// rfid — command-line client for the "Unify Standard" UHF RFID reader.
//
// Usage:
//   rfid --tcp HOST:PORT       <command> [args]
//   rfid --serial PORT[:BAUD]  <command> [args]
//
// See print_usage() below or client/README.md for the command list.
#include <atomic>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "protocol.hpp"
#include "transport.hpp"

using namespace rfid;

static std::atomic<bool> g_stop{false};
static void on_sigint(int) { g_stop = true; }

// ---------------- small arg helpers ----------------

namespace {

struct Args {
    std::vector<std::string> pos;  // positional args (after the command)
    // flags
    bool nosave = false;
    bool tid_match = false;
    Bytes epc;       // --epc (match EPC), empty = match all
    Bytes pwd{0, 0, 0, 0};  // --pwd, defaults to no password
    uint16_t device = 0x0000;  // --device
    double timeout = 3.0;      // --timeout, seconds to wait for a command reply
};

[[noreturn]] void die(const std::string& msg) {
    std::cerr << "error: " << msg << "\n";
    std::exit(2);
}

long to_int(const std::string& s) {
    try {
        size_t idx = 0;
        long v = std::stol(s, &idx, 0);
        if (idx != s.size()) throw std::invalid_argument(s);
        return v;
    } catch (...) {
        die("not a number: " + s);
    }
}

void push_be16(Bytes& b, uint32_t v) {
    b.push_back(static_cast<uint8_t>((v >> 8) & 0xFF));
    b.push_back(static_cast<uint8_t>(v & 0xFF));
}

}  // namespace

// ---------------- reader client ----------------

class Client {
public:
    explicit Client(std::unique_ptr<Transport> t) : t_(std::move(t)) {}
    void open() { t_->open(); }
    void close() { t_->close(); }

    void send(uint8_t cmd, const Bytes& body, uint16_t device_no) {
        t_->write(build_frame(cmd, body, device_no));
    }

    Frame read_one() {
        return read_frame([this](size_t n) { return t_->read_exact(n); });
    }

    void set_request_timeout(double seconds) { request_timeout_ = seconds; }

    // Send a command and return the first matching device reply, skipping
    // asynchronous heartbeat (0x06) and tag-report frames. Applies a read
    // timeout so commands the firmware ignores fail cleanly instead of hanging;
    // blocking mode is restored on the way out (RAII) for streaming reads.
    Frame request(uint8_t cmd, const Bytes& body, uint16_t device_no) {
        send(cmd, body, device_no);
        t_->set_read_timeout(request_timeout_);
        struct RestoreBlocking {
            Transport* t;
            ~RestoreBlocking() { t->set_read_timeout(0); }
        } restore{t_.get()};

        for (int i = 0; i < 64; ++i) {
            Frame f;
            try {
                f = read_one();
            } catch (const Timeout&) {
                char buf[96];
                std::snprintf(buf, sizeof(buf),
                              "no reply from device for command 0x%02X (timeout %.1fs)",
                              cmd, request_timeout_);
                throw ProtocolError(buf);
            }
            if (f.cmd == static_cast<uint8_t>(Cmd::HEARTBEAT)) continue;
            if (is_tag_report_body(f.cmd, f.body)) continue;
            if (f.cmd == cmd) return f;
        }
        throw ProtocolError("no matching reply for command");
    }

    Transport& transport() { return *t_; }

private:
    std::unique_ptr<Transport> t_;
    double request_timeout_ = 3.0;
};

// ---------------- command implementations ----------------

static void warn_checksum(const Frame& f) {
    if (!f.checksum_ok) std::cerr << "warning: checksum mismatch on reply\n";
}

static int cmd_version(Client& c, const Args& a) {
    Frame f = c.request(static_cast<uint8_t>(Cmd::GET_VERSION), {}, a.device);
    warn_checksum(f);
    if (f.body.size() >= 5) {
        std::printf("hardware: %02X %02X\n", f.body[0], f.body[1]);
        std::printf("software: %d.%d.%d\n", f.body[2], f.body[3], f.body[4]);
    } else {
        std::printf("raw: %s\n", to_hex(f.body, true).c_str());
    }
    return 0;
}

static int cmd_info(Client& c, const Args& a) {
    Frame f = c.request(static_cast<uint8_t>(Cmd::GET_DEVICE_INFO), {}, a.device);
    warn_checksum(f);
    std::printf("device info (raw): %s\n", to_hex(f.body, true).c_str());
    return 0;
}

static int cmd_simple_ack(Client& c, uint8_t cmd, const char* name, const Args& a) {
    Frame f = c.request(cmd, {}, a.device);
    warn_checksum(f);
    uint8_t state = f.body.empty() ? 0xFF : f.body[0];
    std::printf("%s: %s (state=0x%02X)\n", name, state == 0 ? "OK" : "ERROR", state);
    return state == 0 ? 0 : 1;
}

static void print_tag(const Frame& f) {
    TagReport r = parse_tag_report(f);
    std::printf("%s\n", r.to_string().c_str());
    std::fflush(stdout);
}

static int cmd_inventory(Client& c, const Args& a) {
    Frame ack = c.request(static_cast<uint8_t>(Cmd::START_INVENTORY), {}, a.device);
    warn_checksum(ack);
    if (!ack.body.empty() && ack.body[0] != 0) {
        std::fprintf(stderr, "start inventory failed (state=0x%02X)\n", ack.body[0]);
        return 1;
    }
    std::fprintf(stderr, "inventory running — Ctrl+C to stop\n");
    try {
        while (!g_stop) {
            Frame f = c.read_one();
            if (is_tag_report_body(f.cmd, f.body)) {
                print_tag(f);
            } else if (f.cmd == static_cast<uint8_t>(Cmd::HEARTBEAT)) {
                // ignore periodic heartbeat
            }
        }
    } catch (const Interrupted&) {
        // fall through to stop
    }
    std::fprintf(stderr, "\nstopping...\n");
    try {
        Frame f = c.request(static_cast<uint8_t>(Cmd::STOP_INVENTORY), {}, a.device);
        std::fprintf(stderr, "stopped (state=0x%02X)\n", f.body.empty() ? 0xFF : f.body[0]);
    } catch (const std::exception& e) {
        std::fprintf(stderr, "stop failed: %s\n", e.what());
    }
    return 0;
}

static int cmd_single(Client& c, const Args& a) {
    c.send(static_cast<uint8_t>(Cmd::SINGLE_INVENTORY), {}, a.device);
    // Device: start-ack, then tag reports, then a stop frame (cmd 0x8E, len 2).
    while (true) {
        Frame f = c.read_one();
        if (is_tag_report_body(f.cmd, f.body)) {
            print_tag(f);
        } else if (f.cmd == static_cast<uint8_t>(Cmd::SINGLE_INVENTORY)) {
            if (f.body.size() >= 2) {  // stop frame: state + substate
                std::fprintf(stderr, "single scan done (state=0x%02X sub=0x%02X)\n",
                             f.body[0], f.body[1]);
                break;
            }
        } else if (f.cmd == static_cast<uint8_t>(Cmd::HEARTBEAT)) {
            continue;
        }
    }
    return 0;
}

static uint8_t rw_area(const std::string& s) {
    if (s == "reserved") return 0x00;
    if (s == "epc") return 0x01;
    if (s == "tid") return 0x02;
    if (s == "user") return 0x03;
    die("area must be one of: reserved|epc|tid|user");
}

static int cmd_read(Client& c, const Args& a) {
    if (a.pos.size() < 3) die("usage: read <reserved|epc|tid|user> <addr> <words>");
    uint8_t area = rw_area(a.pos[0]);
    uint16_t addr = static_cast<uint16_t>(to_int(a.pos[1]));
    uint8_t words = static_cast<uint8_t>(to_int(a.pos[2]));

    Bytes body;
    body.push_back(0x00);                      // model: standard label
    body.push_back(a.tid_match ? 0xA1 : 0x00); // reserve (A1 = match by TID)
    Bytes prefix = build_match_prefix(a.epc, a.pwd);
    body.insert(body.end(), prefix.begin(), prefix.end());
    body.push_back(area);
    push_be16(body, addr);
    body.push_back(words);

    Frame f = c.request(static_cast<uint8_t>(Cmd::READ_TAG_DATA), body, a.device);
    warn_checksum(f);
    uint8_t state = f.body.empty() ? 0xFF : f.body[0];
    if (state != 0) {
        std::printf("read failed (state=0x%02X)\n", state);
        return 1;
    }
    uint8_t rlen = f.body[1];  // in words
    Bytes data(f.body.begin() + 2, f.body.begin() + 2 + rlen * 2);
    std::printf("data: %s\n", to_hex(data, true).c_str());
    if (f.body.size() > 2 + static_cast<size_t>(rlen) * 2)
        std::printf("antenna: %d\n", f.body[2 + rlen * 2]);
    return 0;
}

static int cmd_write(Client& c, const Args& a) {
    if (a.pos.size() < 3) die("usage: write <reserved|epc|tid|user> <addr> <dataHEX>");
    uint8_t area = rw_area(a.pos[0]);
    uint16_t addr = static_cast<uint16_t>(to_int(a.pos[1]));
    Bytes data = from_hex(a.pos[2]);
    if (data.size() % 2 != 0) die("write data must be a whole number of 16-bit words (even bytes)");
    uint8_t words = static_cast<uint8_t>(data.size() / 2);

    Bytes body;
    body.push_back(0x00);                      // type: standard label
    body.push_back(a.tid_match ? 0xA1 : 0x00); // reserve
    Bytes prefix = build_match_prefix(a.epc, a.pwd);
    body.insert(body.end(), prefix.begin(), prefix.end());
    body.push_back(area);
    push_be16(body, addr);
    body.push_back(words);
    body.insert(body.end(), data.begin(), data.end());

    Frame f = c.request(static_cast<uint8_t>(Cmd::WRITE_TAG_DATA), body, a.device);
    warn_checksum(f);
    uint8_t state = f.body.empty() ? 0xFF : f.body[0];
    std::printf("write: %s (state=0x%02X)", state == 0 ? "OK" : "FAILED", state);
    if (state == 0 && f.body.size() >= 2) std::printf(", antenna=%d", f.body[1]);
    std::printf("\n");
    return state == 0 ? 0 : 1;
}

static int cmd_lock(Client& c, const Args& a) {
    if (a.pos.size() < 2)
        die("usage: lock <kill|access|epc|tid|user> <open|permaopen|lock|permalock>");
    uint8_t area;
    const std::string& s = a.pos[0];
    if (s == "kill") area = 0x00;
    else if (s == "access") area = 0x01;
    else if (s == "epc") area = 0x02;
    else if (s == "tid") area = 0x03;
    else if (s == "user") area = 0x04;
    else { die("lock area must be: kill|access|epc|tid|user"); }

    uint8_t action;
    const std::string& act = a.pos[1];
    if (act == "open") action = 0x00;
    else if (act == "permaopen") action = 0x01;
    else if (act == "lock") action = 0x02;
    else if (act == "permalock") action = 0x03;
    else { die("lock action must be: open|permaopen|lock|permalock"); }

    Bytes body;
    body.push_back(0x00);  // type
    body.push_back(0x00);  // reserve
    Bytes prefix = build_match_prefix(a.epc, a.pwd);
    body.insert(body.end(), prefix.begin(), prefix.end());
    body.push_back(area);
    body.push_back(action);

    Frame f = c.request(static_cast<uint8_t>(Cmd::LOCK_TAG), body, a.device);
    warn_checksum(f);
    uint8_t state = f.body.empty() ? 0xFF : f.body[0];
    std::printf("lock: %s (state=0x%02X)\n", state == 0 ? "OK" : "FAILED", state);
    return state == 0 ? 0 : 1;
}

static int cmd_kill(Client& c, const Args& a) {
    // The kill password is the 4-byte --pwd; an all-zero password is invalid to
    // the tag but we let the device reject it rather than second-guess here.
    Bytes body;
    body.push_back(0x00);  // type
    body.push_back(0x00);  // reserve
    Bytes prefix = build_match_prefix(a.epc, a.pwd);
    body.insert(body.end(), prefix.begin(), prefix.end());

    Frame f = c.request(static_cast<uint8_t>(Cmd::KILL_TAG), body, a.device);
    warn_checksum(f);
    uint8_t state = f.body.empty() ? 0xFF : f.body[0];
    std::printf("kill: %s (state=0x%02X)\n", state == 0 ? "OK" : "FAILED", state);
    return state == 0 ? 0 : 1;
}

static int cmd_get_area(Client& c, const Args& a) {
    Frame f = c.request(static_cast<uint8_t>(Cmd::GET_INVENTORY_AREA), {}, a.device);
    warn_checksum(f);
    if (f.body.size() >= 2)
        std::printf("state=0x%02X area=0x%02X\n", f.body[0], f.body[1]);
    else
        std::printf("raw: %s\n", to_hex(f.body, true).c_str());
    return 0;
}

static int cmd_set_area(Client& c, const Args& a) {
    if (a.pos.empty()) die("usage: set-area <0..6>  (0=reserved 1=EPC 2=TID 3=User 4=EPC+TID 5=EPC+User 6=EPC+TID FastID)");
    Bytes body = {static_cast<uint8_t>(to_int(a.pos[0]))};
    Frame f = c.request(static_cast<uint8_t>(Cmd::SET_INVENTORY_AREA), body, a.device);
    warn_checksum(f);
    uint8_t state = f.body.empty() ? 0xFF : f.body[0];
    std::printf("set-area: %s (state=0x%02X)\n", state == 0 ? "OK" : "FAILED", state);
    return state == 0 ? 0 : 1;
}

static int cmd_get_freq(Client& c, const Args& a) {
    Frame f = c.request(static_cast<uint8_t>(Cmd::GET_FREQUENCY), {}, a.device);
    warn_checksum(f);
    if (f.body.size() < 9) { std::printf("raw: %s\n", to_hex(f.body, true).c_str()); return 0; }
    const Bytes& b = f.body;
    int band = b[1];
    int mhz = (b[2] << 8) | b[3];
    int khz = (b[4] << 8) | b[5];
    int step = (b[6] << 8) | b[7];
    int count = b[8];
    std::printf("band=%d start=%d.%03dMHz step=%dkHz hops=%d\n", band, mhz, khz, step, count);
    return 0;
}

static int cmd_set_freq(Client& c, const Args& a) {
    if (a.pos.size() < 5)
        die("usage: set-freq <band> <int_MHz> <dec_kHz> <step_kHz> <count> [--nosave]");
    Bytes body;
    body.push_back(a.nosave ? 0x01 : 0x00);              // store
    body.push_back(static_cast<uint8_t>(to_int(a.pos[0])));  // band
    push_be16(body, static_cast<uint32_t>(to_int(a.pos[1])));  // MHz integer
    push_be16(body, static_cast<uint32_t>(to_int(a.pos[2])));  // kHz decimal
    push_be16(body, static_cast<uint32_t>(to_int(a.pos[3])));  // step kHz
    body.push_back(static_cast<uint8_t>(to_int(a.pos[4])));     // count
    Frame f = c.request(static_cast<uint8_t>(Cmd::SET_FREQUENCY), body, a.device);
    warn_checksum(f);
    uint8_t state = f.body.empty() ? 0xFF : f.body[0];
    std::printf("set-freq: %s (state=0x%02X)\n", state == 0 ? "OK" : "FAILED", state);
    return state == 0 ? 0 : 1;
}

static int cmd_get_antenna(Client& c, const Args& a) {
    Frame f = c.request(static_cast<uint8_t>(Cmd::GET_ANTENNA), {}, a.device);
    warn_checksum(f);
    const Bytes& b = f.body;
    if (b.size() < 2) { std::printf("raw: %s\n", to_hex(b, true).c_str()); return 0; }
    int nch = b[1];
    int enable_bytes = (nch + 7) / 8;
    size_t pos = 2;
    // enable mask is big-endian, bit0 of the last byte = antenna 1
    uint64_t mask = 0;
    for (int i = 0; i < enable_bytes && pos < b.size(); ++i) mask = (mask << 8) | b[pos++];
    std::printf("channels=%d\n", nch);
    for (int ch = 0; ch < nch && pos + 6 <= b.size(); ++ch) {
        uint8_t ant = b[pos];
        uint8_t power = b[pos + 1];
        int on = (b[pos + 2] << 8) | b[pos + 3];
        int gap = (b[pos + 4] << 8) | b[pos + 5];
        pos += 6;
        bool en = (mask >> (ant - 1)) & 1;
        std::printf("  ant %d: %s power=%ddBm on=%dms gap=%dms\n",
                    ant, en ? "ON " : "off", power, on, gap);
    }
    return 0;
}

static int cmd_set_antenna(Client& c, const Args& a) {
    if (a.pos.size() < 2)
        die("usage: set-antenna <enable-mask-hex> <ant:power:onMs:gapMs> ... [--nosave]\n"
            "  mask bit0 = antenna 1; e.g. 09 enables antennas 1 and 4");
    uint64_t mask = static_cast<uint64_t>(std::stoull(a.pos[0], nullptr, 16));
    int nch = static_cast<int>(a.pos.size()) - 1;
    int enable_bytes = (nch + 7) / 8;

    Bytes body;
    body.push_back(a.nosave ? 0x01 : 0x00);       // store
    body.push_back(static_cast<uint8_t>(nch));    // number of channels
    for (int i = enable_bytes - 1; i >= 0; --i)   // big-endian mask
        body.push_back(static_cast<uint8_t>((mask >> (i * 8)) & 0xFF));

    for (int i = 1; i <= nch; ++i) {
        const std::string& spec = a.pos[i];
        int ant = 0, power = 0, on = 0, gap = 0;
        if (std::sscanf(spec.c_str(), "%d:%d:%d:%d", &ant, &power, &on, &gap) != 4)
            die("bad channel spec '" + spec + "', expected ant:power:onMs:gapMs");
        body.push_back(static_cast<uint8_t>(ant));
        body.push_back(static_cast<uint8_t>(power));
        push_be16(body, static_cast<uint32_t>(on));
        push_be16(body, static_cast<uint32_t>(gap));
    }
    Frame f = c.request(static_cast<uint8_t>(Cmd::SET_ANTENNA), body, a.device);
    warn_checksum(f);
    uint8_t state = f.body.empty() ? 0xFF : f.body[0];
    std::printf("set-antenna: %s (state=0x%02X)\n", state == 0 ? "OK" : "FAILED", state);
    return state == 0 ? 0 : 1;
}

static int cmd_detect_antenna(Client& c, const Args& a) {
    Frame f = c.request(static_cast<uint8_t>(Cmd::DETECT_ANTENNA), {}, a.device);
    warn_checksum(f);
    const Bytes& b = f.body;
    if (b.size() < 2) { std::printf("raw: %s\n", to_hex(b, true).c_str()); return 0; }
    int nch = b[1];
    int enable_bytes = (nch + 7) / 8;
    uint64_t mask = 0;
    for (int i = 0; i < enable_bytes && 2 + i < (int)b.size(); ++i) mask = (mask << 8) | b[2 + i];
    std::printf("channels=%d, connected:", nch);
    for (int ant = 1; ant <= nch; ++ant)
        if ((mask >> (ant - 1)) & 1) std::printf(" %d", ant);
    std::printf("\n");
    return 0;
}

static int cmd_raw(Client& c, const Args& a) {
    if (a.pos.empty()) die("usage: raw <cmd-hex> [body-hex]");
    uint8_t cmd = static_cast<uint8_t>(std::stoul(a.pos[0], nullptr, 16));
    Bytes body = a.pos.size() > 1 ? from_hex(a.pos[1]) : Bytes{};
    Frame f = c.request(cmd, body, a.device);
    std::printf("reply cmd=0x%02X eig=0x%02X body=%s checksum=%s\n",
                f.cmd, f.eig, to_hex(f.body, true).c_str(), f.checksum_ok ? "ok" : "BAD");
    return 0;
}

// ---------------- usage / dispatch ----------------

static void print_usage() {
    std::printf(
        "rfid — UHF RFID reader client\n\n"
        "Connection (one required):\n"
        "  --tcp HOST:PORT           connect over TCP\n"
        "  --serial PORT[:BAUD]      connect over serial (default baud 115200)\n\n"
        "Global options:\n"
        "  --device N                device number (default 0)\n"
        "  --timeout SEC             seconds to wait for a command reply (default 3)\n"
        "  --epc HEX                 match a specific EPC (read/write/lock/kill)\n"
        "  --pwd HEX                 4-byte access/kill password (default 00000000)\n"
        "  --tid-match               match by TID instead of EPC (read/write)\n"
        "  --nosave                  don't persist across power cycle (set-freq/set-antenna)\n\n"
        "Commands:\n"
        "  version                                    read hw/sw version (0x01)\n"
        "  info                                       device information dump (0x07)\n"
        "  inventory                                  continuous read, Ctrl+C to stop (0x02/0x03)\n"
        "  single                                     one scan pass per antenna (0x8E)\n"
        "  get-area / set-area <0..6>                 inventory memory bank (0x81/0x80)\n"
        "  read  <reserved|epc|tid|user> <addr> <words>   read tag memory (0x83)\n"
        "  write <reserved|epc|tid|user> <addr> <dataHEX> write tag memory (0x82)\n"
        "  lock  <kill|access|epc|tid|user> <open|permaopen|lock|permalock>  (0x8A)\n"
        "  kill                                       deactivate tag, needs --pwd (0x8B)\n"
        "  get-freq / set-freq <band> <MHz> <kHz> <stepkHz> <count>   (0x85/0x84)\n"
        "  get-antenna                                antenna config (0x87)\n"
        "  set-antenna <mask-hex> <ant:pwr:onMs:gapMs>...             (0x86)\n"
        "  detect-antenna                             which antennas are connected (0x8C)\n"
        "  restart / factory-reset                    (0x04 / 0x05)\n"
        "  raw <cmd-hex> [body-hex]                    send an arbitrary command\n\n"
        "Examples:\n"
        "  rfid --tcp 192.168.1.200:6000 inventory\n"
        "  rfid --serial /dev/ttyUSB0:115200 version\n"
        "  rfid --tcp 192.168.1.200:6000 write user 2 AABBCCDD --pwd 12345678\n");
}

int main(int argc, char** argv) {
    std::string tcp, serial, command;
    Args a;

    // Parse leading options and the command; remaining tokens become positional.
    std::vector<std::string> tokens(argv + 1, argv + argc);
    size_t i = 0;
    auto need = [&](const char* opt) -> std::string {
        if (i + 1 >= tokens.size()) die(std::string("missing value for ") + opt);
        return tokens[++i];
    };
    for (; i < tokens.size(); ++i) {
        const std::string& t = tokens[i];
        if (t == "-h" || t == "--help") { print_usage(); return 0; }
        else if (t == "--tcp") tcp = need("--tcp");
        else if (t == "--serial") serial = need("--serial");
        else if (t == "--device") a.device = static_cast<uint16_t>(to_int(need("--device")));
        else if (t == "--timeout") a.timeout = std::stod(need("--timeout"));
        else if (t == "--epc") a.epc = from_hex(need("--epc"));
        else if (t == "--pwd") {
            a.pwd = from_hex(need("--pwd"));
            if (a.pwd.size() != 4) die("--pwd must be exactly 4 bytes (8 hex digits)");
        }
        else if (t == "--tid-match") a.tid_match = true;
        else if (t == "--nosave") a.nosave = true;
        else if (!t.empty() && t[0] == '-') die("unknown option: " + t);
        else if (command.empty()) command = t;
        else a.pos.push_back(t);
    }

    if (command.empty()) { print_usage(); return 1; }
    if (tcp.empty() == serial.empty())
        die("specify exactly one of --tcp or --serial");

    std::unique_ptr<Transport> transport;
    try {
        if (!tcp.empty()) {
            auto [host, port] = parse_endpoint(tcp, "tcp");
            transport = std::make_unique<TcpTransport>(host, port);
        } else {
            auto [port, baud] = parse_endpoint(serial, "serial");
            transport = std::make_unique<SerialTransport>(port, baud);
        }
    } catch (const std::exception& e) {
        die(e.what());
    }

    // SIGINT without SA_RESTART so a blocking read returns EINTR -> Interrupted.
    struct sigaction sa{};
    sa.sa_handler = on_sigint;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;
    sigaction(SIGINT, &sa, nullptr);

    Client c(std::move(transport));
    c.set_request_timeout(a.timeout);
    try {
        c.open();
    } catch (const std::exception& e) {
        std::fprintf(stderr, "connection failed: %s\n", e.what());
        return 3;
    }

    int rc = 0;
    try {
        if (command == "version") rc = cmd_version(c, a);
        else if (command == "info") rc = cmd_info(c, a);
        else if (command == "inventory") rc = cmd_inventory(c, a);
        else if (command == "single") rc = cmd_single(c, a);
        else if (command == "get-area") rc = cmd_get_area(c, a);
        else if (command == "set-area") rc = cmd_set_area(c, a);
        else if (command == "read") rc = cmd_read(c, a);
        else if (command == "write") rc = cmd_write(c, a);
        else if (command == "lock") rc = cmd_lock(c, a);
        else if (command == "kill") rc = cmd_kill(c, a);
        else if (command == "get-freq") rc = cmd_get_freq(c, a);
        else if (command == "set-freq") rc = cmd_set_freq(c, a);
        else if (command == "get-antenna") rc = cmd_get_antenna(c, a);
        else if (command == "set-antenna") rc = cmd_set_antenna(c, a);
        else if (command == "detect-antenna") rc = cmd_detect_antenna(c, a);
        else if (command == "restart") rc = cmd_simple_ack(c, static_cast<uint8_t>(Cmd::RESTART), "restart", a);
        else if (command == "factory-reset") rc = cmd_simple_ack(c, static_cast<uint8_t>(Cmd::FACTORY_RESET), "factory-reset", a);
        else if (command == "raw") rc = cmd_raw(c, a);
        else { std::fprintf(stderr, "unknown command: %s\n", command.c_str()); rc = 2; }
    } catch (const std::exception& e) {
        std::fprintf(stderr, "error: %s\n", e.what());
        rc = 1;
    }

    c.close();
    return rc;
}
