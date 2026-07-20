#include "protocol.hpp"

#include <cctype>
#include <cstdio>
#include <sstream>

namespace rfid {

const std::map<uint8_t, std::string>& tag_class_names() {
    static const std::map<uint8_t, std::string> names = {
        {0x05, "2.4G tag"},
        {0x07, "2.4G bidirectional tag"},
        {0x08, "UHF tag"},
        {0x09, "ETC plate"},
        {0x0A, "ETC plate (multi-info)"},
        {0x0B, "HF tag"},
        {0x0C, "433M tag"},
        {0x0D, "UHF tag (multi-area)"},
        {0x0E, "Bluetooth"},
        {0x0F, "125K LF tag"},
        {0x10, "undefined"},
    };
    return names;
}

std::string tag_class_name(uint8_t cls) {
    auto it = tag_class_names().find(cls);
    if (it != tag_class_names().end()) return it->second;
    char buf[8];
    std::snprintf(buf, sizeof(buf), "0x%02X", cls);
    return buf;
}

uint8_t xor_checksum(const Bytes& data) {
    uint8_t v = 0;
    for (uint8_t b : data) v ^= b;
    return v;
}

Bytes build_frame(uint8_t cmd, const Bytes& body, uint16_t device_no, bool skip_check) {
    Bytes devno = {static_cast<uint8_t>(device_no & 0xFF),
                   static_cast<uint8_t>((device_no >> 8) & 0xFF)};
    uint16_t length = static_cast<uint16_t>(2 + body.size());

    Bytes frame;
    frame.reserve(8 + body.size() + 1);
    frame.push_back(STX0);
    frame.push_back(STX1);
    frame.push_back(cmd);
    frame.push_back(EIG_HOST);
    frame.push_back(static_cast<uint8_t>(length & 0xFF));
    frame.push_back(static_cast<uint8_t>((length >> 8) & 0xFF));
    frame.push_back(devno[0]);
    frame.push_back(devno[1]);
    frame.insert(frame.end(), body.begin(), body.end());

    uint8_t checksum;
    if (skip_check) {
        checksum = 0xFF;
    } else {
        Bytes chk = devno;
        chk.insert(chk.end(), body.begin(), body.end());
        checksum = xor_checksum(chk);
    }
    frame.push_back(checksum);
    return frame;
}

Frame read_frame(const std::function<Bytes(size_t)>& read_exact) {
    Bytes header = read_exact(8);
    if (header[0] != STX0 || header[1] != STX1) {
        char buf[32];
        std::snprintf(buf, sizeof(buf), "bad start bytes: %02x%02x", header[0], header[1]);
        throw ProtocolError(buf);
    }
    Frame f;
    f.cmd = header[2];
    f.eig = header[3];
    uint16_t length = static_cast<uint16_t>(header[4] | (header[5] << 8));
    f.device_no = static_cast<uint16_t>(header[6] | (header[7] << 8));
    int body_len = static_cast<int>(length) - 2;
    if (body_len < 0) {
        throw ProtocolError("implausible frame length field: " + std::to_string(length));
    }
    Bytes rest = read_exact(static_cast<size_t>(body_len) + 1);
    f.body.assign(rest.begin(), rest.begin() + body_len);
    f.checksum = rest[body_len];

    Bytes chk = {header[6], header[7]};
    chk.insert(chk.end(), f.body.begin(), f.body.end());
    uint8_t expected = xor_checksum(chk);
    f.checksum_ok = (f.checksum == expected || f.checksum == 0xFF);
    return f;
}

static AreaBlock parse_area_block(const Bytes& data, size_t& pos) {
    AreaBlock blk;
    blk.start = data[pos];
    blk.length = data[pos + 1];
    pos += 2;
    if (blk.length) {
        blk.present = true;
        blk.data.assign(data.begin() + pos, data.begin() + pos + blk.length);
        pos += blk.length;
    }
    return blk;
}

static MultiAreaTag parse_multi_area(const Bytes& tag_number) {
    MultiAreaTag m;
    m.category = tag_number[0];
    m.area = tag_number[1];
    uint8_t additional_ctrl = tag_number[2];
    size_t pos = 3;
    m.reserved = parse_area_block(tag_number, pos);
    m.epc = parse_area_block(tag_number, pos);
    m.tid = parse_area_block(tag_number, pos);
    m.user = parse_area_block(tag_number, pos);
    if (additional_ctrl && pos < tag_number.size()) {
        m.has_extra = true;
        m.extra.assign(tag_number.begin() + pos, tag_number.end());
    }
    return m;
}

TagReport parse_tag_report(const Frame& frame) {
    const Bytes& body = frame.body;
    TagReport r;
    r.tag_class = body[0];
    uint8_t tag_len = body[1];
    uint8_t options = body[2];
    size_t pos = 3;
    r.tag_number.assign(body.begin() + pos, body.begin() + pos + tag_len);
    pos += tag_len;

    if (options & 0x80) { r.antenna = body[pos]; pos += 1; }
    if (options & 0x40) { r.rssi_dbm = -static_cast<int>(body[pos]); pos += 1; }
    if (options & 0x20) { r.pc = static_cast<uint16_t>((body[pos] << 8) | body[pos + 1]); pos += 2; }
    if (options & 0x10) { r.tag_area = body[pos]; pos += 1; }
    if (options & 0x08) { r.trigger_source = body[pos]; pos += 1; }
    if (options & 0x04) { r.alarm_or_private = Bytes(body.begin() + pos, body.begin() + pos + 4); pos += 4; }
    if (options & 0x02) {
        r.timestamp = static_cast<uint32_t>((body[pos] << 24) | (body[pos + 1] << 16) |
                                            (body[pos + 2] << 8) | body[pos + 3]);
        pos += 4;
    }
    if (options & 0x01) {
        uint8_t ud_len = body[pos];
        pos += 1;
        r.user_data = Bytes(body.begin() + pos, body.begin() + pos + ud_len);
        pos += ud_len;
    }

    if (r.tag_class == UHF_MULTI_AREA_TAG_CLASS) {
        r.multi_area = parse_multi_area(r.tag_number);
    }
    return r;
}

bool is_tag_report_body(uint8_t cmd, const Bytes& body) {
    if (cmd != static_cast<uint8_t>(Cmd::START_INVENTORY)) return false;
    if (body.size() < 3) return false;
    return tag_class_names().count(body[0]) > 0;
}

Bytes build_match_prefix(const Bytes& match_epc, const Bytes& password) {
    if (password.size() != 4) throw std::invalid_argument("password must be exactly 4 bytes");
    Bytes out;
    if (!match_epc.empty()) {
        out.push_back(static_cast<uint8_t>(match_epc.size() * 8));
        out.insert(out.end(), match_epc.begin(), match_epc.end());
    } else {
        out.push_back(0);
    }
    out.insert(out.end(), password.begin(), password.end());
    return out;
}

std::string TagReport::epc_hex() const {
    if (multi_area && multi_area->epc.present) return to_hex(multi_area->epc.data);
    return to_hex(tag_number);
}

std::string TagReport::to_string() const {
    std::ostringstream os;
    os << "TagReport(class=" << tag_class_name() << ", epc=" << epc_hex();
    if (antenna) os << ", ant=" << static_cast<int>(*antenna);
    if (rssi_dbm) os << ", rssi=" << *rssi_dbm << "dBm";
    if (multi_area && multi_area->tid.present) os << ", tid=" << to_hex(multi_area->tid.data);
    os << ")";
    return os.str();
}

std::string to_hex(const Bytes& b, bool spaced) {
    static const char* H = "0123456789ABCDEF";
    std::string s;
    s.reserve(b.size() * (spaced ? 3 : 2));
    for (size_t i = 0; i < b.size(); ++i) {
        if (spaced && i) s.push_back(' ');
        s.push_back(H[b[i] >> 4]);
        s.push_back(H[b[i] & 0x0F]);
    }
    return s;
}

Bytes from_hex(const std::string& s) {
    Bytes out;
    int hi = -1;
    for (char c : s) {
        if (c == ' ' || c == ':' || c == '-' || c == '_') continue;
        int v;
        if (c >= '0' && c <= '9') v = c - '0';
        else if (c >= 'a' && c <= 'f') v = c - 'a' + 10;
        else if (c >= 'A' && c <= 'F') v = c - 'A' + 10;
        else throw std::invalid_argument(std::string("invalid hex char: ") + c);
        if (hi < 0) hi = v;
        else { out.push_back(static_cast<uint8_t>((hi << 4) | v)); hi = -1; }
    }
    if (hi >= 0) throw std::invalid_argument("odd number of hex digits");
    return out;
}

}  // namespace rfid
