// Wire format for the reader's "Unify Standard" protocol.
//
// Frame layout (see documantation/sdk/.../Document/Protocol/Protocol_20250419-En.pdf):
//
//     43 4D | CMD | EIG | LEN(2, LE) | DEVICE_NO(2, LE) | BODY(N) | CHECKSUM(1)
//
// - LEN = len(DEVICE_NO) + len(BODY) = 2 + N
// - EIG: 0x02 = host -> device, 0x03 = device -> host
// - CHECKSUM = XOR of every byte in DEVICE_NO and BODY (host may send 0xFF to
//   make the device skip verification; the device always computes it for real)
//
// This is a C++17 port of ../reader/protocol.py.
#pragma once

#include <cstdint>
#include <functional>
#include <map>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace rfid {

using Bytes = std::vector<uint8_t>;

constexpr uint8_t STX0 = 0x43;
constexpr uint8_t STX1 = 0x4D;
constexpr uint8_t EIG_HOST = 0x02;
constexpr uint8_t EIG_DEVICE = 0x03;

enum class Cmd : uint8_t {
    GET_VERSION = 0x01,
    START_INVENTORY = 0x02,
    STOP_INVENTORY = 0x03,
    RESTART = 0x04,
    FACTORY_RESET = 0x05,
    HEARTBEAT = 0x06,
    GET_DEVICE_INFO = 0x07,
    SET_INVENTORY_AREA = 0x80,
    GET_INVENTORY_AREA = 0x81,
    WRITE_TAG_DATA = 0x82,
    READ_TAG_DATA = 0x83,
    SET_FREQUENCY = 0x84,
    GET_FREQUENCY = 0x85,
    SET_ANTENNA = 0x86,
    GET_ANTENNA = 0x87,
    DETECT_ANTENNA = 0x8C,
    SINGLE_INVENTORY = 0x8E,
    LOCK_TAG = 0x8A,
    KILL_TAG = 0x8B,
};

enum class TagArea : uint8_t {
    RESERVED = 0x00,
    EPC = 0x01,
    TID = 0x02,
    USER = 0x03,
    EPC_TID = 0x04,
    EPC_USER = 0x05,
    EPC_TID_FASTID = 0x06,  // Impinj Monza 4/6 only
};

constexpr uint8_t UHF_TAG_CLASS = 0x08;
constexpr uint8_t UHF_MULTI_AREA_TAG_CLASS = 0x0D;

const std::map<uint8_t, std::string>& tag_class_names();
std::string tag_class_name(uint8_t cls);

class ProtocolError : public std::runtime_error {
public:
    explicit ProtocolError(const std::string& m) : std::runtime_error(m) {}
};

uint8_t xor_checksum(const Bytes& data);

// Assemble a host -> device frame.
Bytes build_frame(uint8_t cmd, const Bytes& body = {}, uint16_t device_no = 0x0000,
                  bool skip_check = false);

struct Frame {
    uint8_t cmd = 0;
    uint8_t eig = 0;
    uint16_t device_no = 0;
    Bytes body;
    uint8_t checksum = 0;
    bool checksum_ok = false;

    bool is_from_device() const { return eig == EIG_DEVICE; }
};

// read_exact(n) must return exactly n bytes or throw.
//
// Resynchronizes to the start marker: if the stream is misaligned, bytes are
// skipped until the STX (0x43 0x4D) of the next frame. When `skipped` is
// non-null it receives the number of bytes discarded before that STX (0 when
// the stream was already aligned) — useful for logging line noise.
Frame read_frame(const std::function<Bytes(size_t)>& read_exact, size_t* skipped = nullptr);

struct AreaBlock {
    uint8_t start = 0;
    uint8_t length = 0;  // bytes
    bool present = false;
    Bytes data;
};

struct MultiAreaTag {
    uint8_t category = 0;  // 0x00 general, 0x01 encrypted
    uint8_t area = 0;      // see TagArea
    AreaBlock reserved;
    AreaBlock epc;
    AreaBlock tid;
    AreaBlock user;
    Bytes extra;
    bool has_extra = false;
};

struct TagReport {
    uint8_t tag_class = 0;
    Bytes tag_number;
    std::optional<uint8_t> antenna;
    std::optional<int> rssi_dbm;
    std::optional<uint16_t> pc;
    std::optional<uint8_t> tag_area;
    std::optional<uint8_t> trigger_source;
    std::optional<Bytes> alarm_or_private;
    std::optional<uint32_t> timestamp;
    std::optional<Bytes> user_data;
    std::optional<MultiAreaTag> multi_area;

    std::string tag_class_name() const { return rfid::tag_class_name(tag_class); }
    // Best-effort EPC hex: multi-area EPC block if present, else the raw tag number.
    std::string epc_hex() const;
    std::string to_string() const;
};

TagReport parse_tag_report(const Frame& frame);

// CMD 0x02 is overloaded: a 1-byte body is the start/stop ack, >=3 bytes is a tag report.
bool is_tag_report_body(uint8_t cmd, const Bytes& body);

// Shared [match length][match value][password] prefix used by the read/write/
// lock/kill tag-data commands (0x82, 0x83, 0x8A, 0x8B).
Bytes build_match_prefix(const Bytes& match_epc, const Bytes& password);

std::string to_hex(const Bytes& b, bool spaced = false);
Bytes from_hex(const std::string& s);

}  // namespace rfid
