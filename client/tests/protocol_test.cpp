// Unit tests for the frame codec and tag-report parser.
//
// Data is taken verbatim from the vendor protocol PDF (sections 3.2, 3.3, 4.2.1,
// 4.3.4). Uses a hand-rolled CHECK macro rather than assert() so failures are
// still caught in Release builds (where NDEBUG disables assert).
#include <cstdio>
#include <functional>
#include <string>

#include "protocol.hpp"

using namespace rfid;

static int g_failures = 0;

#define CHECK(cond)                                                        \
    do {                                                                   \
        if (!(cond)) {                                                     \
            std::printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);  \
            ++g_failures;                                                  \
        }                                                                  \
    } while (0)

static Frame frame_from_hex(const std::string& hex) {
    Bytes raw = from_hex(hex);
    size_t pos = 0;
    return read_frame([&](size_t n) {
        Bytes b(raw.begin() + pos, raw.begin() + pos + n);
        pos += n;
        return b;
    });
}

static void test_simple_tag_report() {
    // PDF 3.2. NOTE: the PDF prints LEN=0x11, but that is a documentation typo —
    // the body is 17 bytes so LEN must be 0x13; the checksum F8 confirms 17 bytes.
    Frame f = frame_from_hex(
        "43 4D 02 03 13 00 00 00 08 0C C0 01 02 03 04 05 06 07 08 09 0A 0B 0C 01 31 F8");
    CHECK(f.checksum_ok);
    CHECK(is_tag_report_body(f.cmd, f.body));
    TagReport r = parse_tag_report(f);
    CHECK(r.tag_class == 0x08);
    CHECK(r.epc_hex() == "0102030405060708090A0B0C");
    CHECK(r.antenna && *r.antenna == 1);
    CHECK(r.rssi_dbm && *r.rssi_dbm == -49);
}

static void test_multi_area_tag_report() {
    // PDF 3.3: multi-area tag (EPC + TID)
    Frame f = frame_from_hex(
        "43 4D 02 03 2A 00 00 00 0D 23 C0 00 04 00 00 00 02 0C 48 57 22 22 33 44 55 66 "
        "3D 0B 99 22 00 0C E2 00 34 12 01 35 02 00 03 B9 6B 8C 00 00 03 3A A8");
    CHECK(f.checksum_ok);
    TagReport r = parse_tag_report(f);
    CHECK(r.tag_class == 0x0D);
    CHECK(r.multi_area.has_value());
    CHECK(to_hex(r.multi_area->epc.data) == "48572222334455663D0B9922");
    CHECK(to_hex(r.multi_area->tid.data) == "E20034120135020003B96B8C");
    CHECK(r.epc_hex() == "48572222334455663D0B9922");
    CHECK(r.antenna && *r.antenna == 3);
    CHECK(r.rssi_dbm && *r.rssi_dbm == -58);
}

static void test_build_version_frame() {
    // PDF 4.2.1: host issues 43 4D 01 02 02 00 00 00 00
    Bytes vf = build_frame(0x01, {}, 0x0000);
    CHECK(to_hex(vf) == "43" "4D" "01" "02" "02" "00" "00" "00" "00");
}

static void test_build_read_frame() {
    // PDF 4.3.4: read User area, addr 0, 2 words, no match/pwd.
    // Host bytes (ignoring the trailing checksum, which the PDF shows as skip-check FF):
    //   43 4D 83 02 0D 00 00 00 00 00 00 00 00 00 00 03 00 00 02
    Bytes body;
    body.push_back(0x00);  // model
    body.push_back(0x00);  // reserve
    Bytes prefix = build_match_prefix({}, {0, 0, 0, 0});
    body.insert(body.end(), prefix.begin(), prefix.end());
    body.push_back(0x03);  // area = User
    body.push_back(0x00);
    body.push_back(0x00);  // addr 0x0000
    body.push_back(0x02);  // 2 words
    Bytes rf = build_frame(0x83, body);
    Bytes without_checksum(rf.begin(), rf.end() - 1);
    CHECK(to_hex(without_checksum) ==
          to_hex(from_hex("43 4D 83 02 0D 00 00 00 00 00 00 00 00 00 00 03 00 00 02")));
}

static void test_checksum_roundtrip() {
    Bytes f = build_frame(0x02, from_hex("DEADBEEF"), 0x1234);
    size_t pos = 0;
    Frame parsed = read_frame([&](size_t n) {
        Bytes b(f.begin() + pos, f.begin() + pos + n);
        pos += n;
        return b;
    });
    CHECK(parsed.checksum_ok);
    CHECK(parsed.cmd == 0x02);
    CHECK(parsed.device_no == 0x1234);
    CHECK(to_hex(parsed.body) == "DEADBEEF");
}

int main() {
    test_simple_tag_report();
    test_multi_area_tag_report();
    test_build_version_frame();
    test_build_read_frame();
    test_checksum_roundtrip();

    if (g_failures == 0) {
        std::printf("OK: all protocol tests passed\n");
        return 0;
    }
    std::printf("FAILED: %d check(s)\n", g_failures);
    return 1;
}
