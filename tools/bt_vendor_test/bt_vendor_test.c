// Host test: BT vendor-command wire formats (scan result, status, notify).
// Field order, sizes and little-endian encoding are checked byte-for-byte.
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "bt_proto.h"

static int fails = 0;
#define CHECK(c) do { if (!(c)) { printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #c); fails++; } } while (0)

int main(void) {
    CHECK(BT_SCAN_RESULT_SIZE == 32);
    CHECK(BT_STATUS_PACKET_SIZE == 16);
    CHECK(BT_NOTIFY_PAYLOAD_SIZE == 14);

    // ---- scan result ----
    BtScanResult sr = { .addr = {0x94,0xDB,0x56,0x53,0x0C,0x19}, .rssi = -57,
                        .flags = BT_SCAN_FLAG_AUDIO, .cod = 0x240418,
                        .index = 2, .count = 5 };
    strcpy(sr.name, "WH-1000XM4");
    uint8_t b[32];
    bt_scan_result_encode(&sr, b);
    const uint8_t exp_sr[32] = {0x94,0xDB,0x56,0x53,0x0C,0x19, 0xC7, 0x01,
                                0x18,0x04,0x24,0x00, 2, 5,
                                'W','H','-','1','0','0','0','X','M','4'};
    CHECK(memcmp(b, exp_sr, 32) == 0);
    BtScanResult sr2; bt_scan_result_decode(b, &sr2);
    CHECK(memcmp(sr.addr, sr2.addr, 6) == 0);
    CHECK(sr2.rssi == -57 && sr2.flags == BT_SCAN_FLAG_AUDIO && sr2.cod == 0x240418);
    CHECK(sr2.index == 2 && sr2.count == 5 && strcmp(sr2.name, "WH-1000XM4") == 0);

    // Over-long name is truncated and always NUL-terminated.
    BtScanResult lg = {0};
    memset(lg.name, 'x', sizeof lg.name); lg.name[sizeof lg.name - 1] = 0;
    bt_scan_result_encode(&lg, b);
    CHECK(b[31] == 0);

    // ---- status ----
    BtStatus st = { .state = BT_STATE_STREAMING, .flags = BT_STAT_FLAG_ENABLED | BT_STAT_FLAG_RADIO_UP,
                    .codec = BT_CODEC_SBC, .volume = 100,
                    .addr = {1,2,3,4,5,6}, .sample_rate = 44100,
                    .scan_count = 3, .last_status = 0x0D };
    uint8_t s[16];
    bt_status_encode(&st, s);
    const uint8_t exp_st[16] = {5, 0x05, 1, 100, 1,2,3,4,5,6,
                                0x44,0xAC,0x00,0x00, 3, 0x0D};
    CHECK(memcmp(s, exp_st, 16) == 0);
    BtStatus st2; bt_status_decode(s, &st2);
    CHECK(st2.state == BT_STATE_STREAMING && st2.flags == 0x05 && st2.codec == BT_CODEC_SBC);
    CHECK(st2.volume == 100 && st2.sample_rate == 44100 && st2.scan_count == 3 && st2.last_status == 0x0D);
    CHECK(memcmp(st2.addr, st.addr, 6) == 0);

    // ---- notify payload (bytes after the 4-byte v2 header) ----
    uint8_t n[BT_NOTIFY_PAYLOAD_SIZE];
    bt_notify_payload_encode(&st, n);
    const uint8_t exp_n[14] = {5, 1, 100, 0x05, 1,2,3,4,5,6, 0x44,0xAC,0x00,0x00};
    CHECK(memcmp(n, exp_n, 14) == 0);
    BtStatus st3; bt_notify_payload_decode(n, &st3);
    CHECK(st3.state == 5 && st3.codec == 1 && st3.volume == 100 && st3.flags == 0x05 &&
          st3.sample_rate == 44100 && memcmp(st3.addr, st.addr, 6) == 0);

    // ---- REQ_BT (0x00) op encoding and SETUP-stage validation ----
    CHECK(BT_OP_ENABLE == 0x01 && BT_OP_SCAN == 0x02 && BT_OP_CONNECT == 0x03 &&
          BT_OP_DISCONNECT == 0x04 && BT_OP_FORGET == 0x05);
    CHECK(BT_GET_STATUS == 0x01 && BT_GET_SCAN_RESULT == 0x02);
    CHECK(bt_set_request_valid(BT_OP_ENABLE, 1));
    CHECK(bt_set_request_valid(BT_OP_SCAN, 1));
    CHECK(bt_set_request_valid(BT_OP_DISCONNECT, 1));   // 1 dummy byte keeps it a SET
    CHECK(bt_set_request_valid(BT_OP_CONNECT, 6));
    CHECK(bt_set_request_valid(BT_OP_FORGET, 6));
    CHECK(bt_set_request_valid(BT_OP_CONNECT, 8));      // longer payload is fine
    CHECK(!bt_set_request_valid(BT_OP_ENABLE, 0));      // short payload -> STALL
    CHECK(!bt_set_request_valid(BT_OP_DISCONNECT, 0));
    CHECK(!bt_set_request_valid(BT_OP_CONNECT, 5));
    CHECK(!bt_set_request_valid(BT_OP_FORGET, 5));
    CHECK(!bt_set_request_valid(0x00, 1));              // unknown ops -> STALL
    CHECK(!bt_set_request_valid(0x06, 6));
    CHECK(!bt_set_request_valid(0xFF, 6));
    CHECK(!bt_set_request_valid(0x0100 | BT_OP_ENABLE, 1));   // nonzero high byte

    if (fails) { printf("%d FAILED\n", fails); return 1; }
    printf("bt_vendor_test: all passed\n");
    return 0;
}
