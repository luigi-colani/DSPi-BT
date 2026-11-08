#ifndef BT_PROTO_H
#define BT_PROTO_H

// ----------------------------------------------------------------------------
// Bluetooth output — vendor-command / notification wire formats (Task 5)
// ----------------------------------------------------------------------------
// SDK-free (stdint/string only) so tools/bt_vendor_test can round-trip the
// packets host-side.  All multi-byte fields are LITTLE-ENDIAN on the wire and
// are written/read byte-wise by the encode/decode helpers below (the packets
// are never memcpy'd from a native struct, so layout/padding/host endianness
// cannot leak onto the wire).
//
// Vendor request IDs (0x20..0x26; the 0x00-0x2F range was unused):
//   0x20 REQ_BT_SET_ENABLE     SET  payload u8 (0 = off, 1 = on)         deferred
//   0x21 REQ_BT_SCAN           SET  payload u8 (1 = start, 0 = stop)     deferred
//   0x22 REQ_BT_GET_SCAN_RESULT GET wValue = index; 32-byte BtScanResult
//   0x23 REQ_BT_CONNECT        SET  payload 6-byte BD_ADDR (wire order = textual order)  deferred
//   0x24 REQ_BT_DISCONNECT     GET-path no-payload cmd (wLength>=1), returns status byte 0   deferred
//   0x25 REQ_BT_FORGET         SET  payload 6-byte BD_ADDR (drops the bond)  deferred
//   0x26 REQ_BT_GET_STATUS     GET  16-byte BtStatus
// ----------------------------------------------------------------------------

#include <stdint.h>
#include <string.h>

#define BT_REQ_FIRST  0x20
#define BT_REQ_LAST   0x26

// Connection / stream state (BtStatus.state, NOTIFY_EVT_BT_STATE state byte).
typedef enum {
    BT_STATE_OFF        = 0,   // disabled, or radio not (yet) up
    BT_STATE_IDLE       = 1,   // radio up, no link
    BT_STATE_SCANNING   = 2,   // inquiry running (no link)
    BT_STATE_CONNECTING = 3,   // A2DP establish in progress
    BT_STATE_CONNECTED  = 4,   // A2DP signalling up, stream not yet playing
    BT_STATE_STREAMING  = 5,   // A2DP stream started, mains flowing
} BtState;

#define BT_CODEC_NONE  0
#define BT_CODEC_SBC   1

#define BT_STAT_FLAG_ENABLED   0x01
#define BT_STAT_FLAG_SCANNING  0x02
#define BT_STAT_FLAG_RADIO_UP  0x04

#define BT_SCAN_FLAG_AUDIO     0x01   // Class of Device major class = Audio/Video
#define BT_SCAN_FLAG_NAMED     0x02   // name field is a resolved name
#define BT_SCAN_FLAG_BONDED    0x04   // a link key is stored for this device

// BtStatus.last_status: result of the most recent deferred command (0 = ok),
// or the A2DP/HCI status code of the last failed connect.
#define BT_ERR_NONE        0x00
#define BT_ERR_DISABLED    0xF1   // command needs BT enabled + radio up
#define BT_ERR_BUSY        0xF2   // refused: scan/connect while a link is being set up or streaming
#define BT_ERR_RADIO       0xF3   // radio bring-up failed

#define BT_SCAN_NAME_LEN       18
#define BT_SCAN_RESULT_SIZE    32
#define BT_STATUS_PACKET_SIZE  16
#define BT_NOTIFY_PAYLOAD_SIZE 14     // bytes following the 4-byte v2 header

// Scan result (REQ_BT_GET_SCAN_RESULT), 32 bytes:
//  0..5  addr   6  rssi (int8)   7  flags   8..11 cod (LE, 24 bit used)
// 12 index  13 count (total results)  14..31 name (NUL-terminated, UTF-8)
typedef struct {
    uint8_t  addr[6];
    int8_t   rssi;
    uint8_t  flags;
    uint32_t cod;
    uint8_t  index;
    uint8_t  count;
    char     name[BT_SCAN_NAME_LEN];
} BtScanResult;

// Status (REQ_BT_GET_STATUS), 16 bytes:
//  0 state  1 flags  2 codec  3 volume(0..127)  4..9 addr (connected/target)
// 10..13 sample_rate (LE Hz, 0 = none)  14 scan_count  15 last_status
typedef struct {
    uint8_t  state;
    uint8_t  flags;
    uint8_t  codec;
    uint8_t  volume;
    uint8_t  addr[6];
    uint32_t sample_rate;
    uint8_t  scan_count;
    uint8_t  last_status;     // last A2DP/connect status code (0 = ok); not in notify
} BtStatus;

static inline void bt_put_le32(uint8_t *p, uint32_t v) {
    p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);
}
static inline uint32_t bt_get_le32(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static inline void bt_scan_result_encode(const BtScanResult *r, uint8_t out[BT_SCAN_RESULT_SIZE]) {
    memset(out, 0, BT_SCAN_RESULT_SIZE);
    memcpy(&out[0], r->addr, 6);
    out[6] = (uint8_t)r->rssi;
    out[7] = r->flags;
    bt_put_le32(&out[8], r->cod);
    out[12] = r->index;
    out[13] = r->count;
    size_t n = 0;
    while (n < BT_SCAN_NAME_LEN - 1 && r->name[n]) n++;
    memcpy(&out[14], r->name, n);              // out[14+n..31] already 0 (NUL)
}

static inline void bt_scan_result_decode(const uint8_t in[BT_SCAN_RESULT_SIZE], BtScanResult *r) {
    memcpy(r->addr, &in[0], 6);
    r->rssi  = (int8_t)in[6];
    r->flags = in[7];
    r->cod   = bt_get_le32(&in[8]);
    r->index = in[12];
    r->count = in[13];
    memcpy(r->name, &in[14], BT_SCAN_NAME_LEN);
    r->name[BT_SCAN_NAME_LEN - 1] = 0;
}

static inline void bt_status_encode(const BtStatus *s, uint8_t out[BT_STATUS_PACKET_SIZE]) {
    out[0] = s->state; out[1] = s->flags; out[2] = s->codec; out[3] = s->volume;
    memcpy(&out[4], s->addr, 6);
    bt_put_le32(&out[10], s->sample_rate);
    out[14] = s->scan_count; out[15] = s->last_status;
}

static inline void bt_status_decode(const uint8_t in[BT_STATUS_PACKET_SIZE], BtStatus *s) {
    s->state = in[0]; s->flags = in[1]; s->codec = in[2]; s->volume = in[3];
    memcpy(s->addr, &in[4], 6);
    s->sample_rate = bt_get_le32(&in[10]);
    s->scan_count = in[14]; s->last_status = in[15];
}

// NOTIFY_EVT_BT_STATE payload (after [ver=2, evt, flags=0, seq]), 14 bytes:
//  0 state  1 codec  2 volume  3 flags(BT_STAT_FLAG_*)  4..9 addr  10..13 rate (LE)
static inline void bt_notify_payload_encode(const BtStatus *s, uint8_t out[BT_NOTIFY_PAYLOAD_SIZE]) {
    out[0] = s->state; out[1] = s->codec; out[2] = s->volume; out[3] = s->flags;
    memcpy(&out[4], s->addr, 6);
    bt_put_le32(&out[10], s->sample_rate);
}

static inline void bt_notify_payload_decode(const uint8_t in[BT_NOTIFY_PAYLOAD_SIZE], BtStatus *s) {
    memset(s, 0, sizeof *s);
    s->state = in[0]; s->codec = in[1]; s->volume = in[2]; s->flags = in[3];
    memcpy(s->addr, &in[4], 6);
    s->sample_rate = bt_get_le32(&in[10]);
}

#endif // BT_PROTO_H
