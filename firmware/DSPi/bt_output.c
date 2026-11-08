/*
 * bt_output.c — Bluetooth Classic A2DP *source* streaming SBC, fed by the
 * processed main pair of the DSPi audio pipeline (Pico 2 W, DSPI_BT only).
 *
 * Adapted from the BTstack pico-sdk example a2dp_source_demo.c (the proven
 * CYW43 + BTstack A2DP source used by the bring-up spike).  The structure is
 * kept intact — a2dp_source_init, one SBC stream endpoint, AVRCP target +
 * controller, the media-timer send loop, SBC encode — but the audio source is
 * swapped: instead of synthesising a sine/MOD, the media timer pops stereo
 * frames from bt_fifo (the mains, pushed by audio_pipeline.c on Core 0) and
 * SBC-encodes them.  AVRCP (demo stdin, scanning, sample-rate reconfigure UI)
 * is dropped; the sink is a fixed address we auto-connect to at boot.
 *
 * Threading: cyw43_arch + the BTstack run loop live on Core 0 (threadsafe
 * background async context).  bt_fifo_push() runs on the Core-0 audio path
 * (producer); bt_fifo_pop() runs here in the media timer (consumer) — single
 * producer, single consumer, both Core 0, lock-free.
 */

#include "bt_output.h"

#if DSPI_BT

#include <stdint.h>
#include <stdio.h>
#include <inttypes.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
#include "pico/cyw43_arch.h"
#include "tusb.h"          // tud_mounted()
#include "btstack.h"

#include "bt_fifo.h"
#include "bt_metrics.h"
#include "bt_persist.h"       // bt_persist_encode/decode (shared enable contract)
#include "notify.h"

// Boot seed of the persisted BT-enable state.  Forward-declared (not via
// flash_storage.h) so this file keeps its local NUM_CHANNELS==2 A2DP context
// without pulling in config.h's firmware-wide NUM_CHANNELS.  The deferred
// persist write (preset_set_bt_enabled) is performed from main.c.
bool preset_get_bt_enabled(void);

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

// Fixed test sink: the Sony WH-1000XM4 used by the bring-up spike.  For the
// Core-0 budget measurement the firmware auto-connects to this address at boot.
#define BT_TEST_SINK_ADDR   "94:DB:56:53:0C:19"

#define NUM_CHANNELS            2
#define BYTES_PER_AUDIO_SAMPLE  (2 * NUM_CHANNELS)
#define AUDIO_TIMEOUT_MS        10
#define SBC_STORAGE_SIZE        1030

// Preferred sampling rate offered to the sink; the sink selects the final
// value in its SBC configuration (the WH-1000XM4 offers 44.1 kHz).
#define A2DP_SOURCE_PREFERRED_SAMPLING_RATE 44100

// ---------------------------------------------------------------------------
// Metrics (see bt_metrics.h)
// ---------------------------------------------------------------------------

// The audio-callback overrun counter is the shared output-path counter owned
// by main.c (the USB/audio callback could not get an output buffer).
extern volatile uint32_t spdif_overruns;

static volatile uint32_t s_sbc_encode_us_last = 0;
static volatile uint32_t s_sbc_encode_us_max  = 0;

// ---------------------------------------------------------------------------
// The one shared FIFO instance + its global-instance API (declared in
// bt_fifo.h).  Producer: audio_pipeline.c.  Consumer: the media timer below.
// ---------------------------------------------------------------------------

static bt_fifo_t s_fifo;

// Producer entry — called from the Core-0 audio path; kept in RAM so it is
// safe to run while XIP flash is busy (mirrors usb_audio_ring discipline).
void __not_in_flash_func(bt_fifo_push)(const float *l, const float *r, uint32_t n) {
    bt_fifo_push_n(&s_fifo, l, r, n);
}

uint32_t bt_fifo_pop(int16_t *interleaved, uint32_t n_frames) {
    return bt_fifo_pop_drift(&s_fifo, interleaved, n_frames);
}

void bt_fifo_reset(void) {
    bt_fifo_reset_ring(&s_fifo);
}

BtMetrics bt_output_metrics(void) {
    BtMetrics m;
    m.audio_callback_overruns = spdif_overruns;
    m.fifo_overflow_frames    = s_fifo.overflow_frames;
    m.fifo_underflow_frames   = s_fifo.underflow_frames;
    m.fifo_fill_frames        = s_fifo.drift_fill;
    m.fifo_drift_skip_frames  = s_fifo.drift_skipped_frames;
    m.fifo_drift_dup_frames   = s_fifo.drift_duplicated_frames;
    m.sbc_encode_us_last      = s_sbc_encode_us_last;
    m.sbc_encode_us_max       = s_sbc_encode_us_max;
    return m;
}

// ---------------------------------------------------------------------------
// A2DP / AVRCP state (mirrors the demo)
// ---------------------------------------------------------------------------

typedef struct {
    uint16_t a2dp_cid;
    uint8_t  local_seid;
    uint8_t  remote_seid;
    uint8_t  stream_opened;
    uint16_t avrcp_cid;

    uint32_t time_audio_data_sent;     // ms
    uint32_t acc_num_missed_samples;
    uint32_t samples_ready;
    btstack_timer_source_t audio_timer;
    uint8_t  streaming;
    int      max_media_payload_size;
    uint32_t rtp_timestamp;

    uint8_t  sbc_storage[SBC_STORAGE_SIZE];
    uint16_t sbc_storage_count;
    uint8_t  sbc_ready_to_send;

    uint8_t  volume;
} a2dp_media_sending_context_t;

typedef struct {
    int reconfigure;
    int num_channels;
    int sampling_frequency;
    int block_length;
    int subbands;
    int min_bitpool_value;
    int max_bitpool_value;
    btstack_sbc_channel_mode_t      channel_mode;
    btstack_sbc_allocation_method_t allocation_method;
} media_codec_configuration_sbc_t;

// SBC source capabilities: 44.1/48 kHz, stereo, bitpool 2..53 (as the spike).
static uint8_t media_sbc_codec_capabilities[] = {
    (AVDTP_SBC_44100 << 4) | (AVDTP_SBC_48000 << 4) | AVDTP_SBC_STEREO,
    0xFF,
    2, 53
};

static btstack_packet_callback_registration_t hci_event_callback_registration;

static const char * device_addr_string = BT_TEST_SINK_ADDR;
static bd_addr_t    device_addr;

static uint8_t sdp_a2dp_source_service_buffer[150];
static uint8_t sdp_avrcp_target_service_buffer[200];
static uint8_t sdp_avrcp_controller_service_buffer[200];
static uint8_t device_id_sdp_service_buffer[100];

static media_codec_configuration_sbc_t sbc_configuration;
static const btstack_sbc_encoder_t *   sbc_encoder_instance;
static btstack_sbc_encoder_bluedroid_t sbc_encoder_state;

static uint8_t media_sbc_codec_configuration[4];
static a2dp_media_sending_context_t media_tracker;

static int current_sample_rate = A2DP_SOURCE_PREFERRED_SAMPLING_RATE;

// Minimal AVRCP now-playing entry (the sink may query it; volume keys work
// without rich metadata).
static avrcp_track_t tracks[] = {
    {{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01}, 1, "DSPi", "DSPi", "DSPi Main Output", "audio", 0, 1},
};

// True once the A2DP stream is actively playing, so audio_pipeline.c pushes the
// mains into the FIFO only while the media timer is consuming them.
static volatile bool s_bt_enabled = false;

// cyw43_arch_init() done — guards bt_output_led() before the radio is up.
static bool s_cyw43_ready = false;

// Deferred bring-up state.  cyw43_arch_init() blocks (~1 s radio firmware
// download), so it must NOT run before the main loop — that would stall the
// TinyUSB enumeration that only tud_task() (in the main loop) services.  The
// heavy bring-up is deferred to bt_output_poll(), gated on USB being mounted
// so audio + the CDC serial come up first (and the banner can flush).
static bool     s_bringup_done = false;
static uint32_t s_bringup_ticks = 0;

// Periodic [btm] stats line.
static btstack_timer_source_t stats_timer;

// ---------------------------------------------------------------------------
// Control-surface state (Task 5).  Pending flags are written by the vendor
// handlers (EP0 context) and consumed by bt_output_poll() (main loop) — the
// BTstack / radio calls never run in the control-transfer context.
// ---------------------------------------------------------------------------
#define BT_SCAN_MAX        12
#define BT_INQUIRY_UNITS   6        // x1.28 s ~= 7.7 s

typedef struct {
    bd_addr_t addr;
    int8_t    rssi;
    uint8_t   flags;
    uint32_t  cod;
    uint8_t   psrm;
    uint16_t  clock_offset;
    char      name[BT_SCAN_NAME_LEN];
} bt_scan_entry_t;

static bt_scan_entry_t s_scan[BT_SCAN_MAX];
static volatile uint8_t s_scan_count = 0;
static uint8_t s_name_idx = 0;
static volatile bool s_scan_abort = false;
static uint32_t s_redirect_start_ms = 0;

static volatile bool s_enable_pending = false;
static volatile bool s_enable_req = true;
static volatile bool s_scan_pending = false;
static volatile bool s_scan_req = false;
static volatile bool s_connect_pending = false;
static bd_addr_t     s_connect_addr;
static volatile bool s_disconnect_pending = false;
static volatile bool s_forget_pending = false;
static bd_addr_t     s_forget_addr;

// Seeded from the persisted directory value in bt_output_init() (Task 6).
// Default off: an un-provisioned device / a config that predates BT boots with
// the radio idle (bt_persist.h: BT-disabled is the valid default).
static bool s_user_enabled  = false;
static bool s_user_directed = false;  // a command has steered the link: suppress the
                                      // stored-sink boot auto-connect fallback

// Deferred persistence of the enable state (Task 6).  bt_service_commands()
// (main loop) sets these when the state commits; main.c drains them inside its
// flash-write bracket (preset_set_bt_enabled), so the directory write never runs
// nested in the radio stand-down / pipeline-reset bracket.
static volatile bool    s_persist_pending = false;
static volatile uint8_t s_persist_val     = 0;
static void bt_mark_persist(bool on){ s_persist_val = bt_persist_encode_enabled(on); s_persist_pending = true; }
static bool s_radio_ok      = false;  // bring-up fully succeeded
static volatile bool s_hci_working = false;
static volatile bool s_inquiry_active = false;  // inquiry running
static volatile bool s_scanning = false;        // inquiry + name resolution
static volatile bool s_connecting = false;
static volatile bool s_a2dp_connected = false;
static volatile uint8_t s_codec = BT_CODEC_NONE;
static bd_addr_t s_target_addr;
static volatile uint8_t s_last_status = BT_ERR_NONE;

static bool    s_notify_valid = false;
static uint8_t s_notify_last[BT_NOTIFY_PAYLOAD_SIZE];

// ---------------------------------------------------------------------------
// Forward declarations
// ---------------------------------------------------------------------------
static void hci_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);
static void a2dp_source_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t * event, uint16_t event_size);
static void avrcp_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);
static void avrcp_target_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);
static void avrcp_controller_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);

// ---------------------------------------------------------------------------
// Service setup
// ---------------------------------------------------------------------------
static int a2dp_source_and_avrcp_services_init(void){
    // Request role change on reconnecting headset to always use slave mode.
    hci_set_master_slave_policy(0);
    hci_set_inquiry_mode(INQUIRY_MODE_RSSI_AND_EIR);

    l2cap_init();

    a2dp_source_init();
    a2dp_source_register_packet_handler(&a2dp_source_packet_handler);

    avdtp_stream_endpoint_t * local_stream_endpoint = a2dp_source_create_stream_endpoint(
        AVDTP_AUDIO, AVDTP_CODEC_SBC,
        media_sbc_codec_capabilities, sizeof(media_sbc_codec_capabilities),
        media_sbc_codec_configuration, sizeof(media_sbc_codec_configuration));
    if (!local_stream_endpoint){
        printf("A2DP Source: not enough memory to create local stream endpoint\n");
        return 1;
    }
    avdtp_set_preferred_sampling_frequency(local_stream_endpoint, A2DP_SOURCE_PREFERRED_SAMPLING_RATE);
    media_tracker.local_seid = avdtp_local_seid(local_stream_endpoint);
    avdtp_source_register_delay_reporting_category(media_tracker.local_seid);

    avrcp_init();
    avrcp_register_packet_handler(&avrcp_packet_handler);
    avrcp_target_init();
    avrcp_target_register_packet_handler(&avrcp_target_packet_handler);
    avrcp_controller_init();
    avrcp_controller_register_packet_handler(&avrcp_controller_packet_handler);

    sdp_init();

    memset(sdp_a2dp_source_service_buffer, 0, sizeof(sdp_a2dp_source_service_buffer));
    a2dp_source_create_sdp_record(sdp_a2dp_source_service_buffer, sdp_create_service_record_handle(),
                                  AVDTP_SOURCE_FEATURE_MASK_PLAYER, NULL, NULL);
    btstack_assert(de_get_len(sdp_a2dp_source_service_buffer) <= sizeof(sdp_a2dp_source_service_buffer));
    sdp_register_service(sdp_a2dp_source_service_buffer);

    memset(sdp_avrcp_target_service_buffer, 0, sizeof(sdp_avrcp_target_service_buffer));
    uint16_t supported_features = AVRCP_FEATURE_MASK_CATEGORY_PLAYER_OR_RECORDER;
    avrcp_target_create_sdp_record(sdp_avrcp_target_service_buffer, sdp_create_service_record_handle(),
                                   supported_features, NULL, NULL);
    btstack_assert(de_get_len(sdp_avrcp_target_service_buffer) <= sizeof(sdp_avrcp_target_service_buffer));
    sdp_register_service(sdp_avrcp_target_service_buffer);

    memset(sdp_avrcp_controller_service_buffer, 0, sizeof(sdp_avrcp_controller_service_buffer));
    uint16_t controller_supported_features = AVRCP_FEATURE_MASK_CATEGORY_MONITOR_OR_AMPLIFIER;
    avrcp_controller_create_sdp_record(sdp_avrcp_controller_service_buffer, sdp_create_service_record_handle(),
                                       controller_supported_features, NULL, NULL);
    btstack_assert(de_get_len(sdp_avrcp_controller_service_buffer) <= sizeof(sdp_avrcp_controller_service_buffer));
    sdp_register_service(sdp_avrcp_controller_service_buffer);

    memset(device_id_sdp_service_buffer, 0, sizeof(device_id_sdp_service_buffer));
    device_id_create_sdp_record(device_id_sdp_service_buffer, sdp_create_service_record_handle(),
                                DEVICE_ID_VENDOR_ID_SOURCE_BLUETOOTH, BLUETOOTH_COMPANY_ID_BLUEKITCHEN_GMBH, 1, 1);
    btstack_assert(de_get_len(device_id_sdp_service_buffer) <= sizeof(device_id_sdp_service_buffer));
    sdp_register_service(device_id_sdp_service_buffer);

    gap_set_local_name("DSPi A2DP 00:00:00:00:00:00");
    gap_discoverable_control(1);
    gap_set_class_of_device(0x200408);

    hci_event_callback_registration.callback = &hci_packet_handler;
    hci_add_event_handler(&hci_event_callback_registration);

    sscanf_bd_addr(device_addr_string, device_addr);
    return 0;
}

// ---------------------------------------------------------------------------
// Media timer send loop — the only change from the demo is the audio source:
// pop processed mains from bt_fifo and SBC-encode them (silence on underflow).
// ---------------------------------------------------------------------------
static void a2dp_send_media_packet(void){
    int num_bytes_in_frame = sbc_encoder_instance->sbc_buffer_length(&sbc_encoder_state);
    int bytes_in_storage = media_tracker.sbc_storage_count;
    uint8_t num_sbc_frames = bytes_in_storage / num_bytes_in_frame;
    media_tracker.sbc_storage[0] = num_sbc_frames;  // SBC media header
    a2dp_source_stream_send_media_payload_rtp(media_tracker.a2dp_cid, media_tracker.local_seid, 0,
                                              media_tracker.rtp_timestamp,
                                              media_tracker.sbc_storage, bytes_in_storage + 1);
    unsigned int num_audio_samples_per_sbc_buffer = sbc_encoder_instance->num_audio_frames(&sbc_encoder_state);
    media_tracker.rtp_timestamp += num_sbc_frames * num_audio_samples_per_sbc_buffer;
    media_tracker.sbc_storage_count = 0;
    media_tracker.sbc_ready_to_send = 0;
}

static int a2dp_fill_sbc_audio_buffer(a2dp_media_sending_context_t * context){
    int total_num_bytes_read = 0;
    unsigned int num_audio_samples_per_sbc_buffer = sbc_encoder_instance->num_audio_frames(&sbc_encoder_state);
    uint16_t sbc_buffer_length = sbc_encoder_instance->sbc_buffer_length(&sbc_encoder_state);
    while (context->samples_ready >= num_audio_samples_per_sbc_buffer
        && (context->max_media_payload_size - context->sbc_storage_count) >= sbc_buffer_length){

        int16_t pcm_frame[256 * NUM_CHANNELS];

        // Pop the processed mains from the FIFO as interleaved int16; on
        // underflow the FIFO counts the shortfall and we pad with silence so
        // the stream never stalls.
        uint32_t got = bt_fifo_pop(pcm_frame, num_audio_samples_per_sbc_buffer);
        if (got < num_audio_samples_per_sbc_buffer){
            memset(&pcm_frame[got * NUM_CHANNELS], 0,
                   (num_audio_samples_per_sbc_buffer - got) * NUM_CHANNELS * sizeof(int16_t));
        }

        // SBC encode (timed for the Core-0 budget metric).
        uint32_t t0 = time_us_32();
        sbc_encoder_instance->encode_signed_16(&sbc_encoder_state, pcm_frame,
                                               &context->sbc_storage[1 + context->sbc_storage_count]);
        uint32_t dt = time_us_32() - t0;
        s_sbc_encode_us_last = dt;
        if (dt > s_sbc_encode_us_max) s_sbc_encode_us_max = dt;

        total_num_bytes_read += num_audio_samples_per_sbc_buffer;
        context->sbc_storage_count += sbc_buffer_length;
        context->samples_ready     -= num_audio_samples_per_sbc_buffer;
    }
    return total_num_bytes_read;
}

static void a2dp_audio_timeout_handler(btstack_timer_source_t * timer){
    a2dp_media_sending_context_t * context = (a2dp_media_sending_context_t *) btstack_run_loop_get_timer_context(timer);
    btstack_run_loop_set_timer(&context->audio_timer, AUDIO_TIMEOUT_MS);
    btstack_run_loop_add_timer(&context->audio_timer);
    uint32_t now = btstack_run_loop_get_time_ms();

    uint32_t update_period_ms = AUDIO_TIMEOUT_MS;
    if (context->time_audio_data_sent > 0){
        update_period_ms = now - context->time_audio_data_sent;
    }
    uint32_t num_samples = (update_period_ms * current_sample_rate) / 1000;
    context->acc_num_missed_samples += (update_period_ms * current_sample_rate) % 1000;
    while (context->acc_num_missed_samples >= 1000){
        num_samples++;
        context->acc_num_missed_samples -= 1000;
    }
    context->time_audio_data_sent = now;
    context->samples_ready += num_samples;

    if (context->sbc_ready_to_send) return;

    a2dp_fill_sbc_audio_buffer(context);

    if ((context->sbc_storage_count + sbc_encoder_instance->sbc_buffer_length(&sbc_encoder_state)) > context->max_media_payload_size){
        context->sbc_ready_to_send = 1;
        a2dp_source_stream_endpoint_request_can_send_now(context->a2dp_cid, context->local_seid);
    }
}

static void a2dp_timer_start(a2dp_media_sending_context_t * context){
    context->max_media_payload_size = btstack_min(a2dp_max_media_payload_size(context->a2dp_cid, context->local_seid), SBC_STORAGE_SIZE);
    context->sbc_storage_count = 0;
    context->sbc_ready_to_send = 0;
    context->streaming = 1;
    bt_fifo_reset();                 // drop any stale frames from a prior stream
    btstack_run_loop_remove_timer(&context->audio_timer);
    btstack_run_loop_set_timer_handler(&context->audio_timer, a2dp_audio_timeout_handler);
    btstack_run_loop_set_timer_context(&context->audio_timer, context);
    btstack_run_loop_set_timer(&context->audio_timer, AUDIO_TIMEOUT_MS);
    btstack_run_loop_add_timer(&context->audio_timer);
}

static void a2dp_timer_stop(a2dp_media_sending_context_t * context){
    context->time_audio_data_sent = 0;
    context->acc_num_missed_samples = 0;
    context->samples_ready = 0;
    context->streaming = 1;
    context->sbc_storage_count = 0;
    context->sbc_ready_to_send = 0;
    btstack_run_loop_remove_timer(&context->audio_timer);
}

static void dump_sbc_configuration(media_codec_configuration_sbc_t * c){
    printf("[bt] SBC config: ch=%d freq=%d mode=%d block=%d subbands=%d alloc=%d bitpool=[%d,%d]\n",
           c->num_channels, c->sampling_frequency, c->channel_mode, c->block_length,
           c->subbands, c->allocation_method, c->min_bitpool_value, c->max_bitpool_value);
}

static uint8_t bt_compute_state(void){
    if (!s_user_enabled || !s_hci_working) return BT_STATE_OFF;
    if (s_bt_enabled)                      return BT_STATE_STREAMING;
    if (s_a2dp_connected)                  return BT_STATE_CONNECTED;
    if (s_connecting)                      return BT_STATE_CONNECTING;
    if (s_scanning)                        return BT_STATE_SCANNING;
    return BT_STATE_IDLE;
}

void bt_get_status(BtStatus *out){
    memset(out, 0, sizeof *out);
    out->state = bt_compute_state();
    out->flags = (s_user_enabled ? BT_STAT_FLAG_ENABLED : 0)
               | (s_scanning     ? BT_STAT_FLAG_SCANNING : 0)
               | (s_hci_working  ? BT_STAT_FLAG_RADIO_UP : 0);
    out->codec = s_codec;
    out->volume = media_tracker.volume;
    memcpy(out->addr, s_target_addr, 6);
    out->sample_rate = s_codec ? (uint32_t)current_sample_rate : 0;
    out->scan_count = s_scan_count;
    out->last_status = s_last_status;
}

bool bt_get_scan_result(uint8_t index, BtScanResult *out){
    if (index >= s_scan_count) return false;
    const bt_scan_entry_t *e = &s_scan[index];
    memset(out, 0, sizeof *out);
    memcpy(out->addr, e->addr, 6);
    out->rssi = e->rssi; out->flags = e->flags; out->cod = e->cod;
    out->index = index; out->count = s_scan_count;
    memcpy(out->name, e->name, BT_SCAN_NAME_LEN);
    out->name[BT_SCAN_NAME_LEN - 1] = 0;
    return true;
}

static bool bt_is_bonded(const bd_addr_t addr){
    btstack_link_key_iterator_t it;
    if (!gap_link_key_iterator_init(&it)) return false;
    bd_addr_t a; link_key_t k; link_key_type_t t;
    bool found = false;
    while (!found && gap_link_key_iterator_get_next(&it, a, k, &t)){
        if (memcmp(a, addr, 6) == 0) found = true;
    }
    gap_link_key_iterator_done(&it);
    return found;
}

// The stored sink to reconnect to at boot: the first bonded device in the
// BTstack link-key TLV (persisted across power cycles in the dedicated flash
// bank — see flash_storage.c / CMakeLists.txt).  Returns false when no bond is
// stored, so the caller degrades to idle/scannable instead of connecting to a
// hard-coded address.  (One A2DP sink at a time; the first bond is the sink.)
static bool bt_stored_sink(bd_addr_t out){
    btstack_link_key_iterator_t it;
    if (!gap_link_key_iterator_init(&it)) return false;
    bd_addr_t a; link_key_t k; link_key_type_t t;
    bool found = false;
    if (gap_link_key_iterator_get_next(&it, a, k, &t)){
        memcpy(out, a, 6);
        found = true;
    }
    gap_link_key_iterator_done(&it);
    return found;
}

// Resolve the next unnamed scan entry (or finish the scan).
static void bt_scan_next_name(void){
    while (s_name_idx < s_scan_count){
        bt_scan_entry_t *e = &s_scan[s_name_idx];
        if (!(e->flags & BT_SCAN_FLAG_NAMED)){
            if (gap_remote_name_request(e->addr, e->psrm, (uint16_t)(e->clock_offset | 0x8000)) == 0) return;
        }
        s_name_idx++;
    }
    s_scanning = false;
}

// ---------------------------------------------------------------------------
// HCI events — auto-connect to the fixed sink once the controller is working.
// ---------------------------------------------------------------------------
static void hci_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size){
    UNUSED(channel); UNUSED(size);
    if (packet_type != HCI_EVENT_PACKET) return;
    bd_addr_t address;

    switch (hci_event_packet_get_type(packet)){
        case BTSTACK_EVENT_STATE:
            if (btstack_event_state_get_state(packet) != HCI_STATE_WORKING){
                if (btstack_event_state_get_state(packet) == HCI_STATE_OFF) s_hci_working = false;
                break;
            }
            s_hci_working = true;
            // Reconnect-on-boot (Task 6): auto-connect to the STORED bonded sink
            // (the persisted link key) ONLY while no vendor command has directed
            // the link.  Graceful fallback: with no stored bond, stay idle and
            // scannable rather than connect to a hard-coded address; if the
            // establish call errors synchronously, clear the connecting state so
            // a stale/absent bond degrades to idle and never hangs the audio
            // path (the async CONNECTION_ESTABLISHED failure path does the same).
            if (!s_user_directed){
                bd_addr_t sink;
                if (bt_stored_sink(sink)){
                    printf("[bt] HCI working, reconnecting to stored sink %s...\n", bd_addr_to_str(sink));
                    s_connecting = true;
                    memcpy(device_addr, sink, 6);
                    memcpy(s_target_addr, sink, 6);
                    uint8_t st = a2dp_source_establish_stream(device_addr, &media_tracker.a2dp_cid);
                    if (st != ERROR_CODE_SUCCESS){
                        printf("[bt] stored-sink reconnect failed to start (0x%02x); idle/scannable\n", st);
                        s_connecting = false;
                        s_last_status = st;
                    }
                } else {
                    printf("[bt] HCI working, no stored bond; idle/scannable\n");
                }
            } else {
                printf("[bt] HCI working (awaiting vendor-command connect)\n");
            }
            break;
        case HCI_EVENT_PIN_CODE_REQUEST:
            hci_event_pin_code_request_get_bd_addr(packet, address);
            gap_pin_code_response(address, "0000");
            break;

        case GAP_EVENT_INQUIRY_RESULT: {
            gap_event_inquiry_result_get_bd_addr(packet, address);
            uint32_t cod = gap_event_inquiry_result_get_class_of_device(packet);
            if (((cod >> 8) & 0x1F) != 0x04) break;          // Audio/Video major class only
            uint8_t i;
            for (i = 0; i < s_scan_count; i++) if (memcmp(s_scan[i].addr, address, 6) == 0) break;
            if (i == s_scan_count){
                if (s_scan_count >= BT_SCAN_MAX) break;
                memset(&s_scan[i], 0, sizeof s_scan[i]);
                memcpy(s_scan[i].addr, address, 6);
                s_scan_count = (uint8_t)(i + 1);
            }
            bt_scan_entry_t *e = &s_scan[i];
            e->cod = cod;
            e->flags |= BT_SCAN_FLAG_AUDIO;
            if (bt_is_bonded(address)) e->flags |= BT_SCAN_FLAG_BONDED;
            e->psrm = gap_event_inquiry_result_get_page_scan_repetition_mode(packet);
            e->clock_offset = gap_event_inquiry_result_get_clock_offset(packet);
            if (gap_event_inquiry_result_get_rssi_available(packet))
                e->rssi = (int8_t)gap_event_inquiry_result_get_rssi(packet);
            if (gap_event_inquiry_result_get_name_available(packet)){
                uint8_t n = gap_event_inquiry_result_get_name_len(packet);
                if (n > BT_SCAN_NAME_LEN - 1) n = BT_SCAN_NAME_LEN - 1;
                memcpy(e->name, gap_event_inquiry_result_get_name(packet), n);
                e->name[n] = 0;
                e->flags |= BT_SCAN_FLAG_NAMED;
            }
            break;
        }
        case GAP_EVENT_INQUIRY_COMPLETE:
            s_inquiry_active = false;
            s_name_idx = 0;
            if (s_scan_abort){ s_scanning = false; break; }
            bt_scan_next_name();
            break;
        case HCI_EVENT_REMOTE_NAME_REQUEST_COMPLETE:
            if (s_scanning && s_name_idx < s_scan_count){
                bt_scan_entry_t *e = &s_scan[s_name_idx];
                if (hci_event_remote_name_request_complete_get_status(packet) == 0){
                    strncpy(e->name, hci_event_remote_name_request_complete_get_remote_name(packet),
                            BT_SCAN_NAME_LEN - 1);
                    e->name[BT_SCAN_NAME_LEN - 1] = 0;
                }
                e->flags |= BT_SCAN_FLAG_NAMED;      // resolved (or gave up)
                s_name_idx++;
                if (s_scan_abort) s_scanning = false; else bt_scan_next_name();
            }
            break;
        default:
            break;
    }
}

static void a2dp_source_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size){
    UNUSED(channel); UNUSED(size);
    uint8_t status;
    uint8_t local_seid;
    bd_addr_t address;
    uint16_t cid;
    avdtp_channel_mode_t channel_mode;
    uint8_t allocation_method;

    if (packet_type != HCI_EVENT_PACKET) return;
    if (hci_event_packet_get_type(packet) != HCI_EVENT_A2DP_META) return;

    switch (hci_event_a2dp_meta_get_subevent_code(packet)){
        case A2DP_SUBEVENT_SIGNALING_CONNECTION_ESTABLISHED:
            a2dp_subevent_signaling_connection_established_get_bd_addr(packet, address);
            cid = a2dp_subevent_signaling_connection_established_get_a2dp_cid(packet);
            status = a2dp_subevent_signaling_connection_established_get_status(packet);
            s_connecting = false;
            if (status != ERROR_CODE_SUCCESS){
                printf("[bt] A2DP connection failed, status 0x%02x\n", status);
                media_tracker.a2dp_cid = 0;
                s_last_status = status;
                break;
            }
            s_last_status = BT_ERR_NONE;
            s_a2dp_connected = true;
            memcpy(s_target_addr, address, 6);
            media_tracker.a2dp_cid = cid;
            media_tracker.volume = 32;
            printf("[bt] A2DP connected to %s, cid 0x%02x\n", bd_addr_to_str(address), cid);
            break;

        case A2DP_SUBEVENT_SIGNALING_MEDIA_CODEC_SBC_CONFIGURATION:
            cid = avdtp_subevent_signaling_media_codec_sbc_configuration_get_avdtp_cid(packet);
            if (cid != media_tracker.a2dp_cid) return;
            media_tracker.remote_seid = a2dp_subevent_signaling_media_codec_sbc_configuration_get_remote_seid(packet);
            sbc_configuration.reconfigure        = a2dp_subevent_signaling_media_codec_sbc_configuration_get_reconfigure(packet);
            sbc_configuration.num_channels       = a2dp_subevent_signaling_media_codec_sbc_configuration_get_num_channels(packet);
            sbc_configuration.sampling_frequency = a2dp_subevent_signaling_media_codec_sbc_configuration_get_sampling_frequency(packet);
            sbc_configuration.block_length       = a2dp_subevent_signaling_media_codec_sbc_configuration_get_block_length(packet);
            sbc_configuration.subbands           = a2dp_subevent_signaling_media_codec_sbc_configuration_get_subbands(packet);
            sbc_configuration.min_bitpool_value  = a2dp_subevent_signaling_media_codec_sbc_configuration_get_min_bitpool_value(packet);
            sbc_configuration.max_bitpool_value  = a2dp_subevent_signaling_media_codec_sbc_configuration_get_max_bitpool_value(packet);
            channel_mode      = (avdtp_channel_mode_t) a2dp_subevent_signaling_media_codec_sbc_configuration_get_channel_mode(packet);
            allocation_method = a2dp_subevent_signaling_media_codec_sbc_configuration_get_allocation_method(packet);

            // Adapt Bluetooth spec values to the SBC encoder's enums.
            sbc_configuration.allocation_method = (btstack_sbc_allocation_method_t)(allocation_method - 1);
            switch (channel_mode){
                case AVDTP_CHANNEL_MODE_JOINT_STEREO:  sbc_configuration.channel_mode = SBC_CHANNEL_MODE_JOINT_STEREO; break;
                case AVDTP_CHANNEL_MODE_STEREO:        sbc_configuration.channel_mode = SBC_CHANNEL_MODE_STEREO; break;
                case AVDTP_CHANNEL_MODE_DUAL_CHANNEL:  sbc_configuration.channel_mode = SBC_CHANNEL_MODE_DUAL_CHANNEL; break;
                case AVDTP_CHANNEL_MODE_MONO:          sbc_configuration.channel_mode = SBC_CHANNEL_MODE_MONO; break;
                default: btstack_assert(false); break;
            }
            dump_sbc_configuration(&sbc_configuration);

            current_sample_rate = sbc_configuration.sampling_frequency;
            s_codec = BT_CODEC_SBC;
            sbc_encoder_instance = btstack_sbc_encoder_bluedroid_init_instance(&sbc_encoder_state);
            sbc_encoder_instance->configure(&sbc_encoder_state, SBC_MODE_STANDARD,
                                            sbc_configuration.block_length, sbc_configuration.subbands,
                                            sbc_configuration.allocation_method, sbc_configuration.sampling_frequency,
                                            sbc_configuration.max_bitpool_value,
                                            sbc_configuration.channel_mode);
            break;

        case A2DP_SUBEVENT_STREAM_ESTABLISHED:
            a2dp_subevent_stream_established_get_bd_addr(packet, address);
            status = a2dp_subevent_stream_established_get_status(packet);
            if (status != ERROR_CODE_SUCCESS){
                printf("[bt] A2DP stream failed, status 0x%02x\n", status);
                break;
            }
            media_tracker.stream_opened = 1;
            printf("[bt] A2DP stream established, starting playback\n");
            a2dp_source_start_stream(media_tracker.a2dp_cid, media_tracker.local_seid);
            break;

        case A2DP_SUBEVENT_STREAM_STARTED:
            a2dp_timer_start(&media_tracker);
            s_bt_enabled = true;      // pipeline may now push the mains
            if (media_tracker.avrcp_cid){
                avrcp_target_set_now_playing_info(media_tracker.avrcp_cid, &tracks[0], sizeof(tracks)/sizeof(avrcp_track_t));
                avrcp_target_set_playback_status(media_tracker.avrcp_cid, AVRCP_PLAYBACK_STATUS_PLAYING);
            }
            printf("[bt] A2DP stream started\n");
            break;

        case A2DP_SUBEVENT_STREAMING_CAN_SEND_MEDIA_PACKET_NOW:
            a2dp_send_media_packet();
            break;

        case A2DP_SUBEVENT_STREAM_SUSPENDED:
            s_bt_enabled = false;
            a2dp_timer_stop(&media_tracker);
            printf("[bt] A2DP stream suspended\n");
            break;

        case A2DP_SUBEVENT_STREAM_RELEASED:
            s_bt_enabled = false;
            cid = a2dp_subevent_stream_released_get_a2dp_cid(packet);
            if (cid == media_tracker.a2dp_cid) media_tracker.stream_opened = 0;
            a2dp_timer_stop(&media_tracker);
            printf("[bt] A2DP stream released\n");
            break;

        case A2DP_SUBEVENT_SIGNALING_CONNECTION_RELEASED:
            s_bt_enabled = false;
            cid = a2dp_subevent_signaling_connection_released_get_a2dp_cid(packet);
            if (cid == media_tracker.a2dp_cid){
                media_tracker.avrcp_cid = 0;
                media_tracker.a2dp_cid = 0;
            }
            s_a2dp_connected = false;
            s_connecting = false;
            s_codec = BT_CODEC_NONE;
            printf("[bt] A2DP signaling released\n");
            break;

        default:
            break;
    }
}

// ---------------------------------------------------------------------------
// AVRCP (volume + minimal now-playing); not on the audio hot path.
// ---------------------------------------------------------------------------
static void avrcp_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size){
    UNUSED(channel); UNUSED(size);
    bd_addr_t event_addr;
    uint16_t local_cid;
    uint8_t status;

    if (packet_type != HCI_EVENT_PACKET) return;
    if (hci_event_packet_get_type(packet) != HCI_EVENT_AVRCP_META) return;

    switch (packet[2]){
        case AVRCP_SUBEVENT_CONNECTION_ESTABLISHED:
            local_cid = avrcp_subevent_connection_established_get_avrcp_cid(packet);
            status = avrcp_subevent_connection_established_get_status(packet);
            if (status != ERROR_CODE_SUCCESS) return;
            media_tracker.avrcp_cid = local_cid;
            avrcp_subevent_connection_established_get_bd_addr(packet, event_addr);
            avrcp_target_support_event(media_tracker.avrcp_cid, AVRCP_NOTIFICATION_EVENT_PLAYBACK_STATUS_CHANGED);
            avrcp_target_support_event(media_tracker.avrcp_cid, AVRCP_NOTIFICATION_EVENT_TRACK_CHANGED);
            avrcp_target_set_now_playing_info(media_tracker.avrcp_cid, NULL, sizeof(tracks)/sizeof(avrcp_track_t));
            avrcp_controller_enable_notification(media_tracker.avrcp_cid, AVRCP_NOTIFICATION_EVENT_VOLUME_CHANGED);
            return;
        case AVRCP_SUBEVENT_CONNECTION_RELEASED:
            media_tracker.avrcp_cid = 0;
            return;
        default:
            break;
    }
}

static void avrcp_target_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size){
    UNUSED(channel); UNUSED(size);
    if (packet_type != HCI_EVENT_PACKET) return;
    if (hci_event_packet_get_type(packet) != HCI_EVENT_AVRCP_META) return;
    // Volume handling is accepted by the stack; no DSPi-side action needed.
}

static void avrcp_controller_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size){
    UNUSED(channel); UNUSED(size);
    if (packet_type != HCI_EVENT_PACKET) return;
    if (hci_event_packet_get_type(packet) != HCI_EVENT_AVRCP_META) return;
    if (!media_tracker.avrcp_cid) return;
    if (packet[2] == AVRCP_SUBEVENT_NOTIFICATION_VOLUME_CHANGED){
        media_tracker.volume = avrcp_subevent_notification_volume_changed_get_absolute_volume(packet);
        printf("[bt] AVRCP absolute volume %d%%\n",
               avrcp_subevent_notification_volume_changed_get_absolute_volume(packet) * 100 / 127);
    }
}

// ---------------------------------------------------------------------------
// Periodic [btm] stats line (~1/sec) — read by the measurement controller.
// ---------------------------------------------------------------------------
static void stats_timeout_handler(btstack_timer_source_t * ts){
    BtMetrics m = bt_output_metrics();
    printf("[btm] en=%d acb_ovr=%" PRIu32 " fifo_of=%" PRIu32 " fifo_uf=%" PRIu32
           " sbc_us_last=%" PRIu32 " sbc_us_max=%" PRIu32 " avail=%" PRIu32
           " fill=%" PRIu32 " skip=%" PRIu32 " dup=%" PRIu32 "\n",
           (int)s_bt_enabled, m.audio_callback_overruns, m.fifo_overflow_frames,
           m.fifo_underflow_frames, m.sbc_encode_us_last, m.sbc_encode_us_max,
           bt_fifo_available(&s_fifo), m.fifo_fill_frames,
           m.fifo_drift_skip_frames, m.fifo_drift_dup_frames);
    btstack_run_loop_set_timer(ts, 1000);
    btstack_run_loop_add_timer(ts);
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------
void bt_output_init(void){
    // Fast, non-blocking only — this runs before the main loop.
    bt_fifo_init(&s_fifo);

    // Seed the enable state from the persisted device-global directory (Task 6).
    // Runs after preset_boot_load() (main.c ordering), so the directory cache is
    // populated.  Default off: an un-provisioned device stays idle; only a
    // device whose stored config has BT on brings the radio up and reconnects to
    // the stored sink.  This is the boot read; no persist (no change).
    s_user_enabled = preset_get_bt_enabled();
    printf("[bt] boot enable state: %s (persisted)\n", s_user_enabled ? "on" : "off");

    // Bring up USB-serial (CDC) for the [btm] metrics line + BTstack logs.
    // The app already called tud_init(0) in core0_init (usb_sound_card_init),
    // so pico_stdio_usb only enables its stdio driver here — it does not start
    // a second device stack or its own tud_task (LIB_TINYUSB_DEVICE is set).
    // Must run on Core 0 (it is) with TinyUSB already initialised.
    stdio_usb_init();
    // The radio/BTstack bring-up (cyw43_arch_init blocks ~1 s) is deferred to
    // bt_output_poll(), after USB has enumerated.
}

// Deferred heavy bring-up — call every main-loop iteration.  Runs the radio +
// BTstack init exactly once, only after USB is mounted (so audio + the CDC
// serial are up first) and a short settle so the boot banner flushes.
// Restore the wired outputs that bring-up stood down (bracketed so the restart
// is click-free).  bt_radio_standdown(false) re-validates the ADAT pin.
static void bt_restore_outputs(void){
    uint8_t restored = 0;
    bt_pipeline_reset_prepare();
    bt_radio_standdown(false, &restored);
    bt_pipeline_reset_complete();
    printf("[bt] outputs restored mask=0x%02x\n", restored);
}

static void bt_do_disconnect(void){
    if (!(s_a2dp_connected || s_connecting)) return;
    if (media_tracker.a2dp_cid){
        cyw43_arch_lwip_begin();
        a2dp_source_disconnect(media_tracker.a2dp_cid);
        cyw43_arch_lwip_end();
    } else {
        // Connect in flight but no cid yet: nothing addressable to tear down.
        // Clear the flags so the state machine cannot wait on an event that
        // will never arrive (a late CONNECTION_ESTABLISHED re-sets them).
        s_connecting = false;
        s_a2dp_connected = false;
    }
}

static bool bt_link_busy(void){
    return s_connecting || s_a2dp_connected;
}

// Main-loop servicing of the deferred vendor commands (Core 0, never an ISR).
static void bt_service_commands(void){
    // --- enable / disable --------------------------------------------------
    if (s_enable_pending){
        bool want = s_enable_req;
        if (want == s_user_enabled){
            s_enable_pending = false;
        } else if (bt_pipeline_reset_ready()){
            s_enable_pending = false;
            s_user_directed = true;
            if (!want){
                // BT off: drop the link/scan, stop streaming, power the
                // controller down, then give the wired outputs back.
                s_bt_enabled = false;                  // pipeline stops pushing
                cyw43_arch_lwip_begin(); a2dp_timer_stop(&media_tracker); cyw43_arch_lwip_end();
                if (s_inquiry_active){ cyw43_arch_lwip_begin(); gap_inquiry_stop(); cyw43_arch_lwip_end(); }
                s_scan_abort = true; s_scanning = false; s_inquiry_active = false;
                s_connect_pending = s_disconnect_pending = s_forget_pending = s_scan_pending = false;
                cyw43_arch_lwip_begin(); hci_power_control(HCI_POWER_OFF); cyw43_arch_lwip_end();
                s_hci_working = false; s_connecting = false; s_a2dp_connected = false;
                s_codec = BT_CODEC_NONE;
                media_tracker.a2dp_cid = 0; media_tracker.avrcp_cid = 0; media_tracker.stream_opened = 0;
                s_user_enabled = false;
                bt_mark_persist(false);                // persist BT off (deferred)
                printf("[bt] disabled\n");
                bt_restore_outputs();
            } else {
                uint8_t mask = 0;
                bt_pipeline_reset_prepare();
                bt_radio_standdown(true, &mask);
                bt_pipeline_reset_complete();
                s_user_enabled = true;
                bt_mark_persist(true);                 // persist BT on (deferred)
                s_last_status = BT_ERR_NONE;
                cyw43_arch_lwip_begin(); hci_power_control(HCI_POWER_ON); cyw43_arch_lwip_end();  // -> BTSTACK_EVENT_STATE WORKING
                printf("[bt] enabled (stand-down mask=0x%02x)\n", mask);
            }
        }
        return;      // one heavy op per tick
    }
    if (!s_user_enabled){
        // Everything else needs BT on.
        if (s_scan_pending || s_connect_pending || s_disconnect_pending || s_forget_pending){
            s_scan_pending = s_connect_pending = s_disconnect_pending = s_forget_pending = false;
            s_last_status = BT_ERR_DISABLED;
        }
        return;
    }
    if (!s_hci_working) return;      // controller still coming up; keep requests pending

    // --- disconnect --------------------------------------------------------
    if (s_disconnect_pending){
        s_disconnect_pending = false;
        s_user_directed = true;
        bt_do_disconnect();
    }

    // --- forget (drop bond; disconnect first if it is the live peer) --------
    if (s_forget_pending){
        s_forget_pending = false;
        s_user_directed = true;
        if (bt_link_busy() && memcmp(s_target_addr, s_forget_addr, 6) == 0) bt_do_disconnect();
        cyw43_arch_lwip_begin(); gap_drop_link_key_for_bd_addr(s_forget_addr); cyw43_arch_lwip_end();
        for (uint8_t i = 0; i < s_scan_count; i++)
            if (memcmp(s_scan[i].addr, s_forget_addr, 6) == 0) s_scan[i].flags &= (uint8_t)~BT_SCAN_FLAG_BONDED;
        printf("[bt] forgot %s\n", bd_addr_to_str(s_forget_addr));
    }

    // --- connect (stop any scan, drop any other link first) -------------------
    if (s_connect_pending){
        s_user_directed = true;
        if (s_scanning){
            s_scan_abort = true;
            if (s_inquiry_active){ cyw43_arch_lwip_begin(); gap_inquiry_stop(); cyw43_arch_lwip_end(); }
            return;                      // wait for the scan to wind down
        }
        if (bt_link_busy()){
            if (memcmp(s_target_addr, s_connect_addr, 6) == 0){
                s_connect_pending = false;       // already connecting/connected to it
            } else {
                if (s_redirect_start_ms == 0) s_redirect_start_ms = to_ms_since_boot(get_absolute_time()) | 1;
                if (to_ms_since_boot(get_absolute_time()) - s_redirect_start_ms > 5000){
                    // Old link never released: give up waiting, force-clear.
                    s_connecting = false; s_a2dp_connected = false;
                    s_last_status = BT_ERR_BUSY;
                    s_redirect_start_ms = 0;
                    return;
                }
                bt_do_disconnect();
                return;                  // wait for the old link to release
            }
        } else {
            s_connect_pending = false;
            s_redirect_start_ms = 0;
            memcpy(device_addr, s_connect_addr, 6);
            memcpy(s_target_addr, s_connect_addr, 6);
            s_connecting = true;
            s_last_status = BT_ERR_NONE;
            cyw43_arch_lwip_begin();
            uint8_t st = a2dp_source_establish_stream(device_addr, &media_tracker.a2dp_cid);
            cyw43_arch_lwip_end();
            if (st != ERROR_CODE_SUCCESS){
                s_connecting = false;
                s_last_status = st;
            }
            printf("[bt] connect %s -> 0x%02x\n", bd_addr_to_str(device_addr), st);
        }
    }

    // --- scan --------------------------------------------------------------
    if (s_scan_pending){
        s_scan_pending = false;
        if (!s_scan_req){
            s_scan_abort = true;
            if (s_inquiry_active){ cyw43_arch_lwip_begin(); gap_inquiry_stop(); cyw43_arch_lwip_end(); }
        } else if (s_bt_enabled || bt_link_busy()){
            // Inquiry steals radio time slots from an active link: never while streaming.
            s_last_status = BT_ERR_BUSY;
        } else if (!s_scanning){
            s_scan_abort = false;
            s_scan_count = 0;
            s_name_idx = 0;
            s_scanning = true;
            s_inquiry_active = true;
            s_last_status = BT_ERR_NONE;
            cyw43_arch_lwip_begin();
            uint8_t st = (uint8_t)gap_inquiry_start(BT_INQUIRY_UNITS);
            cyw43_arch_lwip_end();
            if (st != ERROR_CODE_SUCCESS){
                s_scanning = false; s_inquiry_active = false; s_last_status = st;
            }
        }
    }
}

// Push NOTIFY_EVT_BT_STATE whenever the (state, codec, volume, flags, addr,
// rate) tuple changes.  Runs here (main loop), never from the BTstack handlers.
static void bt_notify_if_changed(void){
    BtStatus st;
    bt_get_status(&st);
    uint8_t payload[BT_NOTIFY_PAYLOAD_SIZE];
    bt_notify_payload_encode(&st, payload);
    if (s_notify_valid && memcmp(payload, s_notify_last, sizeof payload) == 0) return;
    memcpy(s_notify_last, payload, sizeof payload);
    s_notify_valid = true;
    notify_push_bt_state(payload);
}

// Request recorders (EP0 / transport context): flag only, no stack calls.
void bt_request_enable(bool on){ s_enable_req = on; s_enable_pending = true; }
void bt_request_scan(bool start){ s_scan_req = start; s_scan_pending = true; }
void bt_request_connect(const uint8_t addr[6]){
    memcpy(s_connect_addr, addr, 6); s_connect_pending = true;
}
void bt_request_disconnect(void){ s_disconnect_pending = true; }
void bt_request_forget(const uint8_t addr[6]){
    memcpy(s_forget_addr, addr, 6); s_forget_pending = true;
}

void bt_output_poll(void){
    if (s_bringup_done){
        if (s_radio_ok){
            bt_service_commands();
            bt_notify_if_changed();
        }
        return;
    }
    // Before bring-up the only command that matters is "stay off": a host that
    // disables BT before the radio is up defers the bring-up until it re-enables.
    if (s_enable_pending){
        s_enable_pending = false;
        s_user_enabled = s_enable_req;
        if (!s_user_enabled) s_user_directed = true;
        bt_mark_persist(s_user_enabled);   // persist the new intent (setter dedupes)
    }
    if (!s_user_enabled) return;
    if (!tud_mounted()) return;      // let USB enumerate first

    // Give tud_task() a few iterations after mount so the banner flushes over
    // CDC before the (blocking) radio init stalls the loop briefly.
    if (s_bringup_ticks == 0) {
        printf("[bt] USB mounted; radio bring-up next\n");
    }
    if (++s_bringup_ticks < 200) return;

    // The stand-down + radio init are disruptive: gate on the pipeline being
    // reset-ready like every other reset op, and retry next tick if not (so the
    // mute/settle bracket below is entered from a safe state).
    if (!bt_pipeline_reset_ready()) return;

    s_bringup_done = true;

    // Resource mutual-exclusion MUST run before cyw43_arch_init().  Stand down
    // the PIO1-resident wired outputs (PDM sub, ADAT bulk) as a logical
    // reservation, BRACKETED by prepare/complete_pipeline_reset so the
    // still-active S/PDIF mains stay click-free and the ADAT stream is torn down
    // synchronously (complete -> adat_output_resync) before the blocking init.
    // (Whether the radio actually needs this to find a free SM/DMA — a spare may
    // already exist under USB input — is confirmed by the init line below.)
    uint8_t stood_down = 0;
    bt_pipeline_reset_prepare();
    bt_radio_standdown(true, &stood_down);
    bt_pipeline_reset_complete();
    printf("[bt] output stand-down mask=0x%02x (bit0=PDM bit1=ADAT)\n", stood_down);

    printf("[bt] cyw43_arch_init...\n");
    if (cyw43_arch_init()){
        printf("[bt] cyw43_arch_init FAILED; restoring stood-down outputs\n");
        s_last_status = BT_ERR_RADIO;
        // A radio failure must not leave the user's outputs dead: restore the
        // stand-down (bracketed, so the restart is click-free too).
        uint8_t restored = 0;
        bt_pipeline_reset_prepare();
        bt_radio_standdown(false, &restored);
        bt_pipeline_reset_complete();
        return;
    }
    s_cyw43_ready = true;
    printf("[bt] cyw43 init OK\n");

    if (a2dp_source_and_avrcp_services_init()){
        printf("[bt] A2DP service init failed\n");
        s_last_status = BT_ERR_RADIO;
        return;
    }

    // Periodic [btm] stats line.
    btstack_run_loop_set_timer_handler(&stats_timer, stats_timeout_handler);
    btstack_run_loop_set_timer(&stats_timer, 1000);
    btstack_run_loop_add_timer(&stats_timer);

    // Power on — the background async context drives everything from here,
    // including the auto-connect on HCI_STATE_WORKING.
    s_radio_ok = true;
    cyw43_arch_lwip_begin();
    hci_power_control(HCI_POWER_ON);
    cyw43_arch_lwip_end();
    printf("[bt] A2DP source init done, sink=%s\n", device_addr_string);
}

// Hot path: the audio pipeline reads this every block to decide whether to push
// the mains, so it must be RAM-resident (no flash fetch while XIP is disabled
// during a flash write).  Mirrors the bt_fifo_push placement.
bool __not_in_flash_func(bt_output_is_enabled)(void){
    return s_bt_enabled;
}

void bt_output_led(bool on){
    if (!s_cyw43_ready) return;
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, on);
}

uint32_t bt_output_sink_rate(void){
    return bt_rate_sink_or_default((uint32_t)current_sample_rate);
}

bool bt_output_user_enabled(void){
    return s_user_enabled;
}

bool bt_output_consume_persist_request(bool *on){
    if (!s_persist_pending) return false;
    s_persist_pending = false;
    if (on) *on = bt_persist_decode_enabled(s_persist_val);
    return true;
}

bool bt_radio_resources_held(void){
    // Held while the radio is initialised AND BT is enabled.  After a user
    // disable the wired outputs are restored (the SM/DMA were only logically
    // reserved — see bt_exclusion.h), so the vendor handlers may re-enable them.
    return s_cyw43_ready && s_user_enabled;
}

#endif // DSPI_BT
