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
    return bt_fifo_pop_n(&s_fifo, interleaved, n_frames);
}

void bt_fifo_reset(void) {
    bt_fifo_reset_ring(&s_fifo);
}

BtMetrics bt_output_metrics(void) {
    BtMetrics m;
    m.audio_callback_overruns = spdif_overruns;
    m.fifo_overflow_frames    = s_fifo.overflow_frames;
    m.fifo_underflow_frames   = s_fifo.underflow_frames;
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

// ---------------------------------------------------------------------------
// HCI events — auto-connect to the fixed sink once the controller is working.
// ---------------------------------------------------------------------------
static void hci_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size){
    UNUSED(channel); UNUSED(size);
    if (packet_type != HCI_EVENT_PACKET) return;
    bd_addr_t address;

    switch (hci_event_packet_get_type(packet)){
        case BTSTACK_EVENT_STATE:
            if (btstack_event_state_get_state(packet) != HCI_STATE_WORKING) break;
            printf("[bt] HCI working, connecting to sink %s...\n", device_addr_string);
            a2dp_source_establish_stream(device_addr, &media_tracker.a2dp_cid);
            break;
        case HCI_EVENT_PIN_CODE_REQUEST:
            hci_event_pin_code_request_get_bd_addr(packet, address);
            gap_pin_code_response(address, "0000");
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
            if (status != ERROR_CODE_SUCCESS){
                printf("[bt] A2DP connection failed, status 0x%02x\n", status);
                media_tracker.a2dp_cid = 0;
                break;
            }
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
           " sbc_us_last=%" PRIu32 " sbc_us_max=%" PRIu32 " avail=%" PRIu32 "\n",
           (int)s_bt_enabled, m.audio_callback_overruns, m.fifo_overflow_frames,
           m.fifo_underflow_frames, m.sbc_encode_us_last, m.sbc_encode_us_max,
           bt_fifo_available(&s_fifo));
    btstack_run_loop_set_timer(ts, 1000);
    btstack_run_loop_add_timer(ts);
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------
void bt_output_init(void){
    // Fast, non-blocking only — this runs before the main loop.
    bt_fifo_init(&s_fifo);

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
void bt_output_poll(void){
    if (s_bringup_done) return;
    if (!tud_mounted()) return;      // let USB enumerate first

    // Give tud_task() a few iterations after mount so the banner flushes over
    // CDC before the (blocking) radio init stalls the loop briefly.
    if (s_bringup_ticks == 0) {
        printf("[bt] USB mounted; radio bring-up next\n");
    }
    if (++s_bringup_ticks < 200) return;

    s_bringup_done = true;

    // Resource mutual-exclusion MUST run before cyw43_arch_init(): stand down
    // the PIO1-resident wired outputs (PDM sub, ADAT bulk) so the radio can
    // claim a free PIO state machine + DMA pair + its GPIOs.  Reports which
    // outputs it disabled so the budget/measurement log can account for them.
    uint8_t stood_down = 0;
    bt_output_enable(true, &stood_down);
    printf("[bt] output stand-down mask=0x%02x (bit0=PDM bit1=ADAT)\n", stood_down);

    printf("[bt] cyw43_arch_init...\n");
    if (cyw43_arch_init()){
        printf("[bt] cyw43_arch_init FAILED\n");
        return;
    }
    s_cyw43_ready = true;
    printf("[bt] cyw43 init OK\n");

    if (a2dp_source_and_avrcp_services_init()){
        printf("[bt] A2DP service init failed\n");
        return;
    }

    // Periodic [btm] stats line.
    btstack_run_loop_set_timer_handler(&stats_timer, stats_timeout_handler);
    btstack_run_loop_set_timer(&stats_timer, 1000);
    btstack_run_loop_add_timer(&stats_timer);

    // Power on — the background async context drives everything from here,
    // including the auto-connect on HCI_STATE_WORKING.
    hci_power_control(HCI_POWER_ON);
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

bool bt_radio_resources_held(void){
    return s_cyw43_ready;
}

#endif // DSPI_BT
