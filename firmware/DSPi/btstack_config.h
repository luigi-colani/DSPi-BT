#ifndef _DSPI_BTSTACK_CONFIG_H
#define _DSPI_BTSTACK_CONFIG_H

// BTstack configuration for the DSPi Bluetooth audio output (Pico 2 W only).
// Role: Classic A2DP *source* streaming SBC. Derived from the pico-examples
// pico_w/bt/config/btstack_config.h, trimmed to one ACL link, one AVDTP
// connection/stream endpoint, no AVRCP browsing, no LE.
// ENABLE_CLASSIC is supplied by pico_btstack_classic.

#define ENABLE_LOG_ERROR
#define ENABLE_PRINTF_HEXDUMP  // required by the SDK hci_dump_embedded_stdout.c
#define ENABLE_SCO_OVER_HCI

#ifdef ENABLE_CLASSIC
#define ENABLE_L2CAP_ENHANCED_RETRANSMISSION_MODE
#endif

// BTstack buffers and sizes
#define HCI_OUTGOING_PRE_BUFFER_SIZE 4
#define HCI_ACL_PAYLOAD_SIZE (1691 + 4)
#define HCI_ACL_CHUNK_SIZE_ALIGNMENT 4

#define MAX_NR_AVDTP_CONNECTIONS 1
#define MAX_NR_AVDTP_STREAM_ENDPOINTS 1
#define MAX_NR_AVRCP_CONNECTIONS 1
#define MAX_NR_BTSTACK_LINK_KEY_DB_MEMORY_ENTRIES 2
#define MAX_NR_HCI_CONNECTIONS 1
#define MAX_NR_L2CAP_CHANNELS 4
#define MAX_NR_L2CAP_SERVICES 3
#define MAX_NR_SERVICE_RECORD_ITEMS 4

// Limit ACL/SCO buffers in use by the stack to avoid CYW43 shared-bus overrun
#define MAX_NR_CONTROLLER_ACL_BUFFERS 3
#define MAX_NR_CONTROLLER_SCO_PACKETS 3

// HCI controller-to-host flow control (avoids CYW43 shared-bus overrun)
#define ENABLE_HCI_CONTROLLER_TO_HOST_FLOW_CONTROL
#define HCI_HOST_ACL_PACKET_LEN 1024
#define HCI_HOST_ACL_PACKET_NUM 3
#define HCI_HOST_SCO_PACKET_LEN 120
#define HCI_HOST_SCO_PACKET_NUM 3

// Link key DB using TLV on top of the flash sector interface
#define NVM_NUM_DEVICE_DB_ENTRIES 4
#define NVM_NUM_LINK_KEYS 4

// BTstack HAL configuration
#define HAVE_EMBEDDED_TIME_MS

// Map btstack_assert onto Pico SDK assert()
#define HAVE_ASSERT

#define HCI_RESET_RESEND_TIMEOUT_MS 1000

#endif // _DSPI_BTSTACK_CONFIG_H
