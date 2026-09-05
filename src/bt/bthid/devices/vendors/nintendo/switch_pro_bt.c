// switch_pro_bt.c - Nintendo Switch Pro Controller Bluetooth Driver
// Handles Switch Pro and Joy-Con controllers over Bluetooth
//
// Reference: https://github.com/dekuNukem/Nintendo_Switch_Reverse_Engineering

#include "switch_pro_bt.h"
#include "bt/bthid/bthid.h"
#include "bt/transport/bt_transport.h"
#include "core/input_event.h"
#include "core/router/router.h"
#include "core/buttons.h"
#include "core/services/players/manager.h"
#include "core/services/players/feedback.h"
#include "platform/platform.h"
#include <string.h>
#include <stdio.h>

// ============================================================================
// SWITCH PRO CONSTANTS
// ============================================================================

// Report IDs
#define SWITCH_REPORT_INPUT_STANDARD    0x30    // Standard full input report
#define SWITCH_REPORT_INPUT_SIMPLE      0x3F    // Simple HID mode
#define SWITCH_REPORT_OUTPUT            0x01    // Output report with subcommand
#define SWITCH_REPORT_RUMBLE_ONLY       0x10    // Rumble only (no subcommand)

// Subcommands
#define SWITCH_SUBCMD_SET_INPUT_MODE    0x03
#define SWITCH_SUBCMD_SET_PLAYER_LED    0x30
#define SWITCH_SUBCMD_SET_HOME_LED      0x38
#define SWITCH_SUBCMD_ENABLE_IMU        0x40
#define SWITCH_SUBCMD_ENABLE_VIBRATION  0x48

// Input modes
#define SWITCH_INPUT_MODE_FULL          0x30

// Init delay between subcommands (ms)
#define SWITCH_INIT_DELAY_MS            200

// ============================================================================
// SWITCH PRO REPORT STRUCTURE
// ============================================================================

typedef struct __attribute__((packed)) {
    uint8_t report_id;      // 0x30 or 0x3F
    uint8_t timer;          // Increments by 1 per report

    uint8_t battery_conn;   // Battery level + connection info

    // Button byte 1 (right side buttons)
    struct {
        uint8_t y    : 1;
        uint8_t x    : 1;
        uint8_t b    : 1;
        uint8_t a    : 1;
        uint8_t sr_r : 1;
        uint8_t sl_r : 1;
        uint8_t r    : 1;
        uint8_t zr   : 1;
    };

    // Button byte 2 (system buttons)
    struct {
        uint8_t minus  : 1;
        uint8_t plus   : 1;
        uint8_t rstick : 1;
        uint8_t lstick : 1;
        uint8_t home   : 1;
        uint8_t capture: 1;
        uint8_t pad1   : 2;
    };

    // Button byte 3 (left side buttons + dpad)
    struct {
        uint8_t down  : 1;
        uint8_t up    : 1;
        uint8_t right : 1;
        uint8_t left  : 1;
        uint8_t sr_l  : 1;
        uint8_t sl_l  : 1;
        uint8_t l     : 1;
        uint8_t zl    : 1;
    };

    // Analog sticks (12-bit packed, 3 bytes each)
    uint8_t left_stick[3];
    uint8_t right_stick[3];

    // Vibration ack and subcommand data follow...
} switch_input_report_t;

// Simple HID report (0x3F) - used before handshake
typedef struct __attribute__((packed)) {
    uint8_t report_id;      // 0x3F

    struct {
        uint8_t b      : 1;
        uint8_t a      : 1;
        uint8_t y      : 1;
        uint8_t x      : 1;
        uint8_t l      : 1;
        uint8_t r      : 1;
        uint8_t zl     : 1;
        uint8_t zr     : 1;
    };

    struct {
        uint8_t minus  : 1;
        uint8_t plus   : 1;
        uint8_t lstick : 1;
        uint8_t rstick : 1;
        uint8_t home   : 1;
        uint8_t capture: 1;
        uint8_t pad    : 2;
    };

    uint8_t hat;            // D-pad as hat (0-7, 8=center)
    uint16_t lx, ly;        // Left stick (0-65535, center ~32768)
    uint16_t rx, ry;        // Right stick (0-65535, center ~32768)
} switch_simple_report_t;

// ============================================================================
// INIT STATE MACHINE
// ============================================================================

typedef enum {
    SWITCH_STATE_WAIT_READY,        // Wait before sending first subcommand
    SWITCH_STATE_SET_INPUT_MODE,    // Send set input mode (0x03 → 0x30)
    SWITCH_STATE_ENABLE_VIBRATION,  // Send enable vibration (0x48 → 0x01)
    SWITCH_STATE_SET_PLAYER_LED,    // Send player LED (0x30)
    SWITCH_STATE_ACTIVE,            // Init complete, monitor feedback
} switch_init_state_t;

// ============================================================================
// DRIVER DATA
// ============================================================================

typedef struct {
    input_event_t event;
    bool initialized;
    bool full_report_mode;
    // L/R role for single Joy-Cons, detected from the BT PID at init.
    // -1 = Pro Controller or unknown (all sticks valid), 0 = Joy-Con L
    // (owns LX/LY), 1 = Joy-Con R (owns RX/RY). The driver sets a
    // matching valid_fields mask in switch_process_report so the router
    // does not let the non-physical stick's "0 → 1" raw value clobber
    // the partner Joy-Con's real stick data in MERGE_BLEND.
    int8_t grip_side;
    uint8_t output_seq;     // Sequence counter for output reports
    switch_init_state_t init_state;
    uint32_t init_time;     // Timestamp for init delays
    uint8_t rumble_left;    // Cached rumble state
    uint8_t rumble_right;
} switch_bt_data_t;

static switch_bt_data_t switch_data[BTHID_MAX_DEVICES];

// ============================================================================
// HELPER FUNCTIONS
// ============================================================================

// Unpack 12-bit analog value from 3-byte packed format
static uint16_t unpack_stick_12bit(const uint8_t* data, bool high)
{
    if (high) {
        // High nibble of byte 1 + all of byte 2
        return ((data[1] & 0xF0) >> 4) | (data[2] << 4);
    } else {
        // All of byte 0 + low nibble of byte 1
        return data[0] | ((data[1] & 0x0F) << 8);
    }
}

// Scale 12-bit to 8-bit
static uint8_t scale_12bit_to_8bit(uint16_t val)
{
    if (val == 0) return 1;
    return 1 + ((val * 254) / 4095);
}

// Encode rumble intensity to Switch rumble format (from USB Switch Pro driver)
// Each motor uses 4 bytes: [amplitude, HF_freq, amplitude/2, LF_freq]
// Neutral state: [00 01 40 40]
static void encode_rumble(uint8_t intensity, uint8_t* out)
{
    if (intensity == 0) {
        out[0] = 0x00;
        out[1] = 0x01;
        out[2] = 0x40;
        out[3] = 0x40;
        return;
    }
    uint16_t scaled = ((uint16_t)intensity * 102) / 255 + 64;
    uint8_t amplitude = (uint8_t)(scaled + 64);
    out[0] = amplitude;
    out[1] = 0x88;
    out[2] = amplitude / 2;
    out[3] = 0x61;
}

static void switch_send_subcommand(bthid_device_t* device, uint8_t subcmd,
                                    const uint8_t* data, uint8_t len)
{
    switch_bt_data_t* sw = (switch_bt_data_t*)device->driver_data;
    if (!sw) return;

    uint8_t buf[48];
    memset(buf, 0, sizeof(buf));

    buf[0] = sw->output_seq++ & 0x0F;

    // Neutral rumble data (8 bytes)
    buf[1] = 0x00; buf[2] = 0x01; buf[3] = 0x40; buf[4] = 0x40;
    buf[5] = 0x00; buf[6] = 0x01; buf[7] = 0x40; buf[8] = 0x40;

    buf[9] = subcmd;
    if (data && len > 0 && len < 38) {
        memcpy(&buf[10], data, len);
    }

    bthid_send_output_report(device->conn_index, SWITCH_REPORT_OUTPUT, buf, 10 + len);
}

static void switch_send_rumble(bthid_device_t* device, uint8_t left, uint8_t right)
{
    switch_bt_data_t* sw = (switch_bt_data_t*)device->driver_data;
    if (!sw) return;

    uint8_t buf[9];
    buf[0] = sw->output_seq++ & 0x0F;
    encode_rumble(left, &buf[1]);
    encode_rumble(right, &buf[5]);

    bthid_send_output_report(device->conn_index, SWITCH_REPORT_RUMBLE_ONLY, buf, 9);
}

// ============================================================================
// DRIVER IMPLEMENTATION
// ============================================================================

static bool switch_match(const char* device_name, const uint8_t* class_of_device,
                         uint16_t vendor_id, uint16_t product_id, bool is_ble)
{
    (void)class_of_device;
    (void)is_ble;

    // Match Switch 1 controllers by VID/PID
    // Nintendo VID = 0x057E
    // Switch 1 PIDs: Joy-Con L = 0x2006, Joy-Con R = 0x2007, Pro Controller = 0x2009
    // Do NOT match Switch 2 PIDs (0x2066, 0x2067, 0x2069, 0x2073) - handled by switch2_ble
    if (vendor_id == 0x057E) {
        switch (product_id) {
            case 0x2006:  // Joy-Con L
            case 0x2007:  // Joy-Con R
            case 0x2009:  // Pro Controller
                return true;
        }
        // Don't return true for unknown Nintendo PIDs
        // Let specific drivers handle them
    }

    // Name-based match (fallback for classic BT where VID/PID may be unavailable)
    if (device_name) {
        if (strstr(device_name, "Pro Controller") != NULL) {
            return true;
        }
        if (strstr(device_name, "Joy-Con") != NULL) {
            return true;
        }
    }

    return false;
}

static bool switch_init(bthid_device_t* device)
{
    printf("[SWITCH_BT] Init for device: %s\n", device->name);

    for (int i = 0; i < BTHID_MAX_DEVICES; i++) {
        if (!switch_data[i].initialized) {
            init_input_event(&switch_data[i].event);
            switch_data[i].initialized = true;
            switch_data[i].full_report_mode = false;
// Detect single Joy-Con L/R.
//
// We CANNOT tell L from R at init time. PID-based detection is
// unreliable: (a) the SDP VID/PID query is deferred until AFTER
// bthid->init runs for Classic-BT-HID devices (see btstack_host.c —
// bt_on_hid_ready() is called before the SDP query is fired), so
// vendor_id/product_id are typically still 0 here; and (b) standalone
// Joy-Cons in Classic-BT mode BOTH advertise as PID 0x2006 (the Switch
// console distinguishes L/R by BD_ADDR, not PID). The BT name suffix
// "(L)"/"(R)" is also not reliably present in the advertised EIR name.
//
// Detection is therefore deferred to the first HID report — see
// switch_process_report, which classifies the side from the report
// content (the non-physical stick is filled with raw-zero values for
// both axes). This mirrors the USB Joy-Con Charging Grip driver
// (switch_pro.c), which does the same classification on the first
// 0x30/0x21 report.
//
// Pro Controllers have data on both sticks and stay at grip_side=-1
// ("all fields valid" — legacy behaviour). Switch 2 controllers are
// matched by the switch2_ble driver and never reach this init.
            switch_data[i].grip_side = -1;
            switch_data[i].output_seq = 0;
            switch_data[i].rumble_left = 0;
            switch_data[i].rumble_right = 0;

            // Start init state machine — commands sent from task()
            switch_data[i].init_state = SWITCH_STATE_WAIT_READY;
            switch_data[i].init_time = platform_time_ms();

            switch_data[i].event.type = INPUT_TYPE_GAMEPAD;
            switch_data[i].event.transport = INPUT_TRANSPORT_BT_CLASSIC;
            switch_data[i].event.dev_addr = device->conn_index;
            switch_data[i].event.instance = 0;
            switch_data[i].event.button_count = 10;

            device->driver_data = &switch_data[i];
            return true;
        }
    }

    return false;
}

static void switch_process_report(bthid_device_t* device, const uint8_t* data, uint16_t len)
{
    switch_bt_data_t* sw = (switch_bt_data_t*)device->driver_data;
    if (!sw || len < 1) return;

    uint8_t report_id = data[0];

    if (report_id == SWITCH_REPORT_INPUT_STANDARD && len >= 13) {
        // Full input report (0x30)
        const switch_input_report_t* rpt = (const switch_input_report_t*)data;

        sw->full_report_mode = true;

        // Per-event field-ownership mask (see input_event.h). Initialised
        // to "buttons valid" and tightened per-side after the stick
        // scaling below. D-pad ownership is enforced separately
        // (after the d-pad block, see below) by stripping the bits
        // from R Joy-Con events — we can't use the mask for that
        // because the joystick-style buttons share a single
        // INPUT_VALID_BUTTONS bit.
        uint32_t valid_fields = INPUT_VALID_BUTTONS;

        // Build button state
        uint32_t buttons = 0x00000000;

        // Face buttons (map by position, not label — Nintendo layout is rotated)
        if (rpt->b)      buttons |= JP_BUTTON_B1;  // B = bottom
        if (rpt->a)      buttons |= JP_BUTTON_B2;  // A = right
        if (rpt->y)      buttons |= JP_BUTTON_B3;  // Y = left
        if (rpt->x)      buttons |= JP_BUTTON_B4;  // X = top

        // Shoulder buttons
        if (rpt->l)      buttons |= JP_BUTTON_L1;
        if (rpt->r)      buttons |= JP_BUTTON_R1;
        if (rpt->zl)     buttons |= JP_BUTTON_L2;
        if (rpt->zr)     buttons |= JP_BUTTON_R2;

        // System buttons
        if (rpt->minus)  buttons |= JP_BUTTON_S1;
        if (rpt->plus)   buttons |= JP_BUTTON_S2;
        if (rpt->lstick) buttons |= JP_BUTTON_L3;
        if (rpt->rstick) buttons |= JP_BUTTON_R3;
        if (rpt->home)   buttons |= JP_BUTTON_A1;
        if (rpt->capture) buttons |= JP_BUTTON_A2;

        // D-pad
        if (rpt->up)     buttons |= JP_BUTTON_DU;
        if (rpt->down)   buttons |= JP_BUTTON_DD;
        if (rpt->left)   buttons |= JP_BUTTON_DL;
        if (rpt->right)  buttons |= JP_BUTTON_DR;

        sw->event.buttons = buttons;

        // R Joy-Con has no physical d-pad. The HID report can still
        // carry non-zero d-pad bits (some firmware versions, or when
        // the host has remapped SR + stick to d-pad). Clear them from
        // the event so a paired L Joy-Con is the only source of
        // DU/DD/DL/DR. The clear MUST happen after the assignment to
        // sw->event.buttons (not before) — clearing before the d-pad
        // block above is a no-op because the block OR-sets the bits
        // back in.
        // R Joy-Con has no physical d-pad — single R in horizontal mode
        // encodes stick movement as d-pad bits, and even at "center"
        // it often reports a phantom DD+DL default state. Strip them so
        // only L (the actual d-pad source) contributes to merged
        // d-pad, then re-inject the bits as digital RX/RY axes below
        // so R's stick still drives the right-stick slot.
        if (sw->grip_side == 1) {
            sw->event.buttons &= ~(JP_BUTTON_DU | JP_BUTTON_DD |
                                   JP_BUTTON_DL | JP_BUTTON_DR);
            // Re-inject d-pad bits (now stripped from buttons) as
            // digital RX/RY. 4-way only — no analog precision, but
            // every direction works. Skipped when no d-pad bits are
            // set (vertical-mode normal Joy-Con, idle at center →
            // leave the centered axis values the driver already
            // wrote).
            if (sw->event.buttons & (JP_BUTTON_DU | JP_BUTTON_DD |
                                     JP_BUTTON_DL | JP_BUTTON_DR)) {
                uint8_t ax = 128, ay = 128;
                if (sw->event.buttons & JP_BUTTON_DR) ax = 255;
                else if (sw->event.buttons & JP_BUTTON_DL) ax = 0;
                if (sw->event.buttons & JP_BUTTON_DD) ay = 255;
                else if (sw->event.buttons & JP_BUTTON_DU) ay = 0;
                sw->event.analog[ANALOG_RX] = ax;
                sw->event.analog[ANALOG_RY] = ay;
            }
        }

        // Unpack 12-bit sticks
        uint16_t lx = unpack_stick_12bit(rpt->left_stick, false);
        uint16_t ly = unpack_stick_12bit(rpt->left_stick, true);
        uint16_t rx = unpack_stick_12bit(rpt->right_stick, false);
        uint16_t ry = unpack_stick_12bit(rpt->right_stick, true);

        // Side detection from report content (one-shot, on the first
        // report). Both standalone Joy-Cons advertise PID 0x2006 in
        // Classic-BT mode, so PID-based detection cannot tell L from R.
        // The non-physical stick is filled with raw-zero 12-bit values
        // for both axes — the physical stick reports real values
        // (≥~2048 at rest). This matches the USB Joy-Con Charging Grip
        // driver (switch_pro.c:376-386), which uses the same signal.
        // If both sticks report data, this is a Pro Controller and we
        // stay at grip_side=-1 ("all valid" mask). If both sticks are
        // zero, we wait for a later report (the user might be holding
        // a centered physical stick on first contact, but the
        // non-physical stick is constant, so the asymmetry will appear
        // as soon as the user touches the physical one).
        if (sw->grip_side == -1) {
            bool r_empty = (rx == 0 && ry == 0);
            bool l_empty = (lx == 0 && ly == 0);
            if (r_empty && !l_empty) {
                sw->grip_side = 0;  // Right stick empty → Joy-Con L
            } else if (l_empty && !r_empty) {
                sw->grip_side = 1;  // Left stick empty → Joy-Con R
            }
            if (sw->grip_side != -1) {
                printf("[SWITCH_BT] Grip side detected from report: %s (raw L=%u/%u R=%u/%u)\n",
                       sw->grip_side == 0 ? "L" : "R", lx, ly, rx, ry);
            }
        }

        // Scale to 8-bit and invert Y (Nintendo: up=high, HID: up=low)
        sw->event.analog[ANALOG_LX] = scale_12bit_to_8bit(lx);
        sw->event.analog[ANALOG_LY] = 255 - scale_12bit_to_8bit(ly);
        sw->event.analog[ANALOG_RX] = scale_12bit_to_8bit(rx);
        sw->event.analog[ANALOG_RY] = 255 - scale_12bit_to_8bit(ry);

        // Mark only the owned stick as valid for this event. d-pad
        // ownership was enforced above (R has d-pad bits stripped
        // from the event). Pro Controllers and unknown devices fall
        // back to 0 ("all valid") so we don't drop any fields.
        if (sw->grip_side == 0) {
            valid_fields |= INPUT_VALID_L_STICK;
        } else if (sw->grip_side == 1) {
            valid_fields |= INPUT_VALID_R_STICK;
        } else {
            valid_fields = 0;
        }
        sw->event.valid_fields = valid_fields;

        // Battery: bits 7-4 = level (0/2/4/6/8), bit 3 = charging
        uint8_t bat_raw = rpt->battery_conn >> 4;
        sw->event.battery_level = (bat_raw > 8) ? 100 : bat_raw * 12 + 5;
        sw->event.battery_charging = (rpt->battery_conn & 0x08) != 0;

        // Debug: throttled heartbeat per Joy-Con so we can see the live
        // stick values + active mask in the serial log. Every 64th report
        // = ~1 Hz at the 66 Hz full-report rate, no flood.
        static uint32_t dbg_count_30 = 0;
        if ((dbg_count_30++ & 63) == 0) {
            printf("[SWITCH_BT] 0x30 dev=%d side=%d mask=0x%X "
                   "LX=%3d LY=%3d RX=%3d RY=%3d btn=0x%08lX\n",
                   device->conn_index, sw->grip_side, sw->event.valid_fields,
                   sw->event.analog[ANALOG_LX], sw->event.analog[ANALOG_LY],
                   sw->event.analog[ANALOG_RX], sw->event.analog[ANALOG_RY],
                   (unsigned long)sw->event.buttons);
        }

        router_submit_input(&sw->event);

    } else if (report_id == SWITCH_REPORT_INPUT_SIMPLE && len >= 12) {
        // Simple HID report (0x3F) - used before full mode enabled
        const switch_simple_report_t* rpt = (const switch_simple_report_t*)data;

        uint32_t buttons = 0x00000000;

        if (rpt->b)      buttons |= JP_BUTTON_B1;  // B = bottom
        if (rpt->a)      buttons |= JP_BUTTON_B2;  // A = right
        if (rpt->y)      buttons |= JP_BUTTON_B3;  // Y = left
        if (rpt->x)      buttons |= JP_BUTTON_B4;  // X = top
        if (rpt->l)      buttons |= JP_BUTTON_L1;
        if (rpt->r)      buttons |= JP_BUTTON_R1;
        if (rpt->zl)     buttons |= JP_BUTTON_L2;
        if (rpt->zr)     buttons |= JP_BUTTON_R2;
        if (rpt->minus)  buttons |= JP_BUTTON_S1;
        if (rpt->plus)   buttons |= JP_BUTTON_S2;
        if (rpt->lstick) buttons |= JP_BUTTON_L3;
        if (rpt->rstick) buttons |= JP_BUTTON_R3;
        if (rpt->home)   buttons |= JP_BUTTON_A1;
        if (rpt->capture) buttons |= JP_BUTTON_A2;

        // Hat to D-pad
        if (rpt->hat == 0 || rpt->hat == 1 || rpt->hat == 7) buttons |= JP_BUTTON_DU;
        if (rpt->hat >= 1 && rpt->hat <= 3) buttons |= JP_BUTTON_DR;
        if (rpt->hat >= 3 && rpt->hat <= 5) buttons |= JP_BUTTON_DD;
        if (rpt->hat >= 5 && rpt->hat <= 7) buttons |= JP_BUTTON_DL;

        sw->event.buttons = buttons;

        // R Joy-Con has no physical d-pad in the simple report either.
        // Clear from the event AFTER the hat-to-dpad mapping above, so
        // a paired L Joy-Con is the only source of DU/DD/DL/DR.
        if (sw->grip_side == 1) {
            sw->event.buttons &= ~(JP_BUTTON_DU | JP_BUTTON_DD |
                                   JP_BUTTON_DL | JP_BUTTON_DR);

            // Same horizontal-mode workaround as the 0x30 path: single R
            // Joy-Con encodes stick movement as the hat / d-pad bits
            // (stripped above). Re-inject them as digital RX/RY so the
            // merged player slot still has a usable right stick. 4-way
            // digital only.
            if (sw->event.buttons & (JP_BUTTON_DU | JP_BUTTON_DD |
                                     JP_BUTTON_DL | JP_BUTTON_DR)) {
                uint8_t ax = 128, ay = 128;
                if (sw->event.buttons & JP_BUTTON_DR) ax = 255;
                else if (sw->event.buttons & JP_BUTTON_DL) ax = 0;
                if (sw->event.buttons & JP_BUTTON_DD) ay = 255;
                else if (sw->event.buttons & JP_BUTTON_DU) ay = 0;
                sw->event.analog[ANALOG_RX] = ax;
                sw->event.analog[ANALOG_RY] = ay;
            }
        }

        // Side detection from report content (one-shot, on the first
        // simple report). Same logic as the 0x30 path: the non-physical
        // stick is raw-zero in both axes; the physical stick reports
        // real 16-bit values (≥~32768 at rest). 0x3F reports arrive
        // before SET_INPUT_MODE completes, so this can detect the side
        // a few hundred ms earlier than waiting for 0x30.
        if (sw->grip_side == -1) {
            bool r_empty = (rpt->rx == 0 && rpt->ry == 0);
            bool l_empty = (rpt->lx == 0 && rpt->ly == 0);
            if (r_empty && !l_empty) {
                sw->grip_side = 0;  // Joy-Con L
            } else if (l_empty && !r_empty) {
                sw->grip_side = 1;  // Joy-Con R
            }
            if (sw->grip_side != -1) {
                printf("[SWITCH_BT] Grip side detected from 0x3F: %s (raw L=%u/%u R=%u/%u)\n",
                       sw->grip_side == 0 ? "L" : "R",
                       rpt->lx, rpt->ly, rpt->rx, rpt->ry);
            }
        }

        // Per-event field ownership. Buttons are always "valid" as a
        // bitmap (d-pad stripped above for R); only the stick mask
        // differs by side.
        if (sw->grip_side == 1) {
            sw->event.valid_fields = INPUT_VALID_BUTTONS | INPUT_VALID_R_STICK;
        } else if (sw->grip_side == 0) {
            sw->event.valid_fields = INPUT_VALID_BUTTONS | INPUT_VALID_L_STICK;
        } else {
            sw->event.valid_fields = 0;  // Pro: all valid
        }

        // 16-bit sticks scaled to 8-bit (0-65535 → 0-255)
        sw->event.analog[ANALOG_LX] = rpt->lx >> 8;
        sw->event.analog[ANALOG_LY] = 255 - (rpt->ly >> 8);  // Invert Y (Nintendo: up=high, HID: up=low)
        sw->event.analog[ANALOG_RX] = rpt->rx >> 8;
        sw->event.analog[ANALOG_RY] = 255 - (rpt->ry >> 8);  // Invert Y (Nintendo: up=high, HID: up=low)

        // Same throttled heartbeat for the 0x3F simple-report path so we
        // can see whether the device ever transitions out of init mode.
        static uint32_t dbg_count_3f = 0;
        if ((dbg_count_3f++ & 63) == 0) {
            printf("[SWITCH_BT] 0x3F dev=%d side=%d mask=0x%X "
                   "LX=%3d LY=%3d RX=%3d RY=%3d btn=0x%08lX\n",
                   device->conn_index, sw->grip_side, sw->event.valid_fields,
                   sw->event.analog[ANALOG_LX], sw->event.analog[ANALOG_LY],
                   sw->event.analog[ANALOG_RX], sw->event.analog[ANALOG_RY],
                   (unsigned long)sw->event.buttons);
        }

        router_submit_input(&sw->event);
    }
}

static void switch_task(bthid_device_t* device)
{
    switch_bt_data_t* sw = (switch_bt_data_t*)device->driver_data;
    if (!sw) return;

    uint32_t now = platform_time_ms();

    switch (sw->init_state) {
        case SWITCH_STATE_WAIT_READY:
            // Wait before sending first subcommand
            if (now - sw->init_time >= SWITCH_INIT_DELAY_MS) {
                printf("[SWITCH_BT] Sending set input mode (0x30)\n");
                uint8_t mode = SWITCH_INPUT_MODE_FULL;
                switch_send_subcommand(device, SWITCH_SUBCMD_SET_INPUT_MODE, &mode, 1);
                sw->init_state = SWITCH_STATE_SET_INPUT_MODE;
                sw->init_time = now;
            }
            break;

        case SWITCH_STATE_SET_INPUT_MODE:
            if (now - sw->init_time >= SWITCH_INIT_DELAY_MS) {
                printf("[SWITCH_BT] Sending enable vibration\n");
                uint8_t enable = 0x01;
                switch_send_subcommand(device, SWITCH_SUBCMD_ENABLE_VIBRATION, &enable, 1);
                sw->init_state = SWITCH_STATE_ENABLE_VIBRATION;
                sw->init_time = now;
            }
            break;

        case SWITCH_STATE_ENABLE_VIBRATION:
            if (now - sw->init_time >= SWITCH_INIT_DELAY_MS) {
                // Set player LED based on player index
                int player_idx = find_player_index(sw->event.dev_addr, sw->event.instance);
                uint8_t player_num = (player_idx >= 0) ? player_idx + 1 : 1;
                uint8_t pattern = 0;
                if (player_num >= 1 && player_num <= 4) {
                    pattern = (1 << player_num) - 1;
                }
                printf("[SWITCH_BT] Sending player LED (player %d, pattern 0x%02X)\n",
                       player_num, pattern);
                switch_send_subcommand(device, SWITCH_SUBCMD_SET_PLAYER_LED, &pattern, 1);
                sw->init_state = SWITCH_STATE_SET_PLAYER_LED;
                sw->init_time = now;
            }
            break;

        case SWITCH_STATE_SET_PLAYER_LED:
            if (now - sw->init_time >= SWITCH_INIT_DELAY_MS) {
                printf("[SWITCH_BT] Init complete\n");
                sw->init_state = SWITCH_STATE_ACTIVE;
            }
            break;

        case SWITCH_STATE_ACTIVE: {
            // Monitor feedback system for rumble/LED updates
            int player_idx = find_player_index(sw->event.dev_addr, sw->event.instance);
            if (player_idx < 0) break;

            feedback_state_t* fb = feedback_get_state(player_idx);
            if (!fb) break;

            // Handle rumble updates
            if (fb->rumble_dirty) {
                uint8_t left = fb->rumble.left;
                uint8_t right = fb->rumble.right;
                if (left != sw->rumble_left || right != sw->rumble_right) {
                    switch_send_rumble(device, left, right);
                    sw->rumble_left = left;
                    sw->rumble_right = right;
                }
            }

            // Handle LED updates
            if (fb->led_dirty) {
                uint8_t pattern = fb->led.pattern;
                if (pattern != 0) {
                    printf("[SWITCH_BT] LED update: pattern=0x%02X\n", pattern);
                    switch_send_subcommand(device, SWITCH_SUBCMD_SET_PLAYER_LED, &pattern, 1);
                }
            }

            if (fb->rumble_dirty || fb->led_dirty) {
                feedback_clear_dirty(player_idx);
            }
            break;
        }
    }
}

static void switch_disconnect(bthid_device_t* device)
{
    printf("[SWITCH_BT] Disconnect: %s\n", device->name);

    switch_bt_data_t* sw = (switch_bt_data_t*)device->driver_data;
    if (sw) {
        // Clear router state first (sends zeroed input report)
        router_device_disconnected(sw->event.dev_addr, sw->event.instance);
        // Remove player assignment
        remove_players_by_address(sw->event.dev_addr, sw->event.instance);

        init_input_event(&sw->event);
        sw->initialized = false;
    }
}

// ============================================================================
// DRIVER STRUCT
// ============================================================================

const bthid_driver_t switch_pro_bt_driver = {
    .name = "Switch Pro",
    .match = switch_match,
    .init = switch_init,
    .process_report = switch_process_report,
    .task = switch_task,
    .disconnect = switch_disconnect,
};

void switch_pro_bt_register(void)
{
    bthid_register_driver(&switch_pro_bt_driver);
}
