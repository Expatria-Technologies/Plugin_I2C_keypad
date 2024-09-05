/*
  keypad.h - I2C keypad plugin

  Part of grblHAL

  Copyright (c) 2017-2021 Terje Io

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef _KEYPAD_H_
#define _KEYPAD_H_

#ifdef ARDUINO
#include "../grbl/gcode.h"
#include "../grbl/settings.h"
#else
#include "grbl/gcode.h"
#include "grbl/settings.h"
#endif

#include "pendant.h"

#if N_AXIS > 3
#define N_MACROS 5
#else
#define N_MACROS 7
#endif

#define READ_COUNT_INTERVAL 100

#define KEYBUF_SIZE 8 // must be a power of 2
#define KEYPAD_I2CADDR 0x49
#define STATUSDATA_SIZE 256

#define JOG_XR   'R'
#define JOG_XL   'L'
#define JOG_YF   'F'
#define JOG_YB   'B'
#define JOG_ZU   'U'
#define JOG_ZD   'D'
#define JOG_XRYF 'r'
#define JOG_XRYB 'q'
#define JOG_XLYF 's'
#define JOG_XLYB 't'
#define JOG_XRZU 'w'
#define JOG_XRZD 'v'
#define JOG_XLZU 'u'
#define JOG_XLZD 'x'

#define MACROUP 0x18
#define MACRODOWN 0x19
#define MACROLEFT 0x1B
#define MACRORIGHT 0x1A
#define MACROLOWER  0x7D
#define MACRORAISE 0x7C
#define MACROHOME  0x8E
#define RESET  0x7F
#define UNLOCK 0x80
#define SPINON 0x83

#define NORMAL_MODE         0
#define LASER_MODE          1
#define LATHE_MODE          2

typedef union {
    uint8_t value;                 //!< Bitmask value
    struct {
        uint8_t state        :4, //Machine state machine status
                mode         :3, //machine mode
                disconnected :1; //Connection status 
    };
} machine_state_t;

typedef enum {
    JogMode_Fast = 0,
    JogMode_Slow,
    JogMode_Step
} jogmode_t;

typedef enum {
    JogModify_1 = 0,
    JogModify_01,
    JogModify_001
} jogmodify_t;

typedef union {
    uint8_t value;
    struct {
        uint8_t modifier :4,
                mode     :4;
    };
} jog_mode_t;

typedef union {
    uint8_t value;
    struct {
        uint8_t diameter       :1,
                mpg            :1,
                homed          :1,
                tlo_referenced :1,
                mode           :3; // from machine_mode_t setting
    };
} machine_modes_t;

typedef union {
    float values[4];
    struct {
        float x;
        float y;
        float z;
        float a;
    };
} machine_coords_t;

typedef enum {
    MachineMsg_None = 0,
// 1-127 reserved for message string length
    MachineMsg_Comment = 252,
    MachineMsg_Overrides = 253,
    MachineMsg_WorkOffset = 254,
    MachineMsg_ClearMessage = 255,
} msg_type_t;

typedef struct __attribute__((packed)){
    uint16_t address;  //address is only used for I2C transactions only present in GRBLHAL plugin
    machine_state_t machine_state;
    uint8_t machine_substate;
    axes_signals_t home_state;
    uint16_t feed_override; // size changed in latest version!
    uint16_t spindle_override;
    uint8_t spindle_stop;
    spindle_state_t spindle_state;
    int spindle_rpm;
    float feed_rate;
    coolant_state_t coolant_state;
    jog_mode_t jog_mode;
    control_signals_t signals;
    float jog_stepsize;
    coord_system_id_t current_wcs;  //active WCS or MCS modal state
    axes_signals_t limits;
    status_code_t status_code;
    machine_modes_t machine_modes;
    machine_coords_t coordinate;
    msg_type_t msgtype; //<! 1 - 127 -> msg[] contains a string msgtype long
    uint8_t msg[128];
} machine_status_packet_t;

typedef void (*keycode_callback_ptr)(const char c);
typedef void (*count_callback_ptr)(void);
typedef bool (*on_keypress_preview_ptr)(const char c, uint_fast16_t state);
typedef void (*on_jogmode_changed_ptr)(jogmode_t jogmode);
typedef void (*on_jogmodify_changed_ptr)(jogmodify_t jogmodify);

typedef struct {
    on_keypress_preview_ptr on_keypress_preview;
    on_jogmode_changed_ptr on_jogmode_changed;
    on_jogmodify_changed_ptr on_jogmodify_changed;
} keypad_t;

typedef struct {
    uint8_t port;
    char data[127];
} macro_setting_t;

typedef struct {
    macro_setting_t macro[N_MACROS];
} macro_settings_t;

extern keypad_t keypad;

typedef struct __attribute__((packed)) {
int32_t uptime;
jog_mode_t jog_mode;
int32_t feed_over;
int32_t spindle_over;
int32_t rapid_over;
uint32_t buttons;
float feedrate; 
float spindle_rpm; 
float x_axis;
float y_axis;
float z_axis;
float a_axis;
} pendant_count_packet_t;

ISR_CODE void ISR_FUNC(i2c_enqueue_keycode)(char c);

bool keypad_init (void);
bool keypad_enqueue_keycode (char c);

#endif
