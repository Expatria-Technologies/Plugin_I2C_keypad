/*
  keypad.c - I2C keypad plugin

  Part of grblHAL

  Copyright (c) 2017-2022 Terje Io

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


#ifdef ARDUINO
#include "../driver.h"
#else
#include "driver.h"
#endif

#include <stdio.h>
#include <string.h>
#include <math.h>

#include "keypad.h"
#include "pendant.h"

#ifdef ARDUINO
#include "../i2c.h"
#include "../grbl/report.h"
#include "../grbl/override.h"
#include "../grbl/protocol.h"
#include "../grbl/nvs_buffer.h"
#include "../grbl/state_machine.h"
#else
#include "i2c.h"
#include "grbl/report.h"
#include "grbl/override.h"
#include "grbl/protocol.h"
#include "grbl/nvs_buffer.h"
#include "grbl/state_machine.h"
//#include "grbl/limits.h"
#endif

static jogmode_t jogMode = JogMode_Fast;
static jogmodify_t jogModify = JogModify_1;

static on_spindle_select_ptr on_spindle_select;
spindle_ptrs_t *current_spindle = NULL;

char charbuf[127];

typedef struct {
    char buf[KEYBUF_SIZE];
    volatile uint_fast8_t head;
    volatile uint_fast8_t tail;
} keybuffer_t;

static keybuffer_t keybuf = {0};

static char buf[(STRLEN_COORDVALUE + 1) * N_AXIS];

// Add info about our settings for $help and enumerations.
// Potentially used by senders for settings UI.

// BE WARNED: this function may be dangerous to use...
static char *strrepl (char *str, int c, char *str3)
{
    char tmp[30];
    char *s = strrchr(str, c);

    while(s) {
        strcpy(tmp, str3);
        strcat(tmp, s + 1);
        strcpy(s, tmp);
        s = strrchr(str, c);
    }

    return str;
}

static char *map_coord_system (coord_system_id_t id)
{
    uint8_t g5x = id + 54;

    strcpy(buf, uitoa((uint32_t)(g5x > 59 ? 59 : g5x)));
    if(g5x > 59) {
        strcat(buf, ".");
        strcat(buf, uitoa((uint32_t)(g5x - 59)));
    }

    return buf;
}

static void jog_command (char *cmd, char *to)
{
    strcat(strcpy(cmd, "$J=G91G21"), to);
}

static status_code_t disable_lock (sys_state_t state)
{
    status_code_t retval = Status_OK;

    if(state & (STATE_ALARM|STATE_ESTOP)) {

        control_signals_t control_signals = hal.control.get_state();

        // Block if self-test failed
        if(sys.alarm == Alarm_SelftestFailed)
            retval = Status_SelfTestFailed;
        // Block if e-stop is active.
        else if (control_signals.e_stop)
            retval = Status_EStop;
        // Block if safety door is ajar.
        else if (control_signals.safety_door_ajar)
            retval = Status_CheckDoor;
        // Block if safety reset is active.
        else if(control_signals.reset)
            retval = Status_Reset;
        else {
            grbl.report.feedback_message(Message_AlarmUnlock);
            state_set(STATE_IDLE);
        }
    } // Otherwise, no effect.

    return retval;
}

static bool onSpindleSelect (spindle_ptrs_t *spindle)
{   
    current_spindle = spindle;
    return on_spindle_select == NULL || on_spindle_select(spindle);
}

void prepare_status_info (uint8_t * status_ptr)
{    
    
    machine_status_packet_t * status_packet = (machine_status_packet_t*) status_ptr;
    
    status_packet->current_wcs = gc_state.modal.coord_system.id; 

    int32_t current_position[N_AXIS]; // Copy current state of the system position variable
    float jog_modifier = 0;
    float print_position[N_AXIS];
    status_packet->coordinate.a = 0xffff;

    spindle_ptrs_t *spindle;
    spindle_state_t spindle_state;
   
    memcpy(current_position, sys.position, sizeof(sys.position));

    system_convert_array_steps_to_mpos(print_position, current_position);

    uint_fast8_t idx;
    float wco[N_AXIS];
    for (idx = 0; idx < N_AXIS; idx++) {
        // Apply work coordinate offsets and tool length offset to current position.
        wco[idx] = gc_get_offset(idx);
        print_position[idx] -= wco[idx];
    }  
    
    status_packet->address = 0x01;
    
    switch(jogModify){
        case JogModify_1:
            jog_modifier = 1;
        break;
        case JogModify_01:
            jog_modifier = 0.1;                    
        break;
        case JogModify_001:
            jog_modifier = 0.01;                    
        break;  
    }
    
    switch (state_get()){
        case STATE_ALARM:
            status_packet->machine_state.value = 1;
            break;
        case STATE_ESTOP:
            status_packet->machine_state.value = 1;
            break;            
        case STATE_CYCLE:
            status_packet->machine_state.value = 2;
            break;
        case STATE_HOLD:
            status_packet->machine_state.value = 3;
            break;
        case STATE_TOOL_CHANGE:
            status_packet->machine_state.value = 4;
            break;
        case STATE_IDLE:
            status_packet->machine_state.value = 5;
            break;
        case STATE_HOMING:
            status_packet->machine_state.value = 6;
            break;   
        case STATE_JOG:
            status_packet->machine_state.value = 7;
            break;                                    
        default :
            status_packet->machine_state.value = 254;
            break;                                                        
    }
    status_packet->machine_state.mode = settings.mode;
    status_packet->machine_state.disconnected = 0;

    //check the probe pin, if it is asserted, add it to the state

    status_packet->coolant_state = hal.coolant.get_state();
    status_packet->feed_override = sys.override.feed_rate;

    spindle = spindle_get(0);

    if(spindle->get_state)
        spindle_state = spindle->get_state(spindle);

    if(spindle->cap.variable) {
        status_packet->spindle_rpm = spindle_state.on ? lroundf(spindle->param->rpm_overridden) : 0;
        if(spindle->get_data)
            status_packet->spindle_rpm = spindle->get_data(SpindleData_RPM)->rpm;
    } else
        status_packet->spindle_rpm = spindle->param->rpm;

    status_packet->status_code = (uint8_t) sys.alarm;
    status_packet->home_state.value = (uint8_t)(sys.homing.mask & sys.homed.mask);
    status_packet->jog_mode.value = (uint8_t) jogMode << 4 | (uint8_t) jogModify;
    status_packet->coordinate.x = print_position[0];
    status_packet->coordinate.y = print_position[1];
    status_packet->coordinate.z = print_position[2];
    #if N_AXIS > 3
    status_packet->coordinate.a = print_position[3];
    #else
    status_packet->coordinate.a = 0xFFFFFFFF;
    #endif

    status_packet->feed_rate = st_get_realtime_rate();
    
    switch(jogMode){
        case JogMode_Slow:
        status_packet->jog_stepsize = jog.slow_speed * jog_modifier;
        break;
        case JogMode_Fast:
        status_packet->jog_stepsize = jog.fast_speed * jog_modifier;
        break;
        default:
        status_packet->jog_stepsize = jog.step_distance * jog_modifier;
        break;
    }
    
    status_packet->current_wcs = gc_state.modal.coord_system.id;
}

bool process_count_info (uint8_t * prev_count_ptr, uint8_t * count_ptr)
{    

    bool cmd = 0;
    
    pendant_count_packet_t * count_packet = (pendant_count_packet_t*) count_ptr;
    pendant_count_packet_t * previous_count_packet = (pendant_count_packet_t*) prev_count_ptr;

    char command[35] = ""; 

    static struct {
    float x;
    float y;
    float z;
    float a;
    } deltas;

    double distance;
    double feedrate;
        
        //this reads the data and processes any changes
        hal.stream.write("PROCESS"  ASCII_EOL);

        sprintf(charbuf, "X %f Y %f Z %f UT %d", count_packet->x_axis, count_packet->y_axis, count_packet->z_axis, count_packet->uptime);
        report_message(charbuf, Message_Info);    

        /*sprintf(charbuf, "X %d Y %d Z %d UT %d", count_packet->x_axis - previous_count_packet->x_axis,
                                                 count_packet->y_axis - previous_count_packet->y_axis, 
                                                 count_packet->z_axis - previous_count_packet->z_axis, 
                                                 count_packet->uptime - previous_count_packet->uptime );
        report_message(charbuf, Message_Info);       */ 

        #if 0
        //if deltas are zero, do not start jogging.
        if( count_packet->x_axis - previous_count_packet->x_axis != 0 ||
            count_packet->y_axis - previous_count_packet->y_axis != 0 ||
            count_packet->z_axis - previous_count_packet->z_axis != 0 || 
            count_packet->a_axis - previous_count_packet->a_axis != 0 ){
            
            deltas.x = (float)(count_packet->x_axis - previous_count_packet->x_axis);
            deltas.y = (float)(count_packet->y_axis - previous_count_packet->y_axis);
            deltas.z = (float)(count_packet->z_axis - previous_count_packet->z_axis);
            #if N_AXIS > 3
            deltas.a = (float)(count_packet->a_axis - previous_count_packet->a_axis);
            #else
            deltas.a = 0;
            #endif
            
            //sprintf(charbuf, "X %s UT %d", ftoa(deltas.x, 8), count_packet->uptime - previous_count_packet->uptime);
            //report_message(charbuf, Message_Info);   

            //apply any configuration scaling

            //calculate feed rate so that the movement takes the sampling period to execute.
            //if the distance is longer than can be executed at max rate, limit the distance.            
            distance=sqrt((deltas.x *deltas.x ) + 
                          (deltas.y *deltas.y ) + 
                          (deltas.z *deltas.z ));
            feedrate = (distance/READ_COUNT_INTERVAL)*1000*60;

            jog_command(command, "X?");
            strrepl(command, '?', ftoa(deltas.x, 3));
            strcat(command, "Y?");
            strrepl(command, '?', ftoa(deltas.y, 3));
            strcat(command, "Z?");
            strrepl(command, '?', ftoa(deltas.z, 3));    

            #if N_AXIS > 3
                strcat(command, "A?");
                strrepl(command, '?', ftoa(deltas.a, 3));
                //need to ensure feed isn't zero for A only movement
            #endif

            strcat(command, "F");
            strcat(command, ftoa(feedrate, 3));


            //grbl.enqueue_realtime_command(CMD_JOG_CANCEL);
            //report_message(command, Message_Info);
            grbl.enqueue_gcode((char *)command);
            cmd = 1;
        } else{
            grbl.enqueue_realtime_command(CMD_JOG_CANCEL);
        }

        //deal with overrides
        if( count_packet->feed_over != sys.override.feed_rate ||
            count_packet->spindle_over != sys.override.spindle_rpm ||
            count_packet->rapid_over != sys.override.rapid_rate){
#if 0
        sprintf(charbuf, "PEN F %d S %d R %d", count_packet->feed_over, count_packet->spindle_over, count_packet->rapid_over);
        report_message(charbuf, Message_Info);
        sprintf(charbuf, "SYS F %d S %d R %d", sys.override.feed_rate, sys.override.spindle_rpm, sys.override.rapid_rate);
        report_message(charbuf, Message_Info);
#endif        

            if(count_packet->feed_over > sys.override.feed_rate){
                if(count_packet->feed_over - sys.override.feed_rate >= 10)
                    enqueue_feed_override(CMD_OVERRIDE_FEED_COARSE_PLUS);
                else
                    enqueue_feed_override(CMD_OVERRIDE_FEED_FINE_PLUS);
            } else if (count_packet->feed_over < sys.override.feed_rate){
                if(sys.override.feed_rate - count_packet->feed_over >= 10)
                    enqueue_feed_override(CMD_OVERRIDE_FEED_COARSE_MINUS);
                else
                    enqueue_feed_override(CMD_OVERRIDE_FEED_FINE_MINUS);                
            }

            if(count_packet->spindle_over > sys.override.spindle_rpm){
                if(count_packet->spindle_over - sys.override.spindle_rpm >= 10)
                    enqueue_spindle_override(CMD_OVERRIDE_SPINDLE_COARSE_PLUS);
                else
                    enqueue_spindle_override(CMD_OVERRIDE_SPINDLE_FINE_PLUS);
            } else if (count_packet->spindle_over < sys.override.spindle_rpm){
                if(sys.override.spindle_rpm - count_packet->spindle_over >= 10)
                    enqueue_spindle_override(CMD_OVERRIDE_SPINDLE_COARSE_MINUS);
                else
                    enqueue_spindle_override(CMD_OVERRIDE_SPINDLE_FINE_MINUS);                
            }                
            //sys.override.rapid_rate

            cmd = 1;
        }        
            
        if (count_packet->buttons > 0){
            //assign_button_values(count_packet->buttons);
            if(count_packet->buttons & HALT_PRESSED)
                i2c_enqueue_keycode(RESET);
            if(count_packet->buttons & HOLD_PRESSED)
                i2c_enqueue_keycode(CMD_FEED_HOLD_LEGACY);
            if(count_packet->buttons & CYCLE_START_PRESSED)
                i2c_enqueue_keycode(CMD_CYCLE_START_LEGACY);
            if(count_packet->buttons & SPINDLE_PRESSED)
                i2c_enqueue_keycode(CMD_OVERRIDE_SPINDLE_STOP);  
            if(count_packet->buttons & MIST_PRESSED)
                i2c_enqueue_keycode('m');  
            if(count_packet->buttons & FLOOD_PRESSED)
                i2c_enqueue_keycode('h');  
            if(count_packet->buttons & HOME_PRESSED)
                i2c_enqueue_keycode('H');  
            //if(count_packet->buttons & SPINDLE_OVER_DOWN_PRESSED)
            //    i2c_enqueue_keycode();  
            //if(count_packet->buttons & SPINDLE_OVER_RESET_PRESSED)
            //    i2c_enqueue_keycode();  
            //if(count_packet->buttons & SPINDLE_OVER_UP_PRESSED)
            //    i2c_enqueue_keycode();
            //if(count_packet->buttons & FEED_OVER_DOWN_PRESSED)
            //    i2c_enqueue_keycode();  
            //if(count_packet->buttons & FEED_OVER_RESET_PRESSED)
            //    i2c_enqueue_keycode();  
            //if(count_packet->buttons & FEED_OVER_UP_PRESSED)
            //    i2c_enqueue_keycode();
            if(count_packet->buttons & ALT_HALT_PRESSED)
                i2c_enqueue_keycode(0);  
            if(count_packet->buttons & ALT_HOLD_PRESSED)
                i2c_enqueue_keycode(RESET);  
            if(count_packet->buttons & ALT_HOME_PRESSED)
                i2c_enqueue_keycode(MACROHOME);  
            if(count_packet->buttons & ALT_CYCLE_START_PRESSED)
                i2c_enqueue_keycode(UNLOCK);  
            if(count_packet->buttons & ALT_SPINDLE_PRESSED)
                i2c_enqueue_keycode(SPINON);  
            if(count_packet->buttons & ALT_FLOOD_PRESSED)
                i2c_enqueue_keycode('C');  
            if(count_packet->buttons & ALT_MIST_PRESSED)
                i2c_enqueue_keycode('M');    
            if(count_packet->buttons & ALT_UP_PRESSED)
                i2c_enqueue_keycode(MACROUP);  
            if(count_packet->buttons & ALT_DOWN_PRESSED)
                i2c_enqueue_keycode(MACRODOWN);  
            if(count_packet->buttons & ALT_LEFT_PRESSED)
                i2c_enqueue_keycode(MACROLEFT);  
            if(count_packet->buttons & ALT_RIGHT_PRESSED)
                i2c_enqueue_keycode(MACRORIGHT);  
            if(count_packet->buttons & ALT_RAISE_PRESSED)
                i2c_enqueue_keycode(MACRORAISE);  
            if(count_packet->buttons & ALT_LOWER_PRESSED)
                i2c_enqueue_keycode(MACROLOWER);

            cmd = 1;                                                                                                                                                                                                                                                                                                                                                                                    
        }            
        //if a button is active

#endif

    memcpy(previous_count_packet, count_packet, sizeof(pendant_count_packet_t));
    return cmd;
}