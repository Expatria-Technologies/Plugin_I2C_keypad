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

#ifndef _PENDANT_H_
#define _PENDANT_H_

#ifdef ARDUINO
#include "../grbl/gcode.h"
#include "../grbl/settings.h"
#else
#include "grbl/gcode.h"
#include "grbl/settings.h"
#endif

#include "keypad.h"

#if N_AXIS > 3
#define N_MACROS 5
#else
#define N_MACROS 7
#endif

#define HALT_PRESSED                (1 << (0) )
#define HOLD_PRESSED                (1 << (1) )
#define CYCLE_START_PRESSED         (1 << (2) )
#define SPINDLE_PRESSED             (1 << (3) )
#define MIST_PRESSED                (1 << (4) )
#define FLOOD_PRESSED               (1 << (5) )
#define HOME_PRESSED                (1 << (6) )
#define SPIN_OVER_RESET_PRESSED     (1 << (7) )
#define FEED_OVER_RESET_PRESSED     (1 << (8) )
#define UP_PRESSED                  (1 << (9) )
#define DOWN_PRESSED                (1 << (10) )
#define LEFT_PRESSED                (1 << (11) )
#define RIGHT_PRESSED               (1 << (12) )
#define RAISE_PRESSED               (1 << (13) )
#define LOWER_PRESSED               (1 << (14) )

#define ALT_HALT_PRESSED            (1 << (15) )
#define ALT_HOLD_PRESSED            (1 << (16) )
#define ALT_CYCLE_START_PRESSED     (1 << (17) )
#define ALT_SPINDLE_PRESSED         (1 << (18) )
#define ALT_MIST_PRESSED            (1 << (19) )
#define ALT_FLOOD_PRESSED           (1 << (20) )
#define ALT_HOME_PRESSED            (1 << (21) )
#define ALT_SPIN_OVER_RESET_PRESSED (1 << (22) )
#define ALT_FEED_OVER_RESET_PRESSED (1 << (23) )
#define ALT_UP_PRESSED              (1 << (24) )
#define ALT_DOWN_PRESSED            (1 << (25) )
#define ALT_LEFT_PRESSED            (1 << (26) )
#define ALT_RIGHT_PRESSED           (1 << (27) )
#define ALT_RAISE_PRESSED           (1 << (28) )
#define ALT_LOWER_PRESSED           (1 << (29) )

void PendantRead (uint32_t i2cAddr, uint16_t memaddress, uint16_t size, uint8_t * data, void* callback);
void PendantWrite (uint32_t i2cAddr, uint8_t *buf, uint16_t bytes);

jog_settings_t jog;

#endif
