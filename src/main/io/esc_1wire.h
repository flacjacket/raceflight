/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Ported from https://github.com/4712/BLHeliSuite/blob/master/Interfaces/Arduino1Wire/Source/Arduino1Wire_C/Arduino1Wire.c
 *  by Nathan Tsoi <nathan@vertile.com>
 */

#pragma once

#ifdef USE_ESC_SERIAL

typedef enum {
    MOT_NORMAL = 1,
    MOT_REVERSE,
    MOT_BIDIR,
    MOT_BIDIR_REVERSE, // only on BLHeli_S, I think
} motorDirection_e;

uint8_t esc1WireInitialize(void);
uint8_t esc1WireStart(void);
uint8_t esc1WireDumpEEprom(uint8_t** buf, uint8_t escIndex);
void esc1WireRelease(void);
uint8_t esc1WireSetMotorDirection(uint8_t escIndex, uint8_t motorDirection);
uint8_t esc1WireGetMotorDirection(uint8_t escIndex);

#endif
