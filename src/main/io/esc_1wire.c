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
 * Several updates by 4712 in order to optimize interaction with BLHeliSuite
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#ifdef USE_ESC_SERIAL

#include "drivers/gpio.h"
#include "drivers/pwm_output.h"
#include "drivers/pwm_mapping.h"
#include "drivers/system.h"
#include "io/esc_1wire.h"
#include "io/esc_1wire_protocol.h"
#include "io/beeper.h"
#include "flight/mixer.h"

#include "esc_1wire_blheli.h"

uint8_t escCount; // we detect the hardware dynamically

static escHardware_t escHardware[MAX_PWM_MOTORS];
static const esc1WireProtocol_t *escProtocol[MAX_PWM_MOTORS];

////////////////////////////////////////////////////////
// 1wire functions

void esc1WireInitialize(void) {
    escCount = 0;
    memset(&escHardware, 0, sizeof(escHardware));
    memset(&escProtocol, 0, sizeof(escProtocol));
    pwmOutputConfiguration_t *pwmOutputConfiguration = pwmGetOutputConfiguration();
    for (uint8_t i = 0; i < pwmOutputConfiguration->outputCount; i++) {
        // if setup as motor and motor output set to non-zero value
        if ((pwmOutputConfiguration->portConfigurations[i].flags & PWM_PF_MOTOR) &&
                (motor[pwmOutputConfiguration->portConfigurations[i].index] > 0)) {
            escHardware[escCount].gpio = pwmOutputConfiguration->portConfigurations[i].timerHardware->gpio;
            escHardware[escCount].pin = pwmOutputConfiguration->portConfigurations[i].timerHardware->pin;
            escCount++;
        }
    }
    return escCount;
}

void esc1WireStart(void) {
    pwmDisableMotors();    // prevent updating PWM registers

    uint8_t escActive = 0;

    for (uint8_t escIndex = 0; escIndex < escCount; escIndex++) {
        setEscInput(&escHardware[escIndex]);
        setEscState(&escHardware[escIndex], 1);
    }

    for (uint8_t escIndex = 0; escIndex < escCount; escIndex++) {
        escProtocol[escIndex] = getEscProtocol(&escHardware[escIndex]);
        if (escProtocol[escIndex] != NULL) {
            escActive++;
        }
    }

    return escActive;
}

uint8_t esc1WireDumpEEprom(uint8_t** buf, uint8_t escIndex) {
    const esc1WireProtocol_t *escProto = escProtocol[escIndex];

    if (escProto == NULL) {
        *buf = NULL;
        return 0;
    }

    ioMem_t ioMem = {
        .addr = 0,
        .len = BLHELI_BUF_SIZE,
        .data = blheli_buf,
    };

    if (!escProto->readEEprom(&escHardware[escIndex], &ioMem)) {
        *buf = NULL;
        return 0;
    }

    *buf = blheli_buf;
    return BLHELI_BUF_SIZE;
}

uint8_t esc1WireGetMotorDirection(uint8_t escIndex) {
    escHardware_t *esc = &escHardware[escIndex];
    const esc1WireProtocol_t *proto = escProtocol[escIndex];

    if (proto == NULL) {
        return 0;
    }

    BLHeli_EEprom_t *layout = getEEpromLayout(esc, proto);

    ioMem_t ioMem = {
        .addr = 0,
        .len = BLHELI_BUF_SIZE,
        .data = blheli_buf,
    };

    if (!proto->readEEprom(esc, &ioMem)) {
        return 0;
    }

    return blheli_buf[BLHELI_EEPROM_HEAD + layout->BL_DIRECTION];
}

uint8_t esc1WireSetMotorDirection(uint8_t escIndex, uint8_t motorDirection) {
    escHardware_t *esc = &escHardware[escIndex];
    const esc1WireProtocol_t *proto = escProtocol[escIndex];

    if (proto == NULL) {
        return false;
    }

    BLHeli_EEprom_t *layout = getEEpromLayout(esc, proto);
    ioMem_t ioMem = {
        .addr = 0,
        .len = BLHELI_BUF_SIZE,
        .data = blheli_buf,
    };

    if (!proto->readEEprom(esc, &ioMem)) {
        return 0;
    }
    blheli_buf[BLHELI_EEPROM_HEAD + layout->BL_DIRECTION] = motorDirection;

    uint8_t data = 0x0d;
    ioMem.addr = 0;
    ioMem.len = 1;
    ioMem.data = &data;
    proto->pageErase(esc, &ioMem);

    ioMem.addr = 0;
    ioMem.len = BLHELI_BUF_SIZE;
    ioMem.data = blheli_buf;
    return proto->writeEEprom(esc, &ioMem);
}

// returns all claimed pins back to PWM drivers, re-enables PWM
void esc1WireRelease(void) {
    // stop ESC serial communication
    for(int escIndex = 0; escIndex < escCount; escIndex++) {
        setEscOutput(&escHardware[escIndex]);
        setEscState(&escHardware[escIndex], 0);
    }

    memset(&escProtocol, 0, sizeof(escProtocol));

    pwmEnableMotors();
}

#endif /* USE_ESC_SERIAL */
