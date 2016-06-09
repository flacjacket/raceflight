#pragma once

#include "esc_1wire_protocol.h"

#define BLHELI_EEPROM_HEAD 3

typedef struct {
    uint8_t BL_GOV_P_GAIN;
    uint8_t BL_GOV_I_GAIN;
    uint8_t BL_GOV_MODE;
    uint8_t BL_MOT_GAIN;
    uint8_t BL_STARTUP_PWR;
    uint8_t BL_PWM_FREQ;
    uint8_t BL_DIRECTION;
    uint8_t BL_INPUT_POL;
    uint8_t BL_INIT_L;
    uint8_t BL_INIT_H;
    uint8_t BL_ENABLE_TX;
    uint8_t BL_COMM_TIMING;
    uint8_t BL_PPM_MIN_THROTLE;
    uint8_t BL_PPM_MAX_THROTLE;
    uint8_t BL_BEEP_STRENGTH;
    uint8_t BL_BEACON_STRENGTH;
    uint8_t BL_BEACON_DELAY;
    uint8_t BL_DEMAG_COMP;
    uint8_t BL_BEC_VOLTAGE_HIGH;
    uint8_t BL_PPM_CENTER;
    uint8_t BL_TEMP_PROTECTION;
    uint8_t BL_ENABLE_POWER_PROT;
    uint8_t BL_ENABLE_PWM_INPUT;
    uint8_t BL_PWM_DITHER;
    uint8_t BL_BRAKE_ON_STOP;
    uint8_t BL_LED_CONTROL;
} BLHeli_EEprom_t;

#define BLHELI_BUF_SIZE 112

extern const esc1WireProtocol_t BLHeliAtmelProtocol;
extern const esc1WireProtocol_t BLHeliSiLabsProtocol;

uint8_t blheli_buf[BLHELI_BUF_SIZE];

bool connectBLHeli(escHardware_t *escHardware);
BLHeli_EEprom_t* getEEpromLayout(escHardware_t *escHardware, const esc1WireProtocol_t *protocol);
