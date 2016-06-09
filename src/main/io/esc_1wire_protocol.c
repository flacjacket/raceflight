#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#include "platform.h"

#include "drivers/gpio.h"

#include "esc_1wire_protocol.h"
#include "esc_1wire_blheli.h"

////////////////////////////////////////////////////////
// Get and set digital pin value

void setEscState(escHardware_t *esc, uint8_t state) {
    if (state) {
        digitalHi(esc->gpio, esc->pin);
    } else {
        digitalLo(esc->gpio, esc->pin);
    }
}

uint8_t getEscState(escHardware_t *esc) {
    return digitalIn(esc->gpio, esc->pin) == Bit_SET;
}

////////////////////////////////////////////////////////
// Direct gpio pin configuration

static void escGPIOConfig(GPIO_TypeDef *gpio, uint16_t pin, GPIO_Mode mode) {
    gpio_config_t cfg;

    cfg.pin = pin;
    cfg.mode = mode;
    cfg.speed = Speed_2MHz;
    gpioInit(gpio, &cfg);
}

void setEscInput(escHardware_t *esc) {
#ifdef STM32F10X
    escGPIOConfig(esc->gpio, esc->pin, Mode_IPU);
#else
    escGPIOConfig(esc->gpio, esc->pin, Mode_IPU);
#endif
}

void setEscOutput(escHardware_t *esc) {
    escGPIOConfig(esc->gpio, esc->pin, Mode_Out_PP);
}

////////////////////////////////////////////////////////
// Protocol configuration

static uint16_t signaturesAtmel[] =  {0x9307, 0x930A, 0x930F, 0x940B, 0};
static uint16_t signaturesSilabs[] = {0xF310, 0xF330, 0xF410, 0xF390, 0xF850, 0xE8B1, 0xE8B2, 0};

static bool signatureMatch(uint16_t signature, uint16_t *list) {
    for(; *list; list++) {
        if (signature == *list) {
            return true;
        }
    }
    return false;
}

// Try connecting to device bootloader to set protocol
const esc1WireProtocol_t* getEscProtocol(escHardware_t *escHardware) {
    for (int i = 0; i < 3; i++) {
#if 0
        if (Stk_ConnectEx(pDeviceInfo) && signatureMatch(pDeviceInfo->signature, signaturesAtmel)) {
            escHardware->deviceInfo.bootloaderMode = imSK;
            return true;
        }
#endif
        if (connectBLHeli(escHardware)) {
            // compare to known signatures
            if (signatureMatch(escHardware->deviceInfo.signature, signaturesSilabs)) {
                escHardware->deviceInfo.bootloaderMode = BLHeli_Silabs;
                return &BLHeliSiLabsProtocol;
            } else if (signatureMatch(escHardware->deviceInfo.signature, signaturesAtmel)) {
                escHardware->deviceInfo.bootloaderMode = BLHeli_Atmel;
                return &BLHeliAtmelProtocol;
            } else {
                // some problem with detection of the signature, reset signatures
                escHardware->deviceInfo.signature = escHardware->deviceInfo.signature2 = 0;
            }
        }
    }

    return NULL;
}
