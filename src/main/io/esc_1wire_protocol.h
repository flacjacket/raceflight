#pragma once

typedef enum {
    BLHeli_Silabs = 1,
    BLHeli_Atmel,
    SimonK_Atmel
} escBootloader_e;

typedef struct escDeviceInfo_s {
    uint16_t signature;        // lower 16 bit of signature
    uint8_t  signature2;       // top 8 bit of signature for SK / BootMsg last char from BL
    escBootloader_e bootloaderMode;
} escDeviceInfo_t;

typedef struct {
    GPIO_TypeDef *gpio;
    uint16_t pin;
    escDeviceInfo_t deviceInfo;
} escHardware_t;

typedef struct {
    uint16_t len;
    uint16_t addr;
    uint8_t *data;
} ioMem_t;

typedef struct {
    bool (*pollReadReady)(escHardware_t*, uint32_t timeout);
    bool (*readFlash)(escHardware_t*, ioMem_t*);
    bool (*writeFlash)(escHardware_t*, ioMem_t*);
    bool (*readEEprom)(escHardware_t*, ioMem_t*);
    bool (*writeEEprom)(escHardware_t*, ioMem_t*);
    bool (*pageErase)(escHardware_t*, ioMem_t*);
} esc1WireProtocol_t;

const esc1WireProtocol_t* getEscProtocol(escHardware_t *escHardware);

void setEscState(escHardware_t *esc, uint8_t state);
uint8_t getEscState(escHardware_t *esc);

void setEscInput(escHardware_t *escHardware);
void setEscOutput(escHardware_t *escHardware);
