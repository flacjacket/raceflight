#include <stdbool.h>
#include <string.h>
#include <stdint.h>

#include "platform.h"

#include "common/utils.h"
#include "drivers/light_led.h"
#include "drivers/system.h"
#include "io/esc_1wire_protocol.h"
#include "io/esc_1wire_blheli.h"

// Bootloader commands
// RunCmd
#define RestartBootloader 0
#define ExitBootloader    1

#define CMD_RUN            0x00
#define CMD_PROG_FLASH     0x01
#define CMD_ERASE_FLASH    0x02
#define CMD_READ_FLASH_SIL 0x03
#define CMD_VERIFY_FLASH   0x03
#define CMD_READ_EEPROM    0x04
#define CMD_PROG_EEPROM    0x05
#define CMD_READ_SRAM      0x06
#define CMD_READ_FLASH_ATM 0x07
#define CMD_KEEP_ALIVE     0xFD
#define CMD_SET_ADDRESS    0xFF
#define CMD_SET_BUFFER     0xFE

#define CMD_BOOTINIT       0x07
#define CMD_BOOTSIGN       0x08

// Bootloader result codes
#define RET_SUCCESS        0x30
#define RET_ERRORCOMMAND   0xC1
#define RET_ERRORCRC       0xC2
#define RET_NONE           0xFF

// Bootloader timings
#define BIT_TIME          52               // 52uS -> 19200 baud
#define START_BIT_TIME    ((BIT_TIME >> 1) + 1)
#define START_BIT_TIMEOUT 2000             // 2ms

#define BOOT_MSG_LEN 4
#define SIGNATURE_HIGH BOOT_MSG_LEN
#define SIGNATURE_LOW  (SIGNATURE_HIGH + 1)

// EEPROM layouts
#define NO_CMD 0xff  // value not in EEPROM

#define RX_LED_OFF LED0_OFF
#define RX_LED_ON LED0_ON
#define TX_LED_OFF LED1_OFF
#define TX_LED_ON LED1_ON

// EEPROM layout - BLHeli rev 21
static BLHeli_EEprom_t BLHeli21_EEprom = {
    .BL_GOV_P_GAIN = 0,
    .BL_GOV_I_GAIN = 1,
    .BL_GOV_MODE = 2,
    .BL_MOT_GAIN = 4,
    .BL_STARTUP_PWR = 6,
    .BL_PWM_FREQ = 7,
    .BL_DIRECTION = 8,
    .BL_INPUT_POL = 9,
    .BL_INIT_L = 10,
    .BL_INIT_H = 11,
    .BL_ENABLE_TX = 12,
    .BL_COMM_TIMING = 18,
    .BL_PPM_MIN_THROTLE = 22,
    .BL_PPM_MAX_THROTLE = 23,
    .BL_BEEP_STRENGTH = 24,
    .BL_BEACON_STRENGTH = 25,
    .BL_BEACON_DELAY = 26,
    .BL_DEMAG_COMP = 28,
    .BL_BEC_VOLTAGE_HIGH = 29,
    .BL_PPM_CENTER = 30,
    .BL_TEMP_PROTECTION = 32,
    .BL_ENABLE_POWER_PROT = 33,
    .BL_ENABLE_PWM_INPUT = 34,
    .BL_PWM_DITHER = 35,
    .BL_BRAKE_ON_STOP = 36,
    .BL_LED_CONTROL = NO_CMD,
};

// EEPROM layout - BLHeli_S rev 32
static BLHeli_EEprom_t BLHeliS32_EEprom = {
    .BL_GOV_P_GAIN = NO_CMD,
    .BL_GOV_I_GAIN = NO_CMD,
    .BL_GOV_MODE = NO_CMD,
    .BL_MOT_GAIN = NO_CMD,
    .BL_STARTUP_PWR = 6,
    .BL_PWM_FREQ = NO_CMD,
    .BL_DIRECTION = 8,
    .BL_INPUT_POL = NO_CMD,
    .BL_INIT_L = 10,
    .BL_INIT_H = 11,
    .BL_ENABLE_TX = 12,
    .BL_COMM_TIMING = 18,
    .BL_PPM_MIN_THROTLE = 22,
    .BL_PPM_MAX_THROTLE = 23,
    .BL_BEEP_STRENGTH = 24,
    .BL_BEACON_STRENGTH = 25,
    .BL_BEACON_DELAY = 26,
    .BL_DEMAG_COMP = 28,
    .BL_BEC_VOLTAGE_HIGH = NO_CMD,
    .BL_PPM_CENTER = 30,
    .BL_TEMP_PROTECTION = 32,
    .BL_ENABLE_POWER_PROT = 33,
    .BL_ENABLE_PWM_INPUT = 34,
    .BL_PWM_DITHER = NO_CMD,
    .BL_BRAKE_ON_STOP = 36,
    .BL_LED_CONTROL = 37,
};

BLHeli_EEprom_t* getEEpromLayout(escHardware_t *escHardware, const esc1WireProtocol_t *protocol) {
    uint8_t data[3];

    ioMem_t ioMem = {
        .addr = 0,
        .len = 3,
        .data = data,
    };

    protocol->readEEprom(escHardware, &ioMem);

    if (data[2] == 32) {
        return &BLHeliS32_EEprom;
    }

    if (data[2] == 21) {
        return &BLHeli21_EEprom;
    }

    return NULL;
}

static bool pollReadReadyBLHeli(escHardware_t *escHardware, uint32_t timeout);
static bool readFlashAtmelBLHeli(escHardware_t *escHardware, ioMem_t *ioMem);
static bool readFlashSiLabsBLHeli(escHardware_t *escHardware, ioMem_t *ioMem);
static bool writeFlashBLHeli(escHardware_t *escHardware, ioMem_t *ioMem);
static bool readEEpromAtmelBLHeli(escHardware_t *escHardware, ioMem_t *ioMem);
static bool readEEpromSiLabsBLHeli(escHardware_t *escHardware, ioMem_t *ioMem);
static bool writeEEpromAtmelBLHeli(escHardware_t *escHardware, ioMem_t *ioMem);
static bool writeEEpromSiLabsBLHeli(escHardware_t *escHardware, ioMem_t *ioMem);
static bool pageEraseAtmelBLHeli(escHardware_t *escHardware, ioMem_t *ioMem);
static bool pageEraseSiLabsBLHeli(escHardware_t *escHardware, ioMem_t *ioMem);

const esc1WireProtocol_t BLHeliAtmelProtocol = {
    .pollReadReady = pollReadReadyBLHeli,
    .readFlash = readFlashAtmelBLHeli,
    .writeFlash = writeFlashBLHeli,
    .readEEprom = readEEpromAtmelBLHeli,
    .writeEEprom = writeEEpromAtmelBLHeli,
    .pageErase = pageEraseAtmelBLHeli,
};

const esc1WireProtocol_t BLHeliSiLabsProtocol = {
    .pollReadReady = pollReadReadyBLHeli,
    .readFlash = readFlashSiLabsBLHeli,
    .writeFlash = writeFlashBLHeli,
    .readEEprom = readEEpromSiLabsBLHeli,
    .writeEEprom = writeEEpromSiLabsBLHeli,
    .pageErase = pageEraseSiLabsBLHeli,
};

static uint16_t crc16Byte(uint16_t from, uint8_t byte) {
    uint16_t crc16 = from;
    for (int i = 0; i < 8; i++) {
        if (((byte & 0x01) ^ (crc16 & 0x0001)) != 0) {
            crc16 >>= 1;
            crc16 ^= 0xA001;
        } else {
            crc16 >>= 1;
        }
        byte >>= 1;
    }
    return crc16;
}

//////////////////////////////////////////////////////////////////////
// reading

static int readByteBLHeli(escHardware_t *esc) {
    uint32_t btime;
    uint32_t start_time;

    if (!pollReadReadyBLHeli(esc, START_BIT_TIMEOUT)) {
        return -1;
    }

    // start bit
    start_time = micros();
    btime = start_time + START_BIT_TIME;
    uint16_t bitmask = 0;
    for(int bit = 0; bit < 10; bit++) {
        while (cmp32(micros(), btime) < 0);
        if (getEscState(esc)) {
            bitmask |= (1 << bit);
        }
        btime += BIT_TIME;
    }

    // check start bit and stop bit
    if ((bitmask & (1 << 0)) || (!(bitmask & (1 << 9)))) {
        return -1;
    }
    return bitmask >> 1;
}

static uint8_t readBufBLHeli(escHardware_t *esc, uint8_t *pstring, int len, bool checkCrc) {
    int crc = 0;
    int c;

    RX_LED_ON;

    uint8_t lastACK = RET_NONE;
    for (int i = 0; i < len; i++) {
        if ((c = readByteBLHeli(esc)) < 0) goto timeout;
        crc = crc16Byte(crc, c);
        pstring[i] = c;
    }

    if (checkCrc) {
        // With CRC, read 2 more for checksum
        for (int i = 0; i < 2; i++) {  // checksum 2 CRC bytes
            if ((c = readByteBLHeli(esc)) < 0) goto timeout;
            crc = crc16Byte(crc, c);
        }
    }

    // get ack
    if ((c = readByteBLHeli(esc)) < 0) goto timeout;
    lastACK = c;

    if (checkCrc) {
        if (crc != 0) {  // CRC of correct message is 0
            lastACK = RET_ERRORCRC;
        }
    }

timeout:
    RX_LED_OFF;

    return lastACK == RET_SUCCESS;
}

static uint8_t getAck(escHardware_t *esc, int retry) {
    int c;
    while ((c = readByteBLHeli(esc)) < 0) {
        if (--retry < 0) {    // timeout=1 -> 1 retry
            return RET_NONE;
        }
    }
    return c;
}

//////////////////////////////////////////////////////////////////////
// writing

static void writeByteBLHeli(escHardware_t *esc, uint8_t byte) {
    // send one idle bit first (stopbit from previous byte)
    uint16_t bitmask = (byte << 2) | (1 << 0) | (1 << 10);
    uint32_t btime = micros();

    while (true) {
        setEscState(esc, bitmask & 1);
        btime += BIT_TIME;
        bitmask >>= 1;
        if (bitmask == 0) {
            break; // stopbit shifted out - but don't wait
        }
        while (cmp32(micros(), btime) < 0);
    }
}

static void writeBufBLHeli(escHardware_t *esc, uint8_t *pstring, int len, bool appendCrc) {
    uint16_t crc = 0;
    setEscOutput(esc);
    TX_LED_ON;
    for(int i = 0; i < len; i++) {
        writeByteBLHeli(esc, pstring[i]);
        crc = crc16Byte(crc, pstring[i]);
    }
    if (appendCrc) {
        writeByteBLHeli(esc, crc & 0xff);
        writeByteBLHeli(esc, crc >> 8);
    }
    TX_LED_OFF;
    setEscInput(esc);
}

//////////////////////////////////////////////////////////////////////
// commands

static uint8_t sendCmdSetAddress(escHardware_t *esc, ioMem_t *ioMem) { //supports only 16 bit Adr
    // skip if adr == 0xFFFF
    if ((ioMem->addr == 0xffff)) {
        return 1;
    }

    uint8_t sCMD[] = {CMD_SET_ADDRESS, 0, ioMem->addr >> 8, ioMem->addr & 0xff};
    writeBufBLHeli(esc, sCMD, sizeof(sCMD), true);
    return getAck(esc, 2) == RET_SUCCESS;
}

static uint8_t sendCmdSetBuffer(escHardware_t *esc, ioMem_t *ioMem) {
    uint16_t len = ioMem->len;
    uint8_t sCMD[] = {CMD_SET_BUFFER, 0, len >> 8, len & 0xff};
    writeBufBLHeli(esc, sCMD, sizeof(sCMD), true);
    if (getAck(esc, 2) != RET_NONE) {
        return 0;
    }
    writeBufBLHeli(esc, ioMem->data, len, true);
    return getAck(esc, 40) == RET_SUCCESS;
}

//////////////////////////////////////////////////////////////////////
// reading and writing

static uint8_t sendReadCommand(escHardware_t *esc, uint8_t cmd, ioMem_t *ioMem) {
    if (!sendCmdSetAddress(esc, ioMem)) {
        return 0;
    }

    unsigned len = ioMem->len;
    uint8_t sCMD[] = {cmd, len & 0xff};    // 0x100 is sent a 0x00 here

    writeBufBLHeli(esc, sCMD, sizeof(sCMD), true);
    return readBufBLHeli(esc, ioMem->data, len, true);
}

static uint8_t sendWriteCommand(escHardware_t *esc, uint8_t cmd, ioMem_t *ioMem, uint8_t timeout) {
    if (!sendCmdSetAddress(esc, ioMem)) {
        return 0;
    }
    if (!sendCmdSetBuffer(esc, ioMem)) {
        return 0;
    }
    uint8_t sCMD[] = {cmd, 0x01};
    writeBufBLHeli(esc, sCMD, sizeof(sCMD), true);
    return getAck(esc, timeout) == RET_SUCCESS;
}

//////////////////////////////////////////////
// protocol

bool connectBLHeli(escHardware_t *escHardware) {
    uint8_t bootInfo[BOOT_MSG_LEN + 4];
    const uint8_t bootMsgCheck[BOOT_MSG_LEN - 1] = "471";

    uint8_t bootInit[] = {
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // preface message with long preamble
        0x0D, 'B', 'L', 'H', 'e', 'l', 'i', 0xF4, 0x7D
    };

    writeBufBLHeli(escHardware, bootInit, sizeof(bootInit), false);
    if (!readBufBLHeli(escHardware, bootInfo, sizeof(bootInfo), false)) {
        return false;
    }

    // BootInfo has no CRC (ACK byte already analyzed... )
    // Format = BootMsg("471c"), SIGNATURE_HIGH, SIGNATURE_LOW, BootVersion (always 6), BootPages (,ACK)
    if (memcmp(bootInfo, bootMsgCheck, sizeof(bootMsgCheck)) != 0) { // Check only the first 3 letters -> 471x OK
        return false;
    }

    escHardware->deviceInfo.signature2 = bootInfo[BOOT_MSG_LEN - 1]; // taken from bootloaderMsg part, ascii 'c' now
    escHardware->deviceInfo.signature = (bootInfo[SIGNATURE_HIGH] << 8) | bootInfo[SIGNATURE_LOW]; // SIGNATURE_001, SIGNATURE_002
    return true;
}

static bool pollReadReadyBLHeli(escHardware_t *escHardware, uint32_t timeout) {
    uint32_t wait_time = micros() + timeout;
    while (getEscState(escHardware)) {
        // check for start bit begin
        if (micros() >= wait_time) {
            return false;
        }
    }

    return true;
}

static bool readFlashAtmelBLHeli(escHardware_t *escHardware, ioMem_t *ioMem) {
    return sendReadCommand(escHardware, CMD_READ_FLASH_ATM, ioMem);
}

static bool readFlashSiLabsBLHeli(escHardware_t *escHardware, ioMem_t *ioMem) {
    return sendReadCommand(escHardware, CMD_READ_FLASH_SIL, ioMem);
}

static bool writeFlashBLHeli(escHardware_t *escHardware, ioMem_t *ioMem) {
    return sendWriteCommand(escHardware, CMD_PROG_FLASH, ioMem, 40 * 1000 / START_BIT_TIMEOUT);
}

static bool readEEpromAtmelBLHeli(escHardware_t *escHardware, ioMem_t *ioMem) {
    return sendReadCommand(escHardware, CMD_READ_EEPROM, ioMem);
}

static bool readEEpromSiLabsBLHeli(escHardware_t *escHardware, ioMem_t *ioMem) {
    // SiLabs has no EEPROM, just a flash section at 0x1A00
    ioMem->addr += 0x1A00;
    return readFlashSiLabsBLHeli(escHardware, ioMem);
}

static bool writeEEpromAtmelBLHeli(escHardware_t *escHardware, ioMem_t *ioMem) {
    return sendWriteCommand(escHardware, CMD_PROG_EEPROM, ioMem, 3000 * 1000 / START_BIT_TIMEOUT);
}

static bool writeEEpromSiLabsBLHeli(escHardware_t *escHardware, ioMem_t *ioMem) {
    // SiLabs has no EEPROM, just a flash section at 0x1A00
    ioMem->addr += 0x1A00;
    return writeFlashBLHeli(escHardware, ioMem);
}

static bool pageEraseAtmelBLHeli(escHardware_t *escHardware, ioMem_t *ioMem) {
    if (!sendCmdSetAddress(escHardware, ioMem)) {
        return 0;
    }

    uint8_t sCMD[] = {CMD_ERASE_FLASH, 0x01};
    writeBufBLHeli(escHardware, sCMD, sizeof(sCMD), true);
    return getAck(escHardware, 40 * 1000 / START_BIT_TIMEOUT) == RET_SUCCESS;
}

static bool pageEraseSiLabsBLHeli(escHardware_t *escHardware, ioMem_t *ioMem) {
    // SiLabs has no EEPROM, just a flash section at 0x1A00
    ioMem->addr += 0x1A00;
    return pageEraseAtmelBLHeli(escHardware, ioMem);
}
