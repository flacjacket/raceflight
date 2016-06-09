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
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#ifdef USE_ESC_SERIAL

#include "common/utils.h"

#include "drivers/nvic.h"
#include "drivers/system.h"
#include "drivers/gpio.h"

#include "drivers/serial.h"
#include "drivers/serial_escserial.h"
#include "drivers/timer.h"
#include "drivers/pwm_mapping.h"

#define ESCSERIAL_BUFFER_SIZE 256

#define RX_TOTAL_BITS 10
#define TX_TOTAL_BITS 10

typedef struct escSerial_s {
    serialPort_t     port;

    const timerHardware_t *rxTimerHardware;
    volatile uint8_t rxBuffer[ESCSERIAL_BUFFER_SIZE];
    const timerHardware_t *txTimerHardware;
    volatile uint8_t txBuffer[ESCSERIAL_BUFFER_SIZE];

    uint8_t          isSearchingForStartBit;
    uint8_t          rxBitIndex;
    uint8_t          rxLastLeadingEdgeAtBitIndex;
    uint8_t          rxLastEdge;

    uint8_t          isTransmittingData;
    uint8_t          isReceivingData;
    int8_t           bitsLeftToTransmit;

    uint16_t         internalTxBuffer;  // includes start and stop bits
    uint16_t         internalRxBuffer;  // includes start and stop bits

    uint16_t         receiveTimeout;
    uint16_t         transmissionErrors;
    uint16_t         receiveErrors;

    uint8_t          escSerialPortIndex;

    timerCCHandlerRec_t timerCb;
    timerCCHandlerRec_t edgeCb;
} escSerial_t;

escSerial_t escSerialPorts[MAX_PWM_MOTORS];

extern const struct serialPortVTable escSerialVTable[];

void onEscSerialTxTimer(timerCCHandlerRec_t *cbRec, captureCompare_t capture);
void onEscSerialRxEdge(timerCCHandlerRec_t *cbRec, captureCompare_t capture);
static void serialICConfig(TIM_TypeDef *tim, uint8_t channel, uint16_t polarity);

void setTxSignal(escSerial_t *escSerial, uint8_t state)
{
    if (state) {
        digitalHi(escSerial->rxTimerHardware->gpio, escSerial->rxTimerHardware->pin);
    } else {
        digitalLo(escSerial->rxTimerHardware->gpio, escSerial->rxTimerHardware->pin);
    }
}

///////////////////////////////////////////////////////////////////////////////

static void escSerialGPIOConfig(GPIO_TypeDef *gpio, uint16_t pin, GPIO_Mode mode) {
    gpio_config_t cfg;

    cfg.pin = pin;
    cfg.mode = mode;
    cfg.speed = Speed_2MHz;
    gpioInit(gpio, &cfg);
}

static void escSerialInputConfig(const timerHardware_t *timerHardwarePtr) {
#ifdef STM32F10X
    escSerialGPIOConfig(timerHardwarePtr->gpio, timerHardwarePtr->pin, Mode_IPU);
#else
    escSerialGPIOConfig(timerHardwarePtr->gpio, timerHardwarePtr->pin, Mode_AF_PP_PU);
#endif
    //escSerialGPIOConfig(timerHardwarePtr->gpio, timerHardwarePtr->pin, timerHardwarePtr->gpioInputMode);
    timerChClearCCFlag(timerHardwarePtr);
    timerChITConfig(timerHardwarePtr, ENABLE);
}

//escHardware[escCount].pinpos = getPinPos(escHardware[escCount].pin);

static void escSerialOutputConfig(const timerHardware_t *timerHardwarePtr) {
    escSerialGPIOConfig(timerHardwarePtr->gpio, timerHardwarePtr->pin, Mode_Out_PP);
    timerChITConfig(timerHardwarePtr, DISABLE);
}

static void escSerialInputDeconfig(const timerHardware_t *timerHardwarePtr) {
    escSerialGPIOConfig(timerHardwarePtr->gpio, timerHardwarePtr->pin, Mode_IPU);
    timerChClearCCFlag(timerHardwarePtr);
    timerChITConfig(timerHardwarePtr, DISABLE);
}

///////////////////////////////////////////////////////////////////////////////

static bool isTimerPeriodTooLarge(uint32_t timerPeriod)
{
    return timerPeriod > 0xFFFF;
}

static void escSerialTimerTxConfig(const timerHardware_t *timerHardwarePtr, uint8_t escIndex, uint32_t baud)
{
    uint32_t clock = SystemCoreClock;
    uint32_t timerPeriod;
    TIM_DeInit(timerHardwarePtr->tim);
    do {
        clock /= 2;
        timerPeriod = clock / baud;
    } while (isTimerPeriodTooLarge(timerPeriod) && (clock > 1000000));
    // TODO: if unable to determine clock and timerPeriods for the given baud

    uint8_t mhz = clock / 1000000;
    timerConfigure(timerHardwarePtr, timerPeriod, mhz);
    timerChCCHandlerInit(&escSerialPorts[escIndex].timerCb, onEscSerialTxTimer);
    timerChConfigCallbacks(timerHardwarePtr, &escSerialPorts[escIndex].timerCb, NULL);
}

static void escSerialTimerRxConfig(const timerHardware_t *timerHardwarePtr, uint8_t escIndex, portOptions_t options) {
    // start bit is usually a FALLING signal
    uint8_t mhz = SystemCoreClock / 2000000;
    TIM_DeInit(timerHardwarePtr->tim);
    timerConfigure(timerHardwarePtr, 0xFFFF, mhz);
	serialICConfig(timerHardwarePtr->tim, timerHardwarePtr->channel, (options & SERIAL_INVERTED) ? TIM_ICPolarity_Rising : TIM_ICPolarity_Falling);
    timerChCCHandlerInit(&escSerialPorts[escIndex].edgeCb, onEscSerialRxEdge);
    timerChConfigCallbacks(timerHardwarePtr, &escSerialPorts[escIndex].edgeCb, NULL);
}

///////////////////////////////////////////////////////////////////////////////

static void serialICConfig(TIM_TypeDef *tim, uint8_t channel, uint16_t polarity)
{
    TIM_ICInitTypeDef TIM_ICInitStructure;

    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_Channel = channel;
    TIM_ICInitStructure.TIM_ICPolarity = polarity;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.TIM_ICFilter = 0x0;

    TIM_ICInit(tim, &TIM_ICInitStructure);
}

static void resetBuffers(escSerial_t *escSerial) {
    escSerial->port.rxBufferSize = ESCSERIAL_BUFFER_SIZE;
    escSerial->port.rxBuffer = escSerial->rxBuffer;
    escSerial->port.rxBufferTail = 0;
    escSerial->port.rxBufferHead = 0;

    escSerial->port.txBufferSize = ESCSERIAL_BUFFER_SIZE;
    escSerial->port.txBuffer = escSerial->txBuffer;
    escSerial->port.txBufferTail = 0;
    escSerial->port.txBufferHead = 0;
}

serialPort_t* openEscSerial(uint8_t motorIndex, const timerHardware_t *rxTimer, const timerHardware_t *txTimer,
		uint32_t baud, portOptions_t options) {
    escSerial_t *escSerial = &(escSerialPorts[motorIndex]);

    escSerial->rxTimerHardware = rxTimer;
    escSerial->txTimerHardware = txTimer;

    escSerial->port.vTable = escSerialVTable;
    escSerial->port.baudRate = baud;
    escSerial->port.mode = MODE_RXTX;
    escSerial->port.options = options;
    escSerial->port.callback = NULL;

    resetBuffers(escSerial);

    escSerial->isTransmittingData = false;

    escSerial->isSearchingForStartBit = true;
    escSerial->rxBitIndex = 0;

    escSerial->transmissionErrors = 0;
    escSerial->receiveErrors = 0;
    escSerial->receiveTimeout = 0;

    escSerial->escSerialPortIndex = motorIndex;

    escSerialInputConfig(escSerial->rxTimerHardware);

    setTxSignal(escSerial, ENABLE);
    delay(50);

	escSerialTimerTxConfig(escSerial->txTimerHardware, motorIndex, baud);
	escSerialTimerRxConfig(escSerial->rxTimerHardware, motorIndex, options);

    return &escSerial->port;
}

void closeEscSerial(uint8_t motorIndex) {
    escSerial_t *escSerial = &escSerialPorts[motorIndex];
    escSerialInputDeconfig(escSerial->rxTimerHardware);

    timerChConfigCallbacks(escSerial->txTimerHardware, NULL, NULL);
    timerChConfigCallbacks(escSerial->rxTimerHardware, NULL, NULL);
    TIM_DeInit(escSerial->txTimerHardware->tim);
    TIM_DeInit(escSerial->rxTimerHardware->tim);
}

/* escSerialVTable */

void escSerialWriteByte(serialPort_t *s, uint8_t ch) {
    if ((s->mode & MODE_TX) == 0) {
        return;
    }

    s->txBuffer[s->txBufferHead] = ch;
    s->txBufferHead = (s->txBufferHead + 1) % s->txBufferSize;
}

uint8_t escSerialTotalBytesWaiting(serialPort_t *instance) {
    if ((instance->mode & MODE_RX) == 0) {
        return 0;
    }

    escSerial_t *s = (escSerial_t *)instance;

    return (s->port.rxBufferHead - s->port.rxBufferTail) & (s->port.rxBufferSize - 1);
}

uint8_t escSerialTxBytesFree(serialPort_t *instance) {
    if ((instance->mode & MODE_TX) == 0) {
        return 0;
    }

    escSerial_t *s = (escSerial_t *)instance;

    uint8_t bytesUsed = (s->port.txBufferHead - s->port.txBufferTail) & (s->port.txBufferSize - 1);

    return (s->port.txBufferSize - 1) - bytesUsed;
}

uint8_t escSerialReadByte(serialPort_t *instance) {
    uint8_t ch;

    if ((instance->mode & MODE_RX) == 0) {
        return 0;
    }

    if (escSerialTotalBytesWaiting(instance) == 0) {
        return 0;
    }

    ch = instance->rxBuffer[instance->rxBufferTail];
    instance->rxBufferTail = (instance->rxBufferTail + 1) % instance->rxBufferSize;
    return ch;
}

void escSerialSetBaudRate(serialPort_t *s, uint32_t baudRate) {
    UNUSED(s);
    UNUSED(baudRate);
}

bool isEscSerialTransmitBufferEmpty(serialPort_t *instance) {
	// start listening
    return instance->txBufferHead == instance->txBufferTail;
}

void escSerialSetMode(serialPort_t *instance, portMode_t mode) {
    instance->mode = mode;
}

const struct serialPortVTable escSerialVTable[] = {
    {
        .serialWrite=escSerialWriteByte,
        .serialTotalRxWaiting=escSerialTotalBytesWaiting,
        .serialTotalTxFree=escSerialTxBytesFree,
        .serialRead=escSerialReadByte,
        .serialSetBaudRate=escSerialSetBaudRate,
        .isSerialTransmitBufferEmpty=isEscSerialTransmitBufferEmpty,
        .setMode=escSerialSetMode,
		.beginWrite=NULL,
		.endWrite=NULL,
    }
};

/* end escSerialVTable */

/*-----------------------BL*/

enum {
    TRAILING,
    LEADING
};

static void applyChangedBitsBL(escSerial_t *escSerial) {
    if (escSerial->rxLastEdge == TRAILING) {
        uint8_t bitToSet;
        for (bitToSet = escSerial->rxLastLeadingEdgeAtBitIndex; bitToSet < escSerial->rxBitIndex; bitToSet++) {
            escSerial->internalRxBuffer |= 1 << bitToSet;
        }
    }
}

static void prepareForNextRxByteBL(escSerial_t *escSerial)
{
    // prepare for next byte
    escSerial->rxBitIndex = 0;
    escSerial->isSearchingForStartBit = true;
    if (escSerial->rxLastEdge == LEADING) {
        escSerial->rxLastEdge = TRAILING;
        serialICConfig(
            escSerial->rxTimerHardware->tim,
            escSerial->rxTimerHardware->channel,
            (escSerial->port.options & SERIAL_INVERTED) ? TIM_ICPolarity_Rising : TIM_ICPolarity_Falling
        );
    }
}

#define STOP_BIT_MASK (1 << 0)
#define START_BIT_MASK (1 << (RX_TOTAL_BITS - 1))

static void extractAndStoreRxByteBL(escSerial_t *escSerial) {
    if ((escSerial->port.mode & MODE_RX) == 0) {
        return;
    }

    uint8_t haveStartBit = (escSerial->internalRxBuffer & START_BIT_MASK) == 0;
    uint8_t haveStopBit = (escSerial->internalRxBuffer & STOP_BIT_MASK) == 1;

    if (!haveStartBit || !haveStopBit) {
        escSerial->receiveErrors++;
        return;
    }

    uint8_t rxByte = (escSerial->internalRxBuffer >> 1) & 0xFF;

    if (escSerial->port.callback) {
        escSerial->port.callback(rxByte);
    } else {
        escSerial->port.rxBuffer[escSerial->port.rxBufferHead] = rxByte;
        escSerial->port.rxBufferHead = (escSerial->port.rxBufferHead + 1) % escSerial->port.rxBufferSize;
    }
}

void processEscTxState(escSerial_t *escSerial)
{
    uint8_t txBit;

    if (escSerial->isReceivingData) {
    	return;
    }

    if (!escSerial->isTransmittingData) {
        char byteToSend;
        if (isEscSerialTransmitBufferEmpty((serialPort_t *)escSerial)) {
        	// can receive
            return;
        }

        // data to send
        byteToSend = escSerial->port.txBuffer[escSerial->port.txBufferTail++];
        if (escSerial->port.txBufferTail >= escSerial->port.txBufferSize) {
            escSerial->port.txBufferTail = 0;
        }

        // build internal buffer, MSB = Stop Bit (1) + data bits (MSB to LSB) + start bit(0) LSB
        escSerial->internalTxBuffer = (1 << (TX_TOTAL_BITS - 1)) | (byteToSend << 1);
        escSerial->bitsLeftToTransmit = TX_TOTAL_BITS;
        escSerial->isTransmittingData = true;

        //set output
        escSerialOutputConfig(escSerial->rxTimerHardware);
        return;
    }

    // transmit next bit
    if (escSerial->bitsLeftToTransmit) {
        txBit = escSerial->internalTxBuffer & 1;
        escSerial->internalTxBuffer >>= 1;

        setTxSignal(escSerial, txBit);
        escSerial->bitsLeftToTransmit--;
        return;
    }

    // transmission completed, enable rx if buffer empty
    escSerial->isTransmittingData = false;
    if (isEscSerialTransmitBufferEmpty((serialPort_t *)escSerial)) {
    	escSerialInputConfig(escSerial->rxTimerHardware);
    }
}

void processEscRxState(escSerial_t *escSerial)
{
    if (escSerial->isSearchingForStartBit) {
        return;
    }

    escSerial->rxBitIndex++;

    if (escSerial->rxBitIndex == RX_TOTAL_BITS - 1) {
        applyChangedBitsBL(escSerial);
        return;
    }

    if (escSerial->rxBitIndex == RX_TOTAL_BITS) {

        if (escSerial->rxLastEdge == TRAILING) {
            escSerial->internalRxBuffer |= STOP_BIT_MASK;
        }

        extractAndStoreRxByteBL(escSerial);
        prepareForNextRxByteBL(escSerial);
    }
}

void onEscSerialTxTimer(timerCCHandlerRec_t *cbRec, captureCompare_t capture) {
    UNUSED(capture);
    escSerial_t *escSerial = container_of(cbRec, escSerial_t, timerCb);

    processEscTxState(escSerial);
    processEscRxState(escSerial);
}

void onEscSerialRxEdge(timerCCHandlerRec_t *cbRec, captureCompare_t capture) {
    UNUSED(capture);

    escSerial_t *escSerial = container_of(cbRec, escSerial_t, edgeCb);
    bool inverted = escSerial->port.options & SERIAL_INVERTED;

    if ((escSerial->port.mode & MODE_RX) == 0) {
        return;
    }

    if (escSerial->isSearchingForStartBit) {
        // synchronise bit counter
        // FIXME this reduces functionality somewhat as receiving breaks concurrent transmission on all ports because
        // the next callback to the onEscSerialTxTimer will happen too early causing transmission errors.
        TIM_SetCounter(escSerial->txTimerHardware->tim, escSerial->txTimerHardware->tim->ARR / 2);
        if (escSerial->isTransmittingData) {
            escSerial->transmissionErrors++;
        }

        serialICConfig(escSerial->rxTimerHardware->tim, escSerial->rxTimerHardware->channel, inverted ? TIM_ICPolarity_Falling : TIM_ICPolarity_Rising);
        escSerial->rxLastEdge = LEADING;

        escSerial->rxBitIndex = 0;
        escSerial->rxLastLeadingEdgeAtBitIndex = 0;
        escSerial->internalRxBuffer = 0;
        escSerial->isSearchingForStartBit = false;
        return;
    }

    if (escSerial->rxLastEdge == LEADING) {
        escSerial->rxLastLeadingEdgeAtBitIndex = escSerial->rxBitIndex;
    }

    applyChangedBitsBL(escSerial);

    if (escSerial->rxLastEdge == TRAILING) {
        escSerial->rxLastEdge = LEADING;
        serialICConfig(escSerial->rxTimerHardware->tim, escSerial->rxTimerHardware->channel, inverted ? TIM_ICPolarity_Falling : TIM_ICPolarity_Rising);
    } else {
        escSerial->rxLastEdge = TRAILING;
        serialICConfig(escSerial->rxTimerHardware->tim, escSerial->rxTimerHardware->channel, inverted ? TIM_ICPolarity_Rising : TIM_ICPolarity_Falling);
    }
}
/*-------------------------BL*/

/*void escSerialInitialize()
{pwmDisableMotors
   StopPwmAllMotors();
   for (volatile uint8_t i = 0; i < USABLE_TIMER_CHANNEL_COUNT; i++) {
	   // set outputs to pullup
	   if(timerHardware[i].outputEnable==1)
	   {
		   escSerialGPIOConfig(timerHardware[i].gpio, timerHardware[i].pin, Mode_IPU); //GPIO_Mode_IPU
	   }
   }
}*/

// mode 0=sk, mode 1=bl output=timerHardware PWM channel.
//void escEnablePassthrough(serialPort_t *escPassthroughPort, uint16_t output, uint8_t mode)
/*bool esc1WireEnter(uint16_t output, uint8_t mode)
{
    uint8_t first_output = 0;
    for (volatile uint8_t i = 0;; i++) {
 	   if (timerHardware[i].outputEnable == 1) {
 		   first_output = i;
 		   break;
 	   }

 	   if (i >= USABLE_TIMER_CHANNEL_COUNT) {
 		   return false;
 	   }
    }

    //doesn't work with messy timertable
    uint8_t motor_output = first_output + output - 1;
    if(motor_output >= USABLE_TIMER_CHANNEL_COUNT)
    	return false;

    LED0_OFF;
    LED1_OFF;
    StopPwmAllMotors();
    // passPort = escPassthroughPort;

#ifdef BEEPER
    // fix for buzzer often starts beeping continuously when the ESCs are read
    // switch beeper silent here
    beeperSilence();
#endif

    escPort = openEscSerial(ESCSERIAL1, NULL, motor_output, 19200, 0, mode);
    uint8_t ch;
    while(1) {
        if (serialRxBytesWaiting(escPort)) {
            LED0_ON;
            while(serialRxBytesWaiting(escPort))
            {
            	ch = serialRead(escPort);
            	//serialWrite(escPassthroughPort, ch);
            }
            LED0_OFF;
        }
        //if (serialRxBytesWaiting(escPassthroughPort))
        {
            LED1_ON;
            //while(serialRxBytesWaiting(escPassthroughPort))
            {
            	//ch = serialRead(escPassthroughPort);
            	//exitEsc = ProcessExitCommand(ch);
            	//if(exitEsc)
            	{
            		serialWrite(escPassthroughPort, 0x24);
            		serialWrite(escPassthroughPort, 0x4D);
            		serialWrite(escPassthroughPort, 0x3E);
            		serialWrite(escPassthroughPort, 0x00);
            		serialWrite(escPassthroughPort, 0xF4);
            		serialWrite(escPassthroughPort, 0xF4);
            	    closeEscSerial(ESCSERIAL1, output);
            		return;
            	}
            	if(mode) {
            		//serialWrite(escPassthroughPort, ch); // blheli loopback
            	}
        		serialWrite(escPort, ch);
            }
            LED1_OFF;
        }
        delay(5);
    }
}*/


#endif
