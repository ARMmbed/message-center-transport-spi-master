/* mbed Microcontroller Library
 * Copyright (c) 2006-2015 ARM Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef __MESSAGE_CENTER_SPI_MASTER_H__
#define __MESSAGE_CENTER_SPI_MASTER_H__

#include "mbed-drivers/mbed.h"
#include "mbed-block/BlockStatic.h"

#if 0
#if defined(TARGET_LIKE_STK3700)
#include "swo/swo.h"
#undef printf
#define printf swoprintf
#endif
#endif

using namespace mbed::util;

class MessageCenterSPIMaster
{
public:
    MessageCenterSPIMaster(SPI& _spi, PinName _cs, PinName _irq);


    bool sendTask(BlockStatic* block, void (*callback)(void))
    {
        sendDoneCallback.attach(callback);

        return internalSendTask(block);
    }

private:

    typedef enum {
        STATE_IDLE,
        STATE_SEND_COMMAND,
        STATE_SEND_WAIT,
        STATE_SEND_MESSAGE,
        STATE_SEND_DONE
    } state_t;

    state_t state;

    bool internalSendTask(BlockStatic* block);

    void sendCommandTask(uint32_t length);
    void sendCommandDoneTask(Buffer txBuffer, Buffer rxBuffer, int event);

    void sendMessageTask(void);
    void sendMessageDoneTask(Buffer txBuffer, Buffer rxBuffer, int event);

    void printTask(const char*);

    void irqEnabledIRQ(void);
    void irqEnabledTask(void);
    void irqDisabledIRQ(void);
    void irqDisabledTask(void);

    SPI&            spi;
    DigitalOut      cs;
    InterruptIn     irq;

    FunctionPointer0<void> sendDoneCallback;

    uint8_t cmdTxBuffer[4];

    BlockStatic* sendBlock;
};

#endif // __MESSAGE_CENTER_SPI_MASTER_H__
