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

#include "message-center-transport/MessageCenterTransport.h"

#include "mbed-drivers/mbed.h"
#include "core-util/SharedPointer.h"
#include "mbed-block/BlockStatic.h"


#if 0
#if defined(TARGET_LIKE_STK3700)
#include "swo/swo.h"
#undef printf
#define printf swoprintf
#endif
#endif

using namespace mbed::util;

class MessageCenterSPIMaster : public MessageCenterTransport
{
public:
    MessageCenterSPIMaster(SPI& _spi, PinName _cs, PinName _irq);


private:

    typedef enum {
        STATE_IDLE,
        STATE_IDLE_WAIT,
        STATE_SEND_COMMAND,
        STATE_SEND_WAIT,
        STATE_SEND_MESSAGE,
        STATE_SEND_DONE,
        STATE_RECEIVE_WAIT_COMMAND,
        STATE_RECEIVE_COMMAND,
        STATE_RECEIVE_WAIT_MESSAGE,
        STATE_RECEIVE_MESSAGE
    } state_t;

    state_t state;

    virtual bool internalSendTask(uint16_t port, BlockStatic* block);

    void sendCommandTask(uint16_t port, uint32_t length);
    void sendCommandDoneTask(Buffer txBuffer, Buffer rxBuffer, int event);

    void sendMessageTask(void);
    void sendMessageDoneTask(Buffer txBuffer, Buffer rxBuffer, int event);

    void receiveCommandTask(void);
    void receiveCommandDoneTask(Buffer txBuffer, Buffer rxBuffer, int event);

    void receiveMessageTask(void);
    void receiveMessageDoneTask(Buffer txBuffer, Buffer rxBuffer, int event);

    void printTask(const char*);

    void irqEnabledIRQ(void);
    void irqEnabledTask(void);
    void irqDisabledIRQ(void);
    void irqDisabledTask(void);

    void timeoutTask(void);

    SPI&            spi;
    DigitalOut      cs;
    InterruptIn     irq;

    uint8_t cmdBuffer[6];
    uint16_t callbackPort;

    SharedPointer<Block> receiveBlock;
    BlockStatic* sendBlock;

    minar::callback_handle_t timeoutHandle;
};

#endif // __MESSAGE_CENTER_SPI_MASTER_H__
