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




#include "message-center-transport/MessageCenterSPIMaster.h"


MessageCenterSPIMaster::MessageCenterSPIMaster(SPI& _spi, PinName _cs, PinName _irq)
    :   state(STATE_IDLE),
        spi(_spi),
        cs(_cs),
        irq(_irq)
{
    spi.format(8, 0, SPI_MSB);
    spi.frequency(1000000);
    spi.set_dma_usage(DMA_USAGE_OPPORTUNISTIC);

    cs = 1;

    irq.mode(PullUp);
    irq.fall(this, &MessageCenterSPIMaster::irqEnabledIRQ);
    irq.rise(this, &MessageCenterSPIMaster::irqDisabledIRQ);
}

/*****************************************************************************/
/* IRQ Task handlers                                                         */
/*****************************************************************************/
void MessageCenterSPIMaster::irqEnabledIRQ()
{
    minar::Scheduler::postCallback(this, &MessageCenterSPIMaster::irqEnabledTask);

    FunctionPointer1<void, const char*> fp(this, &MessageCenterSPIMaster::printTask);
    minar::Scheduler::postCallback(fp.bind("fall\r\n"));
}

void MessageCenterSPIMaster::irqEnabledTask()
{
    if (state == STATE_SEND_WAIT)
    {
        state = STATE_SEND_MESSAGE;

        sendMessageTask();
    }
}

void MessageCenterSPIMaster::irqDisabledIRQ()
{
    minar::Scheduler::postCallback(this, &MessageCenterSPIMaster::irqDisabledTask);

    FunctionPointer1<void, const char*> fp(this, &MessageCenterSPIMaster::printTask);
    minar::Scheduler::postCallback(fp.bind("rise\r\n"));
}

void MessageCenterSPIMaster::irqDisabledTask()
{
    if (state == STATE_SEND_DONE)
    {
        state = STATE_IDLE;

        if (sendDoneCallback)
        {
            sendDoneCallback();
        }
    }
}

/*****************************************************************************/
/* Send                                                                      */
/*****************************************************************************/

bool MessageCenterSPIMaster::internalSendTask(BlockStatic* block)
{
    if (state == STATE_IDLE)
    {
        // chip select ASAP to prevent slave from sending
        cs = 0;

        printf("send\r\n");

        uint32_t length = block->getLength();

        state = STATE_SEND_COMMAND;
        sendBlock = block;

        FunctionPointer1<void, uint32_t> fp(this, &MessageCenterSPIMaster::sendCommandTask);
        minar::Scheduler::postCallback(fp.bind(length));

        return true;
    }
    else
    {
        return false;
    }
}


void MessageCenterSPIMaster::sendCommandTask(uint32_t length)
{
    cmdTxBuffer[0] = length;
    cmdTxBuffer[1] = length >> 8;
    cmdTxBuffer[2] = length >> 16;
    cmdTxBuffer[3] = length >> 24;

    SPI::event_callback_t onFinish(this, &MessageCenterSPIMaster::sendCommandDoneTask);

    // send buffer over SPI
    spi.transfer()
        .tx(cmdTxBuffer, 4)
        .callback(onFinish, SPI_EVENT_ALL)
        .apply();
}

void MessageCenterSPIMaster::sendCommandDoneTask(Buffer txBuffer, Buffer rxBuffer, int event)
{
    (void) txBuffer;
    (void) rxBuffer;
    (void) event;

    state = STATE_SEND_WAIT;

    // chip deselect
    cs = 1;
}

void MessageCenterSPIMaster::sendMessageTask()
{
    // chip select
    cs = 0;

    SPI::event_callback_t onFinish(this, &MessageCenterSPIMaster::sendMessageDoneTask);

    // send buffer over SPI
    spi.transfer()
        .tx(sendBlock->getData(), sendBlock->getLength())
        .callback(onFinish, SPI_EVENT_ALL)
        .apply();
}

void MessageCenterSPIMaster::sendMessageDoneTask(Buffer txBuffer, Buffer rxBuffer, int event)
{
    (void) txBuffer;
    (void) rxBuffer;
    (void) event;

    // chip deselect
    cs = 1;

    state = STATE_SEND_DONE;
}

void MessageCenterSPIMaster::printTask(const char* str)
{
    printf("%s", str);
}

