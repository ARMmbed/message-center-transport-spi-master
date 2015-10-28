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

#include "core-util/CriticalSectionLock.h"
#include "mbed-block/BlockDynamic.h"


MessageCenterSPIMaster::MessageCenterSPIMaster(SPI& _spi, PinName _cs, PinName _irq)
    :   MessageCenterTransport(),
        state(STATE_IDLE),
        spi(_spi),
        cs(_cs),
        irq(_irq),
        callbackPort(0),
        timeoutHandle(NULL)
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
//    FunctionPointer1<void, const char*> fp(this, &MessageCenterSPIMaster::printTask);
//    minar::Scheduler::postCallback(fp.bind("fall\r\n"));

    minar::Scheduler::postCallback(this, &MessageCenterSPIMaster::irqEnabledTask);
}

void MessageCenterSPIMaster::irqEnabledTask()
{
    // Slave responded to send command, send message
    if (state == STATE_SEND_WAIT)
    {
        state = STATE_SEND_MESSAGE;

        sendMessageTask();
    }
    // Slave acquired SPI, expect command shortly
    else if (state == STATE_IDLE)
    {
        // change state, this prevents the Master from sending at the same time
        state = STATE_RECEIVE_WAIT_COMMAND;

        // Consistency check
        // if the Slave reboots for some reason, a small pulse might be generated
        // on the IRQ line. Post a callback to check whether this is the case and
        // reset the state to idle if a transfer hasn't commenced.
        timeoutHandle = minar::Scheduler::postCallback(this, &MessageCenterSPIMaster::timeoutTask)
                            .delay(minar::milliseconds(1000))
                            .getHandle();

    }
    // Slave armed message, start reading it
    else if (state == STATE_RECEIVE_WAIT_MESSAGE)
    {
        state = STATE_RECEIVE_MESSAGE;

        receiveMessageTask();
    }
}

void MessageCenterSPIMaster::irqDisabledIRQ()
{
//    FunctionPointer1<void, const char*> fp(this, &MessageCenterSPIMaster::printTask);
//    minar::Scheduler::postCallback(fp.bind("rise\r\n"));

    minar::Scheduler::postCallback(this, &MessageCenterSPIMaster::irqDisabledTask);
}

void MessageCenterSPIMaster::irqDisabledTask()
{
    // Slave signals it is ready for next round of communication
    if (state == STATE_SEND_DONE)
    {
        state = STATE_IDLE;

        if (callbackSend)
        {
            minar::Scheduler::postCallback(callbackSend);
        }
    }
    // Slave signals a command is ready to be read
    else if (state == STATE_RECEIVE_WAIT_COMMAND)
    {
        state = STATE_RECEIVE_COMMAND;

        // cancel timeout task since this was a genuine signal from the Slave
        minar::Scheduler::cancelCallback(timeoutHandle);
        timeoutHandle = NULL;

        receiveCommandTask();
    }
    // Slave signals it is ready for next round
    else if (state == STATE_IDLE_WAIT)
    {
        state = STATE_IDLE;
    }
    // unknown state, reset to known state
    else
    {
        state = STATE_IDLE;
    }
}

/*****************************************************************************/
/* Send                                                                      */
/*****************************************************************************/

bool MessageCenterSPIMaster::internalSendTask(uint16_t port, BlockStatic& block)
{
    bool result = false;

    // begin critical section
    {
        CriticalSectionLock lock;

        if ((state == STATE_IDLE) &&
            (irq == 1))
        {
            // chip select ASAP to prevent slave from sending
            cs = 0;

            // If chip select was successful, IRQ is still high, proceed.
            // If not, don't change state, task for reading slave will be posted
            // when this critical section ends.
            // Let CS remain low in order not to interfere with the transfer.
            if (irq == 1)
            {
                state = STATE_SEND_COMMAND;

                result = true;
            }
        }
    }
    // end critical section

    if (result)
    {
        sendBlock = block;

        uint32_t length = sendBlock.getLength();

        FunctionPointer2<void, uint16_t, uint32_t> fp(this, &MessageCenterSPIMaster::sendCommandTask);
        minar::Scheduler::postCallback(fp.bind(port, length));
    }

    return result;
}


void MessageCenterSPIMaster::sendCommandTask(uint16_t port, uint32_t length)
{
    cmdBuffer[0] = length;
    cmdBuffer[1] = length >> 8;
    cmdBuffer[2] = length >> 16;
    cmdBuffer[3] = length >> 24;

    cmdBuffer[4] = port;
    cmdBuffer[5] = port >> 8;

    SPI::event_callback_t onFinish(this, &MessageCenterSPIMaster::sendCommandDoneTask);

    // send buffer over SPI
    spi.transfer()
        .tx(cmdBuffer, 6)
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
        .tx(sendBlock.getData(), sendBlock.getLength())
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

/*****************************************************************************/
/* Receive                                                                   */
/*****************************************************************************/

void MessageCenterSPIMaster::receiveCommandTask()
{
    // chip select
    cs = 0;

    SPI::event_callback_t onFinish(this, &MessageCenterSPIMaster::receiveCommandDoneTask);

    // send buffer over SPI
    spi.transfer()
        .rx(cmdBuffer, 6)
        .callback(onFinish, SPI_EVENT_ALL)
        .apply();
}

void MessageCenterSPIMaster::receiveCommandDoneTask(Buffer txBuffer, Buffer rxBuffer, int event)
{
    (void) txBuffer;
    (void) rxBuffer;
    (void) event;

    // chip deselect
    cs = 1;

    // received command, wait for Slave to signal message is ready to be read
    state = STATE_RECEIVE_WAIT_MESSAGE;
}

void MessageCenterSPIMaster::receiveMessageTask()
{
    // parse length
    uint32_t length = cmdBuffer[3];
    length = (length << 8) | cmdBuffer[2];
    length = (length << 8) | cmdBuffer[1];
    length = (length << 8) | cmdBuffer[0];

    callbackPort = cmdBuffer[5];
    callbackPort = (callbackPort << 8) | cmdBuffer[4];

    // allocate buffer
    uint8_t* buffer = (uint8_t*) malloc(length);

    // store shared pointer
    receiveBlock = SharedPointer<BlockStatic>(new BlockDynamic(buffer, length));

    // chip select
    cs = 0;

    SPI::event_callback_t onFinish(this, &MessageCenterSPIMaster::receiveMessageDoneTask);

    // send buffer over SPI
    spi.transfer()
        .rx(buffer, length)
        .callback(onFinish, SPI_EVENT_ALL)
        .apply();
}

void MessageCenterSPIMaster::receiveMessageDoneTask(Buffer txBuffer, Buffer rxBuffer, int event)
{
    (void) txBuffer;
    (void) rxBuffer;
    (void) event;

    // chip deselect
    cs = 1;

    // received command, wait for Slave to signal it is ready again
    state = STATE_IDLE_WAIT;

    // post callback to receive handler
    if (callbackReceive)
    {
        minar::Scheduler::postCallback(callbackReceive.bind(callbackPort, receiveBlock));
    }

    // clear reference to shared block
    receiveBlock = SharedPointer<BlockStatic>();
}

/*****************************************************************************/
/* Debug                                                                     */
/*****************************************************************************/

void MessageCenterSPIMaster::timeoutTask()
{
    state = STATE_IDLE;
}

void MessageCenterSPIMaster::printTask(const char* str)
{
    printf("%s", str);
}

