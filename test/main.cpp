/**
 * @file    main.cpp
 * @brief   mbed Connected Home Endpoint main entry point
 * @author  Doug Anson
 * @version 1.0
 * @see
 *
 * Copyright (c) 2014
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

#include "mbed-drivers/mbed.h"

#include "message-center-transport/MessageCenterSPIMaster.h"

#include "core-util/SharedPointer.h"

#if defined(TARGET_LIKE_EFM32GG_STK)
const PinName SPI_MOSI = PD0;
const PinName SPI_MISO = PD1;
const PinName SPI_SCLK = PD2;
const PinName SPI_SSEL = PD3;
const PinName SPI_IRQ  = PD4;
const PinName BUTTON1 = BTN0;
const PinName BUTTON2 = BTN1;
#elif defined(TARGET_LIKE_K64F)
const PinName SPI_MOSI = YOTTA_CFG_HARDWARE_TEST_PINS_SPI_MOSI;
const PinName SPI_MISO = YOTTA_CFG_HARDWARE_TEST_PINS_SPI_MISO;
const PinName SPI_SCLK = YOTTA_CFG_HARDWARE_TEST_PINS_SPI_SCLK;
const PinName SPI_SSEL = YOTTA_CFG_HARDWARE_TEST_PINS_SPI_SSEL;
const PinName SPI_IRQ  = PTC4;
const PinName BUTTON1 = SW2;
const PinName BUTTON2 = SW3;
#else
const PinName SPI_MOSI = YOTTA_CFG_HARDWARE_TEST_PINS_SPI_MOSI;
const PinName SPI_MISO = YOTTA_CFG_HARDWARE_TEST_PINS_SPI_MISO;
const PinName SPI_SCLK = YOTTA_CFG_HARDWARE_TEST_PINS_SPI_SCLK;
const PinName SPI_SSEL = YOTTA_CFG_HARDWARE_TEST_PINS_SPI_SSEL;
const PinName SPI_IRQ  = YOTTA_CFG_HARDWARE_TEST_PINS_SPI_IRQ;
#endif

static SPI spi(SPI_MOSI, SPI_MISO, SPI_SCLK);
static MessageCenterSPIMaster transport(spi, SPI_SSEL, SPI_IRQ);

// enable buttons to initiate transfer
static InterruptIn button1(BUTTON1);
static InterruptIn button2(BUTTON2);

static uint8_t buffer[100];
static BlockStatic block(buffer, sizeof(buffer));

void receivedBlock(uint16_t port, SharedPointer<Block> block)
{
    printf("main:received: %p\r\n", &(block->at(0)));
    printf("%u : ", port);
    for (std::size_t idx = 0; idx < block->getLength(); idx++)
    {
        printf("%02X", block->at(idx));
    }
    printf("\r\n");
}


/*****************************************************************************/
/* Buttons                                                                   */
/*****************************************************************************/

int32_t counter = 0;

void sendDone()
{
    printf("send done\r\n");

    if (--counter > 0)
    {
        transport.sendTask(&block, sendDone);
    }
}

void button1Task()
{
    printf("button 1\r\n");

    for (std::size_t idx = 0; idx < block.getMaxLength(); idx++)
    {
        block.at(idx) = idx;
    }

    counter = 1;

    transport.sendTask(1234, &block, sendDone);
}

void button1ISR()
{
    minar::Scheduler::postCallback(button1Task);
}

void button2ISR()
{
    printf("button 2\r\n");
}

/*****************************************************************************/
/* App start                                                                 */
/*****************************************************************************/


void app_start(int, char *[])
{
    // setup buttons
    button1.fall(button1ISR);
    button2.fall(button2ISR);

    transport.onReceiveTask(receivedBlock);

    printf("SPI Master: %s %s\r\n", __DATE__, __TIME__);
}
