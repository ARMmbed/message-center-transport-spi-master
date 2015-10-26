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
#else
const PinName SPI_MOSI = YOTTA_CFG_HARDWARE_TEST_PINS_SPI_MOSI;
const PinName SPI_MISO = YOTTA_CFG_HARDWARE_TEST_PINS_SPI_MISO;
const PinName SPI_SCLK = YOTTA_CFG_HARDWARE_TEST_PINS_SPI_SCLK;
const PinName SPI_SSEL = YOTTA_CFG_HARDWARE_TEST_PINS_SPI_SSEL;
#endif

static SPI spi(SPI_MOSI, SPI_MISO, SPI_SCLK);
static MessageCenterSPIMaster transport(spi, SPI_SSEL, PD4);

// enable buttons to initiate transfer
static InterruptIn button1(BTN0);
static InterruptIn button2(BTN1);

static uint8_t buffer[100];
static BlockStatic block(buffer, sizeof(buffer));

void receivedBlock(SharedPointer<Block> block)
{
    printf("main:received: ");
    for (std::size_t idx = 0; idx < block->getLength(); idx++)
    {
        printf("%02X", block->at(idx));
    }
    printf("\r\n");
}


/*****************************************************************************/
/* Buttons                                                                   */
/*****************************************************************************/

uint32_t counter = 100;

void sendDone()
{
    printf("send done\r\n");

    if (counter--)
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

    counter = 100;

    transport.sendTask(&block, sendDone);
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

//    transport.onReceive(receivedBlock);

    printf("SPI Master: %s %s\r\n", __DATE__, __TIME__);
}