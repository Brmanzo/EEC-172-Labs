//*****************************************************************************
//
// Copyright (C) 2014 Texas Instruments Incorporated - http://www.ti.com/
//
//
//  Redistribution and use in source and binary forms, with or without
//  modification, are permitted provided that the following conditions
//  are met:
//
//    Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//
//    Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the
//    distribution.
//
//    Neither the name of Texas Instruments Incorporated nor the names of
//    its contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
//  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
//  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
//  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
//  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
//  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
//  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
//*****************************************************************************

//*****************************************************************************
//
// Application Name     - SPI Demo
// Application Overview - The demo application focuses on showing the required
//                        initialization sequence to enable the CC3200 SPI
//                        module in full duplex 4-wire master and slave mode(s).
//
//*****************************************************************************


//*****************************************************************************
//
//! \addtogroup SPI_Demo
//! @{
//
//*****************************************************************************

// Standard includes
#include <string.h>

// Driverlib includes
#include "hw_types.h"
#include "hw_memmap.h"
#include "hw_common_reg.h"
#include "hw_ints.h"
#include "spi.h"
#include "rom.h"
#include "rom_map.h"
#include "utils.h"
#include "prcm.h"
#include "uart.h"
#include "interrupt.h"

// Common interface includes
#include "uart_if.h"
#include "pinmux.h"

#include "Adafruit_GFX.h"
#include "Adafruit_SSD1351.h"
#include "glcdfont.h"


#define APPLICATION_VERSION     "1.4.0"
//*****************************************************************************
//
// Application Master/Slave mode selector macro
//
// MASTER_MODE = 1 : Application in master mode
// MASTER_MODE = 0 : Application in slave mode
//
//*****************************************************************************
#define MASTER_MODE      1

#define SPI_IF_BIT_RATE  100000
#define TR_BUFF_SIZE     100

#define MASTER_MSG       "This is CC3200 SPI Master Application\n\r"
#define SLAVE_MSG        "This is CC3200 SPI Slave Application\n\r"

#define BLACK           0x0000
#define BLUE            0x001F
#define GREEN           0x07E0
#define CYAN            0x07FF
#define RED             0xF800
#define MAGENTA         0xF81F
#define YELLOW          0xFFE0
#define WHITE           0xFFFF

//*****************************************************************************
//                 GLOBAL VARIABLES -- Start
//*****************************************************************************

#if defined(ccs)
extern void (* const g_pfnVectors[])(void);
#endif
#if defined(ewarm)
extern uVectorEntry __vector_table;
#endif
//*****************************************************************************
//                 GLOBAL VARIABLES -- End
//*****************************************************************************

//*****************************************************************************
//
//! Board Initialization & Configuration
//!
//! \param  None
//!
//! \return None
//
//*****************************************************************************
static void
BoardInit(void)
{
/* In case of TI-RTOS vector table is initialize by OS itself */
#ifndef USE_TIRTOS
  //
  // Set vector table base
  //
#if defined(ccs)
    MAP_IntVTableBaseSet((unsigned long)&g_pfnVectors[0]);
#endif
#if defined(ewarm)
    MAP_IntVTableBaseSet((unsigned long)&__vector_table);
#endif
#endif
    //
    // Enable Processor
    //
    MAP_IntMasterEnable();
    MAP_IntEnable(FAULT_SYSTICK);

    PRCMCC3200MCUInit();
}

//*****************************************************************************
//
//! Main function for spi demo application
//!
//! \param none
//!
//! \return None.
//
//*****************************************************************************
void main()
{
    //
    // Initialize Board configurations
    //
    BoardInit();

    //
    // Muxing UART and SPI lines.
    //
    PinMuxConfig();

    //
    // Enable the SPI module clock
    //
    MAP_PRCMPeripheralClkEnable(PRCM_GSPI,PRCM_RUN_MODE_CLK);

    //
    // Reset the peripheral
    //
    MAP_PRCMPeripheralReset(PRCM_GSPI);

    //
    // Reset SPI
    //
    MAP_SPIReset(GSPI_BASE);

    //
    // Configure SPI interface
    //
    MAP_SPIConfigSetExpClk(GSPI_BASE,MAP_PRCMPeripheralClockGet(PRCM_GSPI),
                     SPI_IF_BIT_RATE,SPI_MODE_MASTER,SPI_SUB_MODE_0,
                     (SPI_SW_CTRL_CS |
                     SPI_4PIN_MODE |
                     SPI_TURBO_OFF |
                     SPI_CS_ACTIVELOW |
                     SPI_WL_8));

    //
    // Enable SPI for communication
    //
    MAP_SPIEnable(GSPI_BASE);

    Adafruit_Init();
    //
    // Begin Test Functions
    //
    while(1)
    {
        int i;
        int j;
        // Position in pixels
        int x = 0;
        int y = 0;
        setCursor(x, y);
        setTextSize(1);
        setTextColor(0xFFFF, 0x0000);

        // Print Fontset
        for(i = 0; i < sizeof(font)/5; i++)
        {
            drawChar(x, y, i, 0xFFFF, 0x0000, 1);
            if(x < 120)
                x += 6;
            else
            {
                x = 0;
                if(y < 128)
                    y = y += 8;
                else
                    y = 0;
            }
        }
        // Print Hello world
        fillScreen(0x0000);
        char* Hello = "Hello world ";
        Outstr(Hello);
        MAP_UtilsDelay(10000000);
        fillScreen(0x0000);

        unsigned int colors[8] = {BLACK, BLUE, GREEN, CYAN, RED, MAGENTA, YELLOW, WHITE};
        // Print eight horizontal lines
        y = 0;
        x = 0;

        for(i = 0; i < 8; i++)
        {
            for(j = 0; j < 16; j++)
            {
                drawLine(0, y, 128, y, colors[i]);
                y ++;
            }
        }

        MAP_UtilsDelay(10000000);
        fillScreen(0x0000);

        // Print eight vertical bands
        y = 0;
        x = 0;

        for(i = 0; i < 8; i++)
        {
            for (j = 0; j < 16; j++)
            {
                drawLine(x, 0, x, 128, colors[i]);
                x ++;
            }
        }
        fillScreen(0x0000);

        // Call testlines function
        MAP_UtilsDelay(10000000);
        testlines(0xF800);
        fillScreen(0x0000);

        // Call testfastlines function
        MAP_UtilsDelay(10000000);
        testfastlines(0xF800, 0x07FF);
        fillScreen(0x0000);

        // Call testdrawrects function
        testdrawrects(0x07E0);
        MAP_UtilsDelay(10000000);
        fillScreen(0x0000);

        // Call testfillrects function
        testfillrects(0x07E0, 0x07FF);
        MAP_UtilsDelay(10000000);
        fillScreen(0x0000);

        // Call testfillcircles function
        testfillcircles(4, 0xFFE0);
        MAP_UtilsDelay(10000000);
        fillScreen(0x0000);

        // Call testroundrects function
        testroundrects();
        MAP_UtilsDelay(10000000);
        fillScreen(0x0000);

        // Call testtriangles function
        testtriangles();
        MAP_UtilsDelay(10000000);
        fillScreen(0x0000);
    }
}
