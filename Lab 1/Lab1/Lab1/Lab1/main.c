// Standard includes
#include <stdio.h>

// Driverlib includes
#include "hw_types.h"
#include "hw_ints.h"
#include "hw_memmap.h"
#include "hw_common_reg.h"
#include "interrupt.h"
#include "hw_apps_rcm.h"
#include "prcm.h"
#include "rom.h"
#include "rom_map.h"
#include "prcm.h"
#include "gpio.h"
#include "utils.h"
#include "uart.h"

// Common interface includes
#include "gpio_if.h"
#include "uart_if.h"

#include "pin_mux_config.h"

#define APPLICATION_VERSION     "1.4.0"

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
//                      LOCAL FUNCTION PROTOTYPES                           
//*****************************************************************************
static void BoardInit(void);

//*****************************************************************************
//                      LOCAL FUNCTION DEFINITIONS                         
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
//****************************************************************************
//
//! Main function
//!
//! \param none
//! 
//! This function  
//!    1. Invokes the LEDBlinkyTask
//!
//! \return None.
//
//****************************************************************************
void main()
{
    //
    // Initialize Board configurations
    //
    BoardInit();
    
    //
    // Power on the corresponding GPIO port B for 9,10,11.
    // Set up the GPIO lines to mode 0 (GPIO)
    //
    PinMuxConfig();

    InitTerm();
    //
    // Clearing the Terminal.
    //
    ClearTerm();
    GPIO_IF_LedConfigure(LED1|LED2|LED3);

    Message("\t\t****************************************************\n\r");
    Message("\t\t\t        CC3200 GPIO Application        \n\r");
    Message("\t\t****************************************************\n\r\n\n");
    Message("\t\t****************************************************\n\r");
    Message("\t\t Push SW3 to start LED binary counting \n\r");
    Message("\t\t Push SW2 to blink LEDs on and off \n\r") ;
    Message("\t\t ****************************************************\n\r");
    Message("\n\n\n\r");
    Message("cmd#\n\r");

    GPIO_IF_LedOff(MCU_ALL_LED_IND);
    int red_on = 0;
    int orange_on = 0;
    int green_on = 0;
    
    //
    // Binary 0-7 count routine
    //
    while(1)
    {
        // Checks Switch 3
        if(GPIOPinRead(GPIOA1_BASE, 0x20) == 0x20)
        {
            Message("SW3 is Pressed\n\r");
            // Drives pin 18 low
            GPIOPinWrite(GPIOA3_BASE, 0x10, 0x0);
            // Resets current state of LEDs
            GPIO_IF_LedOff(MCU_RED_LED_GPIO);
            GPIO_IF_LedOff(MCU_ORANGE_LED_GPIO);
            GPIO_IF_LedOff(MCU_GREEN_LED_GPIO);
            // Pauses program until Switch 3 is released
            while(GPIOPinRead(GPIOA1_BASE, 0x20) == 0x20);
            // If red is off
            while(GPIOPinRead(GPIOA2_BASE, 0x40) != 0x40)
            {
                MAP_UtilsDelay(8000000);
                if(!red_on)
                {
                    // Then turn red on
                    GPIO_IF_LedOn(MCU_RED_LED_GPIO);
                    red_on = 1;
                }
                // Otherwise, red is on, so turn it off and check orange
                else
                {
                    // Turn red off
                    GPIO_IF_LedOff(MCU_RED_LED_GPIO);
                    red_on = 0;

                    //If orange is off, turn it on
                    if(!orange_on)
                    {
                        GPIO_IF_LedOn(MCU_ORANGE_LED_GPIO);
                        orange_on = 1;
                    }
                    // Otherwise, Orange is on, so turn it off and check green
                    else
                    {
                        // Turn orange off
                        GPIO_IF_LedOff(MCU_ORANGE_LED_GPIO);
                        orange_on = 0;

                        // If green is off, turn it on
                        if(!green_on)
                        {
                            GPIO_IF_LedOn(MCU_GREEN_LED_GPIO);
                            green_on = 1;
                        }
                        else
                        {
                            // If green is on system is at 111, revert back to 000.
                            GPIO_IF_LedOff(MCU_RED_LED_GPIO);
                            GPIO_IF_LedOff(MCU_ORANGE_LED_GPIO);
                            GPIO_IF_LedOff(MCU_GREEN_LED_GPIO);
                            red_on = 0;
                            orange_on = 0;
                            green_on = 0;
                        }
                    }
                }
            }
        }
        // Checks Switch 2
        if(GPIOPinRead(GPIOA2_BASE, 0x40) == 0x40)
        {
            Message("SW2 is Pressed\n\r");
            // Drives pin 18 High
            GPIOPinWrite(GPIOA3_BASE, 0x10, 0x10);
            // Pauses program until Switch 2 is released
            while(GPIOPinRead(GPIOA2_BASE, 0x40) == 0x40);
            while(GPIOPinRead(GPIOA1_BASE, 0x20) != 0x20)
            {
                GPIO_IF_LedOn(MCU_RED_LED_GPIO);
                GPIO_IF_LedOn(MCU_ORANGE_LED_GPIO);
                GPIO_IF_LedOn(MCU_GREEN_LED_GPIO);
                MAP_UtilsDelay(8000000);
                GPIO_IF_LedOff(MCU_RED_LED_GPIO);
                GPIO_IF_LedOff(MCU_ORANGE_LED_GPIO);
                GPIO_IF_LedOff(MCU_GREEN_LED_GPIO);
                MAP_UtilsDelay(8000000);
            }
        }
    }
}

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************
