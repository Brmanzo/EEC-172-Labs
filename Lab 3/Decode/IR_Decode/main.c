//*****************************************************************************
//
// Application Name     - TV Remote Decoder (TV Code: Zonda 1355)
// Application Overview - The objective of this application is to demonstrate
//                          GPIO interrupts using SW2 and SW3.
//                          NOTE: the switches are not debounced!
//
//*****************************************************************************

// Standard includes
#include <stdio.h>
#include <stdint.h>
#include <string.h>

// Driverlib includes
#include "hw_types.h"
#include "hw_ints.h"
#include "hw_nvic.h"
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
#include "systick.h"
#include "rom_map.h"

// Common interface includes
#include "uart_if.h"

// Pin configurations
#include "pin_mux_config.h"


//*****************************************************************************
//                 GLOBAL VARIABLES -- Start
//*****************************************************************************

// some helpful macros for systick

// the cc3200's fixed clock frequency of 80 MHz
// note the use of ULL to indicate an unsigned long long constant
#define SYSCLKFREQ 80000000ULL

// macro to convert ticks to microseconds
#define TICKS_TO_US(ticks) \
    ((((ticks) / SYSCLKFREQ) * 1000000ULL) + \
    ((((ticks) % SYSCLKFREQ) * 1000000ULL) / SYSCLKFREQ))\

// macro to convert microseconds to ticks
#define US_TO_TICKS(us) ((SYSCLKFREQ / 1000000ULL) * (us))

// systick reload value set to 40ms period
// (PERIOD_SEC) * (SYSCLKFREQ) = PERIOD_TICKS
#define SYSTICK_RELOAD_VAL 3200000UL

// track systick counter periods elapsed
// if it is not 0, we know the transmission ended
volatile int systick_cnt = 1;

extern void (* const g_pfnVectors[])(void);
volatile unsigned char P59_intstatus;
volatile unsigned long P59_intcount;

unsigned long start_int;
unsigned long end_int;

uint64_t delta = 0;
uint64_t delta_us = 0;

uint32_t message;
uint32_t prev_message;
int repetitions = 0;
char character;


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

/**
 * Reset SysTick Counter
 */
static inline void SysTickReset(void) {
    // any write to the ST_CURRENT register clears it
    // after clearing it automatically gets reset without
    // triggering exception logic
    // see reference manual section 3.2.1
    HWREG(NVIC_ST_CURRENT) = 1;

    // clear the global count variable
    systick_cnt = 1;
}

/**
 * SysTick Interrupt Handler
 *
 * Keep track of whether the systick counter wrapped
 */
static void SysTickHandler(void) {
    // increment every time the systick handler fires
    systick_cnt++;
}

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
BoardInit(void) {
    MAP_IntVTableBaseSet((unsigned long)&g_pfnVectors[0]);
    
    // Enable Processor
    //
    MAP_IntMasterEnable();
    MAP_IntEnable(FAULT_SYSTICK);

    PRCMCC3200MCUInit();
}

/**
 * Initializes SysTick Module
 */

// Register Interrupt Handler
static void GPIOA0IntHandler(void) { // P59 handler
    unsigned long ulStatus;

    ulStatus = MAP_GPIOIntStatus(GPIOA0_BASE, true);
    MAP_GPIOIntClear(GPIOA0_BASE, ulStatus);
    delta = systick_cnt*SYSTICK_RELOAD_VAL - SysTickValueGet();
    SysTickReset();
    delta_us = TICKS_TO_US(delta);// clear interrupts on GPIOA0
    P59_intstatus = 1;
    P59_intcount++;

}

static void SysTickInit(void) {

    // configure the reset value for the systick countdown register
    MAP_SysTickPeriodSet(SYSTICK_RELOAD_VAL);

    // register interrupts on the systick module
    MAP_SysTickIntRegister(SysTickHandler);

    // enable interrupts on systick
    // (trigger SysTickHandler when countdown reaches 0)
    MAP_SysTickIntEnable();

    // enable the systick module itself
    MAP_SysTickEnable();
}
//****************************************************************************
//
//! Main function
//!
//! \param none
//! 
//!
//! \return None.
//
//****************************************************************************

//bool signal_detector = 0;

int main() {
    unsigned long ulStatus;

    BoardInit();
    
    PinMuxConfig();

    // Enable SysTick
    SysTickInit();

    // Initialize UART Terminal
    InitTerm();

    // Clear UART Terminal
    ClearTerm();

    // Register Interrupt Handler
    // (Port, pointer to handler function)
    MAP_GPIOIntRegister(GPIOA0_BASE, GPIOA0IntHandler);

    // Configure Falling Edge
    // (Port, bit-packed pin select, interrupt trigger mechanism)
    MAP_GPIOIntTypeSet(GPIOA0_BASE, 0x10, GPIO_FALLING_EDGE);

    // Interrupt Status
    // (Port, True: masked interupt status, false: raw interrupt status)
    // Returns the current interupt status enumerated as a bit field
    // of the values described in GPIOIntEnable()
    ulStatus = MAP_GPIOIntStatus(GPIOA0_BASE, false);

    // Clear Interrupt
    // (Port, with field returned from status above)
    MAP_GPIOIntClear(GPIOA0_BASE, ulStatus);

    // clear global variables
    P59_intstatus = 0;
    P59_intcount = 0;

    Message("\t\t****************************************************\n\r");
    Message("\t\t\t\tIR DECODING\n\r");
    Message("\t\t****************************************************\n\r");
    Message("\n\n\n\r");


    // Enable Interrupt
    // (Port, Flags)
    MAP_GPIOIntEnable(GPIOA0_BASE, 0x10);

    SysTickReset();
    while (1) {
        while ((P59_intstatus==0)) {;}
        // clear flag
        P59_intstatus=0;
        // If longer than standard repeat, stop remembering past input
        if(delta_us > 205000)
        {
            repetitions = 0;
            prev_message = 0;
        }
        // If larger than "1" and a 32bit word
        if((delta_us > 2500) && (delta_us < 205000) && (message != 0))
        {
            // if last remembered word is the same
            // increment repetitions
            // otherwise, message is done repeating and should print
                // Space
            if(message == 0b00000010111111010000000011111111)
            {
                character = ' ';
            }
            // 1 (Font Color Change)
            else if(message == 0b00000010111111011000000001111111)
            {
                character = '1';
            }
            // 2
            else if(message == 0b00000010111111010100000010111111)
            {
                if(prev_message == message)
                {
                    repetitions++;
                }
                else {
                    repetitions = 0;
                }
                char letters = {'A', 'B', 'C'};
                if (repetitions > (sizeof(letters) - 1))
                    repetitions = repetitions - (sizeof(letters) - 1);
                character = letters[repetitions];
                prev_message = message;
            }
            // 3
            else if(message == 0b00000010111111011100000000111111)
            {
                iif(prev_message == message)
                {
                    repetitions++;
                }
                else {
                    repetitions = 0;
                }
                char letters = {'D', 'E', 'F'};
                if (repetitions > (sizeof(letters) - 1))
                    repetitions = repetitions - (sizeof(letters) - 1);
                character = letters[repetitions];
                prev_message = message;
            }
            // 4
            else if(message == 0b00000010111111010010000011011111)
            {
                if(prev_message == message)
                {
                    repetitions++;
                }
                else {
                    repetitions = 0;
                }
                char letters = {'G', 'H', 'I'};
                if (repetitions > (sizeof(letters) - 1))
                    repetitions = repetitions - (sizeof(letters) - 1);
                character = letters[repetitions];
                prev_message = message;
            }
            // 5
            else if(message == 0b00000010111111011010000001011111)
            {
                if(prev_message == message)
                {
                    repetitions++;
                }
                else {
                    repetitions = 0;
                }
                char letters = {'J', 'K', 'L'};
                if (repetitions > (sizeof(letters) - 1))
                    repetitions = repetitions - (sizeof(letters) - 1);
                character = letters[repetitions];
                prev_message = message;
            }
            // 6
            else if(message == 0b00000010111111010110000010011111)
            {
                if(prev_message == message)
                {
                    repetitions++;
                }
                else {
                    repetitions = 0;
                }
                char letters = {'M', 'N', 'O'};
                if (repetitions > (sizeof(letters) - 1))
                    repetitions = repetitions - (sizeof(letters) - 1);
                character = letters[repetitions];
                prev_message = message;
            }
            // 7
            else if(message == 0b00000010111111011110000000011111)
            {
                if(prev_message == message)
                {
                    repetitions++;
                }
                else {
                    repetitions = 0;
                }
                char letters = {'P', 'Q', 'R', 'S'};
                if (repetitions > (sizeof(letters) - 1))
                    repetitions = repetitions - (sizeof(letters) - 1);
                character = letters[repetitions];
                prev_message = message;
            }
            // 8
            else if(message == 0b00000010111111010001000011101111)
            {
                if(prev_message == message)
                {
                    repetitions++;
                }
                else {
                    repetitions = 0;
                }
                char letters = {'T', 'U', 'V'};
                if (repetitions > (sizeof(letters) - 1))
                    repetitions = repetitions - (sizeof(letters) - 1);
                character = letters[repetitions];
                prev_message = message;
            }
            // 9
            else if(message == 0b00000010111111011001000001101111)
            {
                if(prev_message == message)
                {
                    repetitions++;
                }
                else {
                    repetitions = 0;
                }
                char letters = {'W', 'X', 'Y', 'Z'};
                if (repetitions > (sizeof(letters) - 1))
                    repetitions = repetitions - (sizeof(letters) - 1);
                character = letters[repetitions];
                prev_message = message;
            }
            // Enter (MUTE)
            else if(message == 0b00000010111111010000100011110111)
            {
                character = '\n';
            }
            // Delete (LAST)
            else if(message == 0b00000010111111010000001011111101)
            {
                character = 127;
            }
            else
            {
                    character = '!';
            }
            Report("to transmit: %c repetitions: %d \r\n",character, repetitions);
            // Resets repetitions
        }
            message = 0;
        }
        else if(delta_us < 2500 && delta_us > 1300)
        {
            message = message << 1;
            message = message + 1;
        }
        else //if(delta_us > 0 && delta_us < 1300)
        {
            message = message << 1;
        }

        start_int = 0;

    }
}

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************
