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
// Application Name     - UDP Socket
// Application Overview - This particular application illustrates how this
//                        device can be used as a client or server for UDP
//                        communication.
//
//*****************************************************************************

//****************************************************************************
//
//! \addtogroup udp_socket
//! @{
//
//****************************************************************************

#include <stdlib.h>
#include <string.h>

// simplelink includes
#include "simplelink.h"
#include "wlan.h"

// driverlib includes
#include "hw_ints.h"
#include "hw_types.h"
#include "hw_memmap.h"
#include "rom.h"
#include "rom_map.h"
#include "interrupt.h"
#include "prcm.h"
#include "utils.h"
#include "uart.h"
#include "gpio.h"

// common interface includes
#include "udma_if.h"
#include "common.h"
#ifndef NOTERM
#include "uart_if.h"
#endif
#include "utils/network_utils.h"
#include "pinmux.h"

#define APPLICATION_NAME        "UDP Socket"
#define APPLICATION_VERSION     "1.4.0"

#define IP_ADDR            0xc0a80064 /* 192.168.0.100 */
#define PORT_NUM           5001
#define BUF_SIZE           1400
#define UDP_PACKET_COUNT   1000


//*****************************************************************************
//                 GLOBAL VARIABLES -- Start
//*****************************************************************************
#if defined(ccs) || defined(gcc)
extern void (*const g_pfnVectors[])(void);
#endif
#if defined(ewarm)
extern uVectorEntry __vector_table;
#endif
//*****************************************************************************
//                 GLOBAL VARIABLES -- End
//*****************************************************************************

//*****************************************************************************
//                            Motor -- Start
//*****************************************************************************
// IN1 - Pin 15 - GPIO22
#define MOTOR_LEFT_IN1_BASE GPIOA2_BASE
#define MOTOR_LEFT_IN1_PIN  0x40

// IN2 - Pin 64 - GPIO9
#define MOTOR_LEFT_IN2_BASE GPIOA1_BASE
#define MOTOR_LEFT_IN2_PIN  0x02

// IN3 - Pin 6 - GPIO7
#define MOTOR_RIGHT_IN3_BASE GPIOA1_BASE
#define MOTOR_RIGHT_IN3_PIN  0x80

// IN4 - Pin 45 - GPIO31
#define MOTOR_RIGHT_IN4_BASE GPIOA3_BASE
#define MOTOR_RIGHT_IN4_PIN  0x80

void motorStop()
{
    MAP_GPIOPinWrite(MOTOR_LEFT_IN1_BASE, MOTOR_LEFT_IN1_PIN, 0);
    MAP_GPIOPinWrite(MOTOR_LEFT_IN2_BASE, MOTOR_LEFT_IN2_PIN, 0);
    MAP_GPIOPinWrite(MOTOR_RIGHT_IN3_BASE, MOTOR_RIGHT_IN3_PIN, 0);
    MAP_GPIOPinWrite(MOTOR_RIGHT_IN4_BASE, MOTOR_RIGHT_IN4_PIN, 0);
}

void motorForward()
{
    MAP_GPIOPinWrite(MOTOR_LEFT_IN1_BASE, MOTOR_LEFT_IN1_PIN,
    MOTOR_LEFT_IN1_PIN);
    MAP_GPIOPinWrite(MOTOR_LEFT_IN2_BASE, MOTOR_LEFT_IN2_PIN, 0);
    MAP_GPIOPinWrite(MOTOR_RIGHT_IN3_BASE, MOTOR_RIGHT_IN3_PIN,
    MOTOR_RIGHT_IN3_PIN);
    MAP_GPIOPinWrite(MOTOR_RIGHT_IN4_BASE, MOTOR_RIGHT_IN4_PIN, 0);
}

void motorBackward()
{
    MAP_GPIOPinWrite(MOTOR_LEFT_IN1_BASE, MOTOR_LEFT_IN1_PIN, 0);
    MAP_GPIOPinWrite(MOTOR_LEFT_IN2_BASE, MOTOR_LEFT_IN2_PIN,
    MOTOR_LEFT_IN2_PIN);
    MAP_GPIOPinWrite(MOTOR_RIGHT_IN3_BASE, MOTOR_RIGHT_IN3_PIN, 0);
    MAP_GPIOPinWrite(MOTOR_RIGHT_IN4_BASE, MOTOR_RIGHT_IN4_PIN,
    MOTOR_RIGHT_IN4_PIN);
}

void motorLeft()
{
    MAP_GPIOPinWrite(MOTOR_LEFT_IN1_BASE, MOTOR_LEFT_IN1_PIN, 0);
    MAP_GPIOPinWrite(MOTOR_LEFT_IN2_BASE, MOTOR_LEFT_IN2_PIN,
    MOTOR_LEFT_IN2_PIN);
    MAP_GPIOPinWrite(MOTOR_RIGHT_IN3_BASE, MOTOR_RIGHT_IN3_PIN,
    MOTOR_RIGHT_IN4_PIN);
    MAP_GPIOPinWrite(MOTOR_RIGHT_IN4_BASE, MOTOR_RIGHT_IN4_PIN, 0);
}

void motorRight()
{
    MAP_GPIOPinWrite(MOTOR_LEFT_IN1_BASE, MOTOR_LEFT_IN1_PIN,
    MOTOR_LEFT_IN1_PIN);
    MAP_GPIOPinWrite(MOTOR_LEFT_IN2_BASE, MOTOR_LEFT_IN2_PIN, 0);
    MAP_GPIOPinWrite(MOTOR_RIGHT_IN3_BASE, MOTOR_RIGHT_IN3_PIN, 0);
    MAP_GPIOPinWrite(MOTOR_RIGHT_IN4_BASE, MOTOR_RIGHT_IN4_PIN,
    MOTOR_RIGHT_IN4_PIN);
}

void softPWMDual(int dutyCycle, int cycles)
{
    int i;
    for (i = 0; i < cycles; i++)
    {
        motorForward();
        MAP_UtilsDelay(dutyCycle * 1000);
        if (dutyCycle < 100)
        {
            motorStop();
            MAP_UtilsDelay((100 - dutyCycle) * 1000);
        }
    }
}

void softPWMDualBackward(int dutyCycle, int cycles)
{
    int i;
    for (i = 0; i < cycles; i++)
    {
        motorBackward();
        MAP_UtilsDelay(dutyCycle * 1000);
        if (dutyCycle < 100)
        {
            motorStop();
            MAP_UtilsDelay((100 - dutyCycle) * 1000);
        }
    }
}

void softPWMDualLeft(int dutyCycle, int cycles)
{
    int i;
    for (i = 0; i < cycles; i++)
    {
        motorLeft();
        MAP_UtilsDelay(dutyCycle * 1000);
        if (dutyCycle < 100)
        {
            motorStop();
            MAP_UtilsDelay((100 - dutyCycle) * 1000);
        }
    }
}

void softPWMDualRight(int dutyCycle, int cycles)
{
    int i;
    for (i = 0; i < cycles; i++)
    {
        motorRight();
        MAP_UtilsDelay(dutyCycle * 1000);
        if (dutyCycle < 100)
        {
            motorStop();
            MAP_UtilsDelay((100 - dutyCycle) * 1000);
        }
    }
}

void motorForwardPWM(int dutyCycle, int cycles)
{
    motorStop();
    softPWMDual(dutyCycle, cycles);
    motorStop();
}

void motorBackwardPWM(int dutyCycle, int cycles)
{
    motorStop();
    softPWMDualBackward(dutyCycle, cycles);
    motorStop();
}

void motorLeftPWM(int dutyCycle, int cycles)
{
    motorStop();
    softPWMDualLeft(dutyCycle, cycles);
    motorStop();
}

void motorRightPWM(int dutyCycle, int cycles)
{
    motorStop();
    softPWMDualRight(dutyCycle, cycles);
    motorStop();
}
//*****************************************************************************
//                            Motor -- End
//*****************************************************************************


//*****************************************************************************
//
//! Application startup display on UART
//!
//! \param  none
//!
//! \return none
//!
//*****************************************************************************
static void DisplayBanner(char *AppName)
{
    UART_PRINT("\n\n\n\r");
    UART_PRINT("\t\t *************************************************\n\r");
    UART_PRINT("\t\t      CC3200 %s Application       \n\r", AppName);
    UART_PRINT("\t\t *************************************************\n\r");
    UART_PRINT("\n\n\n\r");
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
static void BoardInit(void)
{
    /* In case of TI-RTOS vector table is initialize by OS itself */
#ifndef USE_TIRTOS
    //
    // Set vector table base
    //
#if defined(ccs) || defined(gcc)
    MAP_IntVTableBaseSet((unsigned long) &g_pfnVectors[0]);
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
//                            MAIN FUNCTION
//****************************************************************************
void main()
{
    long lRetVal = -1;

    //
    // Board Initialization
    //
    BoardInit();

    //
    // uDMA Initialization
    //
    UDMAInit();

    //
    // Configure the pinmux settings for the peripherals exercised
    //
    PinMuxConfig();

    //
    // Configuring UART
    //
    InitTerm();

    //
    // Display banner
    //
    DisplayBanner(APPLICATION_NAME);

    lRetVal = connectToAccessPoint();
    udp_socket();
    listen_for_controller();

    while (1)
    {
        char buffer[UDP_MAX_SIZE];
        receive_udp_peer(buffer, UDP_MAX_SIZE);

        if (strlen(buffer) < 2 || strlen(buffer) > 4)
        {
            continue;
        }

        int dutyCycle = strtol(&buffer[1], NULL, 10);
        if (!dutyCycle || dutyCycle < 0 || dutyCycle > 99)
        {
            continue;
        }

        switch (buffer[0])
        {
        case 'W':
            motorForwardPWM(dutyCycle, 100);
            break;
        case 'S':
            motorBackwardPWM(dutyCycle, 100);
            break;
        case 'A':
            motorLeftPWM(dutyCycle, 100);
            break;
        case 'D':
            motorRightPWM(dutyCycle, 100);
            break;
        default:
            return;
        }

    }

    UART_PRINT("Exiting Application ...\n\r");

    //
    // power off the network processor
    //
    lRetVal = sl_Stop(SL_STOP_TIMEOUT);

    while (1)
    {
        _SlNonOsMainLoopTask();
    }
}

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************
