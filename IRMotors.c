// Ashton Wooldridge and Kelvin Wu
// RC Car IR Control

#include <stdio.h>
#include "hw_apps_rcm.h"
#include "hw_common_reg.h"
#include "hw_ints.h"
#include "hw_memmap.h"
#include "hw_types.h"
#include "interrupt.h"
#include "prcm.h"
#include "rom.h"
#include "rom_map.h"
#include "utils.h"
#include "timer.h"
#include "systick.h"
#include "gpio.h"
#include "uart.h"
#include "gpio_if.h"
#include "uart_if.h"
#include "pinmux.h"

#if defined(ccs)
extern void (*const g_pfnVectors[])(void);
#endif

//*****************************************************************************
// IR Remote Button Codes (AT&T Remote)
//*****************************************************************************
#define ONE     0x60DFC837
#define TWO     0x60DF08F7
#define THREE   0x60DF8877
#define FOUR    0x60DFF00F
#define FIVE    0x60DF30CF
#define SIX     0x60DFB04F
#define SEVEN   0x60DFD02F
#define EIGHT   0x60DF10EF
#define NINE    0x60DF906F
#define ZERO    0x60DF926D
#define MUTE    0x60DF48B7  // STOP
#define LAST    0x60DFE01F  // Dead Reckoning

//*****************************************************************************
// IR Decoding Config
//*****************************************************************************
#define TIMER               TIMERA0_BASE
#define MAX_SYSTICK         2 * 1000000
#define GPIO_GROUP          GPIOA0_BASE  // ADDED
#define GPIO_PIN            0x2         // ADDED - changed from 0x10 to match PIN_55

const unsigned long START_WIDTH       = 4430 / 2;
const unsigned long ZERO_WIDTH        = 550 / 2;
const unsigned long ONE_WIDTH         = 1670 / 2;
const unsigned long ACCEPTABLE_JITTER = 100;

static volatile int signalFlag    = 0;
static volatile int stopwatchFlag = 0;
static volatile int inputFlag     = 0;

static volatile long          sysTickValue = 0;
static volatile unsigned long readBuffer   = 0;
static volatile unsigned long readBits     = 0;

//*****************************************************************************
// Motor Control Pin Definitions
//*****************************************************************************

// IN1 - Pin 15 - GPIO22
#define MOTOR_LEFT_IN1_BASE     GPIOA2_BASE
#define MOTOR_LEFT_IN1_PIN      0x40

// IN2 - Pin 64 - GPIO9
#define MOTOR_LEFT_IN2_BASE     GPIOA1_BASE
#define MOTOR_LEFT_IN2_PIN      0x02

// IN3 - Pin 6 - GPIO7
#define MOTOR_RIGHT_IN3_BASE    GPIOA1_BASE
#define MOTOR_RIGHT_IN3_PIN     0x80

// IN4 - Pin 45 - GPIO31
#define MOTOR_RIGHT_IN4_BASE    GPIOA3_BASE
#define MOTOR_RIGHT_IN4_PIN     0x80

//*****************************************************************************
// Software PWM Helpers
//*****************************************************************************
void softPWMDual(int dutyCycle, int cycles) {
    int i;
    for (i = 0; i < cycles; i++) {
        MAP_GPIOPinWrite(MOTOR_LEFT_IN1_BASE,  MOTOR_LEFT_IN1_PIN,  MOTOR_LEFT_IN1_PIN);
        MAP_GPIOPinWrite(MOTOR_RIGHT_IN4_BASE, MOTOR_RIGHT_IN4_PIN, MOTOR_RIGHT_IN4_PIN);
        MAP_UtilsDelay(dutyCycle * 1000);
        if (dutyCycle < 100) {
            MAP_GPIOPinWrite(MOTOR_LEFT_IN1_BASE,  MOTOR_LEFT_IN1_PIN,  0);
            MAP_GPIOPinWrite(MOTOR_RIGHT_IN4_BASE, MOTOR_RIGHT_IN4_PIN, 0);
            MAP_UtilsDelay((100 - dutyCycle) * 1000);
        }
    }
}

void softPWMDualBackward(int dutyCycle, int cycles) {
    int i;
    for (i = 0; i < cycles; i++) {
        MAP_GPIOPinWrite(MOTOR_LEFT_IN2_BASE,  MOTOR_LEFT_IN2_PIN,  MOTOR_LEFT_IN2_PIN);
        MAP_GPIOPinWrite(MOTOR_RIGHT_IN3_BASE, MOTOR_RIGHT_IN3_PIN, MOTOR_RIGHT_IN3_PIN);
        MAP_UtilsDelay(dutyCycle * 1000);
        if (dutyCycle < 100) {
            MAP_GPIOPinWrite(MOTOR_LEFT_IN2_BASE,  MOTOR_LEFT_IN2_PIN,  0);
            MAP_GPIOPinWrite(MOTOR_RIGHT_IN3_BASE, MOTOR_RIGHT_IN3_PIN, 0);
            MAP_UtilsDelay((100 - dutyCycle) * 1000);
        }
    }
}

void softPWMDualLeft(int dutyCycle, int cycles) {
    int i;
    for (i = 0; i < cycles; i++) {
        MAP_GPIOPinWrite(MOTOR_LEFT_IN2_BASE,  MOTOR_LEFT_IN2_PIN,  MOTOR_LEFT_IN2_PIN);
        MAP_GPIOPinWrite(MOTOR_RIGHT_IN4_BASE, MOTOR_RIGHT_IN4_PIN, MOTOR_RIGHT_IN4_PIN);
        MAP_UtilsDelay(dutyCycle * 1000);
        if (dutyCycle < 100) {
            MAP_GPIOPinWrite(MOTOR_LEFT_IN2_BASE,  MOTOR_LEFT_IN2_PIN,  0);
            MAP_GPIOPinWrite(MOTOR_RIGHT_IN4_BASE, MOTOR_RIGHT_IN4_PIN, 0);
            MAP_UtilsDelay((100 - dutyCycle) * 1000);
        }
    }
}

void softPWMDualRight(int dutyCycle, int cycles) {
    int i;
    for (i = 0; i < cycles; i++) {
        MAP_GPIOPinWrite(MOTOR_LEFT_IN1_BASE,  MOTOR_LEFT_IN1_PIN,  MOTOR_LEFT_IN1_PIN);
        MAP_GPIOPinWrite(MOTOR_RIGHT_IN3_BASE, MOTOR_RIGHT_IN3_PIN, MOTOR_RIGHT_IN3_PIN);
        MAP_UtilsDelay(dutyCycle * 1000);
        if (dutyCycle < 100) {
            MAP_GPIOPinWrite(MOTOR_LEFT_IN1_BASE,  MOTOR_LEFT_IN1_PIN,  0);
            MAP_GPIOPinWrite(MOTOR_RIGHT_IN3_BASE, MOTOR_RIGHT_IN3_PIN, 0);
            MAP_UtilsDelay((100 - dutyCycle) * 1000);
        }
    }
}

//*****************************************************************************
// Motor Control Functions
//*****************************************************************************
void motorStop() {
    MAP_GPIOPinWrite(MOTOR_LEFT_IN1_BASE,  MOTOR_LEFT_IN1_PIN,  0);
    MAP_GPIOPinWrite(MOTOR_LEFT_IN2_BASE,  MOTOR_LEFT_IN2_PIN,  0);
    MAP_GPIOPinWrite(MOTOR_RIGHT_IN3_BASE, MOTOR_RIGHT_IN3_PIN, 0);
    MAP_GPIOPinWrite(MOTOR_RIGHT_IN4_BASE, MOTOR_RIGHT_IN4_PIN, 0);
    Message("STOP\n\r");
}

void motorForward() {
    MAP_GPIOPinWrite(MOTOR_LEFT_IN1_BASE,  MOTOR_LEFT_IN1_PIN,  MOTOR_LEFT_IN1_PIN);
    MAP_GPIOPinWrite(MOTOR_LEFT_IN2_BASE,  MOTOR_LEFT_IN2_PIN,  0);
    MAP_GPIOPinWrite(MOTOR_RIGHT_IN3_BASE, MOTOR_RIGHT_IN3_PIN, 0);
    MAP_GPIOPinWrite(MOTOR_RIGHT_IN4_BASE, MOTOR_RIGHT_IN4_PIN, MOTOR_RIGHT_IN4_PIN);
    Message("FORWARD\n\r");
}

void motorBackward() {
    MAP_GPIOPinWrite(MOTOR_LEFT_IN1_BASE,  MOTOR_LEFT_IN1_PIN,  0);
    MAP_GPIOPinWrite(MOTOR_LEFT_IN2_BASE,  MOTOR_LEFT_IN2_PIN,  MOTOR_LEFT_IN2_PIN);
    MAP_GPIOPinWrite(MOTOR_RIGHT_IN3_BASE, MOTOR_RIGHT_IN3_PIN, MOTOR_RIGHT_IN3_PIN);
    MAP_GPIOPinWrite(MOTOR_RIGHT_IN4_BASE, MOTOR_RIGHT_IN4_PIN, 0);
    Message("BACKWARD\n\r");
}

void motorLeft() {
    MAP_GPIOPinWrite(MOTOR_LEFT_IN1_BASE,  MOTOR_LEFT_IN1_PIN,  0);
    MAP_GPIOPinWrite(MOTOR_LEFT_IN2_BASE,  MOTOR_LEFT_IN2_PIN,  MOTOR_LEFT_IN2_PIN);
    MAP_GPIOPinWrite(MOTOR_RIGHT_IN3_BASE, MOTOR_RIGHT_IN3_PIN, 0);
    MAP_GPIOPinWrite(MOTOR_RIGHT_IN4_BASE, MOTOR_RIGHT_IN4_PIN, MOTOR_RIGHT_IN4_PIN);
    Message("LEFT\n\r");
}

void motorRight() {
    MAP_GPIOPinWrite(MOTOR_LEFT_IN1_BASE,  MOTOR_LEFT_IN1_PIN,  MOTOR_LEFT_IN1_PIN);
    MAP_GPIOPinWrite(MOTOR_LEFT_IN2_BASE,  MOTOR_LEFT_IN2_PIN,  0);
    MAP_GPIOPinWrite(MOTOR_RIGHT_IN3_BASE, MOTOR_RIGHT_IN3_PIN, MOTOR_RIGHT_IN3_PIN);
    MAP_GPIOPinWrite(MOTOR_RIGHT_IN4_BASE, MOTOR_RIGHT_IN4_PIN, 0);
    Message("RIGHT\n\r");
}

void motorForwardPWM(int dutyCycle, int cycles) {
    MAP_GPIOPinWrite(MOTOR_LEFT_IN2_BASE,  MOTOR_LEFT_IN2_PIN,  0);
    MAP_GPIOPinWrite(MOTOR_RIGHT_IN3_BASE, MOTOR_RIGHT_IN3_PIN, 0);
    softPWMDual(dutyCycle, cycles);
    Message("FORWARD PWM\n\r");
}

void motorBackwardPWM(int dutyCycle, int cycles) {
    MAP_GPIOPinWrite(MOTOR_LEFT_IN1_BASE,  MOTOR_LEFT_IN1_PIN,  0);
    MAP_GPIOPinWrite(MOTOR_RIGHT_IN4_BASE, MOTOR_RIGHT_IN4_PIN, 0);
    softPWMDualBackward(dutyCycle, cycles);
    Message("BACKWARD PWM\n\r");
}

void motorLeftPWM(int dutyCycle, int cycles) {
    MAP_GPIOPinWrite(MOTOR_LEFT_IN1_BASE,  MOTOR_LEFT_IN1_PIN,  0);
    MAP_GPIOPinWrite(MOTOR_RIGHT_IN3_BASE, MOTOR_RIGHT_IN3_PIN, 0);
    softPWMDualLeft(dutyCycle, cycles);
    Message("LEFT PWM\n\r");
}

void motorRightPWM(int dutyCycle, int cycles) {
    MAP_GPIOPinWrite(MOTOR_LEFT_IN2_BASE,  MOTOR_LEFT_IN2_PIN,  0);
    MAP_GPIOPinWrite(MOTOR_RIGHT_IN4_BASE, MOTOR_RIGHT_IN4_PIN, 0);
    softPWMDualRight(dutyCycle, cycles);
    Message("RIGHT PWM\n\r");
}

//*****************************************************************************
// IR Decode: map button to motor command
//*****************************************************************************
void handleIRCommand(unsigned long button) {
    char buf[50];
    sprintf(buf, "RAW: 0x%08X\n\r", button);
    Message(buf);

    if (button == TWO) {
        motorForwardPWM(75, 500);
    } else if (button == EIGHT) {
        motorBackwardPWM(75, 500);
    } else if (button == FOUR) {
        motorLeftPWM(50, 500);
    } else if (button == SIX) {
        motorRightPWM(50, 500);
    } else if (button == MUTE) {
        motorStop();
    } else if (button == LAST) {                  // ADDED - dead reckoning sequence
        motorForwardPWM(75, 500);
        motorStop();
        MAP_UtilsDelay(10000000);
        motorLeftPWM(50, 300);
        motorStop();
        MAP_UtilsDelay(10000000);
        motorForwardPWM(75, 500);
        motorStop();
    }
}

//*****************************************************************************
// IR Pulse Width Helper
//*****************************************************************************
int inRange(int value, int target, int range) {
    return value >= target - range && value <= target + range;
}

//*****************************************************************************
// SysTick Handler - ADDED
//*****************************************************************************
void SysTickHandler(void) {
    sysTickValue = MAP_SysTickValueGet();
}

//*****************************************************************************
// GPIO Edge Interrupt Handler (IR Receiver)
//*****************************************************************************
void EdgeHandler(void) {
    MAP_GPIOIntClear(GPIO_GROUP, MAP_GPIOIntStatus(GPIO_GROUP, false));

    if (MAP_GPIOPinRead(GPIO_GROUP, GPIO_PIN)) {
    } else {
        if (sysTickValue <= 0) return;

        if (signalFlag) {
            if (inRange(sysTickValue, ZERO_WIDTH, ACCEPTABLE_JITTER)) {
                readBuffer = (readBuffer << 1);
                readBits++;
            } else if (inRange(sysTickValue, ONE_WIDTH, ACCEPTABLE_JITTER)) {
                readBuffer = (readBuffer << 1) + 1;
                readBits++;
            }
            if (readBits >= 32) {
                signalFlag = 0;
                inputFlag  = 1;
                readBits   = 0;
            }
        } else if (inRange(sysTickValue, START_WIDTH, ACCEPTABLE_JITTER)) {
            signalFlag = 1;
        }
    }
}

//*****************************************************************************
// IR Setup - ADDED
//*****************************************************************************
void IRSetup(void) {
    MAP_SysTickPeriodSet(MAX_SYSTICK);
    MAP_SysTickIntRegister(SysTickHandler);
    MAP_SysTickIntEnable();
    MAP_SysTickEnable();

    MAP_GPIOIntRegister(GPIO_GROUP, EdgeHandler);
    MAP_GPIOIntTypeSet(GPIO_GROUP, GPIO_PIN, GPIO_BOTH_EDGES);
    MAP_GPIOIntEnable(GPIO_GROUP, GPIO_PIN);
    MAP_IntEnable(INT_GPIOA0);  // ADDED for Pin 55
}

//*****************************************************************************
// Board Init
//*****************************************************************************
static void BoardInit(void) {
#ifndef USE_TIRTOS
#if defined(ccs)
    MAP_IntVTableBaseSet((unsigned long)&g_pfnVectors[0]);
#endif
#endif
    MAP_IntMasterEnable();
    MAP_IntEnable(FAULT_SYSTICK);
    PRCMCC3200MCUInit();
}

//*****************************************************************************
// Main
//*****************************************************************************

int main(void) {
    BoardInit();
    PinMuxConfig();
    InitTerm();
    ClearTerm();

    Message("RC Car IR Control Ready\n\r");
    Message("2=Forward 8=Backward 4=Left 6=Right MUTE=Stop LAST=DeadReckoning\n\r");

    IRSetup();
    motorStop();

    while (1) {
        if (inputFlag) {
            inputFlag = 0;
            handleIRCommand(readBuffer);
            readBuffer = 0;
        }
    }
}