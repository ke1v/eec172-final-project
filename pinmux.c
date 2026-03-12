//*****************************************************************************
// pinmux.c
//
// configure the device pins for different peripheral signals
//
// Copyright (C) 2014 Texas Instruments Incorporated - http://www.ti.com/
//*****************************************************************************
#include "pinmux.h"
#include "hw_types.h"
#include "hw_memmap.h"
#include "hw_gpio.h"
#include "pin.h"
#include "rom.h"
#include "rom_map.h"
#include "gpio.h"
#include "prcm.h"

/* Pinout
 * 55 - UART0 TX
 * 57 - UART0 RX
 * 01 - I2C SCL
 * 02 - I2C SDA

 * 21 - IN1 Left Motor
 * 64 - IN2 Left Motor
 * 45 - IN3 Right Motor
 * 06 - IN4 Right Motor

 * 18 - Front LiDAR Sensor
 * 21 - Left LiDAR Sensor
 * 15 - Right LiDAR Sensor
 * 16 - Rear LiDAR Sensor
 * 17 - Left Wheel LiDAR Sensor
 */

void PinMuxConfig(void) {
	//
	// Enable Peripheral Clocks
	//
	MAP_PRCMPeripheralClkEnable(PRCM_UARTA0, PRCM_RUN_MODE_CLK);
	MAP_PRCMPeripheralClkEnable(PRCM_GPIOA0, PRCM_RUN_MODE_CLK);
	MAP_PRCMPeripheralClkEnable(PRCM_GPIOA1, PRCM_RUN_MODE_CLK);
	MAP_PRCMPeripheralClkEnable(PRCM_GPIOA3, PRCM_RUN_MODE_CLK);

	//
	// Configure PIN_55 for UART0 TX
	//
	MAP_PinTypeUART(PIN_55, PIN_MODE_3);
	//
	// Configure PIN_57 for UART0 RX
	//
	MAP_PinTypeUART(PIN_57, PIN_MODE_3);

	//
	// Configure PIN_50 for GPIO Input (IR Receiver)
	// This maps to GPIOA0 pin 0x10
	//
	MAP_PinTypeGPIO(PIN_50, PIN_MODE_0, false);
	MAP_GPIODirModeSet(GPIOA0_BASE, 0x10, GPIO_DIR_MODE_IN);

	//
	// Configure PIN_58 for GPIO Output (ENA - Left Motor Enable)
	// P02 = GPIOA0 bit 3 (0x08)
	//
	MAP_PinTypeGPIO(PIN_58, PIN_MODE_0, false);
	MAP_GPIODirModeSet(GPIOA0_BASE, 0x08, GPIO_DIR_MODE_OUT);

	//
	// Configure PIN_21 for GPIO Output (IN1 - Left Motor Direction)
	// P17 = GPIOA0 bit 6 (0x40)
	//
	MAP_PinTypeGPIO(PIN_21, PIN_MODE_0, false);
	MAP_GPIODirModeSet(GPIOA0_BASE, 0x40, GPIO_DIR_MODE_OUT);

	//
	// Configure PIN_64 for GPIO Output (IN2 - Left Motor Direction)
	// P21 = GPIOA3 bit 6 (0x40)
	//
	MAP_PinTypeGPIO(PIN_64, PIN_MODE_0, false);
	MAP_GPIODirModeSet(GPIOA3_BASE, 0x40, GPIO_DIR_MODE_OUT);

	//
	// Configure PIN_45 for GPIO Output (IN3 - Right Motor Direction)
	// P64 = GPIOA1 bit 7 (0x80)
	//
	MAP_PinTypeGPIO(PIN_45, PIN_MODE_0, false);
	MAP_GPIODirModeSet(GPIOA1_BASE, 0x80, GPIO_DIR_MODE_OUT);

	//
	// Configure PIN_06 for GPIO Output (IN4 - Right Motor Direction)
	// P62 = GPIOA1 bit 5 (0x20)
	//
	MAP_PinTypeGPIO(PIN_06, PIN_MODE_0, false);
	MAP_GPIODirModeSet(GPIOA1_BASE, 0x20, GPIO_DIR_MODE_OUT);

	//
	// Configure PIN_63 for GPIO Output (ENB - Right Motor Enable)
	// P18 = GPIOA3 bit 4 (0x10)
	//
	MAP_PinTypeGPIO(PIN_63, PIN_MODE_0, false);
	MAP_GPIODirModeSet(GPIOA3_BASE, 0x10, GPIO_DIR_MODE_OUT);
}