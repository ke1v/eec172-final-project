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
// Application Name     - I2C
// Application Overview - The objective of this application is act as an I2C
//                        diagnostic tool. The demo application is a generic
//                        implementation that allows the user to communicate
//                        with any I2C device over the lines.
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup i2c_demo
//! @{
//
//*****************************************************************************

// Standard includes
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

// Driverlib includes
#include "hw_types.h"
#include "hw_ints.h"
#include "hw_memmap.h"
#include "hw_common_reg.h"
#include "rom.h"
#include "rom_map.h"
#include "interrupt.h"
#include "prcm.h"
#include "utils.h"
#include "uart.h"

// Common interface includes
#include "uart_if.h"
#include "i2c_if.h"

#include "pinmux.h"

#include "vl53lo/core/inc/vl53l0x_api.h"
#include "vl53lo/platform/inc/vl53l0x_platform.h"

//*****************************************************************************
//                      MACRO DEFINITIONS
//*****************************************************************************
#define APPLICATION_VERSION "0.0.0"
#define APP_NAME            "Roomba"
#define UART_PRINT          Report
#define CONSOLE             UARTA0_BASE
#define FAILURE             -1
#define SUCCESS             0
#define RETERR_IF_TRUE(condition) \
	{                             \
		if (condition)            \
			return FAILURE;       \
	}
#define RET_IF_ERR(Func)        \
	{                           \
		int iRetVal = (Func);   \
		if (SUCCESS != iRetVal) \
			return iRetVal;     \
	}

#define LIDAR_MAX_RANGE    500
#define LIDAR_OUT_OF_RANGE 8190

//*****************************************************************************
//                 GLOBAL VARIABLES -- Start
//*****************************************************************************
#if defined(ccs)
extern void (*const g_pfnVectors[])(void);
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
BoardInit(void) {
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
//                            LiDAR -- Start
//*****************************************************************************
VL53L0X_Dev_t dev;
VL53L0X_DeviceInfo_t DeviceInfo;

void LIDAR_INIT() {
	VL53L0X_Dev_t *pDev = &dev;

	uint32_t refSpadCount;
	uint8_t isApertureSpads;
	uint8_t VhvSettings;
	uint8_t PhaseCal;

	VL53L0X_Error status;

	pDev->I2cDevAddr = 0x29;
	pDev->comms_type = 1;
	pDev->comms_speed_khz = 400;

	VL53L0X_WaitDeviceBooted(pDev);

	status = VL53L0X_DataInit(pDev);
	if (status != VL53L0X_ERROR_NONE) {
		UART_PRINT("VL53L0X_DataInit Failed\r\n");
	}

	status = VL53L0X_GetDeviceInfo(pDev, &DeviceInfo);
	if (status != VL53L0X_ERROR_NONE) {
		UART_PRINT("VL53L0X_GetDeviceInfo Failed\r\n");
	}

	status = VL53L0X_StaticInit(pDev);
	if (status != VL53L0X_ERROR_NONE) {
		UART_PRINT("VL53L0X_StaticInit Failed\r\n");
	}

	status = VL53L0X_PerformRefSpadManagement(pDev, &refSpadCount, &isApertureSpads);
	if (status != VL53L0X_ERROR_NONE) {
		UART_PRINT("VL53L0X_PerformRefSpadManagement Failed\r\n");
	}

	status = VL53L0X_PerformRefCalibration(pDev, &VhvSettings, &PhaseCal);
	if (status != VL53L0X_ERROR_NONE) {
		UART_PRINT("VL53L0X_PerformRefCalibration Failed\r\n");
	}

	status = VL53L0X_SetDeviceMode(pDev, VL53L0X_DEVICEMODE_SINGLE_RANGING);
	if (status != VL53L0X_ERROR_NONE) {
		UART_PRINT("VL53L0X_SetDeviceMode Failed\r\n");
	}
}

uint16_t LIDAR_GET_RANGE_MILLIMETER() {
	VL53L0X_RangingMeasurementData_t measure;
	VL53L0X_Error status;

	status = VL53L0X_PerformSingleRangingMeasurement(pDev, &measure);
	if (status != VL53L0X_ERROR_NONE) {
		UART_PRINT("VL53L0X_PerformSingleRangingMeasurement Failed\r\n");
	}

	return = measure.RangeMilliMeter;
}
//*****************************************************************************
//                            LiDAR -- End
//*****************************************************************************







//*****************************************************************************
//                            Main Function
//*****************************************************************************
void main() {
	BoardInit();
	PinMuxConfig();

	InitTerm();

	I2C_IF_Open(I2C_MASTER_MODE_FST);

	UART_PRINT("-----\tSTARTING DEVICE\t-----\r\n");

	while (1) {

		if (distance < 50) {
			motorfoward();
		} else {
			motorstop();
		}

		UART_PRINT("Distance: %d\r\n", distance);
	}
}

//*****************************************************************************
//
// Close the Doxygen group.
//! @
//
//*****************************************************************************
