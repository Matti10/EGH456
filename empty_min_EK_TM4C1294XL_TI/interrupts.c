//*****************************************************************************
//
// interrupts.c - Interrupt preemption and tail-chaining example.
//
// Copyright (c) 2013-2020 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
// 
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
// 
// This is part of revision 2.2.0.295 of the EK-TM4C1294XL Firmware Package.
//
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>


#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_nvic.h"
#include "inc/hw_types.h"

#include <xdc/std.h>
#include <xdc/runtime/Error.h>

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
//#include <ti/sysbios/family/arm/m3/Hwi.h>
#include <ti/drivers/GPIO.h>
#include <ti/drivers/gpio/GPIOTiva.h>

#include "utils/uartstdio.h"
#include "utils/Board.h"

#include "./drivers/motorlib.h"

int g_ui32SysClock;


//*****************************************************************************
//
// Motor Time
//
//*****************************************************************************
#define MOTOR_ACCELERATION_RPMs 750 //Note that this is measured in RPM per Second
#define MOTOR_ESTOP_RPMs 1000 //Note that this is measured in RPM per Second
#define MOTOR_CURR_MAX //we need to decide what this is
#define MOTOR_TEMP_MAX //we need to decide what this is
#define MOTOR_MAX_DUTY 100

#define MOTOR_HALL_A_PIN GPIOTiva_PM_3
#define MOTOR_HALL_B_PIN GPIOTiva_PH_2
#define MOTOR_HALL_C_PIN GPIOTiva_PN_2


int motor_edgeCount = 0;
bool motor_hallStates[3];

void motor_driver(void);
void motor_initHall(void);
void motor_init(void);


void motor_initHall(void)
{
//        GPIO_PinConfig hallPinConfig = GPIO_CFG_INPUT | GPIO_CFG_IN_INT_RISING;

        GPIO_setConfig(MOTOR_HALL_B_PIN, GPIO_CFG_INPUT);
//        GPIO_setConfig(MOTOR_HALL_B_PIN, hallPinConfig);
//        GPIO_setConfig(MOTOR_HALL_C_PIN, hallPinConfig);

//        GPIO_setCallback(MOTOR_HALL_A_PIN,(GPIO_CallbackFxn)motor_driver);
//        GPIO_setCallback(MOTOR_HALL_B_PIN,(GPIO_CallbackFxn)motor_driver);
//        GPIO_setCallback(MOTOR_HALL_C_PIN,(GPIO_CallbackFxn)motor_driver);

//        GPIO_enableInt(MOTOR_HALL_A_PIN);
//        GPIO_enableInt(MOTOR_HALL_B_PIN);
//        GPIO_enableInt(MOTOR_HALL_C_PIN);
}

void
motor_driver(void)
{


    motor_edgeCount++;

    //get hall vals
    motor_hallStates[0] = GPIO_read(MOTOR_HALL_A_PIN);
    motor_hallStates[1] = GPIO_read(MOTOR_HALL_B_PIN);
    motor_hallStates[2] = GPIO_read(MOTOR_HALL_C_PIN);
    updateMotor(motor_hallStates[0],motor_hallStates[1],motor_hallStates[2]); //hall states to be retreived by rpm reader task
}

void motor_init(void)
{
    motor_initHall();

    Error_Block motorError;
    initMotorLib(MOTOR_MAX_DUTY, &motorError);
    enableMotor();
    setDuty(50);
}



int
main(void)
{
    //
    // Run from the PLL at 120 MHz.
    // Note: SYSCTL_CFG_VCO_240 is a new setting provided in TivaWare 2.2.x and
    // later to better reflect the actual VCO speed due to SYSCTL#22.
    //
    //    g_ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
    //                                             SYSCTL_OSC_MAIN |
    //                                             SYSCTL_USE_PLL |
    //                                             SYSCTL_CFG_VCO_240), 120000000);
    //


    //
    // Init motor.
    //
    Board_initGeneral();
    Board_initGPIO();

    motor_init();

    /* HWI Params */
//    struct ti_sysbios_family_arm_m3_Hwi_Params {
//        size_t __size;
//        const void *__self;
//        void *__fxns;
//        xdc_runtime_IInstance_Params *instance;
//        ti_sysbios_interfaces_IHwi_MaskingOption maskSetting;
//        xdc_UArg arg;
//        xdc_Bool enableInt;
//        xdc_Int eventId;
//        xdc_Int priority;
//        xdc_Bool useDispatcher;
//        xdc_runtime_IInstance_Params __iprms;
//    };

//    Hwi_enable();
//
//    Error_Block eb;
//    Error_init(&eb);
//    Hwi_Params hwiParams;
//
//    hwiParams.maskSetting = Hwi_MaskingOption_SELF;  // don't allow this interrupt to nest itself
//    hwiParams.priority = 1;
//    Hwi_Params_init(&hwiParams);
//    Hwi_Handle hwi_HALL_A_Handle = Hwi_create(MOTOR_HALL_A_PIN, (ti_sysbios_interfaces_IHwi_FuncPtr)motor_driver, &hwiParams, &eb);
//
//    if (hwi_HALL_A_Handle == NULL) {
//        // Handle Hwi_create() error
//        // ...
//    }




   BIOS_start();
}
