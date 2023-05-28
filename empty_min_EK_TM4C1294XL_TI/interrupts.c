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
#include <ti/drivers/GPIO.h>
#include <ti/drivers/gpio/GPIOTiva.h>

#include "utils/uartstdio.h"

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
//#define MOTOR_HALL_A_PORT GPIO_PORTM_BASE
#define MOTOR_HALL_A_INT INT_GPIOM
#define MOTOR_HALL_A_PERIPH SYSCTL_PERIPH_GPIOM

#define MOTOR_HALL_B_PIN GPIOTiva_PH_2
//#define MOTOR_HALL_B_PORT GPIO_PORTH_BASE
#define MOTOR_HALL_B_INT INT_GPIOH
#define MOTOR_HALL_B_PERIPH SYSCTL_PERIPH_GPIOH

#define MOTOR_HALL_C_PIN GPIOTiva_PN_2
//#define MOTOR_HALL_C_PORT GPIO_PORTN_BASE
#define MOTOR_HALL_C_INT INT_GPION
#define MOTOR_HALL_C_PERIPH SYSCTL_PERIPH_GPION

int motor_edgeCount = 0;
bool motor_hallStates[3];

void motor_driver(void);
void motor_initHall(void);
void motor_init(void);

//void motor_initHall_butItsWrong(void)
//{
////    // Enable all Ints
////    MAP_IntMasterEnable();
////
////    // Hall A Config
////    MAP_SysCtlPeripheralEnable(MOTOR_HALL_A_PERIPH);
////    MAP_GPIOPinTypeGPIOInput(MOTOR_HALL_A_PORT, MOTOR_HALL_A_PIN);
////    MAP_GPIOIntEnable(MOTOR_HALL_A_PORT, MOTOR_HALL_A_PIN);
////    MAP_GPIOIntTypeSet(MOTOR_HALL_A_PORT, MOTOR_HALL_A_PIN, GPIO_RISING_EDGE);
////    MAP_GPIOIntRegister(MOTOR_HALL_A_PORT, motor_driver);
////    MAP_IntEnable(MOTOR_HALL_A_INT);
////    MAP_IntPrioritySet(MOTOR_HALL_A_INT, 0x00);
////
////    // Hall B Config
////    MAP_SysCtlPeripheralEnable(MOTOR_HALL_B_PERIPH);
////    MAP_GPIOPinTypeGPIOInput(MOTOR_HALL_B_PORT, MOTOR_HALL_B_PIN);
////    MAP_GPIOIntEnable(MOTOR_HALL_B_PORT, MOTOR_HALL_B_PIN);
////    MAP_GPIOIntTypeSet(MOTOR_HALL_B_PORT, MOTOR_HALL_B_PIN, GPIO_RISING_EDGE);
////    MAP_GPIOIntRegister(MOTOR_HALL_B_PORT, motor_driver);
////    MAP_IntEnable(MOTOR_HALL_B_INT);
////    MAP_IntPrioritySet(MOTOR_HALL_B_INT, 0x40);
////
////    // Hall C Config
////    MAP_SysCtlPeripheralEnable(MOTOR_HALL_C_PERIPH);
////    MAP_GPIOPinTypeGPIOInput(MOTOR_HALL_C_PORT, MOTOR_HALL_C_PIN);
////    MAP_GPIOIntEnable(MOTOR_HALL_C_PORT, MOTOR_HALL_C_PIN);
////    MAP_GPIOIntTypeSet(MOTOR_HALL_C_PORT, MOTOR_HALL_C_PIN, GPIO_RISING_EDGE);
////    MAP_GPIOIntRegister(MOTOR_HALL_C_PORT, motor_driver);
////    MAP_IntEnable(MOTOR_HALL_C_INT);
////    MAP_IntPrioritySet(MOTOR_HALL_C_INT, 0x80);
//}
void motor_initHall(void)
{
        GPIO_PinConfig hallPinConfig = GPIO_CFG_INPUT | GPIO_CFG_IN_INT_RISING

        GPIO_setConfig(MOTOR_HALL_A_PIN, hallPinConfig);
        GPIO_setConfig(MOTOR_HALL_B_PIN, hallPinConfig);
        GPIO_setConfig(MOTOR_HALL_C_PIN, hallPinConfig);

        GPIO_setCallback(MOTOR_HALL_A_PIN,(GPIO_CallbackFxn)motor_driver);
        GPIO_setCallback(MOTOR_HALL_B_PIN,(GPIO_CallbackFxn)motor_driver);
        GPIO_setCallback(MOTOR_HALL_C_PIN,(GPIO_CallbackFxn)motor_driver);

        GPIO_enableInt(MOTOR_HALL_A_PIN);
        GPIO_enableInt(MOTOR_HALL_B_PIN);
        GPIO_enableInt(MOTOR_HALL_C_PIN);
}

void
motor_driver(void)
{


    motor_edgeCount++;

    //get hall vals
    motor_hallStates[0] = MAP_GPIOPinRead(MOTOR_HALL_A_PORT, MOTOR_HALL_A_PIN);
    motor_hallStates[1] = MAP_GPIOPinRead(MOTOR_HALL_B_PORT, MOTOR_HALL_B_PIN);
    motor_hallStates[2] = MAP_GPIOPinRead(MOTOR_HALL_C_PORT, MOTOR_HALL_C_PIN);
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

    g_ui32SysClock = MAP_SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
                                             SYSCTL_OSC_MAIN |
                                             SYSCTL_USE_PLL |
                                             SYSCTL_CFG_VCO_240), 120000000);



    //
    // Init motor.
    //
    Board_initGeneral();
    Board_initGPIO();
    motor_init();

    //
    // Loop forever.
    //
    while(1)
    {
    }
}
