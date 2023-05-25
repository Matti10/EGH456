/*
 * Copyright (c) 2015, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ======== empty_min.c ========
 */
/* XDCtools Header files */
#include <xdc/std.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>

/* motor */
#include "./drivers/motorlib.h"

/* TI-RTOS Header files */
// #include <ti/drivers/EMAC.h>
#include <ti/drivers/GPIO.h>
#include <driverlib/gpio.h>
// #include <ti/drivers/I2C.h>
// #include <ti/drivers/SDSPI.h>
// #include <ti/drivers/SPI.h>
#include <ti/drivers/UART.h>
// #include <ti/drivers/USBMSCHFatFs.h>
// #include <ti/drivers/Watchdog.h>
// #include <ti/drivers/WiFi.h>

/* Board Header file */
#include "Board.h"


#include <inc/hw_memmap.h>
#include "driverlib/pin_map.h"
#include "utils/uartstdio.h"
#include "utils/ustdlib.h"
#include "driverlib/sysctl.h"
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Mailbox.h>
#include <ti/sysbios/knl/Event.h>
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/uart.h"
#include <ti/sysbios/knl/GateMutex.h>

uint32_t g_ui32SysClock;

// Setup Tasks
#define TASKSTACKSIZE   1024

Task_Struct task0Struct;
Char task0Stack[TASKSTACKSIZE];

Task_Struct task1Struct;
Char task1Stack[TASKSTACKSIZE];


// init Mutex
GateMutex_Handle gateMutexHandle;

//*****************************************************************************
//
// Motor Control
//
//*****************************************************************************

#define MOTOR_ACCELERATION_RPMs 750 //Note that this is measured in RPM per Second
#define MOTOR_ESTOP_RPMs 1000 //Note that this is measured in RPM per Second
#define MOTOR_CURR_MAX //we need to decide what this is
#define MOTOR_TEMP_MAX //we need to decide what this is
#define MOTOR_MAX_DUTY 100
#define HALL_INT_NUM 1

bool hallStates[3];
hallStates[0] = 0;
hallStates[1] = 0;
hallStates[2] = 0;

double rpm = 0.0;
uint16_t input_rpm = 500;

bool * motor_getHallState();
void motor_initHall();
void motor_GetRPM();
void motor_Driver();
void motor_eStop();

//
// E-Stop Thread / Interupt
//
void motor_eStop()
{
    // This Needs to check the following:
    // 1. Check Current isn't above MOTOR_CURR_MAX
    // 2. Motor isn't above MOTOR_TEMP_MAX
    // 3. get message from acceleration sensor, and check that the acceleration isn't above CRASH_ACCELERATION
    // 4. get message from distance sensor, and check that there isn't an object within CRASH_DISTANCE
    //
    // If any of those are true, it needs to call void stopMotor(bool brakeType);


}

//
// Motor Set RPM - Will likely be called by an interupt from the UI
//
void motor_Driver()
{
    UARTprintf("Starting Motor Driver");
    //This needs to:
    // Set motor RPM using void setDuty(uint16_t duty);
    // Will likely need to be setup in a closed loop with motor_GetRPM()
    // Will likely need to call motor_accelerate() & motor_decelerate
    //init buffer
    uint8_t bufferSize = 10;
    uint16_t rpmBuffer[bufferSize];



    uint8_t duty = 50;
    enableMotor();

    while(1)
    {
//        if (rpm > input_rpm && duty > 0)
//        {
//            duty--;
//        }
//        if (rpm < input_rpm && duty < MOTOR_MAX_DUTY)
//        {
//            duty++;
//        }

        setDuty(duty);
        hallStates[0] = GPIOPinRead(GPIO_PORTM_BASE, GPIO_PIN_3);
        hallStates[1] = GPIOPinRead(GPIO_PORTH_BASE, GPIO_PIN_2);
        hallStates[2] = GPIOPinRead(GPIO_PORTN_BASE, GPIO_PIN_2);
        updateMotor(hallStates[0],hallStates[1],hallStates[2]); //hall states to be retreived by rpm reader task

    }
}

//
// Motor Get RPM
//
void motor_GetRPM()
{
    UARTprintf("Starting RPM Task");

    //this assumes one hall trigger (per sensor) per revolution
    Uint32 startTime, endTime, i, temp1, temp2, cumSum;

    //MAKE THIS A TIMER THAT PERIODIACLLY CHECKS THE EDGE COUNT!?!?!??!


    for (i = 1; i < bufferSize; i++)
    {
        buffer[i] = 0;
    }


    while(1)
    {
        motor_getHallState();
        bool currHallState = hallStates[0];

        startTime = Clock_getTicks();
        do
        {
//            hallStates = {GPIOPinRead(GPIO_PORTM_BASE, GPIO_PIN_3), GPIOPinRead(GPIO_PORTH_BASE, GPIO_PIN_2), GPIOPinRead(GPIO_PORTN_BASE, GPIO_PIN_2)};

        }
        while (hallStates[0] == currHallState);

        endTime = Clock_getTicks();

        //start cumulitave sum to get avg rpm over last 10 samples
        cumSum = 0;

        temp1 = buffer[0];
        buffer[0] = ((endTime-startTime)/g_ui32SysClock)*60; //this is meant to convert rev/tick to rpm
        for (i = 1; i < bufferSize; i++)
        {
            //increment cumSum
            cumSum += temp1;

            //move all items down a place in buffer
            temp2 = buffer[i];
            buffer[i] = temp1;
            temp1 = temp2;
        }


        rpm = (cumSum/bufferSize); //RACE CONDITION add access control later

        UARTprintf("%f",rpm);
    }
}

void motor_getHallState()
{
    IArg key = GateMutex_enter(gateMutexHandle);

    UARTprintf("Updating Hall's");

    hallStates[0] = GPIOPinRead(GPIO_PORTM_BASE, GPIO_PIN_3);
    hallStates[1] = GPIOPinRead(GPIO_PORTH_BASE, GPIO_PIN_2);
    hallStates[2] = GPIOPinRead(GPIO_PORTN_BASE, GPIO_PIN_2);

    GateMutex_leave(gateMutexHandle, key);
}

void motor_initHall()
{
    //set hall pins as inputs
    GPIOPinTypeGPIOInput(GPIO_PORTM_BASE, GPIO_PIN_3);
    GPIOPinTypeGPIOInput(GPIO_PORTH_BASE, GPIO_PIN_2);
    GPIOPinTypeGPIOInput(GPIO_PORTN_BASE, GPIO_PIN_2);

    //set hall pins to interupt	GPIOIntTypeSet(GPIO_PORTM_BASE, GPIO_PIN_3, GPIO_RISING_EDGE);	GPIOIntTypeSet(GPIO_PORTH_BASE, GPIO_PIN_2, GPIO_RISING_EDGE);	GPIOIntTypeSet(GPIO_PORTN_BASE, GPIO_PIN_2, GPIO_RISING_EDGE);
}

//
// Accelerate the Motor
//
void motor_accelerate(bool direction)
{
    // Accelerate the motor at MOTOR_ACCELERATION_RPMs
    //  use the direction to establish if its acceleration or deceleration?
}



//*****************************************************************************
//
// Configure the UART and its pins.  This must be called before UARTprintf().
//
//*****************************************************************************
void ConfigureUART(void)
{
    //
    // Enable the GPIO Peripheral used by the UART.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //
    // Enable UART0
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    //
    // Configure GPIO Pins for UART mode.
    //
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Initialize the UART for console I/O.
    //
    UARTStdioConfig(0, 115200, g_ui32SysClock);a
}


/*
 *  ======== main ========
 */
int main(void)
{
    Task_Params task0Params;
    Task_Params task1Params;

    /* Call board init functions */
    Board_initGeneral();
    // Board_initEMAC();
    Board_initGPIO();
    // Board_initI2C();
    // Board_initSDSPI();
    // Board_initSPI();
    Board_initUART();
    // Board_initUSB(Board_USBDEVICE);
    // Board_initUSBMSCHFatFs();
    // Board_initWatchdog();
    // Board_initWiFi();
    motor_initHall();

    g_ui32SysClock = MAP_SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
                                                SYSCTL_OSC_MAIN |
                                                SYSCTL_USE_PLL |
                                                SYSCTL_CFG_VCO_480), 120000000);

    //
    // Init Hwi for Halls
    //
    Hwi_Params hwiParams;
    Hwi_Params_init(&hwiParams);
    Hwi_create(HALL_INT_NUM,motor_Driver, &hwiParams, NULL) //how do i find the correct int number???

    GPIOIntEnable(GPIO_PORTM_BASE, GPIO_INT_PIN_3)
    GPIOIntEnable(GPIO_PORTH_BASE, GPIO_INT_PIN_2)
    GPIOIntEnable(GPIO_PORTN_BASE, GPIO_INT_PIN_2)

    //
    // Init Mutex
    //
    GateMutex_init();
    gateMutexHandle = GateMutex_create(NULL, NULL);

    if (gateMutexHandle == NULL) {
        System_abort("Gate Mutex Pri create failed");
    }

    //
    // Initialize the UART.
    //
    ConfigureUART();

    UARTprintf("\033[2J\033[H");
    UARTprintf("Welcome to the 'car'\n");
    //
    // Init Motor
    //
    Error_Block motorError;
    uint16_t pwm_period = MOTOR_MAX_DUTY;
    initMotorLib(pwm_period, &motorError);

    if (Error_check(&motorError))
    {
        UARTprintf("GET FUCKED, THERE'S BEEN A MOTOR ERROR RETARD\n");
    }



    /* Construct motorDriver Task  thread */
//    Task_Params_init(&task0Params);
//    task0Params.stackSize = TASKSTACKSIZE;
//    task0Params.stack = &task0Stack;
//    task0Params.priority = 1;
//    Task_construct(&task0Struct, (Task_FuncPtr)motor_Driver, &task0Params, NULL);

    /* Construct RPMr Task  thread */
//    Task_Params_init(&task1Params);
//    task1Params.stackSize = TASKSTACKSIZE;
//    task1Params.stack = &task1Stack;
//    task1Params.priority = 1;
//    Task_construct(&task1Struct, (Task_FuncPtr)motor_GetRPM, &task1Params, NULL);



    /* Turn on user LED  */
    GPIO_write(Board_LED0, Board_LED_ON);



    /* Start BIOS */
    BIOS_start();

    return (0);
}
