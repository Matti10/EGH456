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

//#include "./startup_ccs.c"

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
#include "driverlib/fpu.h"
#include "inc/hw_ints.h"
#include <inc/hw_memmap.h>
#include "driverlib/pin_map.h"
#include "utils/uartstdio.h"
#include "utils/ustdlib.h"
#include "driverlib/sysctl.h"
#include <ti/sysbios/knl/Clock.h>
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/uart.h"
#include "inc/hw_nvic.h"
#include "inc/hw_types.h"
#include "driverlib/timer.h"
#include <ti/sysbios/family/arm/m3/Hwi.h>
#include <ti/sysbios/knl/Mailbox.h>
#include <ti/sysbios/knl/Event.h>
#include <stdint.h>
#include "inc/hw_nvic.h"
#include "inc/hw_types.h"

#define NUMMSGS         1       /* Number of messages */
#define TIMEOUT         12      /* Timeout value */
Mailbox_Params mbxParams;

uint32_t g_ui32SysClock;


// Setup Tasks
#define TASKSTACKSIZE   1024

Task_Struct task0Struct;
Char task0Stack[TASKSTACKSIZE];

Task_Struct task1Struct;
Char task1Stack[TASKSTACKSIZE];


bool arrComp(bool arr1[], bool arr2[], int len)
{
   int ii;
   for (ii = 0; ii < len; ii++) {
       if (arr1[ii] != arr2[ii]) {
           return 0;
       }
   }
   return 1;
}



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


#define MOTOR_HALL_A_PIN GPIO_PIN_3
#define MOTOR_HALL_A_PORT GPIO_PORTM_BASE
#define MOTOR_HALL_A_INT INT_GPIOM
#define MOTOR_HALL_A_PERIPH SYSCTL_PERIPH_GPIOM

#define MOTOR_HALL_B_PIN GPIO_PIN_2
#define MOTOR_HALL_B_PORT GPIO_PORTH_BASE
#define MOTOR_HALL_B_INT INT_GPIOH
#define MOTOR_HALL_B_PERIPH SYSCTL_PERIPH_GPIOH

#define MOTOR_HALL_C_PIN GPIO_PIN_2
#define MOTOR_HALL_C_PORT GPIO_PORTN_BASE
#define MOTOR_HALL_C_INT INT_GPION
#define MOTOR_HALL_C_PERIPH SYSCTL_PERIPH_GPION


#define MOTOR_RPM_TIMER_PERIPH SYSCTL_PERIPH_TIMER3
#define MOTOR_RPM_TIMER_BASE TIMER3_BASE
#define MOTOR_RPM_TIMER TIMER_A
#define MOTOR_RPM_TIMER_INT INT_TIMER3A
#define MOTOR_RPM_TIMER_TIMEOUT TIMER_TIMA_TIMEOUT
#define MOTOR_RPM_REVperRETURN 5
#define MOTOR_RPM_EDGEperREV 9


//int motor_edgeCount = 0;
//bool motor_hallStates[3];
double motor_rpm = 0.0;
uint16_t motor_input_rpm = 500;
int motor_rpm_freq;


void motor_init(void);
void motor_getHallState();
void motor_initHall();
void motor_GetRPM();
void motor_Driver();
void motor_Driver_task();
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
void motor_Driver_task(void)
{


//    UARTprintf("Starting Motor Driver Task\n\n");
    bool oldStates[3];
    bool motor_hallStates[3];
    int i;
    int motor_edgeCount = 0;

    //init motor
    motor_hallStates[0] = GPIOPinRead(MOTOR_HALL_A_PORT, MOTOR_HALL_A_PIN);
    motor_hallStates[1] = GPIOPinRead(MOTOR_HALL_B_PORT, MOTOR_HALL_B_PIN);
    motor_hallStates[2] = GPIOPinRead(MOTOR_HALL_C_PORT, MOTOR_HALL_C_PIN);
    updateMotor(motor_hallStates[0],motor_hallStates[1],motor_hallStates[2]);

    while(1) {
        for (i = 0; i < 3; i++) {
            oldStates[i] = motor_hallStates[i];
        }

        do {
            motor_hallStates[0] = GPIOPinRead(MOTOR_HALL_A_PORT, MOTOR_HALL_A_PIN);
            motor_hallStates[1] = GPIOPinRead(MOTOR_HALL_B_PORT, MOTOR_HALL_B_PIN);
            motor_hallStates[2] = GPIOPinRead(MOTOR_HALL_C_PORT, MOTOR_HALL_C_PIN);
        } while (arrComp(motor_hallStates,oldStates,3));

        updateMotor(motor_hallStates[0],motor_hallStates[1],motor_hallStates[2]); //hall states to be retreived by rpm reader task

        motor_edgeCount++; //nine positions per rotation

        if (motor_edgeCount >= (MOTOR_RPM_REVperRETURN * MOTOR_RPM_EDGEperREV)) {
            return;
        }

    }
}



//
// Motor Get RPM
//
void motor_GetRPM(void)
{
    int time1, time2;
    double debugRPM, revTime;

    double tickPeriod = 1.0/(double)g_ui32SysClock;

    while(1){
        time1 = Clock_getTicks();

        motor_Driver_task();

        time2 = Clock_getTicks();

        revTime = ((time2-time1)*tickPeriod)/MOTOR_RPM_REVperRETURN;

        //mutex
        motor_rpm = 60.0/revTime;
        debugRPM = motor_rpm;
        UARTprintf("%d rpm\n",motor_rpm);

    }
}



void motor_initHall(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOM);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOH);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);

    GPIOPinTypeGPIOInput(MOTOR_HALL_A_PORT, MOTOR_HALL_A_PIN);
    GPIOPinTypeGPIOInput(MOTOR_HALL_B_PORT, MOTOR_HALL_B_PIN);
    GPIOPinTypeGPIOInput(MOTOR_HALL_C_PORT, MOTOR_HALL_C_PIN);


}

void motor_initRPM(void) {
    // init timer and ints for rpm measurement
//    SysCtlPeripheralEnable(MOTOR_RPM_TIMER_PERIPH);
//    IntMasterEnable();
//    TimerConfigure(MOTOR_RPM_TIMER_BASE, TIMER_CFG_PERIODIC);
//    TimerLoadSet(MOTOR_RPM_TIMER_BASE, MOTOR_RPM_TIMER, motor_rpm_freq);
//    IntEnable(MOTOR_RPM_TIMER_INT);
//    TimerIntEnable(MOTOR_RPM_TIMER_BASE, MOTOR_RPM_TIMER_TIMEOUT);
//    TimerEnable(MOTOR_RPM_TIMER_BASE, MOTOR_RPM_TIMER);
}


void motor_init(void)
{
    motor_initHall();
    motor_initRPM();

    Error_Block motorError;
    initMotorLib(MOTOR_MAX_DUTY, &motorError);
    enableMotor();
    setDuty(50);
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
    UARTStdioConfig(0, 115200, g_ui32SysClock);
}



Hwi_Handle myHwi;

/*
 *  ======== main ========
 */
int main(void)
{
    Task_Params task0Params;
    Task_Params task1Params;


    FPUEnable();
    FPULazyStackingEnable();



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

    motor_init();
//    motor_initInterupts();

    g_ui32SysClock = MAP_SysCtlClockFreqSet((SYSCTL_XTAL_12MHZ |
                                                SYSCTL_OSC_MAIN |
                                                SYSCTL_USE_PLL |
                                                SYSCTL_CFG_VCO_240), 12000000);

    motor_rpm_freq = g_ui32SysClock/10;
    //
    // Initialize the UART.
    //
    ConfigureUART();

    UARTprintf("\033[2J\033[H");
    UARTprintf("\nWelcome to the 'car'\n");

    /* Construct a Mailbox Instance */
//    Mailbox_Params_init(&mbxParams);
//    mbxParams.readerEvent = evtHandle;
//    mbxParams.readerEventId = Event_Id_02;
//    Mailbox_construct(&mbxStruct,sizeof(MsgObj), 2, &mbxParams, NULL);
//    mbxHandle = Mailbox_handle(&mbxStruct);

    /* Construct motorDriver Task  thread */
//    Task_Params_init(&task0Params);
//    task0Params.stackSize = TASKSTACKSIZE;
//    task0Params.stack = &task0Stack;
//    Task_construct(&task0Struct, (Task_FuncPtr)motor_Driver_task, &task0Params, NULL);

    /* Construct RPMr Task  thread */
    Task_Params_init(&task1Params);
    task1Params.stackSize = TASKSTACKSIZE;
    task1Params.stack = &task1Stack;
    Task_construct(&task1Struct, (Task_FuncPtr)motor_GetRPM, &task1Params, NULL);


    /* Turn on user LED  */
    GPIO_write(Board_LED0, Board_LED_ON);

    /* Start BIOS */
    BIOS_start();




    return (0);
}


void motor_initInterupts()
{
        // Enable interrupts for button1 and button2 pins
//        GPIOIntEnable(MOTOR_HALL_A_PORT, GPIO_INT_PIN_3);
//        GPIOIntEnable(MOTOR_HALL_B_PORT, GPIO_INT_PIN_2);
//        GPIOIntEnable(MOTOR_HALL_C_PORT, GPIO_INT_PIN_2);
//        GPIOIntTypeSet(MOTOR_HALL_A_PORT, MOTOR_HALL_A_PIN, GPIO_RISING_EDGE);
//        IntPrioritySet(INT_GPIO/M, 0x80);


        // Register the interrupt handler
//        testFuncPtr = testFunc;
//        GPIOIntRegister(MOTOR_HALL_A_PORT,testFuncPtr);
//        GPIOIntRegister(MOTOR_HALL_B_PORT, motor_Driver);
//        GPIOIntRegister(MOTOR_HALL_C_PORT, motor_Driver);

//        IntEnable(INT_GPIOM);
//        IntEnable(INT_GPIOH);
//        IntEnable(INT_GPION);

//        IntMasterEnable();
}
