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
#include <xdc/runtime/system.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>

/* motor */
#include "./drivers/motorlib.h"

/* TI-RTOS Header files */
// #include <ti/drivers/EMAC.h>
#include <ti/drivers/GPIO.h>
#include <driverlib/gpio.h>
#include <ti/drivers/I2C.h>
// #include <ti/drivers/SDSPI.h>
// #include <ti/drivers/SPI.h>
// #include <ti/drivers/UART.h>
// #include <ti/drivers/USBMSCHFatFs.h>
// #include <ti/drivers/Watchdog.h>
// #include <ti/drivers/WiFi.h>

/* Board Header file */
#include "Board.h"


#include <inc/hw_memmap.h>
#include "driverlib/pin_map.h"
#include "utils/uartstdio.h"
#include "driverlib/sysctl.h"

#define TASKSTACKSIZE   1024

/* Slave address */
#define OPT3001_I2C_ADDRESS             0x47

/* Register addresses */
#define REG_RESULT                      0x00
#define REG_CONFIGURATION               0x01

uint32_t g_ui32SysClock;

Task_Struct task0Struct;
Char task0Stack[TASKSTACKSIZE];

// I2C
I2C_Handle i2c;
I2C_Params i2cParams;
I2C_Transaction i2cTransaction;

uint8_t txBuffer[3];
uint8_t rxBuffer[2];

//*****************************************************************************
//
// I2C Sensors - light sensor
//
//*****************************************************************************
bool writeI2C(uint8_t ui8Addr, uint8_t ui8Reg, uint8_t* data) {

    // swap bits?
    txBuffer[0] = ui8Addr;
    txBuffer[1] = data[0];
    txBuffer[2] = data[1];

    i2cTransaction.slaveAddress = ui8Addr;
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 3;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount = 0;

    return I2C_Transfer(i2c, &i2cTransaction);
}

bool readI2C(uint8_t ui8Addr, uint8_t ui8Reg, uint8_t* data) {
    txBuffer[0] = ui8Reg;

    i2cTransaction.slaveAddress = ui8Addr;
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount = 2;
    i2cTransaction.arg = 1;

    bool status = I2C_transfer(i2c, &i2cTransaction);
}

void sensorOpt3001ReadConvertedLux(float convertedLux){
    // This function converts the data read from the optical sensor into a lux value

    uint16_t val;
    uint16_t rawData;

    readI2C(OPT3001_I2C_ADDRESS, REG_RESULT, (uint8_t *)&val);
}

/*
 *  ======== heartBeatFxn ========
 *  Toggle the Board_LED0. The Task_sleep is determined by arg0 which
 *  is configured for the heartBeat Task instance.
 */
Void heartBeatFxn(UArg arg0, UArg arg1)
{
    //uint16_t rpm = 100;
//    enableMotor();
//    setDuty(25);
    //motor_SetRPM(rpm);
    System_printf("heart beat\n");
    float convertedLux = 0;
    sensorOpt3001ReadConvertedLux(convertedLux);
//    System_printf(convertedLux);

}

//*****************************************************************************
//
// Motor Control
//
//*****************************************************************************
#define PIN_HA PM_3
#define PIN_HB PH_2
#define PIN_HC PN_2
#define MOTOR_ACCELERATION_RPMs = 750 //Note that this is measured in RPM per Second
#define MOTOR_ESTOP_RPMs = 1000 //Note that this is measured in RPM per Second
#define MOTOR_CURR_MAX //we need to decide what this is
#define MOTOR_TEMP_MAX //we need to decide what this is
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
void motor_SetRPM(uint16_t rpm)
{
    //This needs to:
    // Set motor RPM using void setDuty(uint16_t duty);
    // Will likely need to be setup in a closed loop with motor_GetRPM()
    // Will likely need to call motor_accelerate() & motor_decelerate
    setDuty(rpm);

    bool outM, outH, outN;

    GPIOPinTypeGPIOInput(GPIO_PORTM_BASE, GPIO_PIN_3);
    GPIOPinTypeGPIOInput(GPIO_PORTH_BASE, GPIO_PIN_2);
    GPIOPinTypeGPIOInput(GPIO_PORTN_BASE, GPIO_PIN_2);

    bool z = 1;
    bool y = 0;
    bool x = 1;

    while(1)
    {
        outM = GPIOPinRead(GPIO_PORTM_BASE, GPIO_PIN_3);
        outH = GPIOPinRead(GPIO_PORTH_BASE, GPIO_PIN_2);
        outN = GPIOPinRead(GPIO_PORTN_BASE, GPIO_PIN_2);
//        z = !z;
//        y = !y;
//        x = !x;
//d updateMotor(bool Hall_a, bool Hall_b, bool Hall_c)
        updateMotor(outM,outH,outN);
        enableMotor();

    }
}

//
// Motor Get RPM
//
int motor_GetRPM()
{
    //This needs to:
    // Get motor RPM
}

//
// Accelerate the Motor
//
void motor_accelerate(bool direction)
{
    // Accelerate the motor at MOTOR_ACCELERATION_RPMs
    //  use the direction to establish if its acceleration or deceleration?
}



void
ConfigureUART(void)
{
    //
    // Enable the GPIO Peripheral used by the UART.
    //
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //
    // Enable UART0
    //
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    //
    // Configure GPIO Pins for UART mode.
    //
    MAP_GPIOPinConfigure(GPIO_PA0_U0RX);
    MAP_GPIOPinConfigure(GPIO_PA1_U0TX);
    MAP_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Initialize the UART for console I/O.
    //
    UARTStdioConfig(0, 115200, g_ui32SysClock);
}



/*
 *  ======== main ========
 */
int main(void)
{
    Task_Params taskParams;

    /* Call board init functions */
    Board_initGeneral();
    // Board_initEMAC();
    Board_initGPIO();
    Board_initI2C();
    // Board_initSDSPI();
    // Board_initSPI();
    Board_initUART();
    // Board_initUSB(Board_USBDEVICE);
    // Board_initUSBMSCHFatFs();
    // Board_initWatchdog();
    // Board_initWiFi();

//    g_ui32SysClock = MAP_SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
//                                                SYSCTL_OSC_MAIN |
//                                                SYSCTL_USE_PLL |
//                                                SYSCTL_CFG_VCO_480), 120000000);


//    UART_config();
//
//    Uartprintf("test");

    // I2C stuff - not sure if it should be moved


    /* create and open i2c port*/
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_400kHz;
    i2c = I2C_open(Board_I2C_OPT3001, &i2cParams);

    if (i2c == NULL) {
        System_abort("Error initializing I2C\n");
    } else {
        System_printf("I2C Initialized!\n");
    }

    //
    // Init Motor
    //
    Error_Block motorError;
    uint16_t pwm_period = 100;//getMotorPWMPeriod();
    initMotorLib(pwm_period, &motorError);

    /* Construct heartBeat Task  thread */
    Task_Params_init(&taskParams);
    taskParams.arg0 = 1000;
    taskParams.stackSize = TASKSTACKSIZE;
    taskParams.stack = &task0Stack;
    Task_construct(&task0Struct, (Task_FuncPtr)heartBeatFxn, &taskParams, NULL);




    /* Turn on user LED  */
    GPIO_write(Board_LED0, Board_LED_ON);

    /* Start BIOS */
    BIOS_start();

    return (0);
}
