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
 *  ======== empty.c ========
 */
/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>

/* TI-RTOS Header files */
// #include <ti/drivers/EMAC.h>
#include <ti/drivers/GPIO.h>
#include <ti/drivers/I2C.h>
// #include <ti/drivers/SDSPI.h>
// #include <ti/drivers/SPI.h>
// #include <ti/drivers/UART.h>
// #include <ti/drivers/USBMSCHFatFs.h>
// #include <ti/drivers/Watchdog.h>
// #include <ti/drivers/WiFi.h>

/* Board Header file */
#include "Board.h"

/* Sensor Header files */
#include "./drivers/OPT3001.h"
#include "./drivers/BMI160.h"

#define TASKSTACKSIZE   512

Task_Struct task0Struct;
Char task0Stack[TASKSTACKSIZE];

I2C_Handle i2c;
I2C_Params i2cParams;

/* BMI160 Accelerometer data registers*/
#define BMI160_I2C_ADDRESS              0x69
#define BMI160_X                        0x12
#define BMI160_Y                        0x14
#define BMI160_Z                        0x16

/* BMI160 CONFIG REGISTERS */
#define ACC_CMD                         0x7E
#define ACC_CONF                        0x40
#define ACC_RANGE                       0x41
#define ACC_INT_OUT_CTRL                0x53
#define ACC_INT_LATCH                   0x54
#define ACC_INT_MAP                     0x55

uint8_t txBufferAcc[2];
uint8_t rxBufferAcc[2];

void initI2CBMI160() {
    // reusable i2c transaction
    I2C_Transaction transaction;

    // set power mode to normal
    txBufferAcc[0] = ACC_CMD;
    txBufferAcc[1] = 0x11; // set data output rate to 1600Hz;
    transaction.slaveAddress = BMI160_I2C_ADDRESS;
    transaction.writeCount = 2;
    transaction.writeBuf = txBufferAcc;
    transaction.readCount = 0;
    transaction.readBuf = NULL;
    bool success = I2C_transfer(i2c, &transaction);
    if (!success) {
        System_printf("Failed to set power mode of accelerometer\n");
    } else {
        System_printf("Successfully set power mode of accelerometer to normal\n");
    }

    // --- configure
    txBufferAcc[0] = ACC_CONF;
    txBufferAcc[1] = 0b00101100; // set data output rate to 1600Hz;
    transaction.slaveAddress = BMI160_I2C_ADDRESS;
    transaction.writeCount = 2;
    transaction.writeBuf = txBufferAcc;
    transaction.readCount = 0;
    transaction.readBuf = NULL;
    success = I2C_transfer(i2c, &transaction);
    if (!success) {
        System_printf("Failed to configure accelerometer\n");
    } else {
        System_printf("Successfully configured accelerometer ODR to 1600Hz\n");
    }

    // --- configure chip interrupts
    txBufferAcc[0] = ACC_INT_OUT_CTRL;
    txBufferAcc[1] = 0b1001;
    transaction.slaveAddress = BMI160_I2C_ADDRESS;
    transaction.writeCount = 2;
    transaction.writeBuf = txBufferAcc;
    transaction.readCount = 0;
    transaction.readBuf = NULL;
    success = I2C_transfer(i2c, &transaction);
    if (!success) {
        System_printf("Failed to configure accelerometer interrupt\n");
    } else {
        System_printf("Successfully configured accelerometer interrupt\n");
    }

    // INT1 latched
    txBufferAcc[0] = ACC_INT_LATCH;
    txBufferAcc[1] = 0b00011111;
    transaction.slaveAddress = BMI160_I2C_ADDRESS;
    transaction.writeCount = 2;
    transaction.writeBuf = txBufferAcc;
    transaction.readCount = 0;
    transaction.readBuf = NULL;
    success = I2C_transfer(i2c, &transaction);
    if (!success) {
        System_printf("Failed to configure accelerometer interrupt (1)\n");
    } else {
        System_printf("Successfully configured accelerometer INT1 to be latched\n");
    }

    // INT1 mapped to any motion
    txBufferAcc[0] = ACC_INT_MAP;
    txBufferAcc[1] = 0b0100;
    transaction.slaveAddress = BMI160_I2C_ADDRESS;
    transaction.writeCount = 2;
    transaction.writeBuf = txBufferAcc;
    transaction.readCount = 0;
    transaction.readBuf = NULL;
    success = I2C_transfer(i2c, &transaction);
    if (!success) {
        System_printf("Failed to map accelerometer INT1\n");
    } else {
        System_printf("Successfully mapped accelerometer INT1\n");
    }

    System_flush();

}

void readI2CBMI160(I2C_Handle handle, uint8_t ui8Reg) {
    I2C_Transaction transaction;
    txBufferAcc[0] = ui8Reg;

    transaction.slaveAddress = BMI160_I2C_ADDRESS;
    transaction.writeBuf = txBufferAcc;
    transaction.writeCount = 1;
    transaction.readBuf = rxBufferAcc;
    transaction.readCount = 2;

    Bool status = I2C_transfer(handle, &transaction);

}

/*
 *  ======== heartBeatFxn ========
 *  Toggle the Board_LED0. The Task_sleep is determined by arg0 which
 *  is configured for the heartBeat Task instance.
 */
Void heartBeatFxn(UArg arg0, UArg arg1)
{
    // init sensors
    initI2CBMI160();
    initOPT3001(i2c);

    uint8_t convertedLux = 0;
    uint8_t *acceleration;
    while (1) {
//        Task_sleep((unsigned int)arg0);
//        GPIO_toggle(Board_LED0);
        convertedLux = readLuxOPT3001(i2c);
//        acceleration = readBMI160(i2c);

//        System_printf("Lux: %d --- Acceleration(x,y,z): %d, %d, %d\n", convertedLux, acceleration[0], acceleration[1], acceleration[2]);

        uint8_t x = 0;
        uint8_t y = 0;
        uint8_t z = 0;
        readI2CBMI160(i2c, BMI160_X);
        x = (int16_t) (rxBufferAcc[1]<<8) + rxBufferAcc[0];
        readI2CBMI160(i2c, BMI160_Y);
        y = (int16_t) (rxBufferAcc[1]<<8) + rxBufferAcc[0];
        readI2CBMI160(i2c, BMI160_Z);
        z = (int16_t) (rxBufferAcc[1]<<8) + rxBufferAcc[0];
        System_printf("Lux: %d; Raw Acc x: %d y: %d z: %d\n", convertedLux, x, y, z);
//        System_printf("Lux: %d\n", convertedLux);
        System_flush();

    }
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
    // Board_initUART();
    // Board_initUSB(Board_USBDEVICE);
    // Board_initUSBMSCHFatFs();
    // Board_initWatchdog();
    // Board_initWiFi();

    /* create and open i2c port*/
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_400kHz;
    i2c = I2C_open(Board_I2C_OPT3001, &i2cParams);
    if (i2c == NULL) {
        System_abort("Error initializing I2C\n");
    } else {
        System_printf("I2C Initialized!\n");
        System_flush();
    }

    /* Construct heartBeat Task  thread */
    Task_Params_init(&taskParams);
    taskParams.arg0 = 1000;
    taskParams.stackSize = TASKSTACKSIZE;
    taskParams.stack = &task0Stack;
    Task_construct(&task0Struct, (Task_FuncPtr)heartBeatFxn, &taskParams, NULL);

     /* Turn on user LED */
    GPIO_write(Board_LED0, Board_LED_ON);

    System_printf("Starting the example\nSystem provider is set to SysMin. "
                  "Halt the target to view any SysMin contents in ROV.\n");
    /* SysMin will only print to the console when you call flush or exit */
    System_flush();

    /* Start BIOS */
    BIOS_start();

    return (0);
}
