/*
 * BMI160 Sensor library
 *
 *
 *
 */

#include <ti/drivers/I2C.h>
#include "Board.h"
#include <xdc/std.h>
#include <xdc/runtime/system.h>
#include "./BMI160.h"

/* I2C Address */
#define BMI160_I2C_ADDRESS              0x47

/* BMI160 Accelerometer data registers*/
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

uint8_t txBuffer[2];
uint8_t rxBuffer[2];

I2C_Transaction transaction;

Bool writeBMI160(I2C_Handle i2c, uint8_t regAddr, uint8_t val, uint8_t count) {
    txBuffer[0] = regAddr;
    txBuffer[1] = val;

    transaction.slaveAddress = BMI160_I2C_ADDRESS;
    transaction.writeCount = count;
    transaction.writeBuf = txBuffer;
    transaction.readCount = 0;
    transaction.readBuf = NULL;
    Bool success = I2C_transfer(i2c, &transaction);

    return success;
}

Bool readBMI160(I2C_Handle i2c, uint8_t regAddr) {
    txBuffer[0] = regAddr;

    transaction.slaveAddress = BMI160_I2C_ADDRESS;
    transaction.writeCount = 1;
    transaction.writeBuf = txBuffer;
    transaction.readBuf = rxBuffer;
    transaction.readCount = 2;

    Bool success = I2C_transfer(i2c, &transaction);

    return success;
}


void initBMI160(I2C_Handle i2c) {
    // Set power to normal mode
    Bool success = writeI2C(i2c, ACC_CMD, 0x11, 2);
    if (!success) {
        System_printf("Failed to set power mode of accelerometer\n");
    } else {
        System_printf("Successfully set power mode of accelerometer to normal\n");
    }

    // configure
    success = writeI2C(i2c, ACC_CONF, 0b00101100, 2);
    if (!success) {
        System_printf("Failed to configure accelerometer\n");
    } else {
        System_printf("Successfully configured accelerometer ODR to 1600Hz\n");
    }

    // configure chip interrupts
    success = writeI2C(i2c, ACC_INT_OUT_CTRL, 0b1001, 2);
    if (!success) {
        System_printf("Failed to configure accelerometer interrupt\n");
    } else {
        System_printf("Successfully configured accelerometer interrupt\n");
    }

    // INT1 latched
    success = writeI2C(i2c, ACC_INT_LATCH, 0b00011111, 2);
    if (!success) {
        System_printf("Failed to configure accelerometer interrupt (1)\n");
    } else {
        System_printf("Successfully configured accelerometer INT1 to be latched\n");
    }

    // INT1 mapped to any motion
    success = writeI2C(i2c, ACC_INT_MAP, 0b0100, 2);
    if (!success) {
        System_printf("Failed to map accelerometer INT1\n");
    } else {
        System_printf("Successfully mapped accelerometer INT1\n");
    }

    System_flush();
}

uint8_t* readAccelerationBMI160(I2C_Handle i2c) {
//    accBMI160 acc;
//    acc.x = 0;
//    acc.y = 0;
//    acc.z = 0;

//    readBMI160(i2c, BMI160_X);
//    acc.x = (int8_t) (rxBuffer[1]<<8) + rxBuffer[0];
//    readI2CBMI160(i2c, BMI160_Y);
//    acc.y = (int8_t) (rxBuffer[1]<<8) + rxBuffer[0];
//    readI2CBMI160(i2c, BMI160_Z);
//    acc.z = (int8_t) (rxBuffer[1]<<8) + rxBuffer[0];

    uint8_t x = 0;
    uint8_t y = 0;
    uint8_t z = 0;
    readBMI160(i2c, BMI160_X);
    x = (int8_t) (rxBuffer[1]<<8) + rxBuffer[0];
    readBMI160(i2c, BMI160_Y);
    y = (int8_t) (rxBuffer[1]<<8) + rxBuffer[0];
    readBMI160(i2c, BMI160_Z);
    z = (int8_t) (rxBuffer[1]<<8) + rxBuffer[0];
    static uint8_t array[3];
    array[0] = x;
    array[1] = y;
    array[2] = z;
    return array;
}



