/*
 *  OPT3001 Ambient Light Sensor
 *
 */

#include <ti/drivers/I2C.h>
#include <math.h>
#include "Board.h"
#include <xdc/std.h>
#include <xdc/runtime/system.h>

/* I2C Address */
#define OPT3001_I2C_ADDRESS     0x47

/* Register Addresses */
#define REG_RESULT              0x00
#define REG_CONFIG              0x01

I2C_Transaction i2cTransaction;

uint8_t txBuffer[3];
uint8_t rxBuffer[2];

void writeI2C(I2C_Handle i2c, uint8_t ui8Addr, uint8_t wBuffer[3], uint8_t count) {


    I2C_Transaction transaction;

    transaction.slaveAddress = ui8Addr;
    transaction.writeCount = count;
    transaction.writeBuf = wBuffer;
    transaction.readCount = 0;
    transaction.readBuf = NULL;

    Bool success = I2C_transfer(i2c, &transaction);

    if (!success) {
        System_printf("I2C write transaction failed\n");
    } else {
        System_printf("I2C write transaction succeeded\n");
    }
}

void readI2C(I2C_Handle i2c, uint8_t ui8Addr, uint8_t ui8Reg) {
    uint8_t writeBuffer[1];
    writeBuffer[0] = ui8Reg;
    i2cTransaction.slaveAddress = ui8Addr;
    i2cTransaction.writeBuf = writeBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount = 2;

    Bool status = I2C_transfer(i2c, &i2cTransaction);


}

void initOPT3001(I2C_Handle i2c) {
    txBuffer[0] = REG_CONFIG;
    txBuffer[1] = 0xC4;
    txBuffer[2] = 0x10;
    writeI2C(i2c, OPT3001_I2C_ADDRESS, txBuffer, 3);
}

uint8_t readLuxOPT3001(I2C_Handle i2c) {
    float convertedLux = 0;
    uint16_t rawData = 0;

    readI2C(i2c, OPT3001_I2C_ADDRESS, REG_RESULT);
    rawData = (rxBuffer[0] << 8) | (rxBuffer[1] >> 8 &0xFF);

    uint16_t e, m;
    m = rawData & 0x0FFF;
    e = (rawData & 0xF000) >> 12;
    convertedLux = m * (0.01 * exp2(e));

    return (uint8_t)convertedLux;
}







