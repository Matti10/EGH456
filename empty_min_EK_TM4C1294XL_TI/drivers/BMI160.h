#ifndef BMI160_H_
#define BMI160_H_

#include <ti/drivers/I2C.h>
#include "Board.h"
#include <xdc/std.h>
#include <xdc/runtime/system.h>

typedef struct _accBMI160 {
    uint8_t x;
    uint8_t y;
    uint8_t z;
} accBMI160;

void initBMI160(I2C_Handle i2c);


uint8_t* readAccelerationBMI160(I2C_Handle i2c);

#endif
