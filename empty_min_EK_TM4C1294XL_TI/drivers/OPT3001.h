#ifndef OPT3001_H_
#define OPT3001_H-

#include <ti/drivers/I2C.h>
#include <math.h>
#include "Board.h"
#include <xdc/std.h>
#include <xdc/runtime/system.h>

void initOPT3001(I2C_Handle i2c);

uint8_t readLuxOPT3001(I2C_Handle i2c);

#endif
