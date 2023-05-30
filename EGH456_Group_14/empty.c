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
#include <stdbool.h>
/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Error.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/family/arm/m3/Hwi.h>
#include <ti/sysbios/family/arm/m3/Timer.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/gates/GateHwi.h>
#include <ti/sysbios/knl/Mailbox.h>

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

/* DriverLib Header files */
#include <driverlib/GPIO.h>
#include <driverlib/sysctl.h>
#include <driverlib/timer.h>

/* inc Headers */
#include <inc/hw_memmap.h>
#include <inc/hw_ints.h>
#include <inc/hw_timer.h>

/* Board Header file */
#include "Board.h"

/* Sensor Header files */
#include "./drivers/OPT3001.h"
#include "./drivers/BMI160.h"

/* Motor Header Files */
#include "./drivers/motorLib/motorlib.h"

/* Task Setup */
#define TASKSTACKSIZE   1024

Task_Struct task0Struct;
Char task0Stack[TASKSTACKSIZE];
Task_Struct taskMotorTester_Struct;
Char taskMotorTester_Stack[TASKSTACKSIZE];

/* I2C Setup */
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
uint32_t ui32SysClock;

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
#define MOTOR_HALL_A_INT GPIO_INT_PIN_3
#define MOTOR_HALL_A_ISR INT_GPIOM_TM4C129

#define MOTOR_HALL_B_PIN GPIO_PIN_2
#define MOTOR_HALL_B_PORT GPIO_PORTH_BASE
#define MOTOR_HALL_B_INT GPIO_INT_PIN_2
#define MOTOR_HALL_B_ISR INT_GPIOH_TM4C129

#define MOTOR_HALL_C_PIN GPIO_PIN_2
#define MOTOR_HALL_C_PORT GPIO_PORTN_BASE
#define MOTOR_HALL_C_INT GPIO_INT_PIN_2
#define MOTOR_HALL_C_ISR INT_GPION_TM4C129

#define MOTOR_RPM_CLOCK_PERIOD 10 //1000 micro secs per tick

#define MOTOR_RPM_CLOCK_PERIODS_PER_MIN 60//000

#define MOTOR_POLES 24

#define NUMMSGS         1


/* Motor Funcs */
void motor_init(void);
void motor_driver();
void motor_initISR();
void motor_start();
void motor_initHall(void);
void motor_initRPM();
void motor_controller();
void motor_initRPM();
void motor_controller();
void motor_tester();
void motor_info();

typedef struct rpm_MsgObj {
    Int         rpm;
} rpm_MsgObj, *Msg;

typedef struct motor_data_MsgObj {
    Int         rpm;
    Int         duty;
} motor_data_MsgObj, *data_Msg;

/* Motor Globals */
bool motor_hallStates[3];
int motor_edgeCount = 0;
rpm_MsgObj rpm_msg;
motor_data_MsgObj motor_info_msg;

/* ONLY USED IN MOTOR CONTORL ISR!!!! */
int motor_rpmEdgeCount_1 = 0;
int motor_rpmEdgeCount_2 = 0;
int motor_rpm = 0;
int motor_input_rpm = 300;
int duty = 50;


Hwi_Handle motor_Hwi_A, motor_Hwi_B, motor_Hwi_C;
Clock_Handle motor_rpm_Timer;
Clock_Handle motor_tester_Timer;
GateHwi_Handle motor_GateHwi;
Mailbox_Handle rpm_mbxHandle;
Mailbox_Handle motor_info_mbxHandle;
Mailbox_Params rpm_mbxParams;
Mailbox_Struct rpm_mbxStruct;
Mailbox_Struct motor_info_mbxStruct;


void motor_initHall(void)
{

    GPIOPinTypeGPIOInput(MOTOR_HALL_A_PORT, MOTOR_HALL_A_PIN);
    GPIOIntTypeSet(MOTOR_HALL_A_PORT, MOTOR_HALL_A_PIN,GPIO_HIGH_LEVEL);
    GPIOIntEnable(MOTOR_HALL_A_PORT,MOTOR_HALL_A_INT);

    GPIOPinTypeGPIOInput(MOTOR_HALL_B_PORT, MOTOR_HALL_B_PIN);
    GPIOIntTypeSet(MOTOR_HALL_B_PORT, MOTOR_HALL_B_PIN,GPIO_HIGH_LEVEL);
    GPIOIntEnable(MOTOR_HALL_B_PORT,MOTOR_HALL_B_INT);

    GPIOPinTypeGPIOInput(MOTOR_HALL_C_PORT, MOTOR_HALL_C_PIN);
    GPIOIntTypeSet(MOTOR_HALL_C_PORT, MOTOR_HALL_C_PIN,GPIO_HIGH_LEVEL);
    GPIOIntEnable(MOTOR_HALL_C_PORT,MOTOR_HALL_C_INT);

}

void motor_start(){
    enableMotor();
    setDuty(duty);

//    motor_hallStates[0] = GPIOPinRead(MOTOR_HALL_A_PORT, MOTOR_HALL_A_PIN);
//    motor_hallStates[1] = GPIOPinRead(MOTOR_HALL_B_PORT, MOTOR_HALL_B_PIN);
//    motor_hallStates[2] = GPIOPinRead(MOTOR_HALL_C_PORT, MOTOR_HALL_C_PIN);
//    updateMotor(motor_hallStates[0],motor_hallStates[1],motor_hallStates[2]);
}


void motor_manageISRStates(UArg isr){
    /* Do Rising Edge Int's job for it */
    if(isr == MOTOR_HALL_A_ISR){
        Hwi_disableInterrupt(MOTOR_HALL_A_ISR);
        Hwi_enableInterrupt(MOTOR_HALL_B_ISR);
    }
    if(isr == MOTOR_HALL_B_ISR){
        Hwi_disableInterrupt(MOTOR_HALL_B_ISR);
        Hwi_enableInterrupt(MOTOR_HALL_C_ISR);
    }
    if(isr == MOTOR_HALL_C_ISR){
        Hwi_disableInterrupt(MOTOR_HALL_C_ISR);
        Hwi_enableInterrupt(MOTOR_HALL_A_ISR);
    }

    motor_driver();
}

void motor_clearISR(){
    Hwi_clearInterrupt(MOTOR_HALL_A_ISR);
    Hwi_clearInterrupt(MOTOR_HALL_B_ISR);
    Hwi_clearInterrupt(MOTOR_HALL_C_ISR);
}

void motor_disableISR(){
    Hwi_disableInterrupt(MOTOR_HALL_A_ISR);
    Hwi_disableInterrupt(MOTOR_HALL_B_ISR);
    Hwi_disableInterrupt(MOTOR_HALL_C_ISR);
}

void motor_enableISR(){
    Hwi_enableInterrupt(MOTOR_HALL_A_ISR);
    Hwi_enableInterrupt(MOTOR_HALL_B_ISR);
    Hwi_enableInterrupt(MOTOR_HALL_C_ISR);
}

void motor_initISR(){
    /* Create Motor Driving HWI's */
    Error_Block motor_HwiError;
    Hwi_Params motor_HwiParams;

    Hwi_Params_init(&motor_HwiParams);
    motor_HwiParams.maskSetting = Hwi_MaskingOption_SELF;
    motor_HwiParams.priority = 250;

    motor_HwiParams.arg = MOTOR_HALL_A_ISR;
    motor_Hwi_A = Hwi_create(MOTOR_HALL_A_ISR,(Hwi_FuncPtr)motor_manageISRStates,&motor_HwiParams,&motor_HwiError);
    errorCheck(&motor_HwiError);

    motor_HwiParams.arg = MOTOR_HALL_B_ISR;
    motor_Hwi_B = Hwi_create(MOTOR_HALL_B_ISR,(Hwi_FuncPtr)motor_manageISRStates,&motor_HwiParams,&motor_HwiError);
    errorCheck(&motor_HwiError);

    motor_HwiParams.arg = MOTOR_HALL_C_ISR;
    motor_Hwi_C = Hwi_create(MOTOR_HALL_C_ISR,(Hwi_FuncPtr)motor_manageISRStates,&motor_HwiParams,&motor_HwiError);
    errorCheck(&motor_HwiError);

}

void motor_sendRPM(int rpm){
    if(rpm_msg.rpm != rpm)
    {
        rpm_msg.rpm = rpm;
        Mailbox_post(rpm_mbxHandle, &rpm_msg, BIOS_NO_WAIT);
    }
}


void motor_initMailbox(){

    // init rpm setter mbx
    Error_Block rpm_mbxError;

    Mailbox_Params_init(&rpm_mbxParams);
    Mailbox_construct(&rpm_mbxStruct,sizeof(rpm_MsgObj), 5, &rpm_mbxParams, &rpm_mbxError);
    rpm_mbxHandle = Mailbox_handle(&rpm_mbxStruct);
    errorCheck(&rpm_mbxError);

    //init motor info getter mbx
    Mailbox_construct(&rpm_mbxStruct,sizeof(motor_data_MsgObj), 5, &rpm_mbxParams, &rpm_mbxError);
    motor_info_mbxHandle = Mailbox_handle(&rpm_mbxStruct);
    errorCheck(&rpm_mbxError);
}

void motor_initGateHwi(){

    Error_Block motor_GateHwiError;
    GateHwi_Params motor_GateHwiParams;

    GateHwi_Params_init(&motor_GateHwiParams);
    motor_GateHwi = GateHwi_create(&motor_GateHwiParams, &motor_GateHwiError);

    errorCheck(&motor_GateHwiError);
}

void motor_initRPM(){


    Error_Block rpm_ClockError;
    Clock_Params rpm_ClockParams;

    Clock_Params_init(&rpm_ClockParams);
    rpm_ClockParams.period = MOTOR_RPM_CLOCK_PERIOD;
    rpm_ClockParams.startFlag = TRUE;
    motor_rpm_Timer = Clock_create((Clock_FuncPtr)motor_controller, MOTOR_RPM_CLOCK_PERIOD, &rpm_ClockParams, &rpm_ClockError);


    errorCheck(&rpm_ClockError);

}


void motor_driver()
{
//    motor_clearISR();

    motor_hallStates[0] = GPIOPinRead(MOTOR_HALL_A_PORT, MOTOR_HALL_A_PIN);
    motor_hallStates[1] = GPIOPinRead(MOTOR_HALL_B_PORT, MOTOR_HALL_B_PIN);
    motor_hallStates[2] = GPIOPinRead(MOTOR_HALL_C_PORT, MOTOR_HALL_C_PIN);
    updateMotor(motor_hallStates[0],motor_hallStates[1],motor_hallStates[2]);


    IArg key = GateHwi_enter(motor_GateHwi);
    motor_edgeCount++;
    GateHwi_leave(motor_GateHwi,key);
}

void motor_controller(){

    //set old edge count
    motor_rpmEdgeCount_1 = motor_rpmEdgeCount_2;

    //get current edge count from counter
    IArg key = GateHwi_enter(motor_GateHwi);
    motor_rpmEdgeCount_2 = motor_edgeCount;
    GateHwi_leave(motor_GateHwi,key);
    int debugRPM = motor_rpm;
    int debugInRPM = motor_input_rpm;
    int debugDuty = duty;

    motor_rpm = (int)(((((double)motor_rpmEdgeCount_2-(double)motor_rpmEdgeCount_1))/(double)MOTOR_POLES)*(double)MOTOR_RPM_CLOCK_PERIODS_PER_MIN);

    if(Mailbox_pend(rpm_mbxHandle, &rpm_msg, BIOS_NO_WAIT)){
        motor_input_rpm = rpm_msg.rpm;
    }

    if (motor_input_rpm == 0)
    {
        disableMotor();
    }
    if (motor_input_rpm > 0 && motor_rpm < 10)
    {
        enableMotor();
    }
    if (motor_rpm > (double)motor_input_rpm && duty > 0)
    {
        duty--;
    }
    else if (motor_rpm < (double)motor_input_rpm && duty < MOTOR_MAX_DUTY)
    {
        duty++;
    }
    setDuty(duty);
    motor_info_msg.rpm = motor_rpm;
    motor_info_msg.duty = duty;
    Mailbox_post(motor_info_mbxHandle, &motor_info_msg, BIOS_NO_WAIT);

}
void motor_initTester(){
    Error_Block tester_ClockError;
    Clock_Params tester_ClockParams;

    int tester_period = MOTOR_RPM_CLOCK_PERIOD*1000;
    Clock_Params_init(&tester_ClockParams);
    tester_ClockParams.period = tester_period;
    tester_ClockParams.startFlag = TRUE;
    motor_tester_Timer= Clock_create((Clock_FuncPtr)motor_tester, tester_period, &tester_ClockParams, &tester_ClockError);


    errorCheck(&tester_ClockError);
}

/* MOtor Tester Vars */
int currDuty= -1;
int currRPM = -1;
int inputRPM = -1;
int rpms[10] = {10, 1000, 900, 30, 50, 2000, 0, 5, 550, 5000};
int i = 0;
void motor_tester(){
    inputRPM = rpms[i];
    motor_sendRPM(inputRPM);

    i++;
}

void motor_info(){
    while(1){

        if(Mailbox_pend(motor_info_mbxHandle, &motor_info_msg, BIOS_NO_WAIT)){
            currDuty = motor_info_msg.duty;
            currRPM = motor_info_msg.rpm;
        }

        System_printf("Input RPM: %d | Motor RPM: %d | Motor Duty: %d\n", inputRPM, currRPM, currDuty);
        System_flush();
        Task_sleep(1000);
    }


}

void motor_init(void){
    motor_initHall();
    motor_initISR();
    motor_initRPM();
    motor_initGateHwi();
    motor_initTester();
    motor_initMailbox();

    Error_Block motorError;
    bool initSuccess = initMotorLib(MOTOR_MAX_DUTY, &motorError);
    errorCheck(&motorError);


    motor_start();

}

void errorCheck(Error_Block *eb){
    if (Error_check(&eb)) {
        // handle the error
        GPIO_toggle(Board_LED1);
    }
}

/*
 *  ======== main ========
 */
int main(void)
{
    Task_Params taskParams;

    ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN | SYSCTL_USE_PLL
    | SYSCTL_CFG_VCO_480), 120000000);

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


    motor_init();

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
    taskParams.priority = 1;
    Task_construct(&task0Struct, (Task_FuncPtr)heartBeatFxn, &taskParams, NULL);

    taskParams.stack = &taskMotorTester_Stack;
    taskParams.priority = 2;
    Task_construct(&taskMotorTester_Struct, (Task_FuncPtr) motor_info, &taskParams, NULL);

    System_printf("Starting the 'Car'\n");
    /* SysMin will only print to the console when you call flush or exit */
    System_flush();

    /* Start BIOS */
    BIOS_start();

    return (0);
}
