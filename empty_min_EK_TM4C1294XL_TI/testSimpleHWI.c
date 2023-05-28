#include <ti/sysbios/family/arm/m3/Hwi.h>
#include <xdc/std.h>
#include <stdbool.h>
#include <xdc/runtime/Error.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/drivers/GPIO.h>
#include <ti/drivers/gpio/GPIOTiva.h>
#include <driverlib/gpio.h>
#include "driverlib/uart.h"
#include <ti/drivers/UART.h>
#include "inc/hw_ints.h"
#include <inc/hw_memmap.h>
#include "Board.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"

uint32_t g_ui32SysClock;



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

Void myHwiFxn(UArg arg)
{
    // Hardware interrupt handler code
    // ...
    int e = 0;

    UARTprintf("testJEFFPLEASE");

}
// Define the GPIO pin for the user switch
#define USER_SWITCH_GPIO_PIN    GPIO_PIN_4
#define USER_SWITCH_GPIO_PORT   GPIO_PORTF_BASE
#define USER_SWITCH_INTERRUPT   INT_GPIOF

// ...

int main(void)
{
    g_ui32SysClock = MAP_SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
                                                   SYSCTL_OSC_MAIN |
                                                   SYSCTL_USE_PLL |
                                                   SYSCTL_CFG_VCO_480), 120000000);

    // Initialize the board and necessary peripherals
    Board_initGeneral();
    Board_initGPIO ();
;
    // Configure the GPIO pin for the user switch as input with pull-up resistor
    GPIOPinTypeGPIOInput(USER_SWITCH_GPIO_PORT, USER_SWITCH_GPIO_PIN);

    // Create the HWI object and associate it with the interrupt number and handler function
    Hwi_Params hwiParams;
    Error_Block eb;
    Error_init(&eb);
    Hwi_Params_init(&hwiParams);
    Hwi_Handle hwiHandle = Hwi_create(USER_SWITCH_INTERRUPT, myHwiFxn, &hwiParams, &eb);

    if (hwiHandle == NULL) {
        // Handle Hwi_create() error
        // ...
    }

    // Start the TI-RTOS kernel
    BIOS_start();

    return 0;
}
