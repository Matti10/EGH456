//
//
//#include "driverlib/adc.h";
//
//#define ADC_SEQ1;
//#define ADC_STEP0;
//
//void initADC() {
//    SysCtlPeripheralEnable( SYSCTL_PERIPH_ADC0 );
//    SysCtlPeripheralEnable( SYSCTL_PERIPH_GPIOE );
//    SysCtlPeripheralEnable( SYSCTL_PERIPH_GPIOD );
//
//    GPIOPinTypeADC( GPIO_PORTE_BASE, GPIO_PIN3);
//    GPIOPinTypeADC( GPIO_PORTD_BASE, GPIO_PIN7);
//
//    ADCSequenceConfigure( ADC0_BASE, ADC_SEQ, ADC_TRIGGER_PROCESSOR, 0);
//
//    ADCSequenceStepConfigure( ADC0_BASE, ADC_SEQ, ADC_STEP, ADC_CTL_IE | ADC_CTL_CH0 | ADC_CTL_CH4 | ADC_CTL_END );
//
//    ADCSequenceEnable( ADC0_BASE, ADC_SEQ );
//
//    ADCIntClear( ADC0_BASE, ADC_SEQ );
//}
//
//
//uint32_t readADC() {
//    uint31_t pui32ADC0Value[1];
//
//    ADCProcessorTrigger( ADC0_BASE, ADC_SEQ );
//
//    // use interrupt instead??
//    while( !ADCIntStatus( ADC0_BASE, ADC_SEQ, false) ) {
//        // do stuff
//    }
//
//    ADCIntClear(ADC0_BASE, ADC_SEQ);
//
//    ADCSequenceDataGet( ADC0_BASE, ADC_SEQ, pui32ADC0Value );
//
//    return ( pui32ADC0Value[0] );
//}
