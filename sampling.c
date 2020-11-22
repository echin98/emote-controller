/*
 * sampling.c
 *
 *  EPWM4A used to trigger ADC channels A2, B2, C2, which are the current sensors
 *  Created on: Nov 21, 2020
 *      Author: ericc
 */
#include "sampling.h"

uint16_t adcAResults[RESULTS_BUFFER_SIZE];   // Buffer for results
uint16_t adcBResults[RESULTS_BUFFER_SIZE];
uint16_t adcCResults[RESULTS_BUFFER_SIZE];
volatile uint16_t indexA2;                              // Index into result buffer
volatile uint16_t bufferFullA2;                // Flag to indicate buffer is full
volatile uint16_t indexB2;                              // Index into result buffer
volatile uint16_t bufferFullB2;                // Flag to indicate buffer is full
volatile uint16_t indexC2;                              // Index into result buffer
volatile uint16_t bufferFullC2;                // Flag to indicate buffer is full

volatile uint16_t test_vals = 0;

void s_resetBufferA2(){indexA2 = 0; bufferFullA2 = 0;}
void s_resetBufferB2(){indexB2 = 0; bufferFullB2 = 0;}
void s_resetBufferC2(){indexC2 = 0; bufferFullC2 = 0;}

bool s_setBufferFullA2(bool val){bufferFullA2=val; return;}
bool s_setBufferFullB2(bool val){bufferFullB2=val; return;}
bool s_setBufferFullC2(bool val){bufferFullC2=val; return;}

bool s_getBufferFullA2(){return bufferFullA2;}
bool s_getBufferFullB2(){return bufferFullB2;}
bool s_getBufferFullC2(){return bufferFullC2;}

uint16_t* s_getA2Buffer(void){return adcAResults;}
uint16_t* s_getB2Buffer(void){return adcBResults;}
uint16_t* s_getC2Buffer(void){return adcCResults;}

void sampling_init(void)
{
    sampling_initADCs();
    sampling_initEPWM();
    sampling_initADCSOCs();
    int index;
    for(index = 0; index < RESULTS_BUFFER_SIZE; index++)
    {
        adcAResults[index] = 7;
        adcBResults[index] = 0;
        adcCResults[index] = 0;
    }
    //
    // Initialize results buffer
    //


    indexA2 = 0;
    bufferFullA2 = 0;
    indexB2 = 0;
    bufferFullB2 = 0;
    indexC2 = 0;
    bufferFullC2 = 0;

    //
    // Enable ADC interrupt for all three ADC's
    //
    Interrupt_enable(INT_ADCA2);
    Interrupt_enable(INT_ADCB2);
    Interrupt_enable(INT_ADCC2);
}

void sampling_initEPWM(void)
{
    //
    // Disable SOCA/B
    //
    EPWM_disableADCTrigger(EPWM4_BASE, EPWM_SOC_A);

    //
    // Configure the SOC to occur on the first up-count event
    //
    EPWM_setADCTriggerSource(EPWM4_BASE, EPWM_SOC_A, EPWM_SOC_TBCTR_U_CMPA);
    EPWM_setADCTriggerEventPrescale(EPWM4_BASE, EPWM_SOC_A, 1);

    //
    // Set the compare A AND B values to 256 and the period to 512
    //

    EPWM_setCounterCompareValue(EPWM4_BASE, EPWM_COUNTER_COMPARE_A, 256); //50% duty cycle
    EPWM_setTimeBasePeriod(EPWM4_BASE, 512); // ~20 kHz

    //
    // Freeze the counter
    //
    EPWM_setTimeBaseCounterMode(EPWM4_BASE, EPWM_COUNTER_MODE_STOP_FREEZE);
}


void sampling_initADCSOCs(void)
{
    //
    // Configure SOCs of ADCA
    // - SOC2 will convert pin A2, B2, C2.
    // - Triggered by EPWM4A.
    // - For 12-bit resolution, a sampling window of 15 (75 ns at a 200MHz
    //   SYSCLK rate) will be used.  For 16-bit resolution, a sampling window
    //   of 64 (320 ns at a 200MHz SYSCLK rate) will be used.
    //
#if(EX_ADC_RESOLUTION == 12)
    ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER2, ADC_TRIGGER_EPWM4_SOCA,
                        ADC_CH_ADCIN2, 15);
    ADC_setupSOC(ADCB_BASE, ADC_SOC_NUMBER2, ADC_TRIGGER_EPWM4_SOCA, //Find out some definitions in this function
                        ADC_CH_ADCIN2, 15);
    ADC_setupSOC(ADCC_BASE, ADC_SOC_NUMBER2, ADC_TRIGGER_EPWM4_SOCA,
                        ADC_CH_ADCIN2, 15);
#elif(EX_ADC_RESOLUTION == 16)
    ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER2, ADC_TRIGGER_EPWM4_SOCA,
                           ADC_CH_ADCIN2, 64);
    ADC_setupSOC(ADCB_BASE, ADC_SOC_NUMBER2, ADC_TRIGGER_EPWM4_SOCA, //Find out some definitions in this function
                        ADC_CH_ADCIN2, 64);
    ADC_setupSOC(ADCC_BASE, ADC_SOC_NUMBER2, ADC_TRIGGER_EPWM4_SOCA,
                        ADC_CH_ADCIN2, 64);
#endif

    //
    // Set SOC2 to set the interrupt 2 flag. Enable the interrupt and make
    // sure its flag is cleared.
    //

    ADC_setInterruptSource(ADCA_BASE, ADC_INT_NUMBER2, ADC_SOC_NUMBER2);
    ADC_enableInterrupt(ADCA_BASE, ADC_INT_NUMBER2);
    ADC_clearInterruptStatus(ADCA_BASE, ADC_INT_NUMBER2);

    ADC_setInterruptSource(ADCB_BASE, ADC_INT_NUMBER2, ADC_SOC_NUMBER2);
    ADC_enableInterrupt(ADCB_BASE, ADC_INT_NUMBER2);
    ADC_clearInterruptStatus(ADCB_BASE, ADC_INT_NUMBER2);

    ADC_setInterruptSource(ADCC_BASE, ADC_INT_NUMBER2, ADC_SOC_NUMBER2);
    ADC_enableInterrupt(ADCC_BASE, ADC_INT_NUMBER2);
    ADC_clearInterruptStatus(ADCC_BASE, ADC_INT_NUMBER2);


}

//
// Function to configure and power up ADCs A and B.
//
void sampling_initADCs(void)
{
    //
    // Set ADCCLK divider to /4
    //
    ADC_setPrescaler(ADCA_BASE, ADC_CLK_DIV_4_0);
    ADC_setPrescaler(ADCB_BASE, ADC_CLK_DIV_4_0);
    ADC_setPrescaler(ADCC_BASE, ADC_CLK_DIV_4_0);

    //
    // Set resolution and signal mode (see #defines above) and load
    // corresponding trims.
    //
#if(EX_ADC_RESOLUTION == 12)
    ADC_setMode(ADCA_BASE, ADC_RESOLUTION_12BIT, ADC_MODE_SINGLE_ENDED);
    ADC_setMode(ADCB_BASE, ADC_RESOLUTION_12BIT, ADC_MODE_SINGLE_ENDED);
    ADC_setMode(ADCC_BASE, ADC_RESOLUTION_12BIT, ADC_MODE_SINGLE_ENDED);
#elif(EX_ADC_RESOLUTION == 16)
    ADC_setMode(ADCA_BASE, ADC_RESOLUTION_16BIT, ADC_MODE_DIFFERENTIAL);
    ADC_setMode(ADCB_BASE, ADC_RESOLUTION_16BIT, ADC_MODE_DIFFERENTIAL);
    ADC_setMode(ADCC_BASE, ADC_RESOLUTION_16BIT, ADC_MODE_DIFFERENTIAL);
#endif

    //
    // Set pulse positions to late
    //
    ADC_setInterruptPulseMode(ADCA_BASE, ADC_PULSE_END_OF_CONV);
    ADC_setInterruptPulseMode(ADCB_BASE, ADC_PULSE_END_OF_CONV);
    ADC_setInterruptPulseMode(ADCC_BASE, ADC_PULSE_END_OF_CONV);

    //
    // Power up the ADCs and then delay for 1 ms
    //
    ADC_enableConverter(ADCA_BASE);
    ADC_enableConverter(ADCB_BASE);
    ADC_enableConverter(ADCC_BASE);

    DEVICE_DELAY_US(1000);
}


/*------------------------ISRS-----------------------------------*/

//
// ADC A Interrupt 1 ISR
//
__interrupt void adcA2ISR(void)
{

    //
    // Set the bufferFull flag if the buffer is full
    //
    if(RESULTS_BUFFER_SIZE <= indexA2)
    {
        bufferFullA2 = 1;
    }
    else
    {
        //
        // Add the latest result to the buffer
        //
        adcAResults[indexA2++] = ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER2);
    }

    //
    // Clear the interrupt flag
    //
    ADC_clearInterruptStatus(ADCA_BASE, ADC_INT_NUMBER2);

    //
    // Check if overflow has occurred
    //
    if(true == ADC_getInterruptOverflowStatus(ADCA_BASE, ADC_INT_NUMBER2))
    {
        ADC_clearInterruptOverflowStatus(ADCA_BASE, ADC_INT_NUMBER2);
        ADC_clearInterruptStatus(ADCA_BASE, ADC_INT_NUMBER2);
    }

    //
    // Acknowledge the interrupt
    //
    /*if(!ADC_getInterruptStatus(ADCB_BASE, ADC_INT_NUMBER2) &&
            !ADC_getInterruptStatus(ADCC_BASE, ADC_INT_NUMBER2))*/
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP10);
}

__interrupt void adcB2ISR(void)
{

    //
    // Set the bufferFull flag if the buffer is full
    //
    if(RESULTS_BUFFER_SIZE <= indexB2)
    {
        bufferFullB2 = 1;
    }
    else
    {
        //
        // Add the latest result to the buffer
        //
        adcBResults[indexB2++] = ADC_readResult(ADCBRESULT_BASE, ADC_SOC_NUMBER2);
    }

    //
    // Clear the interrupt flag
    //
    ADC_clearInterruptStatus(ADCB_BASE, ADC_INT_NUMBER2);

    //
    // Check if overflow has occurred
    //
    if(true == ADC_getInterruptOverflowStatus(ADCB_BASE, ADC_INT_NUMBER2))
    {
        ADC_clearInterruptOverflowStatus(ADCB_BASE, ADC_INT_NUMBER2);
        ADC_clearInterruptStatus(ADCB_BASE, ADC_INT_NUMBER2);
    }

    //
    // Acknowledge the interrupt
    //
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP10);
}

__interrupt void adcC2ISR(void)
{

    //
    // Set the bufferFull flag if the buffer is full
    //
    if(RESULTS_BUFFER_SIZE <= indexC2)
    {
        bufferFullC2 = 1;
    }
    else
    {
        //
        // Add the latest result to the buffer
        //
        adcCResults[indexC2++] = ADC_readResult(ADCCRESULT_BASE, ADC_SOC_NUMBER2);
    }

    //
    // Clear the interrupt flag
    //
    ADC_clearInterruptStatus(ADCC_BASE, ADC_INT_NUMBER2);

    //
    // Check if overflow has occurred
    //
    if(true == ADC_getInterruptOverflowStatus(ADCC_BASE, ADC_INT_NUMBER2))
    {
        ADC_clearInterruptOverflowStatus(ADCC_BASE, ADC_INT_NUMBER2);
        ADC_clearInterruptStatus(ADCC_BASE, ADC_INT_NUMBER2);
    }

    //
    // Acknowledge the interrupt
    //
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP10);
}
