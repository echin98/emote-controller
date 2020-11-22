//#############################################################################
//
// FILE:   empty_driverlib_main.c
//
// TITLE:  Emote Controller
//
//
// Emote controller base firmware
//
//#############################################################################
// $TI Release: F2837xD Support Library v3.11.00.00 $
// $Release Date: Sun Oct  4 15:55:24 IST 2020 $
// $Copyright:
// Copyright (C) 2013-2020 Texas Instruments Incorporated - http://www.ti.com/
//
// Redistribution and use in source and binary forms, with or without 
// modification, are permitted provided that the following conditions 
// are met:
// 
//   Redistributions of source code must retain the above copyright 
//   notice, this list of conditions and the following disclaimer.
// 
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the 
//   documentation and/or other materials provided with the   
//   distribution.
// 
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// $
//#############################################################################

//
// Included Files
//
#include "driverlib.h"
#include "device.h"
#include "board.h"
#include <stdint.h>
#include <stdlib.h>
#include "sampling.h"
#include <stdio.h>
#include <IQmathLib.h>
#include "eqep_ex2_calculation.h"

#define GUARD_PD_US 500

enum switch_commands{
    INCREMENT,
    DECREMENT,
    RESET,
    SET1,
    SET2,
    SET3,
    SET4,
    SET5,
    SET6
};

enum switch_states{
    STATE0,
    STATE1,
    STATE2,
    STATE3,
    STATE4,
    STATE5,
    STATE6
};
#define FORWARD 1
#define REVERSE 0

// 12 for 12-bit conversion resolution, which support (ADC_MODE_SINGLE_ENDED)
// Sample on single pin with VREFLO
// Or 16 for 16-bit conversion resolution, which support (ADC_MODE_DIFFERENTIAL)
// Sample on pair of pins
//
// Globals
//
uint8_t state = STATE0;

float duty_cycle = 0.5;
uint8_t direction = REVERSE;

volatile bool enable = true;

EPWM_SignalParams pwmSignal =
            {60000, 0.5f, 0.5f, false, DEVICE_SYSCLK_FREQ, SYSCTL_EPWMCLK_DIV_2,
            EPWM_COUNTER_MODE_UP_DOWN, EPWM_CLOCK_DIVIDER_1,
            EPWM_HSCLOCK_DIVIDER_1};

uint16_t adcAResult1;

/*----------------------EQEP Stuff -----------------------*/
// .9999 / 4000 converted to IQ26 fixed point format
#define MECH_SCALER     16776
// 2 pole pairs in this example
#define POLE_PAIRS      2
// Angular offset between encoder and Phase A
#define CAL_ANGLE       0
// See Equation 5 in eqep_ex2_calculation.c
#define SPEED_SCALER    ((((uint64_t)32 * DEVICE_SYSCLK_FREQ / 64) * 60) / (24000000))
// Base/max rpm is 6000rpm
#define BASE_RPM        6000

void initEQEP(void);

PosSpeed_Object posSpeed =
{
    0, 0, 0, 0,     // Initialize outputs to zero
    MECH_SCALER,    // mechScaler
    POLE_PAIRS,     // polePairs
    CAL_ANGLE,      // calAngle
    SPEED_SCALER,   // speedScaler
    0,              // Initialize output to zero
    BASE_RPM,       // baseRPM
    0, 0, 0, 0      // Initialize outputs to zero
};

uint16_t interruptCount = 0;
/*--------------------------------------------------------*/

//
// Function Prototypes
//
void initADCs(void);
void initADCSOCs(void);
void configurePhase(uint32_t base, uint32_t masterBase, uint16_t phaseVal);
uint8_t switch_state_machine(enum switch_commands command);
void set_duty_cycle(float val);

//
// Interrupt Handlers
//
__interrupt void gpioInterruptHandler(void);


//
// Main
//
void main(void)
 {
    /*ret_val = add_device("uart", _SSA,
                         uart_open,
                 uart_close,
                 uart_read,
                 uart_write,
                 uart_lseek,
                 uart_unlink,
                 uart_rename);

    fid = fopen("uart", "w");
    freopen("uart:", "w", stdout);     // redirect stdout to uart
    setvbuf(stdout, NULL, _IONBF, 0);  // turn off buffering for stdout
    printf("Hello world!\r\n");*/
    GPIO_setPinConfig(GPIO_32_GPIO32);
    GPIO_setDirectionMode(HALLA_PIN,GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(HALLA_PIN, GPIO_PIN_TYPE_STD);
    GPIO_setMasterCore(HALLA_PIN, GPIO_CORE_CPU1);
    GPIO_setQualificationMode(HALLA_PIN, GPIO_QUAL_ASYNC);


    GPIO_setPinConfig(GPIO_67_GPIO67);
    GPIO_setDirectionMode(HALLB_PIN,GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(HALLB_PIN, GPIO_PIN_TYPE_STD);
    GPIO_setMasterCore(HALLB_PIN, GPIO_CORE_CPU1);
    GPIO_setQualificationMode(HALLB_PIN, GPIO_QUAL_ASYNC);



    GPIO_setPinConfig(GPIO_111_GPIO111);
    GPIO_setDirectionMode(HALLC_PIN,GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(HALLC_PIN, GPIO_PIN_TYPE_STD);
    GPIO_setMasterCore(HALLC_PIN, GPIO_CORE_CPU1);
    GPIO_setQualificationMode(HALLC_PIN, GPIO_QUAL_ASYNC);

    Device_init();
    Interrupt_initModule();
    Interrupt_initVectorTable();

    Interrupt_enableMaster();

    Board_init();

    /* INTERRUPT SETUP */
    GPIO_setInterruptType(GPIO_INT_XINT1, GPIO_INT_TYPE_BOTH_EDGES);
    GPIO_setInterruptType(GPIO_INT_XINT2, GPIO_INT_TYPE_BOTH_EDGES);
    GPIO_setInterruptType(GPIO_INT_XINT3, GPIO_INT_TYPE_BOTH_EDGES);

    GPIO_setInterruptPin(HALLA_PIN, GPIO_INT_XINT1);
    GPIO_setInterruptPin(HALLB_PIN, GPIO_INT_XINT2);
    GPIO_setInterruptPin(HALLC_PIN, GPIO_INT_XINT3);

    GPIO_enableInterrupt(GPIO_INT_XINT1);
    GPIO_enableInterrupt(GPIO_INT_XINT2);
    GPIO_enableInterrupt(GPIO_INT_XINT3);

    Interrupt_register(INT_ADCA2, &adcA2ISR);
    Interrupt_register(INT_ADCB2, &adcB2ISR);
    Interrupt_register(INT_ADCC2, &adcC2ISR);

    Interrupt_register(INT_XINT1, &gpioInterruptHandler); //create interrupt handler that would sync with sensing
    Interrupt_register(INT_XINT2, &gpioInterruptHandler);
    Interrupt_register(INT_XINT3, &gpioInterruptHandler);

    Interrupt_enable(INT_XINT1);
    Interrupt_enable(INT_XINT2);
    Interrupt_enable(INT_XINT3);

    /* END INTERRUPT SETUP */

    /* ADC SETUP */

    sampling_init();
    /*------------------QEP SETUP--------------------*/
    //
    // Initialize GPIOs for use as EQEP1A, EQEP1B, and EQEP1I
    //
    GPIO_setPinConfig(GPIO_104_EQEP3A);
    GPIO_setPadConfig(104, GPIO_PIN_TYPE_STD);

    GPIO_setPinConfig(GPIO_105_EQEP3B);
    GPIO_setPadConfig(105, GPIO_PIN_TYPE_STD);

    /* PWM SETUP */

    //
    // Configuring ePWM module for desired frequency and duty
    //
    EPWM_configureSignal(EPWM1_BASE, &pwmSignal);
    EPWM_configureSignal(EPWM2_BASE, &pwmSignal);
    EPWM_configureSignal(EPWM3_BASE, &pwmSignal);

    //
    // Configure phase between PWM1, PWM2 & PWM3.
    // PWM1 is configured as master and ePWM2 & 3
    // are configured as slaves.
    //
    EPWM_disablePhaseShiftLoad(EPWM1_BASE);
    EPWM_setPhaseShift(EPWM1_BASE, 0U);

    //
    // ePWM1 SYNCO is generated on CTR=0
    //
    EPWM_setSyncOutPulseMode(EPWM1_BASE, EPWM_SYNC_OUT_PULSE_ON_COUNTER_ZERO);

    //
    // Configure phase shift for EPWM2 & 3
    //
    configurePhase(EPWM2_BASE, EPWM1_BASE, 0);
    configurePhase(EPWM3_BASE, EPWM1_BASE, 0);

    EPWM_enablePhaseShiftLoad(EPWM2_BASE);
    EPWM_enablePhaseShiftLoad(EPWM3_BASE);

    //
    // Enable sync and clock to PWM
    //
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);

    //
    // Enable interrupts required for this example
    //
    Interrupt_enable(INT_EPWM1);

    //
    // Set up ADCs, initializing the SOCs to be triggered by software
    //
    initADCs();
    initADCSOCs();

    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM

    initEQEP();

    /* END PWM SETUP */
    float dc = 0;
    float P = 0.01;
    for(;;)
    {
        ADC_forceSOC(ADCA_BASE, ADC_SOC_NUMBER1);
        //
        // Wait for ADCA to complete, then acknowledge flag
        //
        while(ADC_getInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1) == false)
        {
        }
        ADC_clearInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1);

        adcAResult1 = ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER1);
        DEVICE_DELAY_US(3000);
        PosSpeed_calculate(&posSpeed);
        dc = dc+P*(5-posSpeed.speedPR);
        //dc = 1.8*((float)adcAResult1/4096)-0.9;
        if(dc<0){
            direction = REVERSE;
            dc = -dc;
        }
        else{
            direction = FORWARD;
        }

        if(dc>0.8)dc = 0.8;
        if(dc<0.05) dc = 0;
        float dc_temp = dc;
        set_duty_cycle(dc);
        //switch_state_machine(RESET);

        //printf("%i\n",posSpeed.speedPR);
        //DEVICE_DELAY_US(100000);
        //ESTOP0;
        //
        // Start ePWM1/EPWM2, enabling SOCA-b and putting the counter in up-count mode
        //
/* Sampling stuff */

        /*EPWM_enableADCTrigger(EPWM4_BASE, EPWM_SOC_A); //SOCA AND SOCB can trigger ADCA and ADCB synchronously w/o overlapping
        EPWM_setTimeBaseCounterMode(EPWM4_BASE, EPWM_COUNTER_MODE_UP);


        //
        // Wait while ePWM1 causes ADC conversions which then cause interrupts.
        // When the results buffer is filled, the bufferFull flag will be set.
        //
        while(s_getBufferFullA2() == 0)
        {
        }

        //
        // Stop ePWM1/ePWM1, disabling SOCA-B and freezing the counter
        //

        EPWM_disableADCTrigger(EPWM4_BASE, EPWM_SOC_A);
        EPWM_setTimeBaseCounterMode(EPWM4_BASE, EPWM_COUNTER_MODE_STOP_FREEZE);
        s_resetBufferA2();     // Clear the buffer full flag
        s_resetBufferB2();
        s_resetBufferC2();


        uint16_t* tempA = s_getA2Buffer();
        uint16_t* tempB = s_getB2Buffer();
        uint16_t* tempC = s_getC2Buffer();

        uint8_t read_idx;
        printf("starting\n");
        for(read_idx = 0; read_idx<RESULTS_BUFFER_SIZE; read_idx++){
            printf("%u\n",tempA[read_idx]);
        }

        //
        // Software breakpoint. At this point, conversion results are stored in
        // adcAResults.
        //
        // Hit run again to get updated conversions.
        //
        ESTOP0;*/

    }


}

__interrupt void gpioInterruptHandler(void)
{

    uint8_t pinVal = GPIO_readPin(HALLA_PIN)<<2 | GPIO_readPin(HALLB_PIN)<<1 | GPIO_readPin(HALLC_PIN);
    if(direction){
        switch(pinVal){
        case 0b101:
            switch_state_machine(SET1);
            break;
        case 0b100:
            switch_state_machine(SET2);
            break;
        case 0b110:
            switch_state_machine(SET3);
            break;
        case 0b010:
            switch_state_machine(SET4);
            break;
        case 0b011:
            switch_state_machine(SET5);
            break;
        case 0b001:
            switch_state_machine(SET6);
            break;
        default:        //catch errors
            switch_state_machine(RESET);
            break;

        }
    }
    else
    {
        switch(pinVal){
        case 0b101:
            switch_state_machine(SET4);
            break;
        case 0b100:
            switch_state_machine(SET5);
            break;
        case 0b110:
            switch_state_machine(SET6);
            break;
        case 0b010:
            switch_state_machine(SET1);
            break;
        case 0b011:
            switch_state_machine(SET2);
            break;
        case 0b001:
            switch_state_machine(SET3);
            break;
        default:        //catch errors
            switch_state_machine(RESET);
            break;

        }
    }

    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1); //subject to change
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP12); //subject to change
}

//
// Function to configure eQEP1.
//
void initEQEP(void)
{
    //
    // Configure the decoder for quadrature count mode
    //
    EQEP_setDecoderConfig(EQEP3_BASE, (EQEP_CONFIG_1X_RESOLUTION |
                                       EQEP_CONFIG_QUADRATURE |
                                       EQEP_CONFIG_NO_SWAP));
    EQEP_setEmulationMode(EQEP3_BASE, EQEP_EMULATIONMODE_RUNFREE);

    //
    // Configure the position counter to reset on an index event
    //
    EQEP_setPositionCounterConfig(EQEP3_BASE, EQEP_POSITION_RESET_UNIT_TIME_OUT,
                                  0xFFFFFFFF);

    //
    // Enable the unit timer, setting the frequency to 100 Hz
    //
    EQEP_enableUnitTimer(EQEP3_BASE, (DEVICE_SYSCLK_FREQ / 100));

    //
    // Configure the position counter to be latched on a unit time out
    //
    EQEP_setLatchMode(EQEP3_BASE, EQEP_LATCH_UNIT_TIME_OUT);

    //
    // Enable the eQEP1 module
    //
    EQEP_enableModule(EQEP3_BASE);

    //
    // Configure and enable the edge-capture unit. The capture clock divider is
    // SYSCLKOUT/64. The unit-position event divider is QCLK/32.
    //
    EQEP_setCaptureConfig(EQEP3_BASE, EQEP_CAPTURE_CLK_DIV_64,
                          EQEP_UNIT_POS_EVNT_DIV_32);
    EQEP_enableCapture(EQEP3_BASE);
}

//
// set_duty_cycle - Set the duty cycle of the power control PWM
void set_duty_cycle(float val)
{
    uint16_t tbpd = EPWM_getTimeBasePeriod(EPWM1_BASE);
    uint16_t compare_val = tbpd-tbpd*val;
    EPWM_setCounterCompareValue(EPWM1_BASE, EPWM_COUNTER_COMPARE_A, compare_val);
    EPWM_setCounterCompareValue(EPWM1_BASE, EPWM_COUNTER_COMPARE_B, compare_val);
    EPWM_setCounterCompareValue(EPWM2_BASE, EPWM_COUNTER_COMPARE_A, compare_val);
    EPWM_setCounterCompareValue(EPWM2_BASE, EPWM_COUNTER_COMPARE_B, compare_val);
    EPWM_setCounterCompareValue(EPWM3_BASE, EPWM_COUNTER_COMPARE_A, compare_val);
    EPWM_setCounterCompareValue(EPWM3_BASE, EPWM_COUNTER_COMPARE_B, compare_val);

}

//
// configurePhase - Configure ePWMx Phase
//
void configurePhase(uint32_t base, uint32_t masterBase, uint16_t phaseVal)
{
    uint32_t readPrdVal, phaseRegVal;

    //
    // Read Period value to calculate value for Phase Register
    //
    readPrdVal = EPWM_getTimeBasePeriod(masterBase);

    //
    // Caluclate phase register values based on Time Base counter mode
    //
    if((HWREGH(base + EPWM_O_TBCTL) & 0x3U) == EPWM_COUNTER_MODE_UP_DOWN)
    {
        phaseRegVal = (2U * readPrdVal * phaseVal) / 360U;
    }
    else if((HWREGH(base + EPWM_O_TBCTL) & 0x3U) < EPWM_COUNTER_MODE_UP_DOWN)
    {
        phaseRegVal = (readPrdVal * phaseVal) / 360U;
    }

    EPWM_selectPeriodLoadEvent(base, EPWM_SHADOW_LOAD_MODE_SYNC);
    EPWM_setPhaseShift(base, phaseRegVal);
    EPWM_setTimeBaseCounter(base, phaseRegVal);
}

uint8_t switch_state_machine(enum switch_commands command)
{
    if(enable){
        switch(state){
            case STATE0:
                switch(command){
                    case INCREMENT:
                        state = 1;
                        break;
                    case DECREMENT:
                        state = 6;
                        break;
                    case SET1:
                        state = 1;
                        break;
                    case SET2:
                        state = 2;
                        break;
                    case SET3:
                        state = 3;
                        break;
                    case SET4:
                        state = 4;
                        break;
                    case SET5:
                        state = 5;
                        break;
                    case SET6:
                        state = 6;
                        break;
                    default:
                        state = 0;
                        break;
                }
                break;
            case STATE1:
                switch(command){
                case SET1:
                    break;
                case SET4:
                    state = STATE4;
                    break;
                case SET2: case INCREMENT:
                    state = STATE2;
                    break;
                case SET6: case DECREMENT:
                    state = STATE6;
                    break;
                default:
                    state = STATE0;
                    break;
                }
                break;
            case STATE2:
                switch(command){
                case SET2:
                    break;
                case SET5:
                    state = STATE5;
                    break;
                case SET3: case INCREMENT:
                    state = STATE3;
                    break;
                case SET1: case DECREMENT:
                    state = STATE1;
                    break;
                default:
                    state = STATE0;
                    break;
                }
                break;
            case STATE3:
                switch(command){
                case SET3:
                    break;
                case SET6:
                    state = STATE6;
                    break;
                case SET4: case INCREMENT:
                    state = STATE4;
                    break;
                case SET2: case DECREMENT:
                    state = STATE2;
                    break;
                default:
                    state = STATE0;
                    break;
                }
                break;
            case STATE4:
                switch(command){
                case SET4:
                    break;
                case SET1:
                    state = STATE1;
                    break;
                case SET5: case INCREMENT:
                    state = STATE5;
                    break;
                case SET3: case DECREMENT:
                    state = STATE3;
                    break;
                default:
                    state = STATE0;
                    break;
                }
                break;
            case STATE5:
                switch(command){
                case SET5:
                    break;
                case SET2:
                    state = STATE2;
                    break;
                case SET6: case INCREMENT:
                    state = STATE6;
                    break;
                case SET4: case DECREMENT:
                    state = STATE4;
                    break;
                default:
                    state = STATE0;
                    break;
                }
                break;
            case STATE6:
                switch(command){
                case SET6:
                    break;
                case SET3:
                    state = STATE3;
                    break;
                case SET1: case INCREMENT:
                    state = STATE1;
                    break;
                case SET5: case DECREMENT:
                    state = STATE5;
                    break;
                default:
                    state = STATE0;
                    break;
                }
                break;
        }
    } else state = 0;

    switch(state){
        case STATE0:
            GPIO_setPinConfig(HSA_GPIO);
            GPIO_setPinConfig(LSA_GPIO);
            GPIO_setPinConfig(HSB_GPIO);
            GPIO_setPinConfig(LSB_GPIO);
            GPIO_setPinConfig(HSC_GPIO);
            GPIO_setPinConfig(LSC_GPIO);

            GPIO_writePin(HSA_PIN,0);
            GPIO_writePin(LSA_PIN,0);
            GPIO_writePin(HSB_PIN,0);
            GPIO_writePin(LSB_PIN,0);
            GPIO_writePin(HSC_PIN,0);
            GPIO_writePin(LSC_PIN,0);
            break;
        case STATE1:
            GPIO_setPinConfig(LSA_GPIO);
            GPIO_setPinConfig(HSB_GPIO);
            GPIO_setPinConfig(HSC_GPIO);
            GPIO_setPinConfig(LSC_GPIO);
            GPIO_setPinConfig(LSB_PWM);
            GPIO_setPinConfig(HSA_PWM);
            break;
        case STATE2:
            GPIO_setPinConfig(LSA_GPIO);
            GPIO_setPinConfig(HSB_GPIO);
            GPIO_setPinConfig(LSB_GPIO);
            GPIO_setPinConfig(HSC_GPIO);
            GPIO_setPinConfig(LSC_PWM);
            GPIO_setPinConfig(HSA_PWM);
            break;
        case STATE3:
            GPIO_setPinConfig(HSA_GPIO);
            GPIO_setPinConfig(LSA_GPIO);
            GPIO_setPinConfig(LSB_GPIO);
            GPIO_setPinConfig(HSC_GPIO);
            GPIO_setPinConfig(LSC_PWM);
            GPIO_setPinConfig(HSB_PWM);
            break;
        case STATE4:
            GPIO_setPinConfig(HSA_GPIO);
            GPIO_setPinConfig(LSB_GPIO);
            GPIO_setPinConfig(HSC_GPIO);
            GPIO_setPinConfig(LSC_GPIO);
            GPIO_setPinConfig(LSA_PWM);
            GPIO_setPinConfig(HSB_PWM);
            break;
        case STATE5:
            GPIO_setPinConfig(HSA_GPIO);
            GPIO_setPinConfig(HSB_GPIO);
            GPIO_setPinConfig(LSB_GPIO);
            GPIO_setPinConfig(LSC_GPIO);
            GPIO_setPinConfig(LSA_PWM);
            GPIO_setPinConfig(HSC_PWM);
            break;
        case STATE6:
            GPIO_setPinConfig(HSA_GPIO);
            GPIO_setPinConfig(LSA_GPIO);
            GPIO_setPinConfig(HSB_GPIO);
            GPIO_setPinConfig(LSC_GPIO);
            GPIO_setPinConfig(LSB_PWM);
            GPIO_setPinConfig(HSC_PWM);

            break;
    }
    return state;
}
//
// Function to configure and power up ADCs A and B.
//
void initADCs(void)
{
    //
    // Set ADCCLK divider to /4
    //
    ADC_setPrescaler(ADCA_BASE, ADC_CLK_DIV_4_0);

    //
    // Set resolution and signal mode (see #defines above) and load
    // corresponding trims.
    //
#if(EX_ADC_RESOLUTION == 12)
    ADC_setMode(ADCA_BASE, ADC_RESOLUTION_12BIT, ADC_MODE_SINGLE_ENDED);
#elif(EX_ADC_RESOLUTION == 16)
    ADC_setMode(ADCA_BASE, ADC_RESOLUTION_16BIT, ADC_MODE_DIFFERENTIAL);
#endif

    //
    // Set pulse positions to late
    //
    ADC_setInterruptPulseMode(ADCA_BASE, ADC_PULSE_END_OF_CONV);

    //
    // Power up the ADCs and then delay for 1 ms
    //
    ADC_enableConverter(ADCA_BASE);

    DEVICE_DELAY_US(1000);
}

//
// Function to configure SOCs 0 and 1 of ADCs A and B.
//
void initADCSOCs(void)
{
    //
    // Configure SOCs of ADCA
    // - SOC0 will convert pin A0.
    // - SOC1 will convert pin A1.
    // - Both will be triggered by software only.
    // - For 12-bit resolution, a sampling window of 15 (75 ns at a 200MHz
    //   SYSCLK rate) will be used.  For 16-bit resolution, a sampling window
    //   of 64 (320 ns at a 200MHz SYSCLK rate) will be used.
    //
#if(EX_ADC_RESOLUTION == 12)
    ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER1, ADC_TRIGGER_SW_ONLY,       //A1, potentiometer
                 ADC_CH_ADCIN1, 15);
#elif(EX_ADC_RESOLUTION == 16)
    ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER1, ADC_TRIGGER_SW_ONLY,
                 ADC_CH_ADCIN1, 64);
#endif

    //
    // Set SOC1 to set the interrupt 1 flag. Enable the interrupt and make
    // sure its flag is cleared.
    //
    ADC_setInterruptSource(ADCA_BASE, ADC_INT_NUMBER1, ADC_SOC_NUMBER1);
    ADC_enableInterrupt(ADCA_BASE, ADC_INT_NUMBER1);
    ADC_clearInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1);



}


//
// End of File
//
