//#############################################################################
//
// FILE:   empty_driverlib_main.c
//
// TITLE:  Empty Project
//
// Empty Project Example
//
// This example is an empty project setup for Driverlib development.
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

//ISSUE
//Look to see if these are the right configs
EPWM_SignalParams pwmSignal =
            {60000, 0.3f, 0.3f, false, DEVICE_SYSCLK_FREQ, SYSCTL_EPWMCLK_DIV_2,
            EPWM_COUNTER_MODE_UP_DOWN, EPWM_CLOCK_DIVIDER_1,
            EPWM_HSCLOCK_DIVIDER_1};

void configurePhase(uint32_t base, uint32_t masterBase, uint16_t phaseVal);
void switch_state_machine(enum switch_commands command);

//
// Interrupt Handler
//
__interrupt void gpioInterruptHandler(void);

//
// Main
//
void main(void)
{
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


    Interrupt_register(INT_XINT1, &gpioInterruptHandler); //create interrupt handler that would sync with sensing
    Interrupt_register(INT_XINT2, &gpioInterruptHandler);
    Interrupt_register(INT_XINT3, &gpioInterruptHandler);

    Interrupt_enable(INT_XINT1);
    Interrupt_enable(INT_XINT2);
    Interrupt_enable(INT_XINT3);

    /* INTERRUPT SETUP */

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

    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM

    /* END PWM SETUP */

    for(;;)
    {
        ;
        //switch_state_machine(INCREMENT);
        //DEVICE_DELAY_US(1000);
    }


}

__interrupt void gpioInterruptHandler(void)
{
    uint16_t pinValue1;
    uint16_t pinValue2;
    uint16_t pinValue3;

    pinValue1 = GPIO_readPin(HALLA_PIN);
    pinValue2 = GPIO_readPin(HALLB_PIN);
    pinValue3 = GPIO_readPin(HALLC_PIN);

    if (pinValue1 && pinValue2 && pinValue3==0) //case 0
    {

        //errors present when building for setPinConfig. When integrated with switching will go away
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

    }

    if (pinValue1==1 && pinValue2==0 && pinValue3==1) //case 1
    {
                    GPIO_setPinConfig(HSA_PWM); //copied from switching
                    GPIO_setPinConfig(LSA_GPIO);
                    GPIO_setPinConfig(HSB_GPIO);
                    GPIO_setPinConfig(LSB_PWM);
                    GPIO_setPinConfig(HSC_GPIO);
                    GPIO_setPinConfig(LSC_GPIO);
    }

    if (pinValue1==1 && pinValue2==0 && pinValue3==0) //case 2
       {

                   GPIO_setPinConfig(HSA_PWM);
                   GPIO_setPinConfig(LSA_GPIO);
                   GPIO_setPinConfig(HSB_GPIO);
                   GPIO_setPinConfig(LSB_GPIO);
                   GPIO_setPinConfig(HSC_GPIO);
                   GPIO_setPinConfig(LSC_PWM);

       }

    if (pinValue1==1 && pinValue2==1 && pinValue3==0) //case 3
          {
                    GPIO_setPinConfig(HSA_GPIO);
                    GPIO_setPinConfig(LSA_GPIO);
                    GPIO_setPinConfig(HSB_PWM);
                    GPIO_setPinConfig(LSB_GPIO);
                    GPIO_setPinConfig(HSC_GPIO);
                    GPIO_setPinConfig(LSC_PWM);
          }

    if (pinValue1==0 && pinValue2==1 && pinValue3==0) //case 4
          {


                    GPIO_setPinConfig(HSA_GPIO);
                    GPIO_setPinConfig(LSA_PWM);
                    GPIO_setPinConfig(HSB_PWM);
                    GPIO_setPinConfig(LSB_GPIO);
                    GPIO_setPinConfig(HSC_GPIO);
                    GPIO_setPinConfig(LSC_GPIO);

          }
    if (pinValue1==0 && pinValue2==1 && pinValue3==1) //case 5
          {
                    GPIO_setPinConfig(HSA_GPIO);
                    GPIO_setPinConfig(LSA_PWM);
                    GPIO_setPinConfig(HSB_GPIO);
                    GPIO_setPinConfig(LSB_GPIO);
                    GPIO_setPinConfig(HSC_PWM);
                    GPIO_setPinConfig(LSC_GPIO);


          }

    if (pinValue1==0 && pinValue2==0 && pinValue3==1) //case 6
              {

                    GPIO_setPinConfig(HSA_GPIO);
                    GPIO_setPinConfig(LSA_GPIO);
                    GPIO_setPinConfig(HSB_GPIO);
                    GPIO_setPinConfig(LSB_PWM);
                    GPIO_setPinConfig(HSC_PWM);
                    GPIO_setPinConfig(LSC_GPIO);

                  }

    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1); //subject to change
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP2); //subject to change
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

void switch_state_machine(enum switch_commands command)
{
    static uint8_t state = STATE0;
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
                case INCREMENT:
                    state = STATE2;
                    break;
                case DECREMENT:
                    state = STATE6;
                    break;
                case RESET:
                    state = STATE0;
                    break;
            }
            break;
        case STATE2:
            switch(command){
                case INCREMENT:
                    state = STATE3;
                    break;
                case DECREMENT:
                    state = STATE1;
                    break;
                case RESET:
                    state = STATE0;
                    break;
            }
            break;
            case STATE3:
                switch(command){
                    case INCREMENT:
                        state = STATE4;
                        break;
                    case DECREMENT:
                        state = STATE2;
                        break;
                    case RESET:
                        state = STATE0;
                        break;
                }
                break;
            case STATE4:
                switch(command){
                    case INCREMENT:
                        state = STATE5;
                        break;
                    case DECREMENT:
                        state = STATE3;
                        break;
                    case RESET:
                        state = STATE0;
                        break;
                }
                break;
            case STATE5:
                switch(command){
                    case INCREMENT:
                        state = STATE6;
                        break;
                    case DECREMENT:
                        state = STATE4;
                        break;
                    case RESET:
                        state = STATE0;
                        break;
                }
                break;
            case STATE6:
                switch(command){
                    case INCREMENT:
                        state = STATE1;
                        break;
                    case DECREMENT:
                        state = STATE5;
                        break;
                    case RESET:
                        state = STATE0;
                        break;
                }
                break;
    }
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
            GPIO_setPinConfig(HSA_PWM);
            GPIO_setPinConfig(LSA_GPIO);
            GPIO_setPinConfig(HSB_GPIO);
            GPIO_setPinConfig(LSB_PWM);
            GPIO_setPinConfig(HSC_GPIO);
            GPIO_setPinConfig(LSC_GPIO);
            break;
        case STATE2:
            GPIO_setPinConfig(HSA_PWM);
            GPIO_setPinConfig(LSA_GPIO);
            GPIO_setPinConfig(HSB_GPIO);
            GPIO_setPinConfig(LSB_GPIO);
            GPIO_setPinConfig(HSC_GPIO);
            GPIO_setPinConfig(LSC_PWM);
            break;
        case STATE3:
            GPIO_setPinConfig(HSA_GPIO);
            GPIO_setPinConfig(LSA_GPIO);
            GPIO_setPinConfig(HSB_PWM);
            GPIO_setPinConfig(LSB_GPIO);
            GPIO_setPinConfig(HSC_GPIO);
            GPIO_setPinConfig(LSC_PWM);
            break;
        case STATE4:
            GPIO_setPinConfig(HSA_GPIO);
            GPIO_setPinConfig(LSA_PWM);
            GPIO_setPinConfig(HSB_PWM);
            GPIO_setPinConfig(LSB_GPIO);
            GPIO_setPinConfig(HSC_GPIO);
            GPIO_setPinConfig(LSC_GPIO);
            break;
        case STATE5:
            GPIO_setPinConfig(HSA_GPIO);
            GPIO_setPinConfig(LSA_PWM);
            GPIO_setPinConfig(HSB_GPIO);
            GPIO_setPinConfig(LSB_GPIO);
            GPIO_setPinConfig(HSC_PWM);
            GPIO_setPinConfig(LSC_GPIO);
            break;
        case STATE6:
            GPIO_setPinConfig(HSA_GPIO);
            GPIO_setPinConfig(LSA_GPIO);
            GPIO_setPinConfig(HSB_GPIO);
            GPIO_setPinConfig(LSB_PWM);
            GPIO_setPinConfig(HSC_PWM);
            GPIO_setPinConfig(LSC_GPIO);

            break;
    }
    DEVICE_DELAY_US(GUARD_PD_US);
}

//
// End of File
//
