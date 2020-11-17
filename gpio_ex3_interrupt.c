//#############################################################################
//
// FILE:    gpio_ex3_interrupt.c
//
// TITLE:   Device GPIO Interrupt
//
//! \addtogroup driver_example_list
//! <h1> Device GPIO Interrupt </h1>
//!
//! Configures the device GPIOs through the sysconfig file. One GPIO output
//! pin, and one GPIO input pin is configured. The example then configures the
//! GPIO input pin to be the source of an external interrupt which toggles
//! the GPIO output pin.
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

//
// Interrupt Handler
//
__interrupt void gpioInterruptHandler(void);

//
// Main
//
void main(void)
{
    //
    // Initializes system control, device clock, and peripherals
    //

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

    //
    // Initialize the PIE vector table with pointers to the shell interrupt
    // Service Routines (ISR).
    //
    Interrupt_initVectorTable();

    //
    // Board Initialization
    //
    Board_init();

    GPIO_setInterruptType(GPIO_INT_XINT1, GPIO_INT_TYPE_FALLING_EDGE);
    GPIO_setInterruptType(GPIO_INT_XINT2, GPIO_INT_TYPE_FALLING_EDGE);
    GPIO_setInterruptType(GPIO_INT_XINT3, GPIO_INT_TYPE_FALLING_EDGE);

    GPIO_setInterruptPin(32, GPIO_INT_XINT1);
    GPIO_setInterruptPin(67, GPIO_INT_XINT2);
    GPIO_setInterruptPin(111, GPIO_INT_XINT3);

    GPIO_enableInterrupt(GPIO_INT_XINT1);
    GPIO_enableInterrupt(GPIO_INT_XINT2);
    GPIO_enableInterrupt(GPIO_INT_XINT3);


    Interrupt_register(INT_XINT1, &gpioInterruptHandler); //create interrupt handler that would sync with sensing
    Interrupt_register(INT_XINT2, &gpioInterruptHandler);
    Interrupt_register(INT_XINT3, &gpioInterruptHandler);

    Interrupt_enable(INT_XINT1);
    Interrupt_enable(INT_XINT2);
    Interrupt_enable(INT_XINT3);

    //
    // Enables CPU interrupts
    //
    Interrupt_enableMaster();

    //
    // Loop.
    //
    for(;;)
    {

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
}


//
// End of File
//

