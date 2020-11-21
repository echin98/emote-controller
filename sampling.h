#include "driverlib.h"
#include "device.h"
#include "board.h"
#include <stdint.h>
#include <stdlib.h>

#ifndef SAMPLING
#define SAMPLING

#define RESULTS_BUFFER_SIZE     256
#define EX_ADC_RESOLUTION       12


uint16_t* s_getA2Buffer(void);
uint16_t* s_getB2Buffer(void);
uint16_t* s_getC2Buffer(void);

void s_resetBufferA2(void);
void s_resetBufferB2(void);
void s_resetBufferC2(void);

bool s_setBufferFullA2(bool val);
bool s_setBufferFullB2(bool val);
bool s_setBufferFullC2(bool val);

bool s_getBufferFullA2(void);
bool s_getBufferFullB2(void);
bool s_getBufferFullC2(void);
void sampling_init(void);
void sampling_initADCSOCs(void);
void sampling_initEPWM(void);
void sampling_initADCs(void);


//Interrupt handlers
__interrupt void adcA2ISR(void);
__interrupt void adcB2ISR(void);
__interrupt void adcC2ISR(void);
#endif
