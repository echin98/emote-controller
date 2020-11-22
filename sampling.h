#include "driverlib.h"
#include "device.h"
#include "board.h"
#include <stdint.h>
#include <stdlib.h>

#ifndef SAMPLING
#define SAMPLING

#define RESULTS_BUFFER_SIZE     512
#define EX_ADC_RESOLUTION       12


uint16_t* s_getA2Buffer(void);
uint16_t* s_getB2Buffer(void);
uint16_t* s_getC2Buffer(void);
uint16_t* s_getA3Buffer(void);
uint16_t* s_getB3Buffer(void);
uint16_t* s_getC3Buffer(void);

void s_resetBufferA2(void);
void s_resetBufferB2(void);
void s_resetBufferC2(void);
void s_resetBufferA3(void);
void s_resetBufferB3(void);
void s_resetBufferC3(void);

bool s_setBufferFullA2(bool val);
bool s_setBufferFullB2(bool val);
bool s_setBufferFullC2(bool val);
bool s_setBufferFullA3(bool val);
bool s_setBufferFullB3(bool val);
bool s_setBufferFullC3(bool val);

bool s_getBufferFullA2(void);
bool s_getBufferFullB2(void);
bool s_getBufferFullC2(void);
bool s_getBufferFullA3(void);
bool s_getBufferFullB3(void);
bool s_getBufferFullC3(void);

void sampling_init(void);
void sampling_initADCSOCs(void);
void sampling_initEPWM(void);
void sampling_initADCs(void);


//Interrupt handlers
__interrupt void adcA2ISR(void);
__interrupt void adcB2ISR(void);
__interrupt void adcC2ISR(void);
__interrupt void adcA3ISR(void);
__interrupt void adcB3ISR(void);
__interrupt void adcC3ISR(void);
#endif
