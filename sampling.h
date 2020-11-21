#include "driverlib.h"
#include "device.h"
#include "board.h"
#include <stdint.h>
#include <stdlib.h>

#define RESULTS_BUFFER_SIZE     256
#define EX_ADC_RESOLUTION       12


uint16_t adcAResults[RESULTS_BUFFER_SIZE];   // Buffer for results
uint16_t adcBResults[RESULTS_BUFFER_SIZE];
uint16_t adcCResults[RESULTS_BUFFER_SIZE];
volatile uint16_t indexA2;                              // Index into result buffer
volatile uint16_t bufferFullA2;                // Flag to indicate buffer is full
volatile uint16_t indexB2;                              // Index into result buffer
volatile uint16_t bufferFullB2;                // Flag to indicate buffer is full
volatile uint16_t indexC2;                              // Index into result buffer
volatile uint16_t bufferFullC2;                // Flag to indicate buffer is full


void sampling_init(void);
void sampling_initADCSOCs(void);
void sampling_initEPWM(void);
void sampling_initADCs(void);


//Interrupt handlers
__interrupt void adcA1ISR(void);
__interrupt void adcB1ISR(void);
__interrupt void adcC1ISR(void);
