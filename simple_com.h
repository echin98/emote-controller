#ifndef SIMPLE_COM_H_INC
#define SIMPLE_COM_H_INC

#include <stdint.h>

#include "driverlib.h"
#include "device.h"

// Implements simple, ad-hoc parallel data logging using GPIO outputs.
// This can be captured with a logic analyzer or another controller.
// USAGE:
// 1) Fill the SIMPLE_COM_NUM_PINS array with the desired GPIO pin ids.
// 2) Call simple_com_init() to initialize the gpio pins for output.
// 3) Call simple_com_write_out(val) with each value to be output.

// number of GPIO pins used to write out datae
// #define SIMPLE_COM_NUM_PINS 9 // 8 data + one clock bit
#define SIMPLE_COM_NUM_PINS 5 // 4 data + one clock bit
// TODO: expand the number of lines used if more GPIOs are
// available for logging. For now just write 4 bits.

// array of GPIO pin ids used to write out data values in parallel
// The GPIO specified in index 0 is toggled for every value written out,
// The rest of the GPIOs are used to write out each bit of the value, in order;
// the GPIO specified in index 1 is the least significant bit of the value.
extern uint8_t SIMPLE_COM_OUT_PINS[SIMPLE_COM_NUM_PINS];

// Initialize GPIO pins for output.
// IMPORTANT: Set up the pin ids in the SIMPLE_COM_OUT_PINS array 
// before calling this function.
void simple_com_init();

// Write out one bit of the passed value to each of the
// GPIO lines configured in SIMPLE_COM_OUT_PINS, and toggles the "clock"
// GPIO line.
// IMPORTANT: Before calling this function:
// The mapping of GPIO pins must be configured in the
// SIMPLE_COM_OUT_PINS array, then simple_com_init() must be called.
void simple_com_write_out(uint8_t val);

#endif
