#include "simple_com.h"

// array of GPIO pin ids used to write out data values in parallel
// The GPIO specified in index 0 is toggled for every value written out,
// The rest of the GPIOs are used to write out each bit of the value, in order;
// the GPIO specified in index 1 is the least significant bit of the value.
uint8_t SIMPLE_COM_OUT_PINS[SIMPLE_COM_NUM_PINS];

uint8_t SIMPLE_COM_clock_state=0; // state of clock pin

// Initialize GPIO pins for output.
// IMPORTANT: Set up the pin ids in the SIMPLE_COM_OUT_PINS array 
// before calling this function.
void simple_com_init()
{
    uint8_t i;
    for(i=0;i<SIMPLE_COM_NUM_PINS;++i)
    {
        GPIO_setDirectionMode(SIMPLE_COM_OUT_PINS[i],GPIO_DIR_MODE_OUT);
        GPIO_setPadConfig(SIMPLE_COM_OUT_PINS[i], GPIO_PIN_TYPE_STD);
        GPIO_setMasterCore(SIMPLE_COM_OUT_PINS[i], GPIO_CORE_CPU1);
        GPIO_setQualificationMode(SIMPLE_COM_OUT_PINS[i], GPIO_QUAL_ASYNC);
    }
    GPIO_writePin(SIMPLE_COM_OUT_PINS[0],SIMPLE_COM_clock_state);
}

// Write out one bit of the passed value to each of the
// GPIO lines configured in SIMPLE_COM_OUT_PINS, and toggles the "clock"
// GPIO line.
// IMPORTANT: Before calling this function:
// The mapping of GPIO pins must be configured in the
// SIMPLE_COM_OUT_PINS array, then simple_com_init() must be called.
void simple_com_write_out(uint8_t val)
{
    uint8_t i;

    //note: i is supposed to start from 1, not 0.
    // the pin at index 0 is reserved for a clock/indicator line
    for(i=1;i<SIMPLE_COM_NUM_PINS;++i)
    {
        GPIO_writePin(SIMPLE_COM_OUT_PINS[i],(val & 1));
        val >>= 1;
    }

    // toggle clock line
    SIMPLE_COM_clock_state = (!SIMPLE_COM_clock_state);
    GPIO_writePin(SIMPLE_COM_OUT_PINS[0],SIMPLE_COM_clock_state);
}
