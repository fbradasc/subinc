pragma once;

#include <RC/Input/RCInput.h>

#define PWM_NUM_CHANNELS_MAX    GET_MIN(8,RC_INPUT_NUM_CHANNELS_MAX);

class PWM
{
public:
    PWM(RCInput &listener) : _listener(listener)
    {
        reset();
    }

    void process_pulse(const pulse_width_t curr_time, const uint8_t input_pins);

private:
    // Servo pulse start timing
    //
    pulse_width_t _prev_time[PWM_NUM_CHANNELS_MAX];

    // Servo input pin storage 
    //
    uint8_t _old_input_pins;

    inline void reset()
    {
        memset(_prev_time, 0, sizeof(_prev_time)); 

        _old_input_pins = 0x00;
    }
};
