pragma once;

#include <RC/Input/RCInput.h>

class PPM
{
public:
    PPM(RCInput &listener) : _listener(listener)
    {
        reset();
    }

    void process_pulse(pulse_width_t pulse_ticks);

private:
    RCInput       &_listener;

    uint8_t        _channels = 0;
    pulse_width_t  _pulses_ticks[RC_INPUT_NUM_CHANNELS_MAX];

    inline void reset()
    {
        memset(_bytes, 0, sizeof(_bytes)); 
        _bit_ofs         = 0;
        _last_frame_time = 0;
        _channel_shift   = 0;
    }
};

