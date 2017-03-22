pragma once;

#include <RC/Input/RCInput.h>

#define SBUS_STREAM_BITS 12
#define SBUS_FRAME_SIZE  ((SBUS_STREAM_BITS * 2) + 1)

// state of SBUS bit decoder
//
class SBus
{
public:
    SBus(RCInput &listener) : _listener(listener)
    {
        reset();
    }

    void process_pulse(timestamp_t frame_time, pulse_width_t width_s0, pulse_width_t width_s1);

private:
    RCInput  &_listener;
    uint16_t  _bytes[SBUS_FRAME_SIZE]; // including start bit, parity and stop bits
    uint16_t  _bit_ofs;

    inline void reset()
    {
        memset(bytes, 0, sizeof(bytes)); 
        bit_ofs = 0;
    }

    int8_t decode(const uint8_t  frame[SBUS_FRAME_SIZE],
                  pulse_width_t *values
                  uint8_t        max_values);
};

