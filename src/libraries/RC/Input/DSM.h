pragma once;

#include <RC/Input/RCInput.h>

#define DSM_FRAME_SIZE  16  // DSM frame size in bytes

class DSM
{
public:
    DSM(RCInput &listener) : _listener(listener)
    {
        reset();
    }

    void process_pulse(timestamp_t frame_time, pulse_width_t width_s0, pulse_width_t width_s1);

private:
    RCInput     &_listener;

    uint16_t     _bytes[DSM_FRAME_SIZE]; // including start bit, parity and stop bits
    uint16_t     _bit_ofs              ;
    timestamp_t  _last_frame_time      ; // Timestamp for start of last dsm frame
    uint8_t      _channel_shift        ; // Channel resolution, 0=unknown, 1=10 bit, 2=11 bit

    bool decode_channel(uint16_t  raw    ,
                        uint8_t   shift  ,
                        uint16_t &channel,
                        uint16_t &value  );

    void guess_format(bool reset, const uint8_t frame[DSM_FRAME_SIZE]);

    inline void reset()
    {
        memset(_bytes, 0, sizeof(_bytes)); 
        _bit_ofs         = 0;
        _last_frame_time = 0;
        _channel_shift   = 0;
    }

    uint16_t decode(timestamp_t    frame_time           ,
                    const uint8_t  frame[DSM_FRAME_SIZE], 
                    pulse_width_t *values               , 
                    uint16_t       max_values           );
};
