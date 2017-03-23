pragma once;

#include <RC/Input/RCInput.h>

#define PPM_CAPTURE_NUM_CHANNELS_MIN  RC_INPUT_NUM_CHANNELS_MIN 
#define PPM_CAPTURE_NUM_CHANNELS_MAX  min( 16, RC_INPUT_NUM_CHANNELS_MAX )

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

    class PulseStat
    {
    public:
        uint32_t      sum;
        pulse_width_t min;
        pulse_width_t max;
        uint8_t       num;

        PulseStat()
        {
            reset();
        }

        inline void update(pulse_width_t val)
        {
            if (val != PULSE_WIDTH_ERR)
            {
                sum += val;

                if (val < min)
                {
                    min = val;
                }

                if (val > max)
                {
                    max = val;
                }

                num++;
            }
        }

        inline void reset()
        {
            sum=0;
            min=PULSE_WIDTH_MAX;
            max=PULSE_WIDTH_ERR;
            num=0;
        }

        inline pulse_width_t average()
        {
            return sum / num;
        }

        inline pulse_width_t deviation()
        {
            return (max >= min) ? (max - min) : (min - max);
        }
    };

    uint8_t        _channels;
    pulse_width_t  _pulses_ticks[PPM_CAPTURE_NUM_CHANNELS_MAX][2];
    PulseStat      _pulses_stats[2];
    pulse_width_t  _min_pulse_width;
    pulse_width_t  _max_pulse_width;

    inline void reset()
    {
        _channel = 0;
        memset(_pulses_ticks, 0, sizeof(_pulses_ticks)); 
        _pulses_stat[0].reset();
        _pulses_stat[1].reset();
    }

    void flush_pulses();
};

