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
    enum PPMMode
    {
        PPM_UNKNOWN ,
        PPM_STANDARD,
        PPM_EXTENDED,
        PPM_V2      ,
        PPM_V3      ,
    };

    struct PPMProfiles
    {
        PPMMode       mode;
        uint8_t       channels_min;
        uint8_t       channels_max;
        pulse_width_t frame_length;
        pulse_width_t pwm_pulse_min;
        pulse_width_t pwm_pulse_max;
        pulse_width_t pre_pulse_max;
        pulse_width_t syn_pulse_min;
        pulse_width_t syn_pulse_max;
    };

    const struct PPMProfiles PPM_PROFILES[] =
    {
        //+------------+-----------+--------+-------------+-----+---------------+//
        //| PPM mode   | Channels# | Frame  |  PWM Pulse  | Pre | Synchro Pulse |//
        //|            | min | max | length |  min |  max | max |  min  |  max  |//
        //+------------+-----+-----+--------+------+------+-----+-------+-------+//
        {  PPM_UNKNOWN ,   0 ,   0 ,      0 ,    0 ,    0 ,   0 ,     0 ,     0  },
        {  PPM_STANDARD,   4 ,   8 ,  20000 ,  920 , 2120 , 400 ,  3040 , 16320  },
        {  PPM_EXTENDED,   4 ,   9 ,  22500 ,  920 , 2120 , 400 ,  3420 , 18820  },
        {  PPM_V2      ,   4 ,  16 ,  20000 ,  460 , 1060 , 200 ,  3040 , 18160  },
        {  PPM_V3      ,   4 ,  16 ,  25000 ,  750 , 1350 , 400 ,  3400 , 22000  }
    };

    RCInput &_listener;

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

    int8_t         _profile;
    uint8_t        _channels;
    pulse_width_t  _pulses_ticks[PPM_CAPTURE_NUM_CHANNELS_MAX][2];
    PulseStat      _pulses_stats[2];
    uint8_t        _swtch_pulse_ndx;
    

    inline void reset()
    {
        _profile         = 0;
        _channels        = 0;
        _swtch_pulse_ndx = 0;

        memset(_pulses_ticks, 0, sizeof(_pulses_ticks)); 
        _pulses_stat[0].reset();
        _pulses_stat[1].reset();
    }

    bool          guess_ppm_profile()
    pulse_width_t scale            (const pulse_width_t unscaled)
    void          flush_pulses     ();
};

