pragma once;

#include <RC/Input/RCInput.h>

#define PPM_CAPTURE_NUM_CHANNELS_MAX  GET_MIN( 16, RC_INPUT_NUM_CHANNELS_MAX )

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
        PPM_PROFILES
    };

    struct PPMProfiles
    {
        PPMMode       mode;
        uint8_t       channels_min;
        uint8_t       channels_max;
        pulse_width_t frame_length;
        pulse_width_t pwm_pulse_min;
        pulse_width_t pwm_pulse_max;
        pulse_width_t ppm_pulse_avg;
        pulse_width_t syn_pulse_min;
        pulse_width_t syn_pulse_max;
    };

    static struct PPMProfiles _profiles[PPM_PROFILES];

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
                min = GET_MIN(val, min);
                max = GET_MAX(val, max);
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
    uint32_t       _old_switch_bit;
    
    inline void reset_ppm_profile()
    {
        _profile         =  0;
        _channels        = -1;
        _swtch_pulse_ndx =  0;
        _old_switch_bit  =  0;

        memset(_pulses_ticks, 0, sizeof(_pulses_ticks)); 
        _pulses_stat[0].reset();
        _pulses_stat[1].reset();
    }

    inline void reset()
    {
        reset_ppm_profile();

        if (_profiles[0].syn_pulse_max == PULSE_WIDTH_ERR)
        {
            // Calculate starting PPM_CAPTURE_* values
            //
            for (uint8_t i=1; i<ARRAY_SIZE(_profiles); i++)
            {
                _profiles[0].pwm_pulse_min = GET_MIN(_profiles[i].pwm_pulse_min, _profiles[0].pwm_pulse_min);
                _profiles[0].pwm_pulse_max = GET_MAX(_profiles[i].pwm_pulse_max, _profiles[0].pwm_pulse_max);
                _profiles[0].syn_pulse_min = GET_MIN(_profiles[i].syn_pulse_min, _profiles[0].syn_pulse_min);
                _profiles[0].syn_pulse_max = GET_MAX(_profiles[i].syn_pulse_max, _profiles[0].syn_pulse_max);
            }
        }
    }

    bool          guess_ppm_profile();
    pulse_width_t scale            (const pulse_width_t unscaled)
    void          flush_pulses     ();
};

