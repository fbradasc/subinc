#include <RC/Input/PPM.h>

#define PWM_CAPTURE_NUM_CHANNELS_MIN  _profiles[_profile].channels_min;
#define PWM_CAPTURE_PULSE_WIDTH_MIN   _profiles[_profile].pwm_pulse_min;
#define PWM_CAPTURE_PULSE_WIDTH_MAX   _profiles[_profile].pwm_pulse_max;
#define PPM_CAPTURE_MIN_SYNC_PULSE_W  _profiles[_profile].syn_pulse_min;

#define SWITCH_LEVEL   15               // used to auto detect digital data over pre pulse
#define PULSE_CAGE     SWITCH_LEVEL*3   // used to auto detect the PPM profile
#define FRAME_CAGE     500              // used to auto detect the PPM profile

#define MATCH_IN_RANGE(a,b,r)    (((a) >= ((b) - (r))) && ((a) <= ((b) + (r))))

struct PPMProfiles PPM::_profiles[PPM::PPM_PROFILES] =
{
    //+-------------+-----------+-----------------+-----------------------------------+-----------------+-----------------------------------+//
    //|  PPM mode   | Channels# |      Frame      |             PWM Pulse             |    PPM Pulse    |          Synchro Pulse            |//
    //|             | min | max |      length     |       min       |       max       |     average     |       min       |       max       |//
    //+-------------+-----+-----+-----------------+-----------------+-----------------+-----------------+-----------------+-----------------+//
      { PPM_UNKNOWN , 255 ,   0 , PULSE_WIDTH_ERR , PULSE_WIDTH_MAX , PULSE_WIDTH_ERR , PULSE_WIDTH_ERR , PULSE_WIDTH_MAX , PULSE_WIDTH_ERR },
      { PPM_STANDARD,   4 ,   8 ,           20000 ,             920 ,            2120 ,             400 ,            3040 ,           16320 },
      { PPM_EXTENDED,   4 ,   9 ,           22500 ,             920 ,            2120 ,             400 ,            3420 ,           18820 },
      { PPM_V2      ,   4 ,  16 ,           20000 ,             460 ,            1060 ,             200 ,            3040 ,           18160 },
      { PPM_V3      ,   4 ,  16 ,           25000 ,             750 ,            1350 ,             400 ,            3400 ,           22000 }
};

inline bool PPM::guess_ppm_profile()
{
    if (_profile > 0)
    {
        // already detected
        //
        return true;
    }
    else
    if (_channels < 0)
    {
        // not yet synchronized
        //
        return false;
    }
    else
    {
        // detecting
        //
        for (uint8_t i=1; i<ARRAY_SIZE(_profiles); i++)
        {
            if (MATCH_IN_RANGE((_pulses_stat[0].sum + _pulses_stat[1].sum), _profiles[i].frame_length, FRAME_CAGE))
            {
                _swtch_pulse_ndx = (_pulses_stat[0].average() < _pulses_stat[1].average()) ? 0 : 1;

                if (MATCH_IN_RANGE((_pulses_stat[_swtch_pulse_ndx].average()), _profiles[i].ppm_pulse_avg, PULSE_CAGE))
                {
                    _profile = i;

                    return true;
                }
            }
        }
    }

    // unable to detect
    //
    return false;
}

inline pulse_width_t PPM::scale(const pulse_width_t unscaled)
{
    if ( unscaled < PWM_CAPTURE_PULSE_WIDTH_MIN )
    {
        // shall issue an error
        //
        return PULSE_WIDTH_MIN;
    }
    else
    if ( unscaled == PWM_CAPTURE_PULSE_WIDTH_MIN )
    {
        return PULSE_WIDTH_MIN;
    }
    else
    if ( unscaled > PWM_CAPTURE_PULSE_WIDTH_MAX )
    {
        // shall issue an error
        //
        return PULSE_WIDTH_MAX;
    }
    else
    if ( unscaled == PWM_CAPTURE_PULSE_WIDTH_MAX )
    {
        return PULSE_WIDTH_MAX;
    }

    return unscaled * PULSE_WIDTH_MAX / ( PWM_CAPTURE_PULSE_WIDTH_MAX - PWM_CAPTURE_PULSE_WIDTH_MIN );
}

inline void PPM::flush_pulses()
{
    if (!_profile)
    {
        // PPM mode not detected
        //
        return;
    }

    for (uint8_t i=0; i<_channels; i++)
    {
        _listener._pulse_capt[i] = scale(_pulses_ticks[i][0] + _pulses_ticks[i][1]);

        if ( _pulses_ticks[i][_swtch_pulse_ndx] < (_profiles[_profile].ppm_pulse_avg - SWITCH_LEVEL) )
        {
            _listener._switches ^= ( 1 << ( i << 1 ) );
        }
        else
        if ( _pulses_ticks[i][_swtch_pulse_ndx] > (_profiles[_profile].ppm_pulse_avg + SWITCH_LEVEL) )
        {
            _listener._switches ^= ( 1 << ( ( i << 1 ) + 1 ) );
        }
    }

    _listener._num_channels = _channels;

    _listener._new_input    = true;
}

// process a PPM-sum pulse of the given width
//
void PPM::process_pulse(const pulse_width_t width_s0, const pulse_width_t width_s1)
{
    const pulse_width_t pulse_ticks = width_s0 + width_s1;

    if (!_profile && (_channels >= 0))
    {
        _pulses_stat[0].update(width_s0);
        _pulses_stat[1].update(width_s1);
    }

    if (pulse_ticks >= PPM_CAPTURE_MIN_SYNC_PULSE_W)
    {
        // a long pulse indicates the end of a frame. Reset the
        // channel counter so next pulse is channel 0
        //
        if (guess_ppm_profile() && (_channels >= PWM_CAPTURE_NUM_CHANNELS_MIN))
        {
            flush_pulses();
        }

        _channels = 0;

        return;
    }

    if (_channels == -1)
    {
        // we are not synchronised
        //
        return;
    }

    // we limit inputs to a limited range of pulse widths.
    //
    // This allows us to decode SBUS on the same pin,
    // as SBUS will have a maximum pulse width of 100usec
    //
    if ((pulse_ticks > PWM_CAPTURE_PULSE_WIDTH_MIN) && (pulse_ticks < PWM_CAPTURE_PULSE_WIDTH_MAX))
    {
        // take a reading for the current channel
        //
        _pulses_ticks[_channels][0] = width_s0;
        _pulses_ticks[_channels][1] = width_s1;

        // move to next channel
        //
        _channels++;
    }

    // if we have reached the maximum supported channels then
    // mark as unsynchronised, so we wait for a wide pulse
    //
    if (_channels >= RC_INPUT_NUM_CHANNELS_MAX)
    {
        flush_pulses();

        _channels = -1;

        _pulses_stat[0].reset();
        _pulses_stat[1].reset();
    }
}
