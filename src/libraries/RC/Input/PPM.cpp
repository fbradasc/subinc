#define PPM_STANDARD  1  // Standard PPM : 1520 us +/- 600 us -  8 channels - 20   ms frame period
#define PPM_EXTENDED  2  // 9 channels   : 1520 us +/- 600 us -  9 channels - 22.5 ms frame period
#define PPM_V2        3  // PPMv2        :  760 us +/- 300 us - 16 channels - 20   ms frame period
#define PPM_V3        4  // PPMv3        : 1050 us +/- 300 us - 16 channels - 25   ms frame period

// PPM input frame mode
// -------------------------------------------------------------
//#define PPM_MODE PPM_STANDARD
//#define PPM_MODE PPM_EXTENDED
//#define PPM_MODE PPM_V2
//#define PPM_MODE PPM_V3

// PPM input : frame formats definitions
// -------------------------------------------------------------
#if ( PPM_MODE == PPM_STANDARD )

#  define PPM_CH_MIN                 4
#  define PPM_CH_MAX                 8
#  define PPM_PW_FRAME_PERIOD    20000   // frame period (microseconds)
#  define PPM_PW_MIN               920
#  define PPM_PW_MAX              2120
#  define PPM_PW_PREPULSE_LENGHT   400

#elif ( PPM_MODE == PPM_EXTENDED )

#  define PPM_CH_MIN                 4
#  define PPM_CH_MAX                 9
#  define PPM_PW_FRAME_PERIOD    22500   // frame period (microseconds)
#  define PPM_PW_MIN               920
#  define PPM_PW_MAX              2120
#  define PPM_PW_PREPULSE_LENGHT   400

#elif ( PPM_MODE == PPM_V2 ) // PPMv2 is a 50 Hz 16 channels mode

#  define PPM_CH_MIN                 4
#  define PPM_CH_MAX                16
#  define PPM_PW_FRAME_PERIOD    20000   // frame period (microseconds)
#  define PPM_PW_MIN               460
#  define PPM_PW_MAX              1060
#  define PPM_PW_PREPULSE_LENGHT   200

#elif ( PPM_MODE == PPM_V3 ) //  PPMv3 is a 40 Hz slower refresh rate 16 channels mode

#  define PPM_CH_MIN                 4
#  define PPM_CH_MAX                16
#  define PPM_PW_FRAME_PERIOD    25000   // frame period (microseconds)
#  define PPM_PW_MIN               750
#  define PPM_PW_MAX              1350
#  define PPM_PW_PREPULSE_LENGHT   400

#else

#  error "PPM_MODE not defined"

#endif

namespace _RCInput
{
    namespace PPM
    {
        namespace PulseWidth
        {
            DECLARE_FIELD( PPulseWidth, RCInput.PPM.PulseWidth, _frame_period, PPM_PW_FRAME_PERIOD    );
            DECLARE_FIELD( PPulseWidth, RCInput.PPM.PulseWidth, _pre         , PPM_PW_PREPULSE_LENGHT );
            DECLARE_FIELD( PPulseWidth, RCInput.PPM.PulseWidth, _min         , PPM_PW_MIN             );
            DECLARE_FIELD( PPulseWidth, RCInput.PPM.PulseWidth, _max         , PPM_PW_MAX             );
        };
        namespace NumChannels
        {
            DECLARE_FIELD( PUInt8, RCInput.PPM.NumChannels, _min, PPM_CH_MIN );
            DECLARE_FIELD( PUInt8, RCInput.PPM.NumChannels, _max, PPM_CH_MAX );
        };
    };
};

#if 1

#define PPM_CAPTURE_PULSE_WIDTH_MIN   PPM_PROFILES[_profile].pwm_pulse_min;
#define PPM_CAPTURE_PULSE_WIDTH_MAX   PPM_PROFILES[_profile].pwm_pulse_max;
#define PPM_CAPTURE_MIN_SYNC_PULSE_W  PPM_PROFILES[_profile].syn_pulse_min;

#define SWITCH_THRESHOLD_L        50
#define SWITCH_THRESHOLD_H        25

#define PREPULSE_CAGE             50 // used to auto detect the PPM profile
#define FRAME_CAGE               500 // used to auto detect the PPM profile


#define MATCH_IN_RANGE(a,b,r)    (((a) >= ((b) - (r))) && ((a) <= ((b) + (r))))

inline bool PPM::guess_ppm_profile()
{
    if (!_profile)
    {
        for (uint8_t i; i<ARRAY_SIZE(PPM_PROFILES); i++)
        {
            if (MATCH_IN_RANGE((_pulses_stat[0].sum + _pulses_stat[1].sum), PPM_PROFILES[i].frame_length, FRAME_CAGE))
            {
                _swtch_pulse_ndx = (_pulses_stat[0].average < _pulses_stat[1].average) ? 0 : 1;

                if (MATCH_IN_RANGE((_pulses_stat[_swtch_pulse_ndx].average), PPM_PROFILES[i].pre_pulse_max, PREPULSE_CAGE))
                {
                    _profile = i;

                    break;
                }
            }
        }
    }
}

inline pulse_width_t PPM::scale(const pulse_width_t unscaled)
{
    if ( unscaled < PPM_CAPTURE_PULSE_WIDTH_MIN )
    {
        // shall issue an error
        //
        return PULSE_WIDTH_MIN;
    }
    else
    if ( unscaled == PPM_CAPTURE_PULSE_WIDTH_MIN )
    {
        return PULSE_WIDTH_MIN;
    }
    else
    if ( unscaled > PPM_CAPTURE_PULSE_WIDTH_MAX )
    {
        // shall issue an error
        //
        return PULSE_WIDTH_MAX;
    }
    else
    if ( unscaled == PPM_CAPTURE_PULSE_WIDTH_MAX )
    {
        return PULSE_WIDTH_MAX;
    }

    return unscaled * PULSE_WIDTH_MAX / ( PPM_CAPTURE_PULSE_WIDTH_MAX - PPM_CAPTURE_PULSE_WIDTH_MIN );
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

        if ( _pulses_ticks[i][_swtch_pulse_ndx] < (PPM_PROFILES[_profile].pre_pulse_max - SWITCH_THRESHOLD_L) )
        {
            _listener._switches ^= ( 1 << ( i << 1 ) );
        }
        else
        if ( _pulses_ticks[i][_swtch_pulse_ndx] > (PPM_PROFILES[_profile].pre_pulse_max - SWITCH_THRESHOLD_H) )
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

    if (_channels >= 0)
    {
        _pulses_stat[0].update(width_s0);
        _pulses_stat[1].update(width_s1);
    }

    // TODO: calculate PPM_CAPTURE_MIN_SYNC_PULSE_W

    if (pulse_ticks >= PPM_CAPTURE_MIN_SYNC_PULSE_W)
    {
        // a long pulse indicates the end of a frame. Reset the
        // channel counter so next pulse is channel 0
        //
        if (_channels >= PPM_CAPTURE_NUM_CHANNELS_MIN)
        {
            guess_ppm_profile();

            flush_pulses();
        }

        _channels = 0;

        _pulses_stat[0].reset();
        _pulses_stat[1].reset();

        return;
    }

    if (_channels == -1)
    {
        // we are not synchronised
        //
        _pulses_stat[0].reset();
        _pulses_stat[1].reset();

        return;
    }

    // we limit inputs to between 700usec and 2300usec. This allows us
    // to decode SBUS on the same pin, as SBUS will have a maximum
    // pulse width of 100usec
    //
    if ((pulse_ticks > PPM_CAPTURE_PULSE_WIDTH_MIN) && (pulse_ticks < PPM_CAPTURE_PULSE_WIDTH_MAX))
    {
        // take a reading for the current channel
        // buffer these
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
#else
// process a PPM-sum pulse of the given width
//
void PPM::process_pulse(const pulse_width_t width_s0, const pulse_width_t width_s1)
{
    const pulse_width_t pulse_ticks = width_s0 + width_s1;

    static const uint8_t       PPM_CAPTURE_NUM_CHANNELS_MIN = _RCInput::PPM::NumChannels::_min;
    static const uint8_t       PPM_CAPTURE_NUM_CHANNELS_MAX = _RCInput::PPM::NumChannels::_max;
    static const pulse_width_t PPM_CAPTURE_PULSE_WIDTH_MIN  = Timer.usToTicks(_RCInput::PPM::PulseWidth::_min);
    static const pulse_width_t PPM_CAPTURE_PULSE_WIDTH_MAX  = Timer.usToTicks(_RCInput::PPM::PulseWidth::_max);
    static const pulse_width_t PPM_CAPTURE_MIN_SYNC_PULSE_W = Timer.usToTicks
    (
        _RCInput::PPM::PulseWidth::_frame_period
        -
        (
            _RCInput::PPM::NumChannels::_max
            *
            _RCInput::PPM::PulseWidth::_max
        )
        -
        _RCInput::PPM::PulseWidth::_pre
    );

    if (pulse_ticks >= PPM_CAPTURE_MIN_SYNC_PULSE_W)
    {
        // a long pulse indicates the end of a frame. Reset the
        // channel counter so next pulse is channel 0
        //
        if (_channels >= PPM_CAPTURE_NUM_CHANNELS_MIN)
        {
            for (uint8_t i=0; i<_channels; i++)
            {
                _listener._pulse_capt[i] = _pulses_ticks[i];
            }

            _listener._num_channels = _channels;

            _listener._new_input    = true;
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

    // we limit inputs to between 700usec and 2300usec. This allows us
    // to decode SBUS on the same pin, as SBUS will have a maximum
    // pulse width of 100usec
    //
    if ((pulse_ticks > PPM_CAPTURE_PULSE_WIDTH_MIN) && (pulse_ticks < PPM_CAPTURE_PULSE_WIDTH_MAX))
    {
        // take a reading for the current channel
        // buffer these
        //
        _pulses_ticks[_channels] = pulse_ticks;

        // move to next channel
        //
        _channels++;
    }

    // if we have reached the maximum supported channels then
    // mark as unsynchronised, so we wait for a wide pulse
    //
    if (_channels >= RC_INPUT_NUM_CHANNELS_MAX)
    {
        for (uint8_t i=0; i<_channels; i++)
        {
            _listener._pulse_capt[i] = _pulses_ticks[i];
        }

        _listener._num_channels = _channels;

        _listener._new_input    = true;

        _channels = -1;
    }
}
#endif
