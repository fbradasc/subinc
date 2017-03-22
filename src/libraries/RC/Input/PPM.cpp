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

// process a PPM-sum pulse of the given width
//
void PPM::process_pulse(const pulse_width_t pulse_ticks)
{
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
