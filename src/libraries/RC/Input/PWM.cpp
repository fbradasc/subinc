#include <RC/Input/PWM.h>

// -------------------------------------------------------------
// SERVO PWM MODE input settings
// -------------------------------------------------------------

#define PWM_CH_NUM             8
#define PWM_PW_MIN           920
#define PWM_PW_MAX          2120

namespace _RCInput
{
    namespace PWM
    {
        DECLARE_FIELD( PUInt8     , RCInput.PWM, _num_channels  , PWM_CH_NUM );
        DECLARE_FIELD( PPulseWidth, RCInput.PWM, _jitter_filter , SHRT_MIN   );
        DECLARE_FIELD( PBool      , RCInput.PWM, _average_filter, false      );

        namespace PulseWidth
        {
            DECLARE_FIELD( PPulseWidth, RCInput.PWM.PulseWidth, _min, PWM_PW_MIN );
            DECLARE_FIELD( PPulseWidth, RCInput.PWM.PulseWidth, _max, PWM_PW_MAX );
        };
    };
};

void PWM::process_pulse(const pulse_width_t curr_time, const uint8_t input_pins)
{
    static const pulse_width_t PWM_PULSEWIDTH_MIN   = _RCInput::PWM::PulseWidth::_min;
    static const pulse_width_t PWM_PULSEWIDTH_MAX   = _RCInput::PWM::PulseWidth::_max;
    static const pulse_width_t PWM_JITTER_FILTER    = _RCInput::PWM::_jitter_filter;
    static const uint8_t       PWM_NUM_CHANNELS_MAX = _RCInput::PWM::_num_channels;
    static const bool          PWM_AVERAGE_FILTER   = _RCInput::PWM::_average_filter;

    // Calculate input pin change mask
    //
    uint8_t input_change = input_pins ^ _old_input_pins;
    
    if ( input_change )
    {
        for
        (
            unit8_t cur_channel = 0,
                    input_pin   = 1
            ;
            input_change && ( cur_channel < PWM_NUM_CHANNELS_MAX )
            ;
            cur_channel++,
            input_pin <<= 1
        )
        {
            // Check for pin change on current input channel
            //
            if ( input_change & input_pin )
            {
                // Remove processed pin change from bitmask
                //
                input_change &= ~input_pin;

                // High (raising edge)
                //
                if ( input_pins & input_pin )
                {
                    _prev_time[cur_channel] = curr_time;
                }
                else
                {
                    // Get pulse width
                    //
                    pulse_width_t pulse_ticks = Timer.difftime(curr_time, _prev_time[cur_channel]);
                    
                    // Check that pulse signal is valid
                    //
                    if ((pulse_ticks < PWM_PULSEWIDTH_MIN) || (pulse_ticks > PWM_PULSEWIDTH_MAX))
                    {
                        // Invalid input signals --> TODO need to notify ?
                        //
                        continue;
                    }
                    
                    if (PWM_AVERAGE_FILTER)
                    {
                        // Average filter to smooth input jitter
                        //
                        pulse_ticks += _listener._pulse_capt[cur_channel];

                        pulse_ticks >>= 1;
                    }

                    if (PWM_JITTER_FILTER > 0)
                    {
                        // 0.5us cut filter to remove input jitter
                        //
                        pulse_width_t ppm_tmp = _listener._pulse_capt[cur_channel] - pulse_ticks;

                        if ( ppm_tmp <= PWM_JITTER_FILTER && ppm_tmp >= -PWM_JITTER_FILTER )
                        {
                            continue;
                        }
                    }

                    // Update _pulse_capt[..]
                    //
                    _listener._pulse_capt[cur_channel] = pulse_ticks;
                }
            }
        }

        // Store current input pins for next check
        //
        _old_input_pins = input_pins;

        _listener._num_channels = PWM_NUM_CHANNELS_MAX;

        _listener._new_input    = true;
    }
}
