/******************************************************************************
 *
 * Usage example:
 *
 ******************************************************************************
 *
 * // To store current servo input pins
 * uint8_t servo_pins;
 *
 * // Servo input pin storage 
 * static uint8_t servo_pins_old = 0;
 * 
 * // Read current servo timer
 * uint16_t servo_time = SERVO_TIMER_CNT;
 * 
 * // Store current PPM inputs pins
 * servo_pins = SERVO_INPUT;
 *
 * // Calculate servo input pin change mask
 * uint8_t servo_change = servo_pins ^ servo_pins_old;
 *
 * if (servo_input_mode == PPM_REDUDANCY)
 * {
 *     // -------------------------------------
 *     // PPM redundancy mode - variables Init
 *     // -------------------------------------	
 * 
 *     static PPMInput ppm1
 *     (
 *         _RCInput::PPM::Primary::NumChannels::_min,
 *         _RCInput::PPM::Primary::NumChannels::_max,
 *         _RCInput::PPM::Primary::_switchover_channel,
 *         _RCInput::PPM::Primary::PulseWidth::_switch,
 *         _RCInput::PPM::Primary::PulseWidth::_pre,
 *         _RCInput::PPM::Primary::PulseWidth::_min,
 *         _RCInput::PPM::Primary::PulseWidth::_max,
 *         _RCInput::PPM::Primary::PulseWidth::_frame_period
 *     );
 *
 *     static PPMInput ppm2
 *     (
 *         _RCInput::PPM::Secondary::NumChannels::_min,
 *         _RCInput::PPM::Secondary::NumChannels::_max,
 *         0,
 *         0xffff,
 *         _RCInput::PPM::Secondary::PulseWidth::_pre,
 *         _RCInput::PPM::Secondary::PulseWidth::_min,
 *         _RCInput::PPM::Secondary::PulseWidth::_max,
 *         _RCInput::PPM::Secondary::PulseWidth::_frame_period
 *     );
 *
 *     // -----------------------------------
 *     // decoder
 *     // -----------------------------------
 *
 *     // -----------------------------------
 *     // Ch1 decoding
 *     // -----------------------------------
 * 
 *     ppm1.preprocess(servo_change & 1, servo_pins & 1, servo_time);
 *
 *     // -----------------------------------
 *     // Ch2 decoding
 *     // -----------------------------------
 * 
 *     ppm2.preprocess(servo_change & 2, servo_pins & 2, servo_time);
 *
 *     // -----------------------------------
 *     // Post processing
 *     // -----------------------------------
 *     
 *     // Could be eventually run in the main loop for performances improvements if necessary
 *     // sync mode between input and ouptput should clear performance problems
 *     
 *     // -----------------
 *     // Switchover code
 *     // -----------------
 *
 *     const Pulse pulse = ppm1.switch_to(ppm2);
 *
 *     if (pulse != PPMInput::Pulse::Type::INVALID)
 *     {
 *         if (pulse != PPMInput::Pulse::Type::FAILSAFE)
 *         {
 *             ppm[pulse._channel] = pulse._width;
 *         }
 *         else
 *         {
 *             // ppm[...] = failsafe_values[...];
 *         }
 *     }
 *         
 *     // -----------------------------------
 *     // Channel count post processing code
 *     // -----------------------------------
 *     
 *     // To enhance: possible global detection flag to avoid 2 channel_count_detected tests
 * 
 *     // -----------------------------------
 *     // Ch1
 *     // -----------------------------------
 * 
 *     ppm1.channel_count();
 * 
 *     // -----------------------------------
 *     // Ch2
 *     // -----------------------------------
 * 
 *     ppm2.channel_count();
 * }
 *
 * // Store current servo input pins for next check
 * servo_pins_old = servo_pins;
 *
 *****************************************************************************/
pragma once;

class PPMInput
{
public:
    class Pulse
    {
        enum Type
        {
            INVALID  = 0,
            FAILSAFE = 0xffff
        };

        uint8_t  _channel;
        uint16_t _width  ;

        Pulse(): _channel(0), _width(0) {}

        Pulse(const Pulse& ref): _channel(ref._channel), _width(ref._width) {}

        Pulse & operator =(const PPMInput &ref)
        {
            _channel = ref._channel;
            _width   = ref._width[ref._channel];
        }

        Pulse & operator =(const Pulse &ref)
        {
            _channel = ref._channel;
            _width   = ref._width;
        }

        Pulse & operator =(const Pulse::Type &val)
        {
            _channel = 0xff;
            _width   = (uint16_t)val;
        }

        bool operator ==(const Pulse::Type &val)
        {
            return ((_channel == 0xff) && (_width == (uint16_t)val));
        }

        bool operator !=(const Pulse::Type &val)
        {
            return ((_channel != 0xff) || (_width != (uint16_t)val));
        }
    };

    PPMInput();

    PPMInput(uint8_t min_channels, uint8_t max_channels, uint8_t switch_channel,
             uint16_t switch_threshold,
             uint8_t pulsewidth_pre, uint8_t pulsewidth_min, uint8_t pulsewidth_max,
             uint16_t frame_len);

    void preprocess(bool pin_changed, bool pin_high, uint16_t current_time);
    const Pulse switch_to(PPMInput &other);
    void channel_count();

private:
	/* prepulse start
    */
	uint16_t _prepulse_start;
	
	/* prepulse width
    */
	uint16_t _prepulse_width;
	
	/* pulse start time
    */
	uint16_t _start[RC_INPUT_NUM_CHANNELS_MAX];
	
	/* pulse lenght
    */
	uint16_t _width[RC_INPUT_NUM_CHANNELS_MAX];
			
	/* Reset channels ( 0 = Sync Symbol )
    */
	uint8_t _channel;
	
	/* Frame sync flag
    */
	bool _sync;
	
	/* Channel error flags
    */
	bool _channel_error;
	
	/* Sync error flags
    */
	bool _sync_error;
	
	/* switchover flag
    */
	bool _switchover;
	
	/* Channel count detection ready flag
    */
	bool _channel_count_ready;
	
	/* Channel count detected flag
    */
	bool _channel_count_detected;
	
	/* Detected Channel count
    */
	uint8_t _channel_count;
	
	/* Detected Channel count previous value
    */
	uint8_t _previous_channel_count;
	
	/* Channel count detection counter
    */
	uint8_t _channel_count_detection_counter;

    /* Configuration parameters
    */
    uint8_t  _switch_channel  ;
    uint16_t _switch_threshold;
    uint16_t _frame_len       ;
    uint16_t _pulsewidth_pre  ;
    uint16_t _pulsewidth_min  ;
    uint16_t _pulsewidth_max  ;
    uint8_t  _max_channels    ;
    uint8_t  _min_channels    ;
};
