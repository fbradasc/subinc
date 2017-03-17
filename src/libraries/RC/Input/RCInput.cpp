/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <climits> // for SHRT_MIN

#include <RC/Input/RCInput.h>

#include <Debug/Debug.h>

#define PPM_STANDARD  1  // Standard PPM : 1520 us +/- 600 us -  8 channels - 20   ms frame period
#define PPM_EXTENDED  2  // 9 channels   : 1520 us +/- 600 us -  9 channels - 22.5 ms frame period
#define PPM_V2        3  // PPMv2        :  760 us +/- 300 us - 16 channels - 20   ms frame period
#define PPM_V3        4  // PPMv3        : 1050 us +/- 300 us - 16 channels - 25   ms frame period

// PPM input frame mode receiver 1
// -------------------------------------------------------------
//#define PPM_PRI PPM_STANDARD
//#define PPM_PRI PPM_EXTENDED
//#define PPM_PRI PPM_V2
//#define PPM_PRI PPM_V3

// PPM input frame mode receiver 2
// -------------------------------------------------------------
//#define PPM_SEC PPM_STANDARD
//#define PPM_SEC PPM_EXTENDED
//#define PPM_SEC PPM_V2
//#define PPM_SEC PPM_V3

// PPM1 input : frame formats definitions
// -------------------------------------------------------------
#if ( PPM_PRI == PPM_STANDARD )

#  define PPM_PRI_CH_MIN                 4
#  define PPM_PRI_CH_MAX                 8
#  define PPM_PRI_PW_FRAME_PERIOD    20000   // frame period (microseconds)
#  define PPM_PRI_PW_MIN               920
#  define PPM_PRI_PW_MAX              2120
#  define PPM_PRI_PW_SWITCH           1800
#  define PPM_PRI_PW_PREPULSE_LENGHT   400

#elif ( PPM_PRI == PPM_EXTENDED )

#  define PPM_PRI_CH_MIN                 4
#  define PPM_PRI_CH_MAX                 9
#  define PPM_PRI_PW_FRAME_PERIOD    22500   // frame period (microseconds)
#  define PPM_PRI_PW_MIN               920
#  define PPM_PRI_PW_MAX              2120
#  define PPM_PRI_PW_SWITCH           1800
#  define PPM_PRI_PW_PREPULSE_LENGHT   400

#elif ( PPM_PRI == PPM_V2 ) // PPMv2 is a 50 Hz 16 channels mode

#  define PPM_PRI_CH_MIN                 4
#  define PPM_PRI_CH_MAX                16
#  define PPM_PRI_PW_FRAME_PERIOD    20000   // frame period (microseconds)
#  define PPM_PRI_PW_MIN               460
#  define PPM_PRI_PW_MAX              1060
#  define PPM_PRI_PW_SWITCH            900
#  define PPM_PRI_PW_PREPULSE_LENGHT   200

#elif ( PPM_PRI == PPM_V3 ) //  PPMv3 is a 40 Hz slower refresh rate 16 channels mode

#  define PPM_PRI_CH_MIN                 4
#  define PPM_PRI_CH_MAX                16
#  define PPM_PRI_PW_FRAME_PERIOD    25000   // frame period (microseconds)
#  define PPM_PRI_PW_MIN               750
#  define PPM_PRI_PW_MAX              1350
#  define PPM_PRI_PW_SWITCH           1260
#  define PPM_PRI_PW_PREPULSE_LENGHT   400

#else

#  error "PPM_PRI not defined"

#endif

// PPM2 input : frame formats definitions
// -------------------------------------------------------------
#if ( PPM_SEC == PPM_STANDARD )

#  define PPM_SEC_CH_MIN                 4
#  define PPM_SEC_CH_MAX                 8
#  define PPM_SEC_PW_FRAME_PERIOD    20000   // frame period (microseconds)
#  define PPM_SEC_PW_MIN               920
#  define PPM_SEC_PW_MAX              2120
#  define PPM_SEC_PW_SWITCH           1800
#  define PPM_SEC_PW_PREPULSE_LENGHT   400

#elif (PPM_SEC == PPM_EXTENDED )

#  define PPM_SEC_CH_MIN                 4
#  define PPM_SEC_CH_MAX                 9
#  define PPM_SEC_PW_FRAME_PERIOD    22500   // frame period (microseconds)
#  define PPM_SEC_PW_MIN               920
#  define PPM_SEC_PW_MAX              2120
#  define PPM_SEC_PW_SWITCH           1800
#  define PPM_SEC_PW_PREPULSE_LENGHT   400

#elif ( PPM_SEC == PPM_V2 ) // PPMv2 is a 50 Hz 16 channels mode

#  define PPM_SEC_CH_MIN                 4
#  define PPM_SEC_CH_MAX                16
#  define PPM_SEC_PW_FRAME_PERIOD    20000   // frame period (microseconds)
#  define PPM_SEC_PW_MIN               460
#  define PPM_SEC_PW_MAX              1060
#  define PPM_SEC_PW_SWITCH            900
#  define PPM_SEC_PW_PREPULSE_LENGHT   200

#elif ( PPM_SEC == PPM_V3 ) //  PPMv3 is a 40 Hz slower refresh rate 16 channels mode

#  define PPM_SEC_CH_MIN                 4
#  define PPM_SEC_CH_MAX                16
#  define PPM_SEC_PW_FRAME_PERIOD    25000   // frame period (microseconds)
#  define PPM_SEC_PW_MIN               750
#  define PPM_SEC_PW_MAX              1350
#  define PPM_SEC_PW_SWITCH           1260
#  define PPM_SEC_PW_PREPULSE_LENGHT   400

#else

#  error "PPM_SEC not defined"

#endif

#define PPM_SWITCHOVER_CHANNEL 9

// -------------------------------------------------------------
// SERVO PWM MODE input settings
// -------------------------------------------------------------

#define PWM_CH_NUM             8
#define PWM_PW_MIN           920
#define PWM_PW_MAX          2120

namespace _RCInput
{
    namespace PPM
    {
        namespace Primary
        {
            DECLARE_FIELD( PUInt8, RCInput.PPM.Primary, _switchover_channel, PPM_SWITCHOVER_CHANNEL );

            namespace PulseWidth
            {
                DECLARE_FIELD( PUInt16, RCInput.PPM.Primary.PulseWidth, _frame_period, PPM_PRI_PW_FRAME_PERIOD    );
                DECLARE_FIELD( PUInt16, RCInput.PPM.Primary.PulseWidth, _pre         , PPM_PRI_PW_PREPULSE_LENGHT );
                DECLARE_FIELD( PUInt16, RCInput.PPM.Primary.PulseWidth, _min         , PPM_PRI_PW_MIN             );
                DECLARE_FIELD( PUInt16, RCInput.PPM.Primary.PulseWidth, _switch      , PPM_PRI_PW_SWITCH          );
                DECLARE_FIELD( PUInt16, RCInput.PPM.Primary.PulseWidth, _max         , PPM_PRI_PW_MAX             );
            };
            namespace NumChannels
            {
                DECLARE_FIELD( PUInt8, RCInput.PPM.Primary.NumChannels, _min, PPM_PRI_CH_MIN );
                DECLARE_FIELD( PUInt8, RCInput.PPM.Primary.NumChannels, _max, PPM_PRI_CH_MAX );
            };
        };
        namespace Secondary
        {
            namespace PulseWidth
            {
                DECLARE_FIELD( PUInt16, RCInput.PPM.Secondary.PulseWidth, _frame_period, PPM_SEC_PW_FRAME_PERIOD    );
                DECLARE_FIELD( PUInt16, RCInput.PPM.Secondary.PulseWidth, _pre         , PPM_SEC_PW_PREPULSE_LENGHT );
                DECLARE_FIELD( PUInt16, RCInput.PPM.Secondary.PulseWidth, _min         , PPM_SEC_PW_MIN             );
                DECLARE_FIELD( PUInt16, RCInput.PPM.Secondary.PulseWidth, _max         , PPM_SEC_PW_MAX             );
            };
            namespace NumChannels
            {
                DECLARE_FIELD( PUInt8, RCInput.PPM.Secondary.NumChannels, _min, PPM_SEC_CH_MIN );
                DECLARE_FIELD( PUInt8, RCInput.PPM.Secondary.NumChannels, _max, PPM_SEC_CH_MAX );
            };
        };
    };
    namespace PWM
    {
        DECLARE_FIELD( PUInt8, RCInput.PWM, _num_channels   , PWM_CH_NUM );
        DECLARE_FIELD( PUInt8, RCInput.PWM, _jitter_filter  , SHRT_MIN   );
        DECLARE_FIELD( PUInt8, RCInput.PWM, _average_filters, false      );

        namespace PulseWidth
        {
            DECLARE_FIELD( PUInt16, RCInput.PWM.PulseWidth, _min, PWM_PW_MIN );
            DECLARE_FIELD( PUInt16, RCInput.PWM.PulseWidth, _max, PWM_PW_MAX );
        };
    };
};

volatile uint16_t RCInput::_pulse_capt[RC_INPUT_NUM_CHANNELS_MAX] = {0};
volatile bool     RCInput::_new_input                             = false;
volatile uint8_t  RCInput::_num_channels                          = 0;
volatile uint16_t RCInput::_min_pulsewidth                        = 0;
volatile uint16_t RCInput::_max_pulsewidth                        = 0;

/* Constrain captured pulse to be between min and max pulsewidth.
 */
inline uint16_t RCInput::pw_crop(uint16_t p)
{
    if (p > _max_pulsewidth)
    {
        return _max_pulsewidth;
    }

    if (p < _min_pulsewidth)
    {
        return _min_pulsewidth;
    }
                               
    return p;
}

/* Convert the pulse width to an absolute value
 *
 * Input:
 *      p : [_min_pulsewidth,_max_pulsewidth]
 *
 * Return:
 *
 *      v : [0,0x7fff]
 */
inline uint16_t RCInput::pw_value(uint16_t p)
{
    return ( 0x7fff * ( p - _min_pulsewidth ) / ( _max_pulsewidth - _min_pulsewidth ) ) & 0x7fff;
}

RCInput::RCInput()
{
    clear_overrides();
}

// AVR parameters for PhoneDrone and APM2 boards using ATmega32u2
#if defined (__AVR_ATmega16U2__) || defined (__AVR_ATmega32U2__) || defined (__AVR_ATmega328P__) || defined (__AVR_ATmega328__)
#  define enter_cs()  uint8_t __AVR_Critical_Section_SREG__ = SREG; cli();
#  define leave_cs()  SREG = __AVR_Critical_Section_SREG__;
#  define pw_scale(p) ((p) >> 1) // scale pulse from 0.5us units to 1us units.
#  define delay_us(d) _delay_us((d))
#else
#  define enter_cs()
#  define leave_cs()
#  define pw_scale(p) (p)
#  define delay_us(d) 
#endif

#define non_blocking_read(to, from) for(to=(from); to!=(from); to=(from));

#if defined (__AVR_ATmega16U2__) || defined (__AVR_ATmega32U2__)

#  define SERVO_DDR                  DDRB
#  define SERVO_PORT                 PORTB
#  define SERVO_INPUT                PINB
#  define SERVO_INT_VECTOR           PCINT0_vect
#  define SERVO_INT_MASK             PCMSK0
#  define SERVO_INT_CLEAR_FLAG       PCIF0
#  define SERVO_INT_ENABLE           PCIE0
#  define SERVO_TIMER_CNT            TCNT1
                                    
#  define PPM_DDR                    DDRC
#  define PPM_PORT                   PORTC
#  define PPM_OUTPUT_PIN             PC6
#  define PPM_INT_VECTOR             TIMER1_COMPA_vect
#  define PPM_COMPARE                OCR1A
#  define PPM_COMPARE_FLAG           COM1A0
#  define PPM_COMPARE_ENABLE         OCIE1A
#  define PPM_COMPARE_FORCE_MATCH    FOC1A

#  define TIMER_REG                  ICR4

#elif defined (__AVR_ATmega328P__) || defined (__AVR_ATmega328__)

#  define SERVO_DDR                  DDRD
#  define SERVO_PORT                 PORTD
#  define SERVO_INPUT                PIND
#  define SERVO_INT_VECTOR           PCINT2_vect
#  define SERVO_INT_MASK             PCMSK2
#  define SERVO_INT_CLEAR_FLAG       PCIF2
#  define SERVO_INT_ENABLE           PCIE2
#  define SERVO_TIMER_CNT            TCNT1
                                    
#  define PPM_DDR                    DDRB
#  define PPM_PORT                   PORTB
#  define PPM_OUTPUT_PIN             PB2
#  define PPM_INT_VECTOR             TIMER1_COMPB_vect
#  define PPM_COMPARE                OCR1B
#  define PPM_COMPARE_FLAG           COM1B0
#  define PPM_COMPARE_ENABLE         OCIE1B
#  define PPM_COMPARE_FORCE_MATCH    FOC1B

#  define TIMER_REG                  ICR4

#else

#error NO SUPPORTED DEVICE FOUND! (ATmega16u2 / ATmega32u2 / ATmega328p)

#endif

#define TIMER_ROLLUP               40000

#define Servo.enableInput(p)       SERVO_DDR &= ~(1 << (p)); SERVO_PORT |=  (1 << (p));
#define Servo.enableOutput(p)      SERVO_DDR |=  (1 << (p));
#define Servo.get(p)               (SERVO_INPUT &   (1 << (p)))
#define Servo.set(p)               (SERVO_PORT  |=  (1 << (p)))
#define Servo.clear(p)             (SERVO_PORT  &= ~(1 << (p)))
#define Servo.reset()              SERVO_DDR = 0x00; SERVO_PORT |= 0xff;
#define Servo.setInterruptsMask(m) SERVO_INT_MASK = (m);
#define Timer.get()                TIMER_REG_VALUE

void RCInput::init(void* implspecific)
{
    clear_overrides();

    //
    // detect if PPM, PPM redudancy or PWM
    //
    uint8_t pin2_status = 0;
    uint8_t pin4_status = 0;

    Servo.enableInput (1); // Set pin 2 to input
    Servo.enableOutput(2); // Set pin 3 to output
    Servo.enableInput (3); // Set pin 4 to input

    Servo.clear(2); // Set pin 3 output low
    { 
        delay_us(10);
                
        if ( Servo.get(1) == 0 )
        {
            pin2_status++;
        }

        if ( Servo.get(3) == 0 )
        {
            pin4_status++;
        }
    }

    Servo.set(2);
    {
        delay_us(10);
        
        if ( Servo.get(1) != 0 )
        {
            pin2_status++;
        }

        if ( Servo.get(3) != 0 )
        {
            pin4_status++;
        }
    }

    Servo.clear(2);
    {
        delay_us(10);

        if ( Servo.get(1) == 0 )
        {
            pin2_status++;
        }

        if ( Servo.get(3) == 0 )
        {
            pin4_status++;
        }
    }

    // RESET SERVO/PPM PINS AS INPUTS WITH PULLUPS
    //
    Servo.reset();
    
    if ( pin2_status == 3 )
    {
        // PPM single line mode
        //
        // Set servo input interrupt pin mask to servo input channel 1
        //
        Servo.setInterruptsMask(0b00000001);

        register_signal(RCInput::ppm_capture_cb);
    }
    else
    if ( pin4_status == 3 ) 
    {
        // PPM redudancy mode
        //
        // Set servo input interrupt pin mask to servo input channel 1 and 2
        //
        Servo.setInterruptsMask(0b00000011);

        register_signal(RCInput::ppm_redudancy_capture_cb);
    }
    else
    {
        // PWM mode - max 8 channels
        //
        // Set servo input interrupt pin mask to all 8 servo input channels
        //
        Servo.setInterruptsMask(0b11111111);

        register_signal(RCInput::pwm_capture_cb);
    }
}

void RCInput::deinit()
{
}

uint16_t RCInput::read(uint8_t ch)
{
    /* constrain ch
     */
    if (ch >= num_channels())
    {
        return 0;
    }

    if (_overrides[ch] >= 0)
    {
        return _overrides[ch];
    }

    uint16_t capt;

    non_blocking_read(capt, _pulse_capt[ch]);

    return pw_value(pw_crop(pw_scale(capt)));
}

uint8_t RCInput::read(uint16_t* values, uint8_t len)
{
    /* constrain len 
     */
    if (len > num_channels())
    {
        len = num_channels();
    }

    if (len > 0)
    {
        CriticalSection.enter();

        for (uint8_t i = 0; i < len; i++)
        {
            non_blocking_read(values[i], _pulse_capt[i]);
        }

        leave_cs();
    }

    /* Outside of critical section, do the math (in place) to scale and
     * constrain the pulse.
     */
    for (uint8_t i = 0; i < len; i++)
    {
        values[i] = (_overrides[i] >= ChannelValueRange::MIN) ? _overrides[i] : pw_value(pw_crop(pw_scale(values[i])));
    }

    return num_channels();
}

int16_t RCInput::get_override(uint8_t channel) 
{
    if (channel < RC_INPUT_NUM_CHANNELS_MAX)
    {
        return _overrides[channel];
    }

    return ChannelValueRange::NOP;
}

uint8_t RCInput::get_overrides(int16_t *overrides, uint8_t len) 
{
    if (NULL == overrides)
    {
        return 0;
    }

    if (len > RC_INPUT_NUM_CHANNELS_MAX)
    {
        len = RC_INPUT_NUM_CHANNELS_MAX;
    }

    for (uint8_t i = 0; i < len; i++)
    {
        overrides[i] = _overrides[i];
    }
    
    return len;
}

bool RCInput::set_overrides(int16_t *overrides, uint8_t len) 
{
    bool res = false;

    if (len > RC_INPUT_NUM_CHANNELS_MAX)
    {
        len = RC_INPUT_NUM_CHANNELS_MAX;
    }

    for (uint8_t i = 0; i < len; i++)
    {
        res |= set_override(i, overrides[i]);
    }
    
    return res;
}

bool RCInput::set_override(uint8_t channel, int16_t override) 
{
    if ((channel < RC_INPUT_NUM_CHANNELS_MAX) && (_overrides[channel] != override))
    {
        _overrides[channel] = override;

        if (override >= ChannelValueRange::MIN)
        {
            if (channel < num_channels())
            {
                _new_input = true;
            }

            return true;
        }
    }

    return false;
}

void RCInput::clear_overrides()
{
    for (uint8_t i = 0; i < num_channels(); i++)
    {
        _overrides[i] = ChannelValueRange::NOP;
    }
}

bool RCInput::rc_bind(int dsm_mode)
{
    return false;
}

void RCInput::ppm_capture_cb(void)
{
    static const uint8_t PPM_CAPTURE_NUM_CHANNELS_MIN = _RCInput::PPM::Primary::NumChannels::_min;
    static const uint8_t PPM_CAPTURE_NUM_CHANNELS_MAX = _RCInput::PPM::Primary::NumChannels::_max;
    static const uint8_t PPM_CAPTURE_MIN_SYNC_PULSE_W = ( _RCInput::PPM::Primary::PulseWidth::_frame_period -
                                                          ( _RCInput::PPM::Primary::NumChannels::_max * _RCInput::PPM::Primary::PulseWidth::_max ) -
                                                          _RCInput::PPM::Primary::PulseWidth::_pre ) * 2;

    static uint16_t prev_value;
    static uint8_t  num_channels;

    const uint16_t curr_value = Timer.get();

    uint16_t pulse_width;

    if (curr_value < prev_value)
    {
        /* Timer rolls over at TOP=TIMER_ROLLUP
         */
        pulse_width = curr_value + TIMER_ROLLUP - prev_value;
    }
    else
    {
        pulse_width = curr_value - prev_value;
    }

    if (pulse_width > PPM_CAPTURE_MIN_SYNC_PULSE_W)
    {
        /* Sync pulse detected.
         * Pass through values if at least a minimum number of channels received
         */
        if ( num_channels >= PPM_CAPTURE_NUM_CHANNELS_MIN )
        {
            _num_channels = num_channels;

            _new_input    = true;
        }
        num_channels = 0;
    }
    else
    {
        if (num_channels < PPM_CAPTURE_NUM_CHANNELS_MAX)
        {
            _pulse_capt[num_channels] = pulse_width;

            num_channels++;

            if (num_channels == RC_INPUT_NUM_CHANNELS_MAX)
            {
                _num_channels = RC_INPUT_NUM_CHANNELS_MAX;

                _new_input    = true;
            }
        }
    }

    prev_value = curr_value;
}

void RCInput::pwm_capture_cb()
{
    static const uint8_t PWM_PULSEWIDTH_MIN   = _RCInput::PWM::PulseWidth::_min;
    static const uint8_t PWM_PULSEWIDTH_MAX   = _RCInput::PWM::PulseWidth::_max;
    static const uint8_t PWM_NUM_CHANNELS_MAX = _RCInput::PWM::_num_channels;
    static const uint8_t PWM_JITTER_FILTER    = _RCInput::PWM::_jitter_filter;
    static const uint8_t PWM_AVERAGE_FILTER   = _RCInput::PWM::_average_filters;

    // Servo pulse start timing
    static uint16_t prev_time[ PPM_ARRAY_MAX ] = { 0 };

    // Read current servo pulse change time
    const uint16_t curr_time = Timer.get();

    // Servo input pin storage 
    static uint8_t servo_pins_old = 0;

    // Used to store current servo input pins
    uint8_t servo_pins = SERVO_INPUT;

    // Calculate servo input pin change mask
    uint8_t servo_change = servo_pins ^ servo_pins_old;
    
    if ( servo_change )
    {
        for (unit8_t cur_channel = 0, servo_pin = 1;
             servo_change && ( cur_channel < PWM_NUM_CHANNELS_MAX );
             cur_channel++, servo_pin <<= 1)
        {
            // Check for pin change on current servo channel
            //
            if ( servo_change & servo_pin )
            {
                // Remove processed pin change from bitmask
                //
                servo_change &= ~servo_pin;

                // High (raising edge)
                //
                if ( servo_pins & servo_pin )
                {
                    prev_time[ cur_channel ] = curr_time;
                }
                else
                {
                    // Get servo pulse width
                    //
                    uint16_t servo_width = curr_time - prev_time[ cur_channel ];
                    
                    // Check that servo pulse signal is valid before sending to ppm encoder
                    //
                    if ( ( servo_width < PWM_PULSEWIDTH_MIN ) || ( servo_width > PWM_PULSEWIDTH_MAX ) )
                    {
                        // Used to indicate invalid servo input signals
                        //
                        servo_input_errors++;

                        continue;
                    }
                    
                    if (PWM_AVERAGE_FILTER)
                    {
                        // Average filter to smooth input jitter
                        //
                        servo_width += _pulse_capt[cur_channel];

                        servo_width >>= 1;
                    }

                    if (PWM_JITTER_FILTER > 0)
                    {
                        // 0.5us cut filter to remove input jitter
                        //
                        int16_t ppm_tmp = _pulse_capt[cur_channel] - servo_width;

                        if ( ppm_tmp <= PWM_JITTER_FILTER && ppm_tmp >= -PWM_JITTER_FILTER )
                        {
                            continue;
                        }
                    }

                    // Update _pulse_capt[..]
                    //
                    _pulse_capt[cur_channel] = servo_width;
                }
            }
        }

        // Store current servo input pins for next check
        //
        servo_pins_old = servo_pins;

        _num_channels = PWM_NUM_CHANNELS_MAX;

        _new_input    = true;
    }
}
