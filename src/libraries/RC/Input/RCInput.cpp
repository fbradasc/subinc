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
#include <RC/Input/SBus.h>
#include <RC/Input/DSM.h>

#include <Debug/Debug.h>

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

volatile pulse_width_t RCInput::_pulse_capt[RC_INPUT_NUM_CHANNELS_MAX] = {0};
volatile bool          RCInput::_new_input                             = false;
volatile uint8_t       RCInput::_num_channels                          = 0;

RCInput::RCInput()
{
    clear_overrides();
}

// AVR parameters for PhoneDrone and APM2 boards using ATmega32u2
#if defined (__AVR_ATmega16U2__) || defined (__AVR_ATmega32U2__) || defined (__AVR_ATmega328P__) || defined (__AVR_ATmega328__)
#  define enter_cs()         uint8_t __AVR_Critical_Section_SREG__ = SREG; cli();
#  define leave_cs()         SREG = __AVR_Critical_Section_SREG__;
#  define delay_us(d)        _delay_us((d))
#  define Timer.usToTicks(t) ((t) << 1) // scale pulse from 1us units to 0.5us units.
#  define Timer.ticksToUs(t) ((t) >> 1) // scale pulse from 0.5us units to 1us units.
#else
#  define enter_cs()
#  define leave_cs()
#  define delay_us(d) 
#  define Timer.usToTicks(t) (t)
#  define Timer.ticksToUs(t) (t)
#endif

#define non_blocking_read(to, from) for(to=(from); to!=(from); to=(from));

#if defined (__AVR_ATmega16U2__) || defined (__AVR_ATmega32U2__)

#  define SERVO_DDR               DDRB
#  define SERVO_PORT              PORTB
#  define SERVO_INPUT             PINB
#  define SERVO_INT_VECTOR        PCINT0_vect
#  define SERVO_INT_MASK          PCMSK0
#  define SERVO_INT_CLEAR_FLAG    PCIF0
#  define SERVO_INT_ENABLE        PCIE0
#  define SERVO_TIMER_CNT         TCNT1
                                  
#  define PPM_DDR                 DDRC
#  define PPM_PORT                PORTC
#  define PPM_OUTPUT_PIN          PC6
#  define PPM_INT_VECTOR          TIMER1_COMPA_vect
#  define PPM_COMPARE             OCR1A
#  define PPM_COMPARE_FLAG        COM1A0
#  define PPM_COMPARE_ENABLE      OCIE1A
#  define PPM_COMPARE_FORCE_MATCH FOC1A

#  define TIMER_REG               ICR4

#elif defined (__AVR_ATmega328P__) || defined (__AVR_ATmega328__)

#  define SERVO_DDR               DDRD
#  define SERVO_PORT              PORTD
#  define SERVO_INPUT             PIND
#  define SERVO_INT_VECTOR        PCINT2_vect
#  define SERVO_INT_MASK          PCMSK2
#  define SERVO_INT_CLEAR_FLAG    PCIF2
#  define SERVO_INT_ENABLE        PCIE2
#  define SERVO_TIMER_CNT         TCNT1
                                  
#  define PPM_DDR                 DDRB
#  define PPM_PORT                PORTB
#  define PPM_OUTPUT_PIN          PB2
#  define PPM_INT_VECTOR          TIMER1_COMPB_vect
#  define PPM_COMPARE             OCR1B
#  define PPM_COMPARE_FLAG        COM1B0
#  define PPM_COMPARE_ENABLE      OCIE1B
#  define PPM_COMPARE_FORCE_MATCH FOC1B

#  define TIMER_REG               ICR4

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
#define Timer.difftime(c,p)        ((c) < (p)) ? ((c) + TIMER_ROLLUP - (p)) : ((c) - (p))

void RCInput::init(void* implspecific)
{
    clear_overrides();

    //
    // detect if PPM, PPM redudancy or PWM
    //
    uint8_t pin2_status = 0;
#if defined(HANDLE_PPM_REDUDANCY)
    uint8_t pin4_status = 0;
#endif

    Servo.enableInput (1); // Set pin 2 to input
    Servo.enableOutput(2); // Set pin 3 to output
#if defined(HANDLE_PPM_REDUDANCY)
    Servo.enableInput (3); // Set pin 4 to input
#endif

    Servo.clear(2); // Set pin 3 output low
    { 
        delay_us(10);
                
        if ( Servo.get(1) == 0 )
        {
            pin2_status++;
        }

#if defined(HANDLE_PPM_REDUDANCY)
        if ( Servo.get(3) == 0 )
        {
            pin4_status++;
        }
#endif
    }

    Servo.set(2);
    {
        delay_us(10);
        
        if ( Servo.get(1) != 0 )
        {
            pin2_status++;
        }

#if defined(HANDLE_PPM_REDUDANCY)
        if ( Servo.get(3) != 0 )
        {
            pin4_status++;
        }
#endif
    }

    Servo.clear(2);
    {
        delay_us(10);

        if ( Servo.get(1) == 0 )
        {
            pin2_status++;
        }

#if defined(HANDLE_PPM_REDUDANCY)
        if ( Servo.get(3) == 0 )
        {
            pin4_status++;
        }
#endif
    }

    // RESET SERVO/PPM PINS AS INPUTS WITH PULLUPS
    //
    Servo.reset();
    
    if ( pin2_status == 3 )
    {
        // PPM single line mode
        //
        // Set input interrupt pin mask to input channel 1
        //
        Servo.setInterruptsMask(0b00000001);

        register_signal(RCInput::process_rc_pulse);
    }
#if defined(HANDLE_PPM_REDUDANCY)
    else
    if ( pin4_status == 3 ) 
    {
        // PPM redudancy mode
        //
        // Set input interrupt pin mask to input channels 1 and 2
        //
        Servo.setInterruptsMask(0b00000011);

        register_signal(RCInput::ppm_redudancy_capture_cb);
    }
#endif
    else
    {
        // PWM mode - max 8 channels
        //
        // Set input interrupt pin mask to all 8 input channels
        //
        Servo.setInterruptsMask(0b11111111);

        register_signal(RCInput::process_pwm_pulse);
    }
}

void RCInput::deinit()
{
}

pulse_width_t RCInput::read(uint8_t ch)
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

    pulse_width_t capt;

    non_blocking_read(capt, _pulse_capt[ch]);

    return capt;
}

uint8_t RCInput::read(pulse_width_t* values, uint8_t len)
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
        values[i] = (_overrides[i] >= PULSE_WIDTH_MIN) ? _overrides[i] : values[i];
    }

    return num_channels();
}

pulse_width_t RCInput::get_override(uint8_t channel) 
{
    if (channel < RC_INPUT_NUM_CHANNELS_MAX)
    {
        return _overrides[channel];
    }

    return PULSE_WIDTH_ERR;
}

uint8_t RCInput::get_overrides(pulse_width_t *overrides, uint8_t len) 
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

bool RCInput::set_overrides(pulse_width_t *overrides, uint8_t len) 
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

bool RCInput::set_override(uint8_t channel, pulse_width_t override) 
{
    if ((channel < RC_INPUT_NUM_CHANNELS_MAX) && (_overrides[channel] != override))
    {
        _overrides[channel] = override;

        if (override >= PULSE_WIDTH_MIN)
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
        _overrides[i] = PULSE_WIDTH_ERR;
    }
}

bool RCInput::rc_bind(int dsm_mode)
{
    return false;
}

void RCInput::process_rc_pulse(void)
{
    static pulse_width_t pulse_ticks[2] = { 0 };
    static pulse_width_t pulse_us   [2] = { 0 };
    static pulse_width_t prev_ticks     = 0;
    static timestamp_t   timestamp      = 0;
                                       
    const pulse_width_t  curr_ticks = Timer.get();

    // To store current input pins
    //
    const uint8_t input_pins = SERVO_INPUT;
    
    // Servo input pin storage 
    //
    static uint8_t input_pins_old = 0;
    
    // Calculate input pin change mask
    //
    const bool input_change = ( input_pins ^ input_pins_old ) & RC_INPUT_PIN;
    const bool rising_edge  = ( input_pins & RC_INPUT_PIN );

    if (input_change)
    {
        bool index = !rising_edge;

        pulse_ticks[index] = Timer.difftime(curr_ticks, prev_ticks);
        pulse_us   [index] = Timer.ticksToUs(pulse_ticks[index]);

        timestamp += pulse_us[index];

        prev_ticks = curr_ticks;

        if (rising_edge)
        {
            // treat as PPM-Sum
            //
            {
                static PPM ppm(this);

                ppm.process_pulse(pulse_ticks[0] + pulse_ticks[1]);
            }

            // treat as SBUS
            //
            {
                static SBus sbus(this);

                sbus.process_pulse(pulse_us[0], pulse_us[1]);
            }

            // treat as DSM
            //
            {
                static DSM dsm(this);

                dsm.process_pulse(timestamp, pulse_us[0], pulse_us[1]);
            }
        }
    }

    // Store current input pins for next check
    input_pins_old = input_pins;
}

void RCInput::process_pwm_pulse()
{
    static const pulse_width_t PWM_PULSEWIDTH_MIN   = Timer.usToTicks(_RCInput::PWM::PulseWidth::_min);
    static const pulse_width_t PWM_PULSEWIDTH_MAX   = Timer.usToTicks(_RCInput::PWM::PulseWidth::_max);
    static const pulse_width_t PWM_JITTER_FILTER    = Timer.usToTicks(_RCInput::PWM::_jitter_filter);
    static const uint8_t  PWM_NUM_CHANNELS_MAX = _RCInput::PWM::_num_channels;
    static const bool     PWM_AVERAGE_FILTER   = _RCInput::PWM::_average_filter;

    // Servo pulse start timing
    //
    static pulse_width_t prev_time[PWM_NUM_CHANNELS_MAX] = { 0 };

    // Read current pulse change time
    //
    const pulse_width_t curr_time = Timer.get();

    // Servo input pin storage 
    //
    static uint8_t input_pins_old = 0;

    // Used to store current input pins
    //
    const uint8_t input_pins = SERVO_INPUT;

    // Calculate input pin change mask
    //
    uint8_t input_change = input_pins ^ input_pins_old;
    
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
                    prev_time[cur_channel] = curr_time;
                }
                else
                {
                    // Get pulse width
                    //
                    pulse_width_t pulse_ticks = Timer.difftime(curr_time, prev_time[cur_channel]);
                    
                    // Check that pulse signal is valid
                    //
                    if ( ( pulse_ticks < PWM_PULSEWIDTH_MIN ) || ( pulse_ticks > PWM_PULSEWIDTH_MAX ) )
                    {
                        // Invalid input signals --> TODO need to notify ?
                        //
                        continue;
                    }
                    
                    if (PWM_AVERAGE_FILTER)
                    {
                        // Average filter to smooth input jitter
                        //
                        pulse_ticks += _pulse_capt[cur_channel];

                        pulse_ticks >>= 1;
                    }

                    if (PWM_JITTER_FILTER > 0)
                    {
                        // 0.5us cut filter to remove input jitter
                        //
                        pulse_width_t ppm_tmp = _pulse_capt[cur_channel] - pulse_ticks;

                        if ( ppm_tmp <= PWM_JITTER_FILTER && ppm_tmp >= -PWM_JITTER_FILTER )
                        {
                            continue;
                        }
                    }

                    // Update _pulse_capt[..]
                    //
                    _pulse_capt[cur_channel] = pulse_ticks;
                }
            }
        }

        // Store current input pins for next check
        //
        input_pins_old = input_pins;

        _num_channels  = PWM_NUM_CHANNELS_MAX;

        _new_input     = true;
    }
}
