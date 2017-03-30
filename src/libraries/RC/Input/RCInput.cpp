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
#include <RC/Input/PPM.h>
#include <RC/Input/SBus.h>
#include <RC/Input/DSM.h>
#include <RC/Input/PWM.h>

#include <Debug/Debug.h>

volatile pulse_width_t RCInput::_pulse_capt[RC_INPUT_NUM_CHANNELS_MAX] = {0};
volatile uint32_t      RCInput::_switches                              = 0x0000;
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
    // detect if single line mode or PWM mode
    //
    uint8_t pin2_status = 0;

    Servo.enableInput (1); // Set pin 2 to input
    Servo.enableOutput(2); // Set pin 3 to output

    Servo.clear(2); // Set pin 3 output low
    { 
        delay_us(10);
                
        if ( Servo.get(1) == 0 )
        {
            pin2_status++;
        }
    }

    Servo.set(2);
    {
        delay_us(10);
        
        if ( Servo.get(1) != 0 )
        {
            pin2_status++;
        }
    }

    Servo.clear(2);
    {
        delay_us(10);

        if ( Servo.get(1) == 0 )
        {
            pin2_status++;
        }
    }

    // RESET SERVO/PPM PINS AS INPUTS WITH PULLUPS
    //
    Servo.reset();
    
    if ( pin2_status == 3 )
    {
        // Single line mode
        //
        // Set input interrupt pin mask to input channel 1
        //
        Servo.setInterruptsMask(0b00000001);

        register_signal(RCInput::process_rc_pulse);
    }
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
    static pulse_width_t pulse_width[2] = { 0 };
    static pulse_width_t prev_time      = 0;
    static timestamp_t   timestamp      = 0;
    static uint8_t       input_pins_old = 0;
                                       
    const  pulse_width_t curr_time      = Timer.ticksToUs(Timer.get());
    const  uint8_t       input_pins     = SERVO_INPUT;
    
    if ( ( input_pins ^ input_pins_old ) & RC_INPUT_PIN )
    {
        const bool cur_pin_level = ( input_pins & RC_INPUT_PIN ) ? true : false ;
        const bool old_pin_level = !cur_pin_level;

        pulse_width[old_pin_level] = Timer.difftime(curr_time, prev_time);

        timestamp += pulse_width[old_pin_level];

        prev_time = curr_time;

        if ( cur_pin_level )
        {
            // treat as PPM-Sum
            //
            {
                static PPM ppm(this);

                ppm.process_pulse(pulse_width[0], pulse_width[1]);
            }

            // treat as SBUS
            //
            {
                static SBus sbus(this);

                sbus.process_pulse(pulse_width[0], pulse_width[1]);
            }

            // treat as DSM
            //
            {
                static DSM dsm(this);

                dsm.process_pulse(timestamp, pulse_width[0], pulse_width[1]);
            }
        }
    }

    // Store current input pins for next check
    input_pins_old = input_pins;
}

void RCInput::process_pwm_pulse()
{
    static PWM pwm(this);

    pmw.process_pulses(Timer.ticksToUs(Timer.get()), SERVO_INPUT);
}
