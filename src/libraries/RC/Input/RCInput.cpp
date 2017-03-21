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
        DECLARE_FIELD( PUInt8, RCInput.PPM, _switchover_channel, PPM_SWITCHOVER_CHANNEL );

        namespace PulseWidth
        {
            DECLARE_FIELD( PUInt16, RCInput.PPM.PulseWidth, _frame_period, PPM_PRI_PW_FRAME_PERIOD    );
            DECLARE_FIELD( PUInt16, RCInput.PPM.PulseWidth, _pre         , PPM_PRI_PW_PREPULSE_LENGHT );
            DECLARE_FIELD( PUInt16, RCInput.PPM.PulseWidth, _min         , PPM_PRI_PW_MIN             );
            DECLARE_FIELD( PUInt16, RCInput.PPM.PulseWidth, _switch      , PPM_PRI_PW_SWITCH          );
            DECLARE_FIELD( PUInt16, RCInput.PPM.PulseWidth, _max         , PPM_PRI_PW_MAX             );
        };
        namespace NumChannels
        {
            DECLARE_FIELD( PUInt8, RCInput.PPM.NumChannels, _min, PPM_PRI_CH_MIN );
            DECLARE_FIELD( PUInt8, RCInput.PPM.NumChannels, _max, PPM_PRI_CH_MAX );
        };
    };
    namespace PWM
    {
        DECLARE_FIELD( PUInt8, RCInput.PWM, _num_channels  , PWM_CH_NUM );
        DECLARE_FIELD( PInt16, RCInput.PWM, _jitter_filter , SHRT_MIN   );
        DECLARE_FIELD( PBool , RCInput.PWM, _average_filter, false      );

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
#  define delay_us(d) _delay_us((d))
#  define Timer.usToTicks(t)         ((t) << 1) // scale pulse from 1us units to 0.5us units.
#  define Timer.ticksToUs(t)         ((t) >> 1) // scale pulse from 0.5us units to 1us units.
#else
#  define enter_cs()
#  define leave_cs()
#  define delay_us(d) 
#  define Timer.usToTicks(t)         (t)
#  define Timer.ticksToUs(t)         (t)
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
#define Timer.difftime(c,p)        ((c) < (p)) ? ((c) + TIMER_ROLLUP - (p)) : ((c) - (p))

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
        // Set input interrupt pin mask to input channel 1
        //
        Servo.setInterruptsMask(0b00000001);

        register_signal(RCInput::process_rc_pulse);
    }
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

    return pw_value(pw_crop(Timer.ticksToUs(capt)));
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
        values[i] = (_overrides[i] >= ChannelValueRange::MIN) ? _overrides[i] : pw_value(pw_crop(Timer.ticksToUs(values[i])));
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

// process a PPM-sum pulse of the given width
//
void RCInput::process_ppm_pulse(const uint16_t pulse_ticks)
{
    static const uint8_t PPM_CAPTURE_NUM_CHANNELS_MIN = _RCInput::PPM::NumChannels::_min;
    static const uint8_t PPM_CAPTURE_NUM_CHANNELS_MAX = _RCInput::PPM::NumChannels::_max;
    static const uint8_t PPM_CAPTURE_PULSE_WIDTH_MIN  = _RCInput::PPM::PulseWidth::_min;
    static const uint8_t PPM_CAPTURE_PULSE_WIDTH_MAX  = _RCInput::PPM::PulseWidth::_max;
    static const uint8_t PPM_CAPTURE_MIN_SYNC_PULSE_W = Timer.usToTicks
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

    static uint8_t  channel_counter = 0;
    static uint16_t pulses_ticks[RC_INPUT_NUM_CHANNELS_MAX];

    if (pulse_ticks >= PPM_CAPTURE_MIN_SYNC_PULSE_W)
    {
        // a long pulse indicates the end of a frame. Reset the
        // channel counter so next pulse is channel 0
        //
        if (channel_counter >= PPM_CAPTURE_NUM_CHANNELS_MIN)
        {
            for (uint8_t i=0; i<channel_counter; i++)
            {
                _pulse_capt[i] = pulses_ticks[i];
            }

            _num_channels = channel_counter;

            _new_input    = true;
        }

        channel_counter = 0;

        return;
    }

    if (channel_counter == -1)
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
        pulses_ticks[channel_counter] = pulse_ticks;

        // move to next channel
        //
        channel_counter++;
    }

    // if we have reached the maximum supported channels then
    // mark as unsynchronised, so we wait for a wide pulse
    //
    if (channel_counter >= RC_INPUT_NUM_CHANNELS_MAX)
    {
        for (uint8_t i=0; i<channel_counter; i++)
        {
            _pulse_capt[i] = pulses_ticks[i];
        }

        _num_channels   = channel_counter;

        _new_input      = true;

        channel_counter = -1;
    }
}

// process a SBUS input pulse of the given width
//
void RCInput::process_sbus_pulse(uint16_t width_s0, uint16_t width_s1)
{
    static SBus sbus;

    // convert to bit widths, allowing for up to 1usec error, assuming 100000 bps
    //
    uint16_t bits_s0 = (width_s0+1) / 10;
    uint16_t bits_s1 = (width_s1+1) / 10;
    uint16_t nlow;

    uint8_t byte_ofs = sbus.bit_ofs / SBUS_STREAM_BITS;
    uint8_t bit_ofs  = sbus.bit_ofs % SBUS_STREAM_BITS;

    if ((bits_s0 != 0) && (bits_s1 != 0) && (bits_s0+bit_ofs <= 10))
    {
        // pull in the high bits
        //
        sbus.bytes[byte_ofs] |= ((1U<<bits_s0)-1) << bit_ofs;
        sbus.bit_ofs += bits_s0;
        bit_ofs      += bits_s0;

        // pull in the low bits
        //
        nlow = bits_s1;

        if (nlow + bit_ofs > SBUS_STREAM_BITS)
        {
            nlow = SBUS_STREAM_BITS - bit_ofs;
        }

        bits_s1      -= nlow;
        sbus.bit_ofs += nlow;

        if ((sbus.bit_ofs == SBUS_BFRAME_SIZE*SBUS_STREAM_BITS) && (bits_s1 > SBUS_STREAM_BITS))
        {
            // we have a full frame
            //
            uint8_t bytes[SBUS_FRAME_SIZE];
            uint8_t i;

            for (i=0; i<SBUS_FRAME_SIZE; i++)
            {
                // get inverted data
                //
                uint16_t v = ~sbus.bytes[i];

                // check start bit
                //
                if ((v & 1) != 0)
                {
                    sbus.reset();

                    return;
                }

                // check stop bits
                //
                if ((v & 0xC00) != 0xC00)
                {
                    sbus.reset();

                    return;
                }

                // check parity
                //
                uint8_t parity = 0, j;

                for (j=1; j<=8; j++)
                {
                    parity ^= (v & (1U<<j)) ? 1 : 0;
                }

                if (parity != (v&0x200)>>9)
                {
                    sbus.reset();

                    return;
                }

                bytes[i] = ((v>>1) & 0xFF);
            }

            uint16_t values[RC_INPUT_NUM_CHANNELS_MAX];

            int8_t num_values = SBus::decode(bytes, values, RC_INPUT_NUM_CHANNELS_MAX);

            if ((num_values >= RC_INPUT_NUM_CHANNELS_MIN) &&
                (num_values <= RC_INPUT_NUM_CHANNELS_MAX))
            {
                for (i=0; i<num_values; i++)
                {
                    _pulse_capt[i] = values[i];
                }

                _num_channels = num_values;
                _new_input    = true;
            }

            sbus.reset();

            return;
        }
        else if (bits_s1 > SBUS_STREAM_BITS)
        {
            // break
            //
            sbus.reset();

            return;
        }

        return;
    }

    sbus.reset();
}

void RCInput::process_dsm_pulse(uint16_t width_s0, uint16_t width_s1)
{
    static DSM dsm;

    // convert to bit widths, allowing for up to 1usec error, assuming 115200 bps
    //
    uint16_t bits_s0 = ((width_s0+4)*(uint32_t)115200) / 1000000;
    uint16_t bits_s1 = ((width_s1+4)*(uint32_t)115200) / 1000000;
    uint8_t  bit_ofs ;
    uint8_t  byte_ofs;
    uint16_t nbits   ;

    if ((bits_s0 == 0) || (bits_s1 == 0))
    {
        // invalid data
        //
        dsm.reset();

        return;
    }

    byte_ofs = dsm.bit_ofs / DSM_FRAME_BITS;
    bit_ofs  = dsm.bit_ofs % DSM_FRAME_BITS;
    
    if (byte_ofs > 15)
    {
        // invalid data
        //
        dsm.reset();

        return;
    }

    // pull in the high bits
    //
    nbits = bits_s0;

    if (nbits+bit_ofs > DSM_FRAME_BITS)
    {
        nbits = DSM_FRAME_BITS - bit_ofs;
    }

    dsm.bytes[byte_ofs] |= ((1U<<nbits)-1) << bit_ofs;
    dsm.bit_ofs         += nbits;
    bit_ofs             += nbits;

    if ((bits_s0 - nbits) > DSM_FRAME_BITS)
    {
        if (dsm.bit_ofs == DSM_FRAME_SIZE*DSM_FRAME_BITS)
        {
            // we have a full frame
            uint8_t bytes[DSM_FRAME_SIZE];
            uint8_t i;

            for (i=0; i<16; i++)
            {
                // get raw data
                //
                uint16_t v = dsm.bytes[i];
                
                // check start bit || stop bit
                //
                if (((v & 1) != 0) || ((v & 0x200) != 0x200))
                {
                    dsm.reset();

                    return;
                }

                bytes[i] = ((v>>1) & 0xFF);
            }

            uint16_t values[8];

            uint16_t num_values = DSM::decode(hal.scheduler->micros64(), bytes, values, 8);

            if ((num_values >= RC_INPUT_NUM_CHANNELS_MIN) &&
                (num_values <= RC_INPUT_NUM_CHANNELS_MAX))
            {
                for (i=0; i<num_values; i++)
                {
                    _pulse_capt[i] = values[i];
                }

                _num_channels = num_values;                

                new_rc_input = true;
            }
        }

        dsm.reset();
    }

    byte_ofs = dsm.bit_ofs / DSM_FRAME_BITS;
    bit_ofs  = dsm.bit_ofs % DSM_FRAME_BITS;

    if (bits_s1+bit_ofs > DSM_FRAME_BITS)
    {
        // invalid data
        //
        dsm.reset();

        return;
    }

    // pull in the low bits
    //
    dsm.bit_ofs += bits_s1;

    return;
}

void RCInput::process_rc_pulse(void)
{
    static uint16_t pulse_ticks[2] = { 0 };

    static uint16_t prev_ticks = 0;

    const uint16_t curr_ticks = Timer.get();

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
        pulse_ticks[!rising_edge] = Timer.difftime(curr_ticks, prev_ticks);

        prev_ticks = curr_ticks;

        if (rising_edge)
        {
            // treat as PPM-Sum
            //
            process_ppm_pulse(pulse_ticks[0] + pulse_ticks[1]);

            // treat as SBUS
            //
            process_sbus_pulse(pulse_ticks[0], pulse_ticks[1]);

            // treat as DSM
            //
            process_dsm_pulse (pulse_ticks[0], pulse_ticks[1]);
        }
    }

    // Store current input pins for next check
    input_pins_old = input_pins;
}

void RCInput::process_pwm_pulse()
{
    static const uint8_t PWM_PULSEWIDTH_MIN   = Timer.usToTicks(_RCInput::PWM::PulseWidth::_min);
    static const uint8_t PWM_PULSEWIDTH_MAX   = Timer.usToTicks(_RCInput::PWM::PulseWidth::_max);
    static const uint8_t PWM_NUM_CHANNELS_MAX = _RCInput::PWM::_num_channels;
    static const uint8_t PWM_JITTER_FILTER    = Timer.usToTicks(_RCInput::PWM::_jitter_filter);
    static const bool    PWM_AVERAGE_FILTER   = _RCInput::PWM::_average_filter;

    // Servo pulse start timing
    //
    static uint16_t prev_time[PWM_NUM_CHANNELS_MAX] = { 0 };

    // Read current pulse change time
    //
    const uint16_t curr_time = Timer.get();

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
                    uint16_t pulse_ticks = Timer.difftime(curr_time, prev_time[cur_channel]);
                    
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
                        int16_t ppm_tmp = _pulse_capt[cur_channel] - pulse_ticks;

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

        _num_channels = PWM_NUM_CHANNELS_MAX;

        _new_input    = true;
    }
}
