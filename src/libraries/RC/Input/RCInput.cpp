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

// -------------------------------------------------------------
// SERVO PWM MODE input settings
// -------------------------------------------------------------

#define PWM_CH_NUM             8
#define PWM_PW_MIN           920
#define PWM_PW_MAX          2120

#define PWM_AVERAGE_FILTER  (0x01 << 0)
#define PWM_JITTER_FILTER   (0x01 << 1)

namespace _RCInput
{
    namespace PPM
    {
        namespace Primary
        {
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
        DECLARE_FIELD( PUInt8, RCInput.PWM, _num_channels, PWM_CH_NUM         );
        DECLARE_FIELD( PUInt8, RCInput.PWM, _filters     , PWM_AVERAGE_FILTER );

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

// AVR parameters for PhoneDrone and APM2 boards using ATmega32u2
#if defined (__AVR_ATmega16U2__) || defined (__AVR_ATmega32U2__)
#  define enter_cs()  uint8_t __AVR_Critical_Section_SREG__ = SREG; cli();
#  define leave_cs()  SREG = __AVR_Critical_Section_SREG__;
#  define pw_scale(p) ((p) >> 1) // scale pulse from 0.5us units to 1us units.
#else
#  define enter_cs()
#  define leave_cs()
#  define pw_scale(p) (p)
#endif

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

void RCInput::init(void* implspecific)
{
    clear_overrides();
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

    {
        enter_cs();

        capt = _pulse_capt[ch];

        leave_cs();
    }

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
        enter_cs();

        for (uint8_t i = 0; i < len; i++)
        {
            values[i] = _pulse_capt[i];
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
