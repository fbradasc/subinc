#include <RC/Input/PPM.h>

#define PWM_CAPTURE_NUM_CHANNELS_MIN  _profiles[_profile].channels_min;
#define PWM_CAPTURE_PULSE_WIDTH_MIN   _profiles[_profile].pwm_pulse_min;
#define PWM_CAPTURE_PULSE_WIDTH_MAX   _profiles[_profile].pwm_pulse_max;
#define PWM_CAPTURE_PULSE_WIDTH_LEN   _profiles[_profile].pwm_pulse_len;
#define PPM_CAPTURE_MIN_SYNC_PULSE_W  _profiles[_profile].syn_pulse_min;

#define SWITCH_LEVEL   25                      // used to auto detect digital data over mark pulse
#define PULSE_CAGE     SWITCH_LEVEL*2          // used to auto detect the PPM profile
#define FRAME_CAGE     500                     // used to auto detect the PPM profile

#define MATCH_IN_RANGE(a,b,r)    (((a) >= ((b) - (r))) && ((a) <= ((b) + (r))))

#define GetData(p)               (((p) < (_profiles[_profile].ppm_pulse_avg - PULSE_CAGE  )) ? -1 :   \
                                  ((p) < (_profiles[_profile].ppm_pulse_avg - SWITCH_LEVEL)) ?  0 :   \
                                  ((p) < (_profiles[_profile].ppm_pulse_avg               )) ?  1 :   \
                                  ((p) < (_profiles[_profile].ppm_pulse_avg + SWITCH_LEVEL)) ?  2 :   \
                                  ((p) < (_profiles[_profile].ppm_pulse_avg + PULSE_CAGE  )) ?  3 : -1)

// _data.byte[0].bits[0..3] -> major ID
// _data.byte[0].bits[4..7] -> minor ID
// _data.byte[1].bits[0..7] -> switches
// _data.byte[2].bits[0..7] -> checksum high byte
// _data.byte[3].bits[0..7] -> checksum low  byte
//
// sync_data.bits[0]        -> parity bit for _data.byte[0..1]
// sync_data.bits[0]        -> parity bit for _data.byte[2..3]
//
// a0.a1.a2.a3|a4.a5.a6.a7|b0.b1.b2.b3.b4.b5.b6.b7|c0.c1.c2.c3.c4.c5.c6.c7.d0.d1.d2.d3.d4.d5.d6.d7|s0.s1
//  dev major | dev minor |        switches       |                   checksum                    | 

#define ID_MAJOR_POS     28
#define ID_MAJOR_MASK    0x0f
#define ID_MINOR_POS     24
#define ID_MINOR_MASK    0x0f
#define SWITCHES_POS     16
#define SWITCHES_MASK    0xff
#define PAYLOADS_POS     16
#define PAYLOADS_MASK    0xffff
#define CHECKSUM_POS     0
#define CHECKSUM_MASK    0xffff
#define PARITY_L_POS     1
#define PARITY_L_MASK    0x01
#define PARITY_H_POS     0
#define PARITY_H_MASK    0x01

#define GetDevMajor(d)   (uint8_t )(((d) >> ID_MAJOR_POS) & ID_MAJOR_MASK)
#define GetDevMinor(d)   (uint8_t )(((d) >> ID_MINOR_POS) & ID_MINOR_MASK)
#define GetSwitches(d)   (uint8_t )(((d) >> SWITCHES_POS) & SWITCHES_MASK)
#define GetPayloads(d)   (uint16_t)(((d) >> PAYLOADS_POS) & PAYLOADS_MASK)
#define GetChecksum(d)   (uint16_t)(((d) >> CHECKSUM_POS) & CHECKSUM_MASK)
#define GetHiParity(d)   (uint8_t )(((d) >> PARITY_H_POS) & PARITY_H_MASK)
#define GetLoParity(d)   (uint8_t )(((d) >> PARITY_L_POS) & PARITY_L_MASK)

                const uint16_t payloads = 
                const uint16_t checksum = (uint16_t)((_data >>  0) & 0xffff);

struct PPMProfiles PPM::_profiles[PPM::PPM_PROFILES] =
{
    //+-------------+-----------+-----------------+-----------------------------------------------------+-----------------+-----------------------------------+//
    //|  PPM mode   | Channels# |      Frame      |                      PWM Pulse                      |    PPM Pulse    |          Synchro Pulse            |//
    //|             | min | max |      length     |       min       |       max       |       len       |     average     |       min       |       max       |//
    //+-------------+-----+-----+-----------------+-----------------+-----------------+-----------------+-----------------+-----------------+-----------------+//
      { PPM_UNKNOWN , 255 ,   0 , PULSE_WIDTH_ERR , PULSE_WIDTH_MAX , PULSE_WIDTH_ERR , PULSE_WIDTH_ERR , PULSE_WIDTH_ERR , PULSE_WIDTH_MAX , PULSE_WIDTH_ERR },
      { PPM_STANDARD,   4 ,   8 ,           20000 ,             920 ,            2120 ,               0 ,             400 ,               0 ,               0 },
      { PPM_EXTENDED,   4 ,   9 ,           22500 ,             920 ,            2120 ,               0 ,             400 ,               0 ,               0 },
      { PPM_V2      ,   4 ,  16 ,           20000 ,             460 ,            1060 ,               0 ,             200 ,               0 ,               0 },
      { PPM_V3      ,   4 ,  16 ,           25000 ,             750 ,            1350 ,               0 ,             400 ,               0 ,               0 }
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
                _mark_pulse_ndx = (_pulses_stat[0].average() < _pulses_stat[1].average()) ? 0 : 1;

                if (MATCH_IN_RANGE((_pulses_stat[_mark_pulse_ndx].average()), _profiles[i].ppm_pulse_avg, PULSE_CAGE))
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

    return PULSE_WIDTH_MIN
           +
           ( 
              ( unscaled - PWM_CAPTURE_PULSE_WIDTH_MIN )
              * 
              ( PULSE_WIDTH_LEN / PWM_CAPTURE_PULSE_WIDTH_LEN )
           );
}

// process a PPM-sum pulse of the given width
//
void PPM::process_pulse(const pulse_width_t width_s0, const pulse_width_t width_s1)
{
    const pulse_width_t pulse_ticks = width_s0 + width_s1;

    if (/* !_profile && */ (_channels >= 0))
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
            {
                // take a reading for the sync channel
                //
                _pulses_ticks[_channels][0] = width_s0;
                _pulses_ticks[_channels][1] = width_s1;

                const uint8_t  parities = GetData(_pulses_ticks[_channels][_mark_pulse_ndx]);
                const uint16_t payloads = GetPayloads(_data);
                const uint16_t checksum = GetChecksum(_data);

                if ((calculateParity(payloads) == GetHiParity(parities)) &&
                    (calculateParity(checksum) == GetLoParity(parities)))
                {
                    // TODO
                }
            }

            for (uint8_t i=0; i<_channels; i++)
            {
                _listener._pulse_capt[i] = scale(_pulses_ticks[i][0] + _pulses_ticks[i][1]);
            }

            _listener._num_channels = _channels;

            _listener._new_input    = true;
        }

        _channels = 0;
        _data     = 0;

        _pulses_stat[0].reset();
        _pulses_stat[1].reset();

        return;
    }

    // if we are not synchronised
    // or
    // if we have reached the maximum supported channels
    // then
    // wait for a wide pulse
    //
    if ((_channels == -1) || (_channels >= RC_INPUT_NUM_CHANNELS_MAX))
    {
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

        uint32_t data = GetData(_pulses_ticks[_channels][_mark_pulse_ndx]);

        if (data >= 0)
        {
            _data |= (data & 0x3) << ( _channels << 1 );
        }

        // move to next channel
        //
        _channels++;
    }

}
