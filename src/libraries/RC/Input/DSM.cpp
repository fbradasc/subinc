// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: -*- t -*-
/*
  DSM decoder, based on src/modules/px4iofirmware/dsm.c from PX4Firmware
  modified for use in subinc by Francesco Bradascio
 */
/****************************************************************************
 *
 *   Copyright (c) 2012-2014 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <stdint.h>

#include <Utils/Utils.h>
#include <RC/Input/RCInput.h>
#include <RC/Input/DSM.h>

#define DSM_FRAME_BITS     10      // DSM stream size in bits
#define DSM_FRAME_CHANNELS  7      // Max supported DSM channels

uint16_t    DSM::_bytes[DSM_FRAME_SIZE] = { 0 }; // including start bit, parity and stop bits
uint16_t    DSM::_bit_ofs               = 0;
timestamp_t DSM::_last_frame_time       = 0;     // Timestamp for start of last dsm frame
uint8_t     DSM::_channel_shift         = 0;     // Channel resolution, 0=unknown, 1=10 bit, 2=11 bit

/**
 * Attempt to decode a single channel raw channel datum
 *
 * The DSM* protocol doesn't provide any explicit framing,
 * so we detect dsm frame boundaries by the inter-dsm frame delay.
 *
 * The minimum dsm frame spacing is 11ms; with 16 bytes at 115200bps
 * dsm frame transmission time is ~1.4ms.
 *
 * We expect to only be called when bytes arrive for processing,
 * and if an interval of more than 5ms passes between calls,
 * the first byte we read will be the first byte of a dsm frame.
 *
 * In the case where byte(s) are dropped from a dsm frame, this also
 * provides a degree of protection. Of course, it would be better
 * if we didn't drop bytes...
 *
 * Upon receiving a full dsm frame we attempt to decode it
 *
 * @param[in] raw 16 bit raw channel value from dsm frame
 * @param[in] shift position of channel number in raw data
 * @param[out] channel pointer to returned channel number
 * @param[out] value pointer to returned channel value
 * @return true=raw value successfully decoded
 */
bool DSM::decode_channel(uint16_t raw, uint8_t shift, uint8_t &channel, pulse_width_t &value)
{
    if (raw == 0xffff)
    {
        return false;
    }

    channel = (raw >> shift) & 0xf;

    pulse_width_t data_mask = ( (1 << shift) - 1 ) & PULSE_WIDTH_MAX;

    value = raw & data_mask;

    // DBG_MSG_F("DSM: %d 0x%04x -> %d %d", shift, raw, channel, value);

    return true;
}

/**
 * Attempt to guess if receiving 10 or 11 bit channel values
 *
 * @param[in] reset true=reset the 10/11 bit state to unknown
 */
void DSM::guess_format(bool reset, const uint8_t frame[DSM_FRAME_SIZE])
{
    static uint32_t cs10   ;
    static uint32_t cs11   ;
    static uint8_t  samples;

    // reset the 10/11 bit sniffed channel masks
    //
    if (reset)
    {
        cs10           = 0;
        cs11           = 0;
        samples        = 0;
        _channel_shift = 0;

        return;
    }

    // scan the channels in the current frame in both 10- and 11-bit mode
    //
    for (uint8_t i = 0; i < DSM_FRAME_CHANNELS; i++)
    {
        const uint8_t *dp  = &frame[2 + (2 * i)];
        uint16_t       raw = (dp[0] << 8) | dp[1];

        uint8_t       channel;
        pulse_width_t value  ;

        // if the channel decodes, remember the assigned number
        //
        if (decode_channel(raw, 10, channel, value) && (channel < 31))
        {
            cs10 |= (1 << channel);
        }

        if (decode_channel(raw, 11, channel, value) && (channel < 31))
        {
            cs11 |= (1 << channel);
        }

        // XXX if we cared, we could look for the phase bit here to decide 1 vs. 2-frame format
    }

    // wait until we have seen plenty of frames - 5 should normally be enough
    //
    if (samples++ < RC_INPUT_NUM_CHANNELS_MIN)
    {
        return;
    }

    /*
     * Iterate the set of sensible sniffed channel sets and see whether
     * decoding in 10 or 11-bit mode has yielded anything we recognize.
     *
     * XXX Note that due to what seem to be bugs in the DSM2 high-resolution
     *     stream, we may want to sniff for longer in some cases when we think we
     *     are talking to a DSM2 receiver in high-resolution mode (so that we can
     *     reject it, ideally).
     *     See e.g. http://git.openpilot.org/cru/OPReview-116 for a discussion
     *     of this issue.
     */
    static uint32_t masks[] =
    {
        0x003f, //  6 channels (DX6)
        0x007f, //  7 channels (DX7)
        0x00ff, //  8 channels (DX8)
        0x01ff, //  9 channels (DX9, etc.)
        0x03ff, // 10 channels (DX10)
        0x1fff, // 13 channels (DX10t)
        0x3fff  // 18 channels (DX10)
    };

    uint8_t votes10 = 0;
    uint8_t votes11 = 0;

    for (uint8_t i = 0; i < ARRAY_SIZE(masks); i++)
    {
        if (cs10 == masks[i])
        {
            votes10++;
        }

        if (cs11 == masks[i])
        {
            votes11++;
        }
    }

    if ((votes11 == 1) && (votes10 == 0))
    {
        _channel_shift = 11;

        DBG_MSG_F("DSM: 11-bit format");

        return;
    }

    if ((votes10 == 1) && (votes11 == 0))
    {
        _channel_shift = 10;

        DBG_MSG_F("DSM: 10-bit format");

        return;
    }

    DBG_INF_F("DSM: format detect fail, 10: 0x%08x %d 11: 0x%08x %d", cs10, votes10, cs11, votes11);

    // call ourselves to reset our state ... we have to try again
    //
    guess_format(true, frame);
}

/**
 * Decode the entire dsm frame (all contained channels)
 *
 */
uint8_t DSM::decode(timestamp_t frame_time, const uint8_t frame[DSM_FRAME_SIZE], pulse_width_t *values, uint8_t max_values)
{
    uint8_t num_values=0;

    /*
    DBG_MSG_F("DSM frame %02x%02x %02x%02x %02x%02x %02x%02x %02x%02x %02x%02x %02x%02x %02x%02x",
              frame[0], frame[1], frame[ 2], frame[ 3], frame[ 4], frame[ 5], frame[ 6], frame[ 7],
              frame[8], frame[9], frame[10], frame[11], frame[12], frame[13], frame[14], frame[15]);
    */

    // If we have lost signal for at least a second, reset the
    // format guessing heuristic.
    //
    if (((frame_time - _last_frame_time) > 1000000) && (_channel_shift != 0))
    {
        guess_format(true, frame);
    }

    // we have received something we think is a frame
    //
    _last_frame_time = frame_time;

    // if we don't know the frame format, update the guessing state machine
    //
    if (_channel_shift == 0)
    {
        guess_format(false, frame);

        return false;
    }

    // The encoding of the first two bytes is uncertain, so we're
    // going to ignore them for now.
    // 
    // Each channel is a 16-bit unsigned value containing either a 10-
    // or 11-bit channel value and a 4-bit channel number, shifted
    // either 10 or 11 bits. The MSB may also be set to indicate the
    // second frame in variants of the protocol where more than
    // seven channels are being transmitted.
    // 
    for (uint8_t i = 0; i < DSM_FRAME_CHANNELS; i++)
    {
        const uint8_t *dp = &frame[2 + (2 * i)];
        uint16_t raw      = (dp[0] << 8) | dp[1];

        uint8_t       channel;
        pulse_width_t value  ;

        if (!decode_channel(raw, _channel_shift, channel, value))
        {
            continue;
        }

        // ignore channels out of range
        //
        if (channel >= max_values)
        {
            continue;
        }

        // update the decoded channel count
        //
        if (channel >= num_values)
        {
            num_values = channel + 1;
        }

        // convert 0-1023 / 0-2047 values to 0-PULSE_WIDTH_MAX
        //
        value <<= ( PULSE_WIDTH_BITS - _channel_shift );

        // Store the decoded channel into the R/C input buffer, taking into
        // account the different ideas about channel assignement that we have.
        // 
        // Specifically, the first four channels in rc_channel_data are roll, pitch, thrust, yaw,
        // but the first four channels from the DSM receiver are thrust, roll, pitch, yaw.
        // 
        switch (channel)
        {
            case 0:
                channel = 2;
                break;

            case 1:
                channel = 0;
                break;

            case 2:
                channel = 1;

            default:
                break;
        }

        values[channel] = value;
    }

    // Spektrum likes to send junk in higher channel numbers to fill
    // their packets. We don't know about a 13 channel model in their TX
    // lines, so if we get a channel count of 13, we'll return 12 (the last
    // data index that is stable).
    // 
    if (num_values == 13)
    {
        num_values = 12;
    }

#if 0
    if (_channel_shift == 11)
    {
        // Set the 11-bit data indicator
        //
        num_values |= 0x8000;
    }
#endif

    // XXX Note that we may be in failsafe here; we need to work out how to detect that.
    // 
    return true;
}

void DSM::process_pulse(timestamp_t frame_time, pulse_width_t width_s0, pulse_width_t width_s1)
{
    // convert to bit widths, allowing for up to 1usec error, assuming 115200 bps
    //
    uint16_t bits_s0 = ((width_s0+4)*(uint32_t)115200) / 1000000;
    uint16_t bits_s1 = ((width_s1+4)*(uint32_t)115200) / 1000000;

    if ((bits_s0 == 0) || (bits_s1 == 0))
    {
        // invalid data
        //
        reset();

        return;
    }

    uint8_t  byte_ofs = _bit_ofs / DSM_FRAME_BITS;
    
    if (byte_ofs > 15)
    {
        // invalid data
        //
        reset();

        return;
    }

    uint8_t  bit_ofs = _bit_ofs % DSM_FRAME_BITS;

    // pull in the high bits
    //
    uint16_t nbits = bits_s0;

    if ((nbits + bit_ofs) > DSM_FRAME_BITS)
    {
        nbits = DSM_FRAME_BITS - bit_ofs;
    }

    _bytes[byte_ofs] |= ((1U<<nbits)-1) << bit_ofs;
    _bit_ofs         += nbits;
    bit_ofs          += nbits;

    if ((bits_s0 - nbits) > DSM_FRAME_BITS)
    {
        if (_bit_ofs == DSM_FRAME_SIZE*DSM_FRAME_BITS)
        {
            // we have a full frame
            //
            uint8_t bytes[DSM_FRAME_SIZE];
            uint8_t i;

            for (i=0; i<DSM_FRAME_SIZE; i++)
            {
                // get raw data
                //
                uint16_t v = _bytes[i];
                
                // check start bit || stop bit
                //
                if (((v & 1) != 0) || ((v & 0x200) != 0x200))
                {
                    reset();

                    return;
                }

                bytes[i] = ((v>>1) & 0xFF);
            }

            pulse_width_t values[RC_INPUT_NUM_CHANNELS_MAX];

            uint16_t num_values = decode(frame_time, bytes, values, ARRAY_SIZE(values));

            if ((num_values >= RC_INPUT_NUM_CHANNELS_MIN) &&
                (num_values <= RC_INPUT_NUM_CHANNELS_MAX))
            {
                for (i=0; i<num_values; i++)
                {
                    _listener._pulse_capt[i] = values[i];
                }

                _listener._num_channels = num_values;                

                _listener._new_input    = true;
            }
        }

        reset();
    }

    byte_ofs = _bit_ofs / DSM_FRAME_BITS;
    bit_ofs  = _bit_ofs % DSM_FRAME_BITS;

    if ((bits_s1 + bit_ofs) > DSM_FRAME_BITS)
    {
        // invalid data
        //
        reset();

        return;
    }

    // pull in the low bits
    //
    _bit_ofs += bits_s1;

    return;
}
