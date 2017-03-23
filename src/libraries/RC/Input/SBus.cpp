// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: -*- t -*-
/*
  SBUS decoder, based on src/modules/px4iofirmware/sbus.c from PX4Firmware
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
#include <RC/Input/SBus.h>


#define SBUS_INPUT_CHANNELS   16
#define SBUS_FLAGS_BYTE       23
#define SBUS_FRAME_BITS       11

#define SBUS_DIGITAL_CH1_BIT   0
#define SBUS_DIGITAL_CH2_BIT   1
#define SBUS_FRAMELOST_BIT     2
#define SBUS_FAILSAFE_BIT      3

struct BitPick
{
    uint8_t byte;
    uint8_t rshift;
    uint8_t mask;
    uint8_t lshift;
};

// S.bus decoder matrix.
//
// Each channel value can come from up to 3 input bytes. Each row in the
// matrix describes up to three bytes, and each entry gives:
//
// - byte offset in the data portion of the frame
// - right shift applied to the data byte
// - mask for the data byte
// - left shift applied to the result into the channel value
//
const struct BitPick decoder[SBUS_INPUT_CHANNELS][3] =
{
    /*  0 */ { { 0, 0, 0xff, 0}, { 1, 0, 0x07, 8}, { 0, 0, 0x00,  0} },
    /*  1 */ { { 1, 3, 0x1f, 0}, { 2, 0, 0x3f, 5}, { 0, 0, 0x00,  0} },
    /*  2 */ { { 2, 6, 0x03, 0}, { 3, 0, 0xff, 2}, { 4, 0, 0x01, 10} },
    /*  3 */ { { 4, 1, 0x7f, 0}, { 5, 0, 0x0f, 7}, { 0, 0, 0x00,  0} },
    /*  4 */ { { 5, 4, 0x0f, 0}, { 6, 0, 0x7f, 4}, { 0, 0, 0x00,  0} },
    /*  5 */ { { 6, 7, 0x01, 0}, { 7, 0, 0xff, 1}, { 8, 0, 0x03,  9} },
    /*  6 */ { { 8, 2, 0x3f, 0}, { 9, 0, 0x1f, 6}, { 0, 0, 0x00,  0} },
    /*  7 */ { { 9, 5, 0x07, 0}, {10, 0, 0xff, 3}, { 0, 0, 0x00,  0} },
    /*  8 */ { {11, 0, 0xff, 0}, {12, 0, 0x07, 8}, { 0, 0, 0x00,  0} },
    /*  9 */ { {12, 3, 0x1f, 0}, {13, 0, 0x3f, 5}, { 0, 0, 0x00,  0} },
    /* 10 */ { {13, 6, 0x03, 0}, {14, 0, 0xff, 2}, {15, 0, 0x01, 10} },
    /* 11 */ { {15, 1, 0x7f, 0}, {16, 0, 0x0f, 7}, { 0, 0, 0x00,  0} },
    /* 12 */ { {16, 4, 0x0f, 0}, {17, 0, 0x7f, 4}, { 0, 0, 0x00,  0} },
    /* 13 */ { {17, 7, 0x01, 0}, {18, 0, 0xff, 1}, {19, 0, 0x03,  9} },
    /* 14 */ { {19, 2, 0x3f, 0}, {20, 0, 0x1f, 6}, { 0, 0, 0x00,  0} },
    /* 15 */ { {20, 5, 0x07, 0}, {21, 0, 0xff, 3}, { 0, 0, 0x00,  0} }
};

int8_t SBus::decode(const uint8_t frame[SBUS_FRAME_SIZE], pulse_width_t *values, uint8_t max_values)
{
    // check frame boundary markers to avoid out-of-sync cases
    //
    if (frame[0] != 0x0f)
    {
        return 0;
    }

    int8_t channels = (max_values > SBUS_INPUT_CHANNELS) ? SBUS_INPUT_CHANNELS
                                                          : max_values;

    // use the decoder matrix to extract channel data
    //
    for (int8_t channel = 0; channel < channels; channel++)
    {
        pulse_width_t value = 0;

        for (int8_t pick = 0; pick < 3; pick++)
        {
            const struct SBus::BitPick *decode = &decoder[channel][pick];

            if (decode->mask != 0)
            {
                pulse_width_t piece = frame[1 + decode->byte];

                piece >>= decode->rshift;
                piece  &= decode->mask  ;
                piece <<= decode->lshift;

                value |= piece;
            }
        }

        // convert 0-2047 values to PULSE_WIDTH_MIN-PULSE_WIDTH_MAX
        //
        values[channel] = value << ( PULSE_WIDTH_BITS - SBUS_FRAME_BITS );
    }

    // decode switch channels if data fields are wide enough
    //
    if (max_values > 17 && channels > 15)
    {
        channels = 18;

        // channel 17 (index 16)
        //
        values[16] = ( frame[SBUS_FLAGS_BYTE] & ( 1 << SBUS_DIGITAL_CH1_BIT ) ) ? PULSE_WIDTH_MAX
                                                                                : PULSE_WIDTH_MIN;

        // channel 18 (index 17)
        //
        values[17] = ( frame[SBUS_FLAGS_BYTE] & ( 1 << SBUS_DIGITAL_CH2_BIT ) ) ? PULSE_WIDTH_MAX
                                                                                : PULSE_WIDTH_MIN;
    }

    // decode and handle failsafe and frame-lost flags
    //
    if (frame[SBUS_FLAGS_BYTE] & (1 << SBUS_FAILSAFE_BIT))
    {
        // failsafe
        //
        // report that we failed to read anything valid off the receiver
        //
        channels += max_values;
    }
    else if (frame[SBUS_FLAGS_BYTE] & (1 << SBUS_FRAMELOST_BIT))
    {
        // a frame was lost
        //
        // set a special warning flag
        // 
        // Attention! This flag indicates a skipped frame only, not a total link loss! Handling this 
        // condition as fail-safe greatly reduces the reliability and range of the radio link, 
        // e.g. by prematurely issueing return-to-launch!!!
        //
        channels *= -1;
    }

    // return the number of channels decoded
    //
    return channels;
}

// process a SBUS input pulse of the given width
//
void SBus::process_pulse(const pulse_width_t width_s0, const pulse_width_t width_s1)
{
    // convert to bit widths, allowing for up to 1usec error, assuming 100000 bps
    //
    pulse_width_t bits_s0 = (width_s0 + 1) / 10;
    pulse_width_t bits_s1 = (width_s1 + 1) / 10;
    pulse_width_t nlow;

    uint8_t byte_ofs = _bit_ofs / SBUS_STREAM_BITS;
    uint8_t bit_ofs  = _bit_ofs % SBUS_STREAM_BITS;

    if ((bits_s0 != 0) && (bits_s1 != 0) && ((bits_s0 + bit_ofs) <= 10))
    {
        // pull in the high bits
        //
        _bytes[byte_ofs] |= ( ( 1U << bits_s0 ) - 1 ) << bit_ofs;
        _bit_ofs         += bits_s0;
        bit_ofs          += bits_s0;

        // pull in the low bits
        //
        nlow = bits_s1;

        if ((nlow + bit_ofs) > SBUS_STREAM_BITS)
        {
            nlow = SBUS_STREAM_BITS - bit_ofs;
        }

        bits_s1  -= nlow;
        _bit_ofs += nlow;

        if ((_bit_ofs == (SBUS_BFRAME_SIZE * SBUS_STREAM_BITS)) && (bits_s1 > SBUS_STREAM_BITS))
        {
            // we have a full frame
            //
            uint8_t bytes[SBUS_FRAME_SIZE];
            uint8_t i;

            for (i=0; i<SBUS_FRAME_SIZE; i++)
            {
                // get inverted data
                //
                pulse_width_t v = ~_bytes[i];

                // check start bit
                //
                if ((v & 1) != 0)
                {
                    reset();

                    return;
                }

                // check stop bits
                //
                if ((v & 0xC00) != 0xC00)
                {
                    reset();

                    return;
                }

                // check parity
                //
                uint8_t parity = 0, j;

                for (j=1; j<=8; j++)
                {
                    parity ^= ( v & ( 1U << j ) ) ? 1 : 0;
                }

                if (parity != ( v & 0x200 ) >> 9 )
                {
                    reset();

                    return;
                }

                bytes[i] = ( ( v >> 1 ) & 0xFF );
            }

            pulse_width_t values[RC_INPUT_NUM_CHANNELS_MAX];

            int8_t num_values = SBus::decode(bytes, values, RC_INPUT_NUM_CHANNELS_MAX);

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

            reset();

            return;
        }
        else if (bits_s1 > SBUS_STREAM_BITS)
        {
            // break
            //
            reset();

            return;
        }

        return;
    }

    reset();
}

