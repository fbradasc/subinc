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

#pragma once

#include <Config/Config.h>

#define RC_INPUT_NUM_CHANNELS_MAX 16

class RCInput
{
public:
    enum ChannelValueRange
    {
        NOP =     -1,
        MIN =      0,
        MAX = 0x7fff
    };

    RCInput();

    /* Call init from the platform hal instance init, so that both the type of
     * the RCInput implementation and init argument (e.g. ISRRegistry) are
     * known to the programmer. (Its too difficult to describe this dependency
     * in the C++ type system.)
     */
    void init(void* implspecific);

    void deinit();

    /* Return true if there has been new input since the last read()
     * call. This call also clears the new_input flag, so once it
     * returns true it won't return true again until another frame is
     * received.
     */
    inline bool new_input()
    {
        if (_new_input)
        {
            _new_input = false;

            return true;
        }
        return false;
    }

    /* Return the number of valid channels in the last read
     */
    inline uint8_t num_channels() { return _num_channels; }

    /* Read a single channel at a time
     */
    uint16_t read(uint8_t ch);

    /* Read an array of channels, return the valid count
     */
    uint8_t read(uint16_t* periods, uint8_t len);

    /**
     * Overrides: these are really grody and don't belong here but we need
     * them at the moment to make the port work.
     *
     * case v of:
     *   v == ChannelValueRange::NOP -> do not override this channel
     *   v >= ChannelValueRange::MIN -> set v as override.
     */

    /* get overrides: array starts at ch 0, for len channels
     */
    uint8_t get_overrides(int16_t *overrides, uint8_t len);

    /* set overrides: array starts at ch 0, for len channels
     */
    bool set_overrides(int16_t *overrides, uint8_t len);

    /* get override value for channel
     */
    int16_t get_override(uint8_t channel);

    /* set override value for channel
     */
    bool set_override(uint8_t channel, int16_t override);

    /* clear_overrides: equivelant to setting all overrides to ChannelValueRange::NOP
     */
    void clear_overrides();

    /* execute receiver bind
     */
    bool rc_bind(int dsm_mode);

private:
    /* New input detected since last incoming frame
     */
    static volatile bool _new_input;

    /* private variables to communicate with input capture isr
     */
    static volatile uint16_t _pulse_capt[RC_INPUT_NUM_CHANNELS_MAX];

    /* Autodetected num channels
     */
    static volatile uint8_t _num_channels;

    /* Autodetected minimum pulse width [uS]
     */
    static volatile uint16_t _min_pulsewidth;

    /* Autodetected maximum pulse width [uS]
     */
    static volatile uint16_t _max_pulsewidth;

    /* Current override values in
     *
     * { ChannelValueRange::NOP U [ChannelValueRange::MIN, ChannelValueRange::MAX] }
     */
    int16_t _overrides[RC_INPUT_NUM_CHANNELS_MAX];

    /* Constrain captured pulse to be between min and max pulsewidth.
     */
    static inline uint16_t pw_crop(uint16_t p);

    /* Convert the pulse width to an absolute value
     *
     * Input:
     *      p : [_min_pulsewidth,_max_pulsewidth]
     *
     * Return:
     *
     *      v : [ChannelValueRange::MIN, ChannelValueRange::MAX]
     */
    static inline uint16_t pw_value(uint16_t p);
};
