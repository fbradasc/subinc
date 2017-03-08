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

class RCInput
{
public:
    /**
     * Call init from the platform hal instance init, so that both the type of
     * the RCInput implementation and init argument (e.g. ISRRegistry) are
     * known to the programmer. (Its too difficult to describe this dependency
     * in the C++ type system.)
     */
    virtual void init(void* implspecific) {}
    virtual void deinit() {}

    /**
     * Return true if there has been new input since the last read()
     * call. This call also clears the new_input flag, so once it
     * returns true it won't return true again until another frame is
     * received.
     */
    virtual bool new_input() { return false; }

    /**
     * Return the number of valid channels in the last read
     */
    virtual uint8_t  num_channels() { return 0; }

    /* Read a single channel at a time */
    virtual uint16_t read(uint8_t ch) { return 0; }

    /* Read an array of channels, return the valid count */
    virtual uint8_t read(uint16_t* periods, uint8_t len) { return 0; }

    /**
     * Overrides: these are really grody and don't belong here but we need
     * them at the moment to make the port work.
     * case v of:
     *  v == -1 -> no change to this channel
     *  v == 0  -> do not override this channel
     *  v > 0   -> set v as override.
     */

    /* set_overrides: array starts at ch 0, for len channels */
    virtual bool set_overrides(int16_t *overrides, uint8_t len) { return false; }
    /* set_override: set just a specific channel */
    virtual bool set_override(uint8_t channel, int16_t override) { return false; }
    /* clear_overrides: equivelant to setting all overrides to 0 */
    virtual void clear_overrides() {}

    /* execute receiver bind */
    virtual bool rc_bind(int dsmMode) { return false; };
};
