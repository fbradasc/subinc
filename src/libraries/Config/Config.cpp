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

//
//

// total up and check overflow
// check size of group var_info

/// @file   Config.cpp
/// @brief  The Sub variable store.

#include <stdio.h>

#include "Config.h"

Config::EEprom Config::_config = { 0 };

void Config::load()
{
    Config::_config._magic                  = Config::CONFIG_MAGIC;
    Config::_config._version                = 1;
    Config::_config.Scheduler._loop_rate_hz = 281;
}

void Config::save()
{
}

void Config::dump()
{
    printf("_magic                  = 0x%04x\n", Config::_config._magic                 );
    printf("_version                = %d\n"    , Config::_config._version               );
    printf("Scheduler._loop_rate_hz = %d\n"    , Config::_config.Scheduler._loop_rate_hz);
}
