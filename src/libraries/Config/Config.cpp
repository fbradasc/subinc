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
/// @file   Config.cpp
/// @brief  The Sub variable store.

#include <stdio.h>

#include "Config.h"

Config::EEprom Config::_config = { 0 };

void Config::load()
{
    Config::_config._magic                  = Config::CONFIG_MAGIC;
    Config::_config._version                = 1;
    
#if 0
    Config::_config.Scheduler._loop_rate_hz = 281;

    // +--------------+----------------------------------------------+-------------+
    // |              |                  PulseWidth (us)             | NumChannels |
    // |     Mode     +---------------+------+------+---------+------+------+------+
    // |              | _frame_period | _pre | _min | _switch | _max | _min | _max |
    // +--------------+---------------+------+------+---------+------+------+------+
    // | Standard PPM | 20000 (50 Hz) |  400 |  920 |   1800  | 2120 |   4  |   8  |
    // +--------------+---------------+------+------+---------+------+------+------+
    //
    Config::_config.RCInput.PPM.Primary.PulseWidth._frame_period = 20000;
    Config::_config.RCInput.PPM.Primary.PulseWidth._pre          =   400;
    Config::_config.RCInput.PPM.Primary.PulseWidth._min          =   920;
    Config::_config.RCInput.PPM.Primary.PulseWidth._switch       =  1800;
    Config::_config.RCInput.PPM.Primary.PulseWidth._max          =  2120;
    Config::_config.RCInput.PPM.Primary.NumChannels._min         =     4;
    Config::_config.RCInput.PPM.Primary.NumChannels._max         =     8;

    // +--------------+----------------------------------------------+-------------+
    // |              |                  PulseWidth (us)             | NumChannels |
    // |     Mode     +---------------+------+------+---------+------+------+------+
    // |              | _frame_period | _pre | _min | _switch | _max | _min | _max |
    // +--------------+---------------+------+------+---------+------+------+------+
    // | PPMv3        | 25000 (40 Hz) |  400 |  750 |   1260  | 1350 |   4  |  16  |  
    // +--------------+---------------+------+------+---------+------+------+------+
    //
    Config::_config.RCInput.PPM.Secondary.PulseWidth._frame_period = 20000;
    Config::_config.RCInput.PPM.Secondary.PulseWidth._pre          =   200;
    Config::_config.RCInput.PPM.Secondary.PulseWidth._min          =   460;
    Config::_config.RCInput.PPM.Secondary.PulseWidth._max          =  1060;
    Config::_config.RCInput.PPM.Secondary.NumChannels._min         =     4;
    Config::_config.RCInput.PPM.Secondary.NumChannels._max         =    16;

    // +--------------+---------------+-----------------+
    // |              |               | PulseWidth (us) |
    // |     Mode     | _num_channels +--------+--------|
    // |              |               |  _min  |  _max  |
    // +--------------+---------------+--------+--------+
    // | Standard PPM |       8       |   920  |  2120  |
    // +--------------+---------------+--------+--------+
    //
    Config::_config.RCInput.PWM._num_channels    =     8;
    Config::_config.RCInput.PWM._jitter_filter   =     0;
    Config::_config.RCInput.PWM._average_filter  = false;
    Config::_config.RCInput.PWM.PulseWidth._min  =   920;
    Config::_config.RCInput.PWM.PulseWidth._max  =  2120;
#endif
}

void Config::save()
{
}

void Config::dump()
{
    printf("_magic                = 0x%04x\n", Config::_config._magic  );
    printf("_version              = %d\n"    , Config::_config._version);
    printf("\n");
    printf("Scheduler {\n");
    printf("  _loop_rate_hz       = %d\n"    , Config::_config.Scheduler._loop_rate_hz);
    printf("}\n");
    printf("\n");
    printf("RCInput {\n");
    printf("  PPM {\n");
    printf("    Primary {\n");
    printf("      PulseWidth {\n");
    printf("        _frame_period = %d\n", Config::_config.RCInput.PPM.Primary.PulseWidth._frame_period);
    printf("        _pre          = %d\n", Config::_config.RCInput.PPM.Primary.PulseWidth._pre         );
    printf("        _min          = %d\n", Config::_config.RCInput.PPM.Primary.PulseWidth._min         );
    printf("        _switch       = %d\n", Config::_config.RCInput.PPM.Primary.PulseWidth._switch      );
    printf("        _max          = %d\n", Config::_config.RCInput.PPM.Primary.PulseWidth._max         );
    printf("      }\n");
    printf("\n");
    printf("      NumChannels {\n");
    printf("        _min          = %d\n", Config::_config.RCInput.PPM.Primary.NumChannels._min);
    printf("        _max          = %d\n", Config::_config.RCInput.PPM.Primary.NumChannels._max);
    printf("      }\n");
    printf("    }\n");
    printf("\n");
    printf("    Secondary {\n");
    printf("      PulseWidth {\n");
    printf("        _frame_period = %d\n", Config::_config.RCInput.PPM.Secondary.PulseWidth._frame_period);
    printf("        _pre          = %d\n", Config::_config.RCInput.PPM.Secondary.PulseWidth._pre         );
    printf("        _min          = %d\n", Config::_config.RCInput.PPM.Secondary.PulseWidth._min         );
    printf("        _max          = %d\n", Config::_config.RCInput.PPM.Secondary.PulseWidth._max         );
    printf("      }\n");
    printf("\n");
    printf("      NumChannels {\n");
    printf("        _min          = %d\n", Config::_config.RCInput.PPM.Secondary.NumChannels._min);
    printf("        _max          = %d\n", Config::_config.RCInput.PPM.Secondary.NumChannels._max);
    printf("      }\n");
    printf("    }\n");
    printf("  }\n");
    printf("\n");
    printf("  PWM {\n");
    printf("    _num_channels     = %d\n", Config::_config.RCInput.PWM._num_channels);
    printf("    _jitter_filter    = %d\n", Config::_config.RCInput.PWM._jitter_filter);
    printf("    _average_filter   = %s\n", ( Config::_config.RCInput.PWM._average_filter ) ? "true" : "false");
    printf("\n");
    printf("    PulseWidth {\n");
    printf("      _min            = %d\n", Config::_config.RCInput.PWM.PulseWidth._min);
    printf("      _max            = %d\n", Config::_config.RCInput.PWM.PulseWidth._max);
    printf("    }\n");
    printf("  }\n");
    printf("}\n");
}
