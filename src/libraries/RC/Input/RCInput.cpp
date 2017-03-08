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
#include <Config/Config.h>

#include <stdio.h>

// PPM input frame mode receiver 1
// -------------------------------------------------------------
//#define PPM_PRI "STANDARD"    // Standard PPM : 1520 us +/- 600 us - 8 channels - 20 ms frame period
//#define PPM_PRI "EXTENDED"    // 9 channels : 1520 us +/- 600 us - 9 channels - 22.1 ms slower frame period
//#define PPM_PRI "V2"          // PPMv2 : 760 us +/- 300 us - 16 Channels - normal 20 ms frame period
#define PPM_PRI "V3"            // PPMv3 16 channels with long sync symbol : 1050 us +/- 300 us - 25 ms frame period

// PPM input frame mode receiver 2
// -------------------------------------------------------------
//#define PPM_SEC "STANDARD"
#define PPM_SEC "EXTENDED"
//#define PPM_SEC "V2"
//#define PPM_SEC "V3"

// PPM1 input : frame formats definitions
// -------------------------------------------------------------
#if ( PPM_PRI == "STANDARD" )

#  define PPM_PRI_CH_MIN                 4
#  define PPM_PRI_CH_MAX                 8
#  define PPM_PRI_PW_FRAME_PERIOD    20000   // frame period (microseconds)
#  define PPM_PRI_PW_MIN               920
#  define PPM_PRI_PW_MAX              2120
#  define PPM_PRI_PW_SWITCH           1800
#  define PPM_PRI_PW_PREPULSE_LENGHT   400

#elif ( PPM_PRI "EXTENDED" )

#  define PPM_PRI_CH_MIN                 4
#  define PPM_PRI_CH_MAX                 9
#  define PPM_PRI_PW_FRAME_PERIOD    22500   // frame period (microseconds)
#  define PPM_PRI_PW_MIN               920
#  define PPM_PRI_PW_MAX              2120
#  define PPM_PRI_PW_SWITCH           1800
#  define PPM_PRI_PW_PREPULSE_LENGHT   400

#elif ( PPM_PRI == "V2" ) // PPMv2 is a 50 Hz 16 channels mode

#  define PPM_PRI_CH_MIN                 4
#  define PPM_PRI_CH_MAX                16
#  define PPM_PRI_PW_FRAME_PERIOD    20000   // frame period (microseconds)
#  define PPM_PRI_PW_MIN               460
#  define PPM_PRI_PW_MAX              1060
#  define PPM_PRI_PW_SWITCH            900
#  define PPM_PRI_PW_PREPULSE_LENGHT   200

#elif ( PPM_PRI == "V3" ) //  PPMv3 is a 40 Hz slower refresh rate 16 channels mode

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
#if ( PPM_SEC == "STANDARD" )

#  define PPM_SEC_CH_MIN                 4
#  define PPM_SEC_CH_MAX                 8
#  define PPM_SEC_PW_FRAME_PERIOD    20000   // frame period (microseconds)
#  define PPM_SEC_PW_MIN               920
#  define PPM_SEC_PW_MAX              2120
#  define PPM_SEC_PW_SWITCH           1800
#  define PPM_SEC_PW_PREPULSE_LENGHT   400

#elif (PPM_SEC == "EXTENDED" )

#  define PPM_SEC_CH_MIN                 4
#  define PPM_SEC_CH_MAX                 9
#  define PPM_SEC_PW_FRAME_PERIOD    22500   // frame period (microseconds)
#  define PPM_SEC_PW_MIN               920
#  define PPM_SEC_PW_MAX              2120
#  define PPM_SEC_PW_SWITCH           1800
#  define PPM_SEC_PW_PREPULSE_LENGHT   400

#elif ( PPM_SEC == "V2" ) // PPMv2 is a 50 Hz 16 channels mode

#  define PPM_SEC_CH_MIN                 4
#  define PPM_SEC_CH_MAX                16
#  define PPM_SEC_PW_FRAME_PERIOD    20000   // frame period (microseconds)
#  define PPM_SEC_PW_MIN               460
#  define PPM_SEC_PW_MAX              1060
#  define PPM_SEC_PW_SWITCH            900
#  define PPM_SEC_PW_PREPULSE_LENGHT   200

#elif ( PPM_SEC == "V3" ) //  PPMv3 is a 40 Hz slower refresh rate 16 channels mode

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
#define PWM_AVERAGE_FILTER  true
#define PWM_JITTER_FILTER  false

DECLARE_FIELD( PUint16, RCInput.PPM.Primary.PulseWidth   , _frame_period, PPM_PRI_PW_FRAME_PERIOD    );
DECLARE_FIELD( PUint16, RCInput.PPM.Primary.PulseWidth   , _pre         , PPM_PRI_PW_PREPULSE_LENGHT );
DECLARE_FIELD( PUint16, RCInput.PPM.Primary.PulseWidth   , _min         , PPM_PRI_PW_MIN             );
DECLARE_FIELD( PUint16, RCInput.PPM.Primary.PulseWidth   , _switch      , PPM_PRI_PW_SWITCH          );
DECLARE_FIELD( PUint16, RCInput.PPM.Primary.PulseWidth   , _max         , PPM_PRI_PW_MAX             );
DECLARE_FIELD( PUint8 , RCInput.PPM.Primary.NumChannels  , _min         , PPM_PRI_CH_MIN             );
DECLARE_FIELD( PUint8 , RCInput.PPM.Primary.NumChannels  , _max         , PPM_PRI_CH_MAX             );

DECLARE_FIELD( PUint16, RCInput.PPM.Secondary.PulseWidth , _frame_period, PPM_SEC_PW_FRAME_PERIOD    );
DECLARE_FIELD( PUint16, RCInput.PPM.Secondary.PulseWidth , _pre         , PPM_SEC_PW_PREPULSE_LENGHT );
DECLARE_FIELD( PUint16, RCInput.PPM.Secondary.PulseWidth , _min         , PPM_SEC_PW_MIN             );
DECLARE_FIELD( PUint16, RCInput.PPM.Secondary.PulseWidth , _max         , PPM_SEC_PW_MAX             );
DECLARE_FIELD( PUint8 , RCInput.PPM.Secondary.NumChannels, _min         , PPM_SEC_CH_MIN             );
DECLARE_FIELD( PUint8 , RCInput.PPM.Secondary.NumChannels, _max         , PPM_SEC_CH_MAX             );
                                                                          
DECLARE_FIELD( PUint8 , RCInput.PWM                      , _num_channels, PWM_CH_NUM                 );
DECLARE_FIELD( PUint16, RCInput.PWM.PulseWidth           , _min         , PWM_PW_MIN                 );
DECLARE_FIELD( PUint16, RCInput.PWM.PulseWidth           , _max         , PWM_PW_MAX                 );
DECLARE_FIELD( PBool  , RCInput.PWM.Filters              , _average     , PWM_AVERAGE_FILTER         );
DECLARE_FIELD( PBool  , RCInput.PWM.Filters              , _jitter      , PWM_JITTER_FILTER          );
