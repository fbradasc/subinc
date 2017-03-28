#include <RC/Input/PWM.h>

// -------------------------------------------------------------
// SERVO PWM MODE input settings
// -------------------------------------------------------------

#define PWM_CH_NUM             8
#define PWM_PW_MIN           920
#define PWM_PW_MAX          2120

namespace _RCInput
{
    namespace PWM
    {
        DECLARE_FIELD( PUInt8     , RCInput.PWM, _num_channels  , PWM_CH_NUM );
        DECLARE_FIELD( PPulseWidth, RCInput.PWM, _jitter_filter , SHRT_MIN   );
        DECLARE_FIELD( PBool      , RCInput.PWM, _average_filter, false      );

        namespace PulseWidth
        {
            DECLARE_FIELD( PPulseWidth, RCInput.PWM.PulseWidth, _min, PWM_PW_MIN );
            DECLARE_FIELD( PPulseWidth, RCInput.PWM.PulseWidth, _max, PWM_PW_MAX );
        };
    };
};

