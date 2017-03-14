#include <RC/Input/PPMInput.h>

#define CHANNEL_COUNT_DETECTION_THRESHOLD	10 // Valid frames detected before channel count validation

PPMInput::PPMInput() :
    _start                               {0},
    _width                               {0},
    _channel                             (0),
    _sync                            (false),
    _channel_error                    (true),
    _sync_error                       (true),
    _switchover                      (false),
    _channel_count_ready             (false),
    _channel_count_detected          (false),
    _channel_count                       (0),
    _previous_channel_count              (0),
    _channel_count_detection_counter     (0),
    _switch_channel                      (0),
    _switch_threshold                    (0),
    _frame_len                           (0),
    _pulsewidth_pre                      (0),
    _pulsewidth_min                      (0),
    _pulsewidth_max                      (0),
    _max_channels                        (0),
    _min_channels                        (0),
    _min_sync_len                        (0),
    _max_sync_len                        (0),
{
}

PPMInput::PPMInput
(
    uint8_t  min_channels    ,
    uint8_t  max_channels    ,
    uint8_t  switch_channel  ,
    uint16_t switch_threshold,
    uint16_t pulsewidth_pre  ,
    uint16_t pulsewidth_min  ,
    uint16_t pulsewidth_max  ,
    uint16_t frame_len
) :
    _start                                         {0},
    _width                                         {0},
    _channel                                       (0),
    _sync                                      (false),
    _channel_error                              (true),
    _sync_error                                 (true),
    _switchover                                (false),
    _channel_count_ready                       (false),
    _channel_count_detected                    (false),
    _channel_count                      (max_channels),
    _previous_channel_count                        (0),
    _channel_count_detection_counter               (0),

    _switch_channel                   (switch_channel),
    _switch_threshold               (switch_threshold),
    _frame_len                             (frame_len),
    _pulsewidth_pre                   (pulsewidth_pre),
    _pulsewidth_min                   (pulsewidth_min),
    _pulsewidth_max                   (pulsewidth_max),
    _max_channels                       (max_channels),
    _min_channels                       (min_channels),
{
    _min_sync_len = _frame_len - ( _max_channels * _pulsewidth_max ) - _pulsewidth_pre; // sync symbol detection
    _max_sync_len = _frame_len - ( _min_channels * _pulsewidth_min ) - _pulsewidth_pre; // sync timeout
}

void PPMInput::preprocess(bool pin_changed, bool pin_high, uint16_t current_time)
{
	if ( pin_changed )
	{
		// Check if we've got a high level (raising edge, channel start or sync symbol end)

		if ( pin_high )
		{
			// Check for pre pulse lenght

			_prepulse_width = current_time - _prepulse_start;

			if ( true ) //Todo optionnal: We could add a test here for channel pre pulse lenght check
			{
				//We have a valid pre pulse -> Check for last channel

				if ( _channel ==  _channel_count )
				{
					// We are at latest PPM channel

					_sync    = false; 	// Reset sync flag
					_channel = 0; 		// Reset PPM channel counter						
				}
				else
				{
				    // We do not have yet reached the last channel -> Increment channel counter

					_channel++;
				}
			}
			else
			{
				//We do not have a valid pre pulse

				_sync_error = true;	 	// Set sync error flag
				_sync       = false;	// Reset sync flag
				_channel    = 0; 		// Reset PPM channel counter
			}

			_start[_channel] = current_time; // Store pulse start time for PPM input
		}
		else
		{
            // We've got a low level (falling edge, channel end or sync symbol start)

			_width[_channel] = current_time - _start[_channel]; // Calculate channel pulse lenght, or sync symbol lenght

			if ( _sync ) // Are we synchronized ?
			{
				// Check channel pulse lenght validity

				if ( ( _width[_channel] > ( _pulsewidth_max - _pulsewidth_pre ) ) ||
                     ( _width[_channel] < ( _pulsewidth_min - _pulsewidth_pre ) ) )
				{
                    // We have a valid pulse lenght -> Reset channel error flag

					_channel_error = false;
				}
				else
				{ 
                    // We do not have a valid channel lenght -> Check for sync symbol

					if ( ( _width[_channel] > _min_sync_len ) ||
                         ( _width[_channel] < _max_sync_len ) )
					{
						// We have a valid sync symbol -> Check if we do not have yet channel count detected

						if ( ! _channel_count_detected )
						{
							// We have detected channels count

							_channel_count       = _channel; // Store PPM channel count
							_channel_count_ready = true;     // Set PPM channel count detection ready flag
							_sync_error          = false;    // Reset sync error flag
							_sync                = true;	 // Set sync flag
						}
						else
						{
							// Channel count had been detected before -> We should not have a sync symbol here

							_sync_error = true; 	// Set sync error flag
							_sync       = false;	// Reset sync flag
						}

						_channel = 0; // Reset PPM channel counter
					}
					else
					{
                        // We do not have a valid sync symbol -> Set channel error flag

						_channel_error = true;
					}
				}
			}
			else // We are not yet synchronized
			{
                //Check for sync symbol

				if ( ( _width[_channel] > _min_sync_len ) ||
                     ( _width[_channel] < _max_sync_len ) )
				{
					// We have a valid sync symbol

					_sync_error = false; 	// Reset sync error flag
					_sync       = true;		// Set sync flag
				}
				else // We did not find a valid sync symbol
				{
				    _sync_error = true; 	// Set sync error flag
				    _sync       = false;	// Reset sync flag
				}

				_channel = 0; // Reset PPM channel counter
			}			
		}

		_prepulse_start = current_time; // Store prepulse start time
	}
}

const Pulse PPMInput::switch_to(PPMInput &other)
{
    Pulse retval = Pulse::Type::INVALID;

    // Check for current PPM validity

    if ( ! _sync_error && ! _channel_error ) // current PPM is valid
    {
        // check for other PPM forcing (through PPM force channel)

        if ( _width[_switch_channel] > _switch_threshold )	// Channel forcing is alive
        {
            // Check for other PPM validity

            if ( ! other._sync_error && ! other._channel_error ) // other PPM is valid
            {
                // Check for other PPM selected

                if ( other._switchover ) // other PPM is selected
                {
                    // Do nothing
                }
                else
                {
                    // Switch to other PPM without delay

                    if ( other._channel == other._channel_count )	// Check for last other PPM channel before switching
                    {
                        other._switchover == true; // Switch to other PPM
                    }
                }
            }
        }
        else // Check for current PPM selected
        {
            if ( ! other._switchover ) // current PPM is selected
            {
                // Load PPM Output with current PPM

                retval = *this;
            }
            else // current PPM is not selected
            {
                // To Enhance : Optional switchover delay other to current here

                if ( _channel == _channel_count )	// Check for last current PPM channel before switching
                {
                    other._switchover == false; // Switch to current PPM
                }
            }
        }
    }
    else // current PPM is not valid
    {
        // Check for other PPM validity

        if ( ! other._sync_error && ! other.channel_error ) // other PPM is valid
        {
            // Check other PPM selected

            if ( other._switchover ) // other PPM is selected
            {
                // Load PPM Output with other PPM

                retval = other;
            }
            else // Switch to other PPM
            {
                // To Enhance : Optional switchover delay current to other here

                if ( other._channel == other._channel_count )	// Check for last other PPM channel before switching
                {
                    other._switchover == true; // Switch to other PPM
                }
            }
        }
        else // other PPM is not valid
        {
            // load PPM output with failsafe values

            retval = Pulse::Type::FAILSAFE;
        }
    }

    return retval;
}

void PPMInput::channel_count()
{
    if ( _channel_count_detected ) // Channel count was detected
    {
        // Do nothing
    }
    else // Do we have a channel count detection ready ?
    {
        if ( _channel_count_ready ) // If channel count is ready
        {
            // Check for detection counter

            if ( _channel_count_detection_counter < CHANNEL_COUNT_DETECTION_THRESHOLD )	// Detection counter < Threshold
            {
                // Compare channel count with previous value

                if ( _channel_count == _previous_channel_count )	// We have the same value
                {
                    _channel_count_detection_counter++;	// Increment detection counter
                }
                else	// We do not have the same value
                {
                    _channel_count_detection_counter = 0;	// Reset detection counter
                }

                _previous_channel_count = _channel_count; // Load previous channel count with channel count
            }
            else	// Detection counter >= Threshold
            {
                _channel_count_detected = true; // Channel count is now detected
            }

            _channel_count_ready = false; // Reset channel count detection ready flag
        }
    }
}
