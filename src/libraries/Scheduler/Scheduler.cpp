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

/*
 *  main loop scheduler
 *
 *  Author: Andrew Tridgell, January 2013
 *  Adaptation for Sub: Francesco Bradascio, November 2016
 *
 */
#include <Scheduler/Scheduler.h>
#include <Config/Config.h>

#include <Utils/Utils.h>
#include <HAL/HAL.h>
#include <stdio.h>

DECLARE_FIELD( PUInt16, Scheduler, _loop_rate_hz, DEFAULT_MAIN_LOOP_RATE );

// constructor
Scheduler::Scheduler(void)
{
    _loop_rate_hz = constraint(_loop_rate_hz,MIN_LOOP_RATE_HZ,MAX_LOOP_RATE_HZ);
}

// initialise the scheduler
void Scheduler::init(const Scheduler::Task *tasks, uint8_t num_tasks)
{
    _tasks        = tasks;
    _num_tasks    = num_tasks;
    _tick_counter = 0;
    _last_run     = new uint16_t[_num_tasks];

    memset(_last_run, 0, sizeof(_last_run[0]) * _num_tasks);
}

// one tick has passed
void Scheduler::tick(void)
{
    _tick_counter++;
}

/*
  run one tick
  this will run as many scheduler tasks as we can in the specified time
 */
void Scheduler::run(uint32_t time_available)
{
    uint32_t run_started_usec = HAL::micros();
    uint32_t now              = run_started_usec;

    for (uint8_t i=0; i<_num_tasks; i++)
    {
        uint16_t dt             = _tick_counter - _last_run[i];
        uint16_t interval_ticks = _loop_rate_hz / _tasks[i]._rate_hz;

        if (interval_ticks < 1)
        {
            interval_ticks = 1;
        }
        
        if (dt >= interval_ticks)
        {
            // this task is due to run. Do we have enough time to run it?
            _task_time_allowed = _tasks[i]._max_time_us;

            if (_task_time_allowed <= time_available)
            {
                // run it
                _task_time_started = now;

                _tasks[i]._function();

                // record the tick counter when we ran. This drives
                // when we next run the event
                _last_run[i] = _tick_counter;

                // work out how long the event actually took
                now = HAL::micros();
                uint32_t time_taken = now - _task_time_started;

                if (time_taken >= time_available)
                {
                    goto update_spare_ticks;
                }

                time_available -= time_taken;
            }
        }
    }

    // update number of spare microseconds
    _spare_us += time_available;

update_spare_ticks:

    _spare_ticks++;

    if (_spare_ticks == 32)
    {
        _spare_ticks /= 2;
        _spare_us /= 2;
    }
}
