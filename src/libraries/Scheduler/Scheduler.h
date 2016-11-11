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
 *  Adaptation for Sub: Francesco Bradascio 2016
 *
 */
#pragma once

#include <Utils/functor.h>

#define MIN_LOOP_RATE_HZ    50
#define MAX_LOOP_RATE_HZ    400

#define SCHEDULER_TASK_CLASS(class_name, class_ptr, func, rate_hz, max_time_us) \
{ \
    ._function    = FUNCTOR_BIND(class_ptr, &class_name::func, void),\
    ._name        = #func                                           ,\
    ._rate_hz     = rate_hz                                         ,\
    ._max_time_us = max_time_us\
}

/*
  A task scheduler for APM main loops

  Sketches should call scheduler.init() on startup, then call
  scheduler.tick() at regular intervals (typically every 10ms).

  To run tasks use scheduler.run(), passing the amount of time that
  the scheduler is allowed to use before it must return
 */
class Scheduler
{
public:
    Scheduler();

    FUNCTOR_TYPEDEF(Task_func_ptr_T, void);

    class Task
    {
        Task_func_ptr_T  _function;
        const char      *_name;
        float            _rate_hz;
        uint16_t         _max_time_us;
    };

    // initialise scheduler
    void init(const Task *tasks, uint8_t num_tasks);

    // call when one tick has passed
    void tick();

    // run the tasks. Call this once per 'tick'.
    // time_available is the amount of time available to run
    // tasks in microseconds
    void run(uint32_t time_available);

private:
    const Task *_tasks;             // progmem list of tasks to run
    uint8_t     _num_tasks;         // number of tasks in _tasks list
    uint16_t    _tick_counter;      // number of 'ticks' that have passed (number of times that
                                    // tick() has been called
    uint16_t   *_last_run;          // tick counter at the time we last ran each task
    uint32_t    _task_time_allowed; // number of microseconds allowed for the current task
    uint32_t    _task_time_started; // the time in microseconds when the task started
    uint32_t    _spare_us;          // number of spare microseconds accumulated
    uint8_t     _spare_ticks;       // number of ticks that _spare_micros is counted over
};
