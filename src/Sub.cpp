#include <Sub.h>
#include <HAL/HAL.h>
#include <Config.h>

#define SCHEDULER_TASK(func, rate_hz, max_time_us) SCHEDULER_TASK_CLASS(Sub, &sub, func, rate_hz, max_time_us)

/*
  scheduler table for fast CPUs - all regular tasks apart from the fast_loop()
  should be listed here, along with how often they should be called (in hz)
  and the maximum time they are expected to take (in microseconds)
 */
const Scheduler::Task tasks[] =
{
/*
    SCHEDULER_TASK(func, rate_hz, max_time_us),
    ...
*/
};

Sub::Sub()
{
}

void Sub::setup()
{
    _scheduler.init(tasks);
}

void Sub::loop()
{
    // wait for an INS sample
    //
    ins.wait_for_sample();

    uint32_t timer = HAL::micros();

    // for mainloop failure monitoring
    //
    _main_loop_count++;

    // Execute the fast loop
    // ---------------------
    //
    fast_loop();

    // tell the scheduler one tick has passed
    //
    _scheduler.tick();

    // run all the tasks that are due to run. Note that we only
    // have to call this once per loop, as the tasks are scheduled
    // in multiples of the main loop tick. So if they don't run on
    // the first call to the scheduler they won't run on a later
    // call until scheduler.tick() is called again
    //
    uint32_t time_available = (timer + MAIN_LOOP_MICROS) - micros();

    _scheduler.run(time_available);
}
