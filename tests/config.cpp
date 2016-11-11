#include <stdio.h>

#include <Config/Config.h>

DECLARE_FIELD( PUInt16, Scheduler, _loop_rate_hz, DEFAULT_MAIN_LOOP_RATE );

class Scheduler
{
public:
    Scheduler();

    void update()
    {
        _loop_rate_hz = 765;
        DBG_TRACE_PF("_loop_rate_hz=%d", (int)_loop_rate_hz);
        _loop_rate_hz += 10;
        DBG_TRACE_PF("_loop_rate_hz=%d", (int)_loop_rate_hz);
        _loop_rate_hz -=  5;
        DBG_TRACE_PF("_loop_rate_hz=%d", (int)_loop_rate_hz);
    }
};

Scheduler::Scheduler()
{
    DBG_TRACE_F();
    _loop_rate_hz = 314;
}

int main(int argc, char *argv[])
{
    // Config::dump();

    // Scheduler sched;

    Config::dump();

    Config::load();

    Scheduler sched;

    sched.update();

    Config::dump();
}
