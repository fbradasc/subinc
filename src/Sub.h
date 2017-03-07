#pragma once

#include <Scheduler/Scheduler.h>

class Sub
{
public:
    Sub();

    void setup();
    void loop ();

private:
    Scheduler _scheduler;
    PPMSum    _ppmsum;
    Servos    _servos;
};
