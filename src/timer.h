#ifndef _TIMER_H
#define _TIMER_H

#include <iostream>
#include <ctime>

class Timer
{
    public:
        Timer();
        ~Timer();

    private:
        const std::clock_t m_startTime;
};

#endif 