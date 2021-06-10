#include "timer.h"

Timer::Timer()
    : m_startTime(std::clock()) {};

Timer::~Timer()
{
    const double l_totalTime = (std::clock() - m_startTime) / double(CLOCKS_PER_SEC);
    std::cout << "Time: " << l_totalTime << "\n";
}
