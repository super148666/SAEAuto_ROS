#include "Timer.h"

Timer::Timer() {}

void Timer::start()   { start_time = Clock::now(); }
void Timer::refresh() { start_time = Clock::now(); }

double Timer::getSeconds() {
	Duration elapsed = Clock::now() - start_time;
    return elapsed.count()/1000.0;
}