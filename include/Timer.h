#ifndef TIMER_H
#define TIMER_H

#include <chrono>

typedef std::chrono::system_clock Clock;
typedef std::chrono::duration<double, std::milli> Duration;

class Timer {
public:
    Timer();
    virtual ~Timer(){};
    
    void start();
    void refresh();

    double getSeconds();

private:
    std::chrono::time_point<Clock> start_time;
};

#endif /* TIMER_H */