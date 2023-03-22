//
// Created by sentry on 2023/3/6.
//
#include "../inc/Systime.h"



static systime getsystime(){
    timeval tv;
    gettimeofday(&tv, nullptr);
    return tv.tv_usec / 1000.0 + tv.tv_sec * 1000.0;
}



void getsystime(systime &t) {
    static systime time_base = getsystime();
    timeval tv;
    gettimeofday(&tv, nullptr);
    t = tv.tv_usec / 1000.0 + tv.tv_sec * 1000.0 - time_base;
}
double getTimeIntervalms(const systime &now, const systime &last) {
    return now - last;
}


void Timer::getSystime(double &t)
{
    static double time_base = getsystime();
    timeval tv;
    gettimeofday(&tv, nullptr);
    t = tv.tv_usec / 1000.0 + tv.tv_sec * 1000.0 - time_base;
}

double Timer::getTimeIntervalms(const double &now, const double &last)
{
    return now - last;
}
