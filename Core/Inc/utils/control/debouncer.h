/*
 * debouncer.h
 *
 * Created : 6/4/2019
 *  Author : n-is
 *   email : 073bex422.nischal@pcampus.edu.np
 */

#ifndef _DEBOUNCE_H_
#define _DEBOUNCE_H_

#include "stm32f4xx.h"

class Debouncer
{
public:
        Debouncer(uint32_t (&get_Time)(void), uint32_t time_mills) {
                get_Time_ = &get_Time;
                time_mills_ = time_mills;
                is_held_ = false;
                is_first_ = true;
                start_time_ = 0;
        }
        Debouncer() = default;
        Debouncer(Debouncer &&) = default;
        Debouncer(const Debouncer &) = default;
        Debouncer &operator=(Debouncer &&) = default;
        Debouncer &operator=(const Debouncer &) = default;
        ~Debouncer() { }

        bool is_Ready();
        void hold();
        void release();

private:
        bool is_held_;
        bool is_first_;
        uint32_t time_mills_;
        uint32_t (*get_Time_)(void);
        uint32_t start_time_;
};

#endif // !_DEBOUNCE_H_
