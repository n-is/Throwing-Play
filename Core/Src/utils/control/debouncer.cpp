/*
 * debouncer.cpp
 *
 * Created : 6/4/2019
 *  Author : n-is
 *   email : 073bex422.nischal@pcampus.edu.np
 */

#include "debouncer.h"

bool Debouncer::is_Ready()
{
        if (is_first_) {
                is_first_ = false;
                if (!is_held_) {
                        start_time_ = get_Time_();
                }
                is_held_ = false;
        }
        else {
                if (is_held_) {
                        uint32_t curr_time = get_Time_();
                        if (curr_time - start_time_ > time_mills_) {
                                is_held_ = false;
                        }
                }
        }

        return !is_held_;
}

void Debouncer::hold()
{
        start_time_ = get_Time_();
        is_held_ = true;
}

void Debouncer::release()
{
        is_held_ = false;
}
