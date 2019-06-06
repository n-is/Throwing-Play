/*
 * throwing.h
 *
 * Created : 6/2/2019
 *  Author : n-is
 *   email : 073bex422.nischal@pcampus.edu.np
 */

#ifndef _THROWING_H_
#define _THROWING_H_

#include "stm32f4xx.h"

#include "commander.h"
#include "processor.h"
#include "actuator.h"

class Throwing final
{
public:
        Throwing(Throwing &&) = default;
        Throwing(const Throwing &) = default;
        Throwing &operator=(Throwing &&) = default;
        Throwing &operator=(const Throwing &) = default;

        ~Throwing() { }
        static Throwing& get_Instance();

        int init(uint32_t dt_millis);
        void update(uint32_t dt_millis);
        void run(uint32_t dt_millis);
        bool is_Initiated() const;

private:
        Commander *general_;
        Processor *brain_;
        Actuator *soldier_;

        Actuation_Packet act_pack_;

        bool initiated_;
        Throwing() {
                initiated_ = false;
        }
};
  
#endif // !_THROWING_H_
