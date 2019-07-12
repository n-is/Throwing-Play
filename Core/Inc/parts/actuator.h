/*
 * actuator.h
 *
 * Created : 6/2/2019
 *  Author : rishi and nis-ane
 *   email : 073bex433.rishav@pcampus.edu.np
 *   email : 073bex421.nischal@pcampus.edu.np
 */

#ifndef _ACTUATOR_H_
#define _ACTUATOR_H_

#include "wheel.h"
#include "actuation_packet.h"

class Actuator final
{
public:
        Actuator(Actuator &&) = default;
        Actuator(const Actuator &) = default;
        Actuator &operator=(Actuator &&) = default;
        Actuator &operator=(const Actuator &) = default;
        ~Actuator() { }

        static Actuator& get_Instance();

        int init(Actuation_Packet pack);
        void actuate(Actuation_Packet pack, uint32_t dt_millis);
        uint32_t stop(uint32_t dt_millis, uint32_t max_time = 1000);
        void clear();

private:
        Wheel wheels_[2];

        Actuator() { }
        void actuate_Pneumatics(Actuation_Packet pack);
        void wheels_Init();
        void pid_Init();
};

#endif // !_ACTUATOR_H_
