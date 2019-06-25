/*
 * processor.h
 *
 * Created : 6/2/2019
 *  Author : n-is
 *   email : 073bex422.nischal@pcampus.edu.np
 */

#ifndef _PROCESSOR_H_
#define _PROCESSOR_H_

#include "actuation_packet.h"
#include "commander.h"

class Processor
{
public:
        Processor(Processor &&) = default;
        Processor(const Processor &) = default;
        Processor &operator=(Processor &&) = default;
        Processor &operator=(const Processor &) = default;
        ~Processor() { }

        static Processor& get_Instance();
        
        int init(uint32_t dt_millis);

        Actuation_Packet process(uint8_t cmd, uint32_t dt_millis);

private:
        Actuation_Packet last_pack_;

        Processor() { reset_Actuation_Packet(last_pack_); }

        void process_Gerege(Actuation_Packet &pack, uint8_t cmd);
        void process_Grip(Actuation_Packet &pack);
        void process_Extend(Actuation_Packet &pack, Throw_Commands cmd);
        void process_Shoot(Actuation_Packet &pack);

        void process_Arm(Actuation_Packet &pack);
        void process_Platform(Actuation_Packet &pack, uint8_t cmd);
};

#endif // !_PROCESSOR_H_
