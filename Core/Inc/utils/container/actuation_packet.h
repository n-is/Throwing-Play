/*
 * actuation_packet.h
 *
 * Created : 6/2/2019
 *  Author : n-is
 *   email : 073bex422.nischal@pcampus.edu.np
 */

#ifndef _ACTUATION_PACKET_H_
#define _ACTUATION_PACKET_H_

#include "stm32f4xx.h"

struct Actuation_Packet
{
        bool gerege;
        bool rotate_gerege;
        bool extend;
        bool shoot;
        bool grip;
        bool extend_shoot;

        float platform_angle;
        float arm_angle;
};

void reset_Actuation_Packet(Actuation_Packet &pack);
void print_Actuation_Packet(Actuation_Packet &pack);

#endif // !_ACTUATION_PACKET_H_
