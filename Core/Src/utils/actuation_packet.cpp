/*
 * actuation_packet.cpp
 *
 * Created : 6/2/2019
 *  Author : n-is
 *   email : 073bex422.nischal@pcampus.edu.np
 */

#include "actuation_packet.h"

void reset_Actuation_Packet(Actuation_Packet &pack)
{
        pack.gerege = false;
        pack.rotate_gerege = false;
        pack.extend = false;
        pack.shoot = false;
        pack.grip = false;
        pack.extend_shoot = false;

        pack.platform_angle = 0;
        pack.arm_angle = 0;
}

void print_Actuation_Packet(Actuation_Packet &pack)
{
        printf("%d, %d, %d, %d, %d, %d, %d, %d\n",  pack.gerege,
                                                pack.rotate_gerege,
                                                pack.extend,
                                                pack.shoot,
                                                pack.grip,
                                                pack.extend_shoot,
                                                (int16_t)pack.platform_angle,
                                                (int16_t)pack.arm_angle);
}
