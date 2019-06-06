/*
 * commander.h
 *
 * Created : 6/2/2019
 *  Author : n-is
 *   email : 073bex422.nischal@pcampus.edu.np
 */

#ifndef _COMMANDER_H_
#define _COMMANDER_H_

#include "queue_custom.h"

enum Throw_Commands {
        GRIP = 0x01,
        ACTUATE = 0x02,
        EXTEND = 0x03 ,
        PASS_GEREGE = 0x04,
        THROW = 0x05,
        RETRIEVE = 0x06,
        MOVE_PLATFORM_LEFT = 0x07,
        MOVE_PLATFORM_RIGHT = 0x08
};

class Commander
{
public:
        Commander(Commander &&) = default;
        Commander(const Commander &) = default;
        Commander &operator=(Commander &&) = default;
        Commander &operator=(const Commander &) = default;
        ~Commander() { }

        static Commander& get_Instance();
        int init();
        bool is_Empty();
        void clear();

        uint8_t get_Command();

private:
        Commander() { }
};

void Commander_Handle_RxCplt();

#endif //! _COMMANDER_H_
