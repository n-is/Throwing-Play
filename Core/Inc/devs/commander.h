#ifndef _COMMANDER_H_
#define _COMMANDER_H_

#include "queue_custom.h"


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

        uint8_t read();

private:
        Commander() { }
};

void Commander_Handle_RxCplt();

#endif //! _COMMANDER_H_
