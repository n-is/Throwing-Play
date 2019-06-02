/*
 * commander.cpp
 *
 * Created : 6/2/2019
 *  Author : n-is
 *   email : 073bex422.nischal@pcampus.edu.np
 */

#include "commander.h"
#include "usart.h"

#include "defines.h"

#define COMMANDER_UART  (huart2)

Queue<uint8_t, 2> gCommands;
static uint8_t gRxData;

Commander& Commander::get_Instance()
{
        static Commander sCommander;
        
        return sCommander;
}

int Commander::init()
{
        HAL_UART_Receive_DMA(&huart2, &gRxData, 1);
        return 0;
}

bool Commander::is_Empty()
{
        return gCommands.is_Empty();
}

uint8_t Commander::get_Command()
{
        return gCommands.lookup();
}

void Commander::clear()
{
        gCommands.clear();
}

static bool gStart_Byte_Received = false;

void Commander_Handle_RxCplt()
{
        __HAL_UART_FLUSH_DRREGISTER(&COMMANDER_UART);
        // printf("%d\n", gRxData);
        if (!gStart_Byte_Received) {
                if (gRxData == START_BYTE) {
                        gStart_Byte_Received = true;
                }
        }
        else {
                gStart_Byte_Received = false;
                gCommands.insert(gRxData);
        }
}
