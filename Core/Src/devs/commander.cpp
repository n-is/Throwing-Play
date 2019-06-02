#include "commander.h"
#include "usart.h"

#define COMMANDER_UART  (huart2)
#define START_BYTE      (0xA5)

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

uint8_t Commander::read()
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