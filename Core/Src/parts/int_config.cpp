#include "commander.h"
#include "main.h"

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
        if (huart->Instance == USART2) {
                Commander_Handle_RxCplt();
        }
}
