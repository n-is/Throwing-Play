/*
 * robo_tasks.cpp
 * 
 * Created : **
 *  Author : n-is
 *   email : 073bex422.nischal@pcampus.edu.np
 */

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

#include "throwing.h"

/* Export Functions Used in C */
extern "C" void StartDefaultTask(void const *argument);
extern "C" void ThrowingThread(void const *argument);
extern "C" void ActuationThread(void const *argument);

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const *argument)
{

        /* USER CODE BEGIN StartDefaultTask */
        /* Infinite loop */
        for (;;)
        {
                osDelay(1);
        }
        /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_ThrowingThread */
/**
* @brief Function implementing the ThrowingTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ThrowingThread */

Throwing &Throwing_Mechanism = Throwing::get_Instance();

void ThrowingThread(void const *argument)
{
        /* USER CODE BEGIN RobotThread */
        uint32_t sample_period = 10;

        //! Initialize the throwing part of the robot here
        Throwing_Mechanism.init(sample_period);

        uint32_t dt = HAL_GetTick();
        uint32_t dt_tmp = HAL_GetTick();
        uint32_t last_run_time = 0;
        HAL_GPIO_WritePin(B_OrangeLED_GPIO_Port, B_OrangeLED_Pin, GPIO_PIN_SET);
        osDelay(sample_period);

        /* Infinite loop */
        for (;;)
        {
                // Since this is the highest priority task, we can be sure that
                // another task won't start when this task is running
                dt_tmp = HAL_GetTick();
                dt = dt_tmp - dt;

                //! Update the sensing and reasoning part here
                Throwing_Mechanism.update(sample_period);

                // for (uint32_t i = 0; i < 100; ++i) {
                //         printf("%ld, %ld\n", dt + last_run_time, i);
                // }
  
                dt = HAL_GetTick();
                dt_tmp = dt - dt_tmp;
                last_run_time = dt_tmp;

                // Check for timing Issues
                if (last_run_time > sample_period / 2) {
                        // Timing Issue Occured since run time is more than half
                        // of sample time
                }

                // Sleep for remaining time of the sampling period if there is
                // time left
                if (dt_tmp < sample_period) {
                        osDelay(sample_period - dt_tmp);
                }
        }
        /* USER CODE END ThrowingThread */
}

/* USER CODE BEGIN Header_ActuationThread */
/**
* @brief Function implementing the ActuationTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ActuationThread */
void ActuationThread(void const *argument)
{
        /* USER CODE BEGIN ActuationThread */
        uint32_t sample_period = 10;
        

        uint32_t dt = HAL_GetTick();
        uint32_t dt_tmp = HAL_GetTick();
        uint32_t last_run_time = 0;
        /* Infinite loop */
        for (;;)
        {
                if (Throwing_Mechanism.is_Initiated()) {
                        // Since this is the highest priority task, we can be sure that
                        // another task won't start when this task is running
                        dt_tmp = HAL_GetTick();
                        dt = dt_tmp - dt;

                        //! Update the actuation part of the Throwing_Mechanism
                        Throwing_Mechanism.run(sample_period);

                        // for (uint32_t i = 0; i < 100; ++i) {
                        //         printf("%ld, %ld\n", dt + last_run_time, i);
                        // }

                        dt = HAL_GetTick();
                        dt_tmp = dt - dt_tmp;
                        last_run_time = dt_tmp;

                        // Check for timing Issues
                        if (last_run_time > sample_period / 2) {
                                // Timing Issue Occured since run time is more than half
                                // of sample time
                        }

                        // Sleep for remaining time of the sampling period if there is
                        // time left
                        if (dt_tmp < sample_period) {
                                osDelay(sample_period - dt_tmp);
                        }
                }
        }
        /* USER CODE END ActuationThread */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
