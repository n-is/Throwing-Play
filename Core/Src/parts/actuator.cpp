#include <math.h>

#include "actuator.h"
#include "pid_algorithms.h"
#include "pid.h"
#include "stdio.h"

// We should make sure that the Actuator ony have one instance and it is properly
// instantiated
Actuator& Actuator::get_Instance()
{
        static Actuator sBase_Instance;

        return sBase_Instance;
}


/**
 * @brief Function that initializes all the required components for the robot's
 *        actuator(omni-base)
 * @retval None
 * 
 * 
 * <pre>
 * Tasks performed by this function
 * 1) Call the devices initializations in correct order
 * 2) Call the software initializations for utilities like pid for wheels
 * </pre>
 */
int Actuator::init(Actuation_Packet pack)
{
        // Initialize all wheels of the robot
        wheels_Init();
        // Initializes PID parameters for all wheels
        pid_Init();
        // printf("sp   value   cal\n");
        actuate_Pneumatics(pack);

        return 0;
}

void Actuator::actuate(Actuation_Packet pack, uint32_t dt_millis)
{
        actuate_Pneumatics(pack);
        
        wheels_[1].update_DeltaCount();

        // *** Correct Motor's Moving *** //

        if (fabsf(pack.arm_angle - ARM_MOTOR_HOME_ANGLE) < 10) {

                // Read Limit Switch and Correct Arm Angle to zero if set
                GPIO_PinState ls_state = HAL_GPIO_ReadPin(THROWING_SWITCH_GPIO_Port, THROWING_SWITCH_Pin);
                if (ls_state != GPIO_PIN_SET) {
                        wheels_[1].set_Angle(pack.arm_angle);
                        printf("Limit Switch On!!\n");
                }

                float arm_angle = fabsf(ARM_MOTOR_HOME_ANGLE - wheels_[1].get_Angle());

                if (arm_angle < 5) {

                        // if arm_angle < 1 and Limit Switch is clear, set arm angle to
                        // some value in between home position and shagai picking pos,
                        // but closer to home position

                        if (ls_state == GPIO_PIN_SET) {
                                wheels_[1].set_Angle(ARM_MOTOR_ERROR_ANGLE);
                                printf("Limit Switch Off!!\n");
                        }
                }

                printf("%ld, %ld\n", (int32_t)(pack.arm_angle), (int32_t)(wheels_[1].get_Angle()));
        }
        // *** Correct Motor's Moving End *** //

        float set_points[2] = { pack.platform_angle, pack.arm_angle };

        float omega;
        float angle;
        float calc_omega;
        float error;
        float err;
        float voltage;
        float new_omega;
        PID *pid;
        float max_voltage;
        float max_omega;

        // printf("%ld   ", HAL_GetTick());
        
        angle = wheels_[1].get_Angle();
        err = set_points[1] - angle;

        if (fabs(err) <= 0.02) {
                err = 0;
        }

        pid = wheels_[1].get_AnglePIDController();
        // Use angle pid to compute omega
        calc_omega = pid->compute_PID(err, dt_millis);

        omega = wheels_[1].get_Omega(dt_millis);
        error = calc_omega - omega;
        
        /**/
        // error = omega_set_points[i] - omega;
        /**/

        pid = wheels_[1].get_PIDController();
        // The controller's output is voltage
        voltage = pid->compute_PID(error, dt_millis);

        // Max Omega corresponds to the max voltage value
        max_voltage = fabsf(pid->get_Algorithm()->get_Upper());
        max_omega = wheels_[1].get_MaxOmega();
        // Controller's output voltage is converted to the corresponding
        // omega according to linear relation since we will just be
        // output-ting voltage and this is just a abstraction of the
        // motor driver
        new_omega = voltage * max_omega / max_voltage;

        wheels_[1].set_Omega(new_omega);
        // if(i == 0)
        //         printf("%ld   %ld   %ld   ", (int32_t)(set_points[i]*1000), (int32_t)(angle*1000), (int32_t)(new_omega*1000));
        // wheels_[i].log(omega[i], new_omega[i]);

        // printf("\n");


        // We don't want to delete the pointer since it was not us who allocated it
        pid = 0;

        for (uint8_t i = 0; i < 2; ++i) {
                wheels_[i].update();
        }

        
}

void Actuator::actuate_Pneumatics(Actuation_Packet pack)
{
        if (pack.gerege) {
                HAL_GPIO_WritePin(Gerege_GPIO_Port, Gerege_Pin, GPIO_PIN_SET);
        }
        else {
                HAL_GPIO_WritePin(Gerege_GPIO_Port, Gerege_Pin, GPIO_PIN_RESET);
        }

        if (pack.extend) {
                HAL_GPIO_WritePin(Extend_GPIO_Port, Extend_Pin, GPIO_PIN_SET);
        }
        else {
                HAL_GPIO_WritePin(Extend_GPIO_Port, Extend_Pin, GPIO_PIN_RESET);
        }
        
        if (pack.grip) {
                HAL_GPIO_WritePin(Grip_GPIO_Port, Grip_Pin, GPIO_PIN_SET);
        }
        else {
                HAL_GPIO_WritePin(Grip_GPIO_Port, Grip_Pin, GPIO_PIN_RESET);
        }

        if (pack.shoot) {
                HAL_GPIO_WritePin(Shoot_GPIO_Port, Shoot_Pin, GPIO_PIN_SET);
        }
        else {
                HAL_GPIO_WritePin(Shoot_GPIO_Port, Shoot_Pin, GPIO_PIN_RESET);
        }

        if (pack.extend_shoot) {
                HAL_GPIO_WritePin(Extend_Shoot_GPIO_Port, Extend_Shoot_Pin, GPIO_PIN_SET);
        }
        else {
                HAL_GPIO_WritePin(Extend_Shoot_GPIO_Port, Extend_Shoot_Pin, GPIO_PIN_RESET);
        }
}


uint32_t Actuator::stop(uint32_t dt_millis, uint32_t max_time)
{
        return 0;
}

void Actuator::clear()
{
}


static Wheel_Config gWheel_Configurations[2];
/**
 * @brief Function that initializes all the required components for the wheels
 *        of the robot
 * @retval None
 * 
 * 
 * <pre>
 * Tasks performed by this function
 * 1) Assigns appropriate IDs to each wheel
 * 2) Give each wheel the respective radius
 * 3) Assign a timer and a channel to each wheel for it's motor to run in PWM
 *    mode. A timer is shared among all the available wheels since we have
 *    four wheels.
 * 4) Assigns the ppr of each wheel's encoder
 * 5) Assigns the direction pins for each motor's direction
 * 6) Assigns timer for each wheel's encoder
 * 7) Starts the timers to run each wheel in respective configured modes like PWM
 *    mode and the Encoder mode
 * </pre>
 */
void Actuator::wheels_Init(void)
{
        int i;
        for (i = 0; i < 2; i++)
        {
                gWheel_Configurations[i].id = i + 1;
                gWheel_Configurations[i].radius = 0.0675;
                // Both motors are connected to same timer : TIM2
                gWheel_Configurations[i].htim = &htim2;
                //! Need to make sure the following value is correct
                
        }
        gWheel_Configurations[0].in2_port = GPIOC;
        gWheel_Configurations[0].in2_pin = GPIO_PIN_3;
        gWheel_Configurations[0].in1_port = GPIOC;
        gWheel_Configurations[0].in1_pin = GPIO_PIN_1; 
        gWheel_Configurations[0].channel = TIM_CHANNEL_2;
        gWheel_Configurations[0].henc = &htim1;
        gWheel_Configurations[0].max_omega = 19.37 / 5.16; // 3.75
        gWheel_Configurations[0].enc_ppr = 567 * 5.16;

        gWheel_Configurations[1].in2_port = GPIOC;
        gWheel_Configurations[1].in2_pin = GPIO_PIN_0;
        gWheel_Configurations[1].in1_port = GPIOC;
        gWheel_Configurations[1].in1_pin = GPIO_PIN_2;
        gWheel_Configurations[1].channel = TIM_CHANNEL_1;
        gWheel_Configurations[1].henc = &htim3;
        gWheel_Configurations[1].max_omega = 63.86 / 4.0;  // 15.965
        gWheel_Configurations[1].enc_ppr = 249.6 * 4.0;

        for (uint8_t i = 0; i < 2; ++i) {
                wheels_[i].set_Config(&gWheel_Configurations[i]);
                wheels_[i].start_Periphs();
        }
}


static Discrete_PID gSpd_PID[2];
static PID gS_PID[2];

// static float gI_Factor = 3.125;
// static float gP_Factor = 1;

//* Robot's Angle Control parameters
static Angle_PID gAng_PID[2];
static PID gA_PID[2];
/**
 * @brief Function that initializes all the required components for the wheel's
 *        pid controller
 * @retval None
 * 
 * 
 * <pre>
 * Tasks performed by this function
 * 1) Assigns gains for PID controller of each wheel
 * 2) Assigns separate PID controllers for each wheel
 * </pre>
 */
void Actuator::pid_Init()
{
        gSpd_PID[0].set_PID(5.5, 79.345, 0.0);
        gSpd_PID[0].set_Limits(12, -12);
        gSpd_PID[1].set_PID(0.523, 12.25, 0.0);
        gSpd_PID[1].set_Limits(24, -24);

        gAng_PID[0].set_PID(0.05, 0, 0);
        gAng_PID[0].set_Limits(1, -1);
        gAng_PID[1].set_PID(0.1, 0, 0);
        gAng_PID[1].set_Limits(2, -2);

        for (uint8_t i = 0; i < 2; ++i) {
                gS_PID[i].set_Algorithm(&gSpd_PID[i]);
                gA_PID[i].set_Algorithm(&gAng_PID[i]);

                wheels_[i].set_PIDController(&gS_PID[i]);
                wheels_[i].set_AnglePIDController(&gA_PID[i]);
        }
}

// *** EOF ***
