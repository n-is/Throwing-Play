/*
 * wheel.h
 *
 * Created : 10/1/2018
 */

#ifndef _WHEEL_H_
#define _WHEEL_H_

#include "stm32f4xx_hal.h"
#include "tim.h"

#include "defines.h"

#include "pid_algorithms.h"
#include "pid.h"

enum Direction
{
        AHEAD,
        BACK,
        HALT
};

struct Wheel_Config
{
        uint8_t id;
        float radius;

        GPIO_TypeDef *in1_port;
        uint16_t in1_pin;
        GPIO_TypeDef *in2_port;
        uint16_t in2_pin;

        TIM_HandleTypeDef *htim;        // PWM Generating Timer
        uint32_t channel;               // PWM Channel

        TIM_HandleTypeDef *henc;
        
        float tolerance;                // Tolerance level for zero
        float max_omega;                // Max Omega Attained by the wheel(motor)
        uint32_t enc_ppr;

        PID *speed_pid;                 // Pointer to the wheel's PID controller
        PID *angle_pid;
};

class Wheel
{
public:
        Wheel() { wheel_ = 0; }
        Wheel(Wheel_Config *wheel) { wheel_ = wheel; }
        Wheel(Wheel &&) = default;
        Wheel(const Wheel &) = default;
        Wheel &operator=(Wheel &&) = default;
        Wheel &operator=(const Wheel &) = default;
        ~Wheel() { }
        
        void set_Config(Wheel_Config *wheel) { wheel_ = wheel; }
        void set_Direction(enum Direction d) { dir_ = d; }
        void set_Omega(float omega);
        void update() const;

        float get_Omega(uint32_t dt_millis);
        float get_MaxOmega() { return wheel_->max_omega; }

        float get_Radius() { return wheel_->radius; }

        void start_Periphs();

        void set_PIDController(PID *pid) { wheel_->speed_pid = pid; }
        PID * get_PIDController() {
                return wheel_->speed_pid;
        }
        void set_AnglePIDController(PID *pid) { wheel_->angle_pid = pid; }
        PID * get_AnglePIDController() {
                return wheel_->angle_pid;
        }

        inline void set_Angle(float a) { angle_ = a; }
        void goto_Angle(float theta);
        float get_Angle();

        void update_DeltaCount();
private:
       Wheel_Config *wheel_;
       enum Direction dir_;
       float omega_;    // This value should always be positive
       float angle_;    // Angle in degrees
       int16_t delta_count_;
};


#endif // _WHEEL_H_
