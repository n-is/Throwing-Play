#ifndef _DEFINES_H_
#define _DEFINES_H_


#define MOTOR_DRIVER_FREQUENCY  (8000)
#define PWM_TIMER_FREQUENCY     (168000000)


#define MAX_OMEGA       70    // w = 2*pi*f

#define MAX_POSSIBLE_OMEGA	(70)    // Maximum value that may be computed

#define START_BYTE      (0xA5)

//* Motor Default Configs
#define ARM_MOTOR_PICK_ANGLE    (-170)
#define ARM_MOTOR_HOME_ANGLE    (0)
#define ARM_MOTOR_ERROR_ANGLE   (-10)

#define PLATFORM_MOTOR_RED_ANGLE        (20)
#define PLATFORM_MOTOR_BLUE_ANGLE       (-20)
#define PLATFORM_MOTOR_HOME_ANGLE       (0)

#endif  // _DEFINES_H_
