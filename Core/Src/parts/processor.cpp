/*
 * processor.cpp
 *
 * Created : 6/2/2019
 *  Author : n-is
 *   email : 073bex422.nischal@pcampus.edu.np
 */

#include "processor.h"
#include "debouncer.h"
#include "defines.h"
#include "commander.h"

#include "main.h"

Processor& Processor::get_Instance()
{
        static Processor sRobo_CPU;
        
        return sRobo_CPU;
}

int Processor::init(uint32_t dt_millis)
{

        return 0;
}

Actuation_Packet Processor::process(uint8_t cmd, uint32_t dt_millis)
{
        Actuation_Packet pack = last_pack_;

        if (cmd == Throw_Commands::GRIP) {
                process_Grip(pack);
        }
        else if (cmd == Throw_Commands::EXTEND) {
                process_Extend(pack);
        }
        else if (cmd == Throw_Commands::PASS_GEREGE) {
                process_Gerege(pack);
        }
        else if (cmd == Throw_Commands::THROW) {
                process_Shoot(pack);
        }
        else if (cmd == Throw_Commands::ACTUATE) {
                process_Arm(pack);
        }
        else if (cmd == Throw_Commands::MOVE_PLATFORM_LEFT ||
                 cmd == Throw_Commands::MOVE_PLATFORM_RIGHT) {

                process_Platform(pack, cmd);
        }

        last_pack_ = pack;

        return pack;
}

enum class Mechanism_State
{
        HOME,
        RELEASE,
        LEFT,
        RIGHT
};

static Debouncer gArm_Debouncer(HAL_GetTick, 1000);
static Mechanism_State gArm_State = Mechanism_State::HOME;
void Processor::process_Arm(Actuation_Packet &pack)
{
        Mechanism_State arm_state = gArm_State;
        if (gArm_Debouncer.is_Ready()) {
                // HAL_GPIO_TogglePin(B_BlueLED_GPIO_Port, B_BlueLED_Pin);
                if (gArm_State == Mechanism_State::HOME) {
                        arm_state = Mechanism_State::RELEASE;
                }
                else if (gArm_State == Mechanism_State::RELEASE) {
                        arm_state = Mechanism_State::HOME;
                }
                gArm_Debouncer.hold();
        }

        if (arm_state != gArm_State) {
                gArm_State = arm_state;
                if (gArm_State == Mechanism_State::HOME) {
                        pack.arm_angle = ARM_MOTOR_HOME_ANGLE;
                }
                else if (gArm_State == Mechanism_State::RELEASE) {
                        pack.arm_angle = ARM_MOTOR_PICK_ANGLE;
                }
        }
}

static Debouncer gGerege_Debouncer(HAL_GetTick, 1000);
static Mechanism_State gGerege_State = Mechanism_State::HOME;
void Processor::process_Gerege(Actuation_Packet &pack)
{
        Mechanism_State gerege_state = gGerege_State;
        if (gGerege_Debouncer.is_Ready()) {
                if (gGerege_State == Mechanism_State::HOME) {
                        gerege_state = Mechanism_State::RELEASE;
                }
                else if (gGerege_State == Mechanism_State::RELEASE) {
                        gerege_state = Mechanism_State::HOME;
                }
                gGerege_Debouncer.hold();
        }

        if (gerege_state != gGerege_State) {
                gGerege_State = gerege_state;
                if (gGerege_State == Mechanism_State::HOME) {
                        pack.gerege = false;
                }
                else if (gGerege_State == Mechanism_State::RELEASE) {
                        pack.gerege = true;
                }
        }
}

static Debouncer gGrip_Debouncer(HAL_GetTick, 1000);
static Mechanism_State gGrip_State = Mechanism_State::HOME;
void Processor::process_Grip(Actuation_Packet &pack)
{
        Mechanism_State grip_state = gGrip_State;
        if (gGrip_Debouncer.is_Ready()) {
                if (gGrip_State == Mechanism_State::HOME) {
                        grip_state = Mechanism_State::RELEASE;
                }
                else if (gGrip_State == Mechanism_State::RELEASE) {
                        grip_state = Mechanism_State::HOME;
                }
                gGrip_Debouncer.hold();
        }

        if (grip_state != gGrip_State) {
                gGrip_State = grip_state;
                if (gGrip_State == Mechanism_State::HOME) {
                        pack.grip = false;
                }
                else if (gGrip_State == Mechanism_State::RELEASE) {
                        pack.grip = true;
                }
        }
}

static Debouncer gExtend_Debouncer(HAL_GetTick, 1000);
static Mechanism_State gExtend_State = Mechanism_State::HOME;
void Processor::process_Extend(Actuation_Packet &pack)
{
        Mechanism_State extend_state = gExtend_State;
        if (gExtend_Debouncer.is_Ready()) {
                if (gExtend_State == Mechanism_State::HOME) {
                        extend_state = Mechanism_State::RELEASE;
                }
                else if (gExtend_State == Mechanism_State::RELEASE) {
                        extend_state = Mechanism_State::HOME;
                }
                gExtend_Debouncer.hold();
        }

        if (extend_state != gExtend_State) {
                gExtend_State = extend_state;
                if (gExtend_State == Mechanism_State::HOME) {
                        pack.extend = false;
                }
                else if (gExtend_State == Mechanism_State::RELEASE) {
                        pack.extend = true;
                }
        }
}

static Debouncer gShoot_Debouncer(HAL_GetTick, 1000);
static Mechanism_State gShoot_State = Mechanism_State::HOME;
void Processor::process_Shoot(Actuation_Packet &pack)
{
        Mechanism_State shoot_state = gShoot_State;
        if (gShoot_Debouncer.is_Ready()) {
                if (gShoot_State == Mechanism_State::HOME) {
                        shoot_state = Mechanism_State::RELEASE;
                }
                else if (gShoot_State == Mechanism_State::RELEASE) {
                        shoot_state = Mechanism_State::HOME;
                }
                gShoot_Debouncer.hold();
        }

        if (shoot_state != gShoot_State) {
                gShoot_State = shoot_state;
                if (gShoot_State == Mechanism_State::HOME) {
                        pack.shoot = false;
                }
                else if (gShoot_State == Mechanism_State::RELEASE) {
                        pack.shoot = true;
                }
        }
}

static Debouncer gPlatform_Debouncer(HAL_GetTick, 1000);
static Mechanism_State gPlatform_State = Mechanism_State::HOME;
void Processor::process_Platform(Actuation_Packet &pack, uint8_t cmd)
{
        Mechanism_State platform_state = gPlatform_State;
        if (gPlatform_Debouncer.is_Ready()) {
                if (gPlatform_State == Mechanism_State::HOME) {
                        platform_state = Mechanism_State::RELEASE;
                }
                else if (gPlatform_State == Mechanism_State::RELEASE) {
                        platform_state = Mechanism_State::HOME;
                }
                gPlatform_Debouncer.hold();
        }

        if (platform_state != gPlatform_State) {
                gPlatform_State = platform_state;
                if (gPlatform_State == Mechanism_State::HOME) {
                        pack.platform_angle = PLATFORM_MOTOR_HOME_ANGLE;
                }
                else if (gPlatform_State == Mechanism_State::RELEASE) {
                        pack.platform_angle = PLATFORM_MOTOR_RED_ANGLE;
                }
        }
}
