/*
 * throwing.cpp
 *
 * Created : 6/2/2019
 *  Author : n-is
 *   email : 073bex422.nischal@pcampus.edu.np
 */

#include "throwing.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"

// We should make sure that the throwing ony have one instance and it is properly
// instantiated
Throwing& Throwing::get_Instance()
{
        static Throwing sRobo_Instance;
        
        Commander &cmd = Commander::get_Instance();
        sRobo_Instance.general_ = &cmd;

        Processor &cpu = Processor::get_Instance();
        sRobo_Instance.brain_ = &cpu;

        Actuator &sol = Actuator::get_Instance();
        sRobo_Instance.soldier_ = &sol;
        
        return sRobo_Instance;
}

int Throwing::init(uint32_t dt_millis)
{
        int status = 0;

        general_->init();
        Actuation_Packet pack = brain_->init(dt_millis);
        soldier_->init(pack);

        initiated_ = true;

        return status;
}

// This function is called by the ThrowingThread
void Throwing::update(uint32_t dt_millis)
{
        uint8_t cmd = general_->get_Command();
        Actuation_Packet pack = brain_->process(cmd, dt_millis);

        taskENTER_CRITICAL();
        act_pack_ = pack;
        taskEXIT_CRITICAL();
}

// This function is called by the ActuationThread
void Throwing::run(uint32_t dt_millis)
{
        soldier_->actuate(act_pack_, dt_millis);
}

bool Throwing::is_Initiated() const
{
        return initiated_;
}