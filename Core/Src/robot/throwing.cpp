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
        
        return sRobo_Instance;
}

int Throwing::init(uint32_t dt_millis)
{
        int status = 0;

        initiated_ = true;

        return status;
}

// This function is called by the ThrowingThread
void Throwing::update(uint32_t dt_millis)
{
        
}

// This function is called by the ActuationThread
void Throwing::run(uint32_t dt_millis)
{
        
}

bool Throwing::is_Initiated() const
{
        return initiated_;
}