#ifndef _PID_H_
#define _PID_H_

#include "pid_algorithms.h"

class PID
{
public:
        PID() { }
        PID(PID_Algorithm *algo) { set_Algorithm(algo); }
        PID(PID &&) = default;
        PID(const PID &) = default;
        PID &operator=(PID &&) = default;
        PID &operator=(const PID &) = default;
        ~PID() { }

        void set_Algorithm(PID_Algorithm *algo) { algo_ = algo; }
        PID_Algorithm * get_Algorithm() { return algo_; }

        float compute_PID(float err, uint32_t dt_millis) {
                return algo_->compute(err, dt_millis);
        }

private:
        PID_Algorithm *algo_;
};

#endif // !_PID_H_
