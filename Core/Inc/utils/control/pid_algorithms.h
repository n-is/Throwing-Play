#ifndef _PID_ALGORITHMS_H_
#define _PID_ALGORITHMS_H_

class PID_Algorithm
{
      public:
        PID_Algorithm() { set_PID(0, 0, 0); }
        PID_Algorithm(float p, float i, float d) { set_PID(p, i, d); }
        virtual float compute(float error, uint32_t dt_millis) = 0;

        void set_P(float p) { p_ = p; }
        void set_I(float i) { i_ = i; }
        void set_D(float d) { d_ = d; }
        void set_PID(float p, float i, float d)
        {
                set_P(p);
                set_I(i);
                set_D(d);
        }
        float get_P() { return p_; }
        float get_I() { return i_; }
        float get_D() { return d_; }

        void set_Limits(float max_out, float min_out)
        {
                max_ = max_out;
                min_ = min_out;
        }

        float get_Upper() { return max_; }
        float get_Lower() { return min_; }

        virtual ~PID_Algorithm() {}

      protected:
        float p_, i_, d_;
        float max_, min_;
};

class Discrete_PID : public PID_Algorithm
{
      public:
        Discrete_PID(float p, float i, float d) : PID_Algorithm(p, i, d)
        {
                l_output_ = 0;
                l_err_ = 0;
                ll_err_ = 0;
        }
        Discrete_PID()
        {
                l_output_ = 0;
                l_err_ = 0;
                ll_err_ = 0;
        }
        Discrete_PID(Discrete_PID &&) = default;
        Discrete_PID(const Discrete_PID &) = default;
        Discrete_PID &operator=(Discrete_PID &&) = default;
        Discrete_PID &operator=(const Discrete_PID &) = default;
        ~Discrete_PID() {}

        /* *** PID Algorithm Description ***
         * 1) Discrete PID control Algorithm
         * 2) Integrator Method : Forward Euler
         * 3) //! Filtered Derivative not used
         * 4) Output Limited
         * 5) Form : Parallel
         * 6) Compensator Formula : Dz = P + I*Ts/(z-1) + D*(z-1)/(Ts*z)
         * 7) In Time Domain :
         *      y(t) - y(t-1) = a*x(t) + b*x(t-1) + c*x(t-2)
         *      where,
         *              a = P + D/Ts
         *              b = -P + I*Ts - 2*D/Ts
         *              c = D/Ts
         */
        float compute(float error, uint32_t dt_millis)
        {
                float Ts = (float)dt_millis / 1000.0;

                float P = get_P();
                float I = get_I();
                float D_by_Ts = get_D() / Ts;

                float a = P + D_by_Ts;
                float b = -P + I * Ts - 2 * D_by_Ts;
                float c = D_by_Ts;

                l_output_ += a * error + b * l_err_ + c * ll_err_;

                if (l_output_ > get_Upper())
                {
                        l_output_ = get_Upper();
                }
                else if (l_output_ < get_Lower())
                {
                        l_output_ = get_Lower();
                }

                ll_err_ = l_err_;
                l_err_ = error;

                return l_output_;
        }

      private:
        float l_output_;
        float l_err_;
        float ll_err_;
};

class FilteredDiscrete_PID : public PID_Algorithm
{
      public:
        FilteredDiscrete_PID(float p, float i, float d, float N = 0) : PID_Algorithm(p, i, d)
        {
                l_output_ = 0;
                l_err_ = 0;
                ll_err_ = 0;
                set_N(N);
        }
        FilteredDiscrete_PID()
        {
                l_output_ = 0;
                l_err_ = 0;
                ll_err_ = 0;
                set_N(0);
        }
        FilteredDiscrete_PID(FilteredDiscrete_PID &&) = default;
        FilteredDiscrete_PID(const FilteredDiscrete_PID &) = default;
        FilteredDiscrete_PID &operator=(FilteredDiscrete_PID &&) = default;
        FilteredDiscrete_PID &operator=(const FilteredDiscrete_PID &) = default;
        ~FilteredDiscrete_PID() {}

        void set_N(float N) { N_ = N; }

        /* *** PID Algorithm Description ***
         * 1) Filtered Discrete PID control Algorithm
         * 2) Integrator Method : Forward Euler
         * 3) Filtered Derivative used
         * 4) Output Limited
         * 5) Form : Parallel
         * 6) Compensator Formula : Dz = P + I*Ts/(z-1) + D*N*(z-1)/{(z -1) + N*Ts}
         * 7) In Time Domain :
         *      y(t) = (1 + a)*y(t-1) - a*y(t-2) + alpha*x(t) + beta*x(t-1) + gamma*x(t-2)
         *      where,
         *              a = 1 - N_*Ts
         *              alpha = P + D*N
         *              beta = -(1 + a)*P + I*Ts - 2*D*N
         *              gamma = a*P - a*I*Ts + D*N
         */
        float compute(float error, uint32_t dt_millis)
        {
                float Ts = (float)dt_millis / 1000.0;

                float a, alpha, beta, gamma;
                float output;

                float P = get_P();
                float I = get_I();
                float D = get_D();

                a = 1 - N_ * Ts;
                alpha = P + D * N_;
                beta = -(1 + a) * P + I * Ts - 2 * D * N_;
                gamma = a * P - a * I * Ts + D * N_;

                output = (1 + a) * l_output_ - a * ll_output_ + alpha * error + beta * l_err_ + gamma * ll_err_;

                ll_output_ = l_output_;
                l_output_ = output;
                ll_err_ = l_err_;
                l_err_ = error;

                if (l_output_ > get_Upper())
                {
                        l_output_ = get_Upper();
                }
                else if (l_output_ < get_Lower())
                {
                        l_output_ = get_Lower();
                }
                return l_output_;
        }

      private:
        float N_;

        float l_output_;
        float ll_output_;
        float l_err_;
        float ll_err_;
};

class Angle_PID : public PID_Algorithm
{
      public:
        Angle_PID(float p, float i, float d) : PID_Algorithm(p, i, d)
        {
                l_output_ = 0;
                err_sum_ = 0;
        }

        Angle_PID()
        {
                l_output_ = 0;
                err_sum_ = 0;
        }

        Angle_PID(Angle_PID &&) = default;
        Angle_PID(const Angle_PID &) = default;
        Angle_PID &operator=(Angle_PID &&) = default;
        Angle_PID &operator=(const Angle_PID &) = default;
        ~Angle_PID() { }

        /* *** PID Algorithm Description ***
                * 1) Angle PID control Algorithm
                * 2) Use of Proportional Controller Only 
                * 3) Output Limited
                * 4) Compensator formula in Time Domain: 
                *              y(t) = P * {(x(t) - x(t-1))/Ts}
                */

        float compute(float error, uint32_t dt_millis)
        {
                // float Ts = (float)dt_millis / 1000.0;

                float P = get_P();
                float I = get_I();

                err_sum_ += error;
                float i_term = I * err_sum_;
                // Clamping Integral Term
                if (i_term > get_Upper()) {
                        i_term = get_Upper();
                        err_sum_ = 0;
                }
                else if (i_term < get_Lower()) {
                        i_term = get_Lower();
                        err_sum_ = 0;
                }

                l_output_ = P * error + i_term;

                if (l_output_ > get_Upper())
                {
                        l_output_ = get_Upper();
                }

                else if (l_output_ < get_Lower())
                {
                        l_output_ = get_Lower();
                }

                return l_output_;
        }

      private:
        float l_output_;
        float err_sum_;
};

#endif // !_PID_ALGORITHMS_H_
