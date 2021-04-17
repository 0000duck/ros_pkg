#include <stdio.h>
#include <math.h>
#include <vector>
#include <iostream>

#define PI 3.14159265


class Oscillator
{
    public:
        Oscillator(float* t);

        void sample_sine(std::vector<std::vector<float>>& in_vector);

        void set_trans_freq(const float& trans_freq);
        void set_rot_freq(const float& rot_freq);
        void set_surge_A(const float& surge_A);
        void set_sway_A(const float& sway_A);
        void set_heave_A(const float& heave_A);
        void set_roll_A(const float& roll_A);
        void set_pitch_A(const float& pitch_A);
        void set_yaw_A(const float& yaw_A);

    private:
        float* t_ptr;

        float trans_freq = 0.25;
        float rot_freq = 0.3;

        float surge_A = 0.;
        float surge_bias = 0.;
        float surge_offset = 0.;

        float sway_A = 0.;
        float sway_bias = 0.;
        float sway_offset = -PI/2;

        float heave_A = 0.02;
        float heave_bias = 0.04853;
        float heave_offset = 0.;

        float roll_A = 0.;
        float roll_bias = 0.;
        float roll_offset = 0.;

        float pitch_A = 0.;
        float pitch_bias = 0.;
        float pitch_offset = -PI/2;

        float yaw_A = 0.;
        float yaw_bias = 0.;
        float yaw_offset = 0.;
};