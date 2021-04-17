#include "oscillator.h"

Oscillator::Oscillator(float* t)
{
    this->t_ptr = t;
}

void Oscillator::sample_sine(std::vector<std::vector<float>>& dof)
{
    static const uint8_t POS = 0;
    static const uint8_t VEL = 1;
    
    dof[POS][0] = surge_A * sin(2.*PI * trans_freq * *t_ptr + surge_offset) + surge_bias;
    dof[POS][1] = sway_A * sin(2.*PI * trans_freq * *t_ptr + sway_offset) + sway_bias;
    dof[POS][2] = heave_A * sin(2.*PI * trans_freq * *t_ptr + heave_offset) + heave_bias;
    
    dof[POS][3] = roll_A * (PI/180.) * sin(2.*PI * rot_freq * *t_ptr + roll_offset) + roll_bias;
    dof[POS][4] = pitch_A * (PI/180.) * sin(2.*PI * rot_freq * *t_ptr + pitch_offset) + pitch_bias;
    dof[POS][5] = yaw_A * (PI/180.) * sin(2.*PI * rot_freq * *t_ptr + yaw_offset) + yaw_bias;

    
    dof[VEL][0] = surge_A * 2.*PI * trans_freq * cos(2*PI * trans_freq * *t_ptr + surge_offset);
    dof[VEL][1] = sway_A * 2.*PI * trans_freq * cos(2*PI * trans_freq * *t_ptr + sway_offset);
    dof[VEL][2] = heave_A * 2.*PI * trans_freq * cos(2*PI * trans_freq * *t_ptr + heave_offset);
    
    dof[VEL][3] = roll_A * 2.*PI * rot_freq * cos(2*PI * rot_freq * *t_ptr + roll_offset);
    dof[VEL][4] = pitch_A * 2.*PI * rot_freq * cos(2*PI * rot_freq * *t_ptr + pitch_offset);
    dof[VEL][5] = yaw_A * 2.*PI * rot_freq * cos(2*PI * rot_freq * *t_ptr + yaw_offset);
    
}

void Oscillator::set_trans_freq(const float& trans_freq) { this->trans_freq = trans_freq; }

void Oscillator::set_rot_freq(const float& rot_freq) { this->rot_freq = rot_freq; }

void Oscillator::set_surge_A(const float& surge_A) { this->surge_A = surge_A; }

void Oscillator::set_sway_A(const float& sway_A) { this->sway_A = sway_A; }

void Oscillator::set_heave_A(const float& heave_A) { this->heave_A = heave_A; }

void Oscillator::set_roll_A(const float& roll_A) { this->roll_A = roll_A; }

void Oscillator::set_pitch_A(const float& pitch_A) { this->pitch_A = pitch_A; }

void Oscillator::set_yaw_A(const float& yaw_A) { this->yaw_A = yaw_A; }
