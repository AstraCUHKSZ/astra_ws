#ifndef DUAL_MOTOR_H
#define DUAL_MOTOR_H

#include <Arduino.h>

#define JOINT_NUM 2
#define NONE_JOINT_NUM 4

void dualMotorUpdatePos(uint16_t pos[]);

void dualMotorSetup();

void dualMotorReadPos(uint16_t read_pos[]);

void setupTorque(int enable);

void doSetupTorque(int enable);

extern float pidtune_kp, pidtune_ki, pidtune_kd, pidtune_ki_max, pidtune_kp2, pidtune_kp2_err_point, pidtune_ki_clip_thres, pidtune_ki_clip_coef;

extern volatile bool debug_mode;
void debugSetServoPWM(int id, int pwm);
void debugSetServoPosition(int id, int position);
void debugGetServoStatus(int id, int16_t* pos, int16_t* speed, int16_t* load, uint8_t* voltage, uint8_t* temper);

#endif