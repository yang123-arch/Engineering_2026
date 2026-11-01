#ifndef __Function__
#define __Function__

#include "main.h"

void Limit_float(float *Input, float Up, float Down);
void Limit_int(int *Input, int Up, int Down);

void Ignor_float(float *Input,  float Up, float Down);

void Deadline_Float(float *Input,  float Up, float Down);

void Deadline_Zero_Float(float *Input,  float DeadLine);

void Acc_Slow(float Target, float *Output, float Acc, float Dt);

float Caculate_Distence(float X, float Y);

#endif
