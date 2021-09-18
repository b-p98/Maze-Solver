/*
 * right_motor.h
 *
 * Created: 3/11/2021 6:22:32 PM
 *  Author: Andrew
 */ 


#ifndef RIGHT_MOTOR_H_
#define RIGHT_MOTOR_H_


#include <avr/io.h>

void set_right_motor_duty_cycle(uint8_t duty_cycle);
void configure_right_motor();
void brake_right_motor();
void coast_right_motor();
void forward_right_motor();
void reverse_right_motor();



#endif /* RIGHT_MOTOR_H_ */