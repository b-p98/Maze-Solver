/*
 * left_motor.h
 *
 * Created: 3/11/2021 6:21:55 PM
 *  Author: Andrew
 */ 


#ifndef LEFT_MOTOR_H_
#define LEFT_MOTOR_H_

#include <avr/io.h>

void set_left_motor_duty_cycle(uint8_t duty_cycle);
void configure_left_motor();
void brake_left_motor();
void coast_left_motor();
void forward_left_motor();
void reverse_left_motor();



#endif /* LEFT_MOTOR_H_ */