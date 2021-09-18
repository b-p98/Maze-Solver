/*
 * right_motor.c
 *
 * Created: 3/11/2021 6:22:43 PM
 *  Author: Andrew
 */ 

#include <avr/io.h>
#include "right_motor.h"

void stop_timer2(){
	TCCR2B=0X00;
}

void start_timer2(){
	TCCR2B=0X02;
}

void set_right_motor_duty_cycle(uint8_t duty_cycle){
	OCR2A = duty_cycle;
	OCR2B = duty_cycle;
}

void configure_right_motor(){
	brake_right_motor();
	set_right_motor_duty_cycle(0);
	DDRB |= (1<<3);
	DDRD |= (1<<3);
}

void brake_right_motor(){
	stop_timer2();
	PORTB |= (1<<3);
	PORTD |= (1<<3);
}

void coast_right_motor(){
	stop_timer2();
	PORTB &= ~(1<<3);
	PORTD &= ~(1<<3);
}

void forward_right_motor(){
	brake_right_motor();
	TCCR2A = 0X33;
	start_timer2();
}

void reverse_right_motor(){
	brake_right_motor();
	TCCR2A=0XC3;
	start_timer2();
}