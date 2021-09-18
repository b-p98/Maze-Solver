/*
 * left_motor.c
 *
 * Created: 3/11/2021 6:22:17 PM
 *  Author: Andrew
 */ 

#include <avr/io.h>
#include "left_motor.h"

void stop_timer0(){
	TCCR0B=0X00;	
}

void start_timer0(){
	TCCR0B=0X02;
}

void set_left_motor_duty_cycle(uint8_t duty_cycle){
	OCR0A = duty_cycle;
	OCR0B = duty_cycle;
}

void configure_left_motor(){
	brake_left_motor();
	set_left_motor_duty_cycle(0);
	DDRD |= ((1<<6)|(1<<5));
}

void brake_left_motor(){
	stop_timer0();
	PORTD |= (1<<5);
	PORTD |= (1<<6);
}

void coast_left_motor(){
	stop_timer0();
	PORTD &= ~(1<<5);
	PORTD &= ~(1<<6);
}

void forward_left_motor(){
	brake_left_motor();
	TCCR0A = 0X33;
	start_timer0();
}



void reverse_left_motor(){
	brake_left_motor();
	TCCR0A=0XC3;
	start_timer0();
}
