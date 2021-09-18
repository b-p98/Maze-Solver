/*
 * sensor.c
 *
 * Created: 3/29/2021 12:06:17 PM
 *  Author: Andrew
 */ 

#include <avr/io.h>
#include "sensor.h"
#include <stdint.h>

void configure_sensors(){
	DDRC &= ~(0x3F);
	PORTC |= 0x1F;
}

uint8_t get_line_sensor(){
	return PINC & 0x1F;
}