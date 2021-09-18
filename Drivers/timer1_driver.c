/*
 * timer1_driver.c
 *
 * Created: 3/8/2021 11:15:15 AM
 *  Author: Andrew
 */ 

#include <avr/io.h>
#include "timer1_driver.h"

void configure_timer1(){
	ICR1 = 20000; // 20,000,000/1,000
	TCCR1A = 0x00;
	TCCR1B = 0x19;
}

void wait_for_next_timer_period(){
	while((TIFR1 & (ICF1)) == 0);
	TIFR1 |= (1<<ICF1);
}
