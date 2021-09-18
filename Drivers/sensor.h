/*
 * sensor.h
 *
 * Created: 3/29/2021 12:06:00 PM
 *  Author: Andrew
 */ 


#ifndef SENSOR_H_
#define SENSOR_H_

#include <avr/io.h>
void configure_sensors();
uint8_t get_line_sensor();


#endif /* SENSOR_H_ */