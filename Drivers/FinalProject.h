/*
 * FinalProject.h
 *
 * Created: 5/19/2021 2:43:38 PM
 *  Author: Ben Picone and Andrew Benner
 */ 


#ifndef FINALPROJECT_H_
#define FINALPROJECT_H_



// standard libraries
#include <stdint.h>
#include <stdbool.h>
#include <stdfix.h>


// sensor declarations
#define min_duty_cycle 0x00
#define max_duty_cycle 0x64
enum sensor_state {LINE_FOUND,LINE_LOST, LEFT_INTERSECTION,RIGHT_INTERSECTION,T_INTERSECTION,MINE_FOUND};
enum sensor_state read_line_sensor(short fract * position);
void steer_robot_forward(uint8_t speed, short fract steer);
void move_robot_forward();
short fract line_pos;


// turning variables
_Bool turning = false;
_Bool creeping_up = false;
_Bool justMine = false;
_Bool backing_up = false;


uint16_t backUpCounter = 0;
uint16_t mineCounter = 0;


// back_tracking flags
_Bool backTrackFlag = false;
_Bool first_backTracking_turn = true;

// movement declarations
typedef struct {
	
	int8_t left_duty_cycle;
	int8_t right_duty_cycle;
	uint16_t time;
	uint8_t type_of_turn;
	
} movement_t;

#define RIGHT_TURN 0
#define LEFT_TURN 1
#define TURN_AROUND 2
#define FORW 3
#define REVERSE 4
// these values work well enough
movement_t left_turn  = {0x30,0x30,350,LEFT_TURN};
movement_t right_turn  = {0x30,0x30,350,RIGHT_TURN};
movement_t fullSpin = {0x30,0x30,675,TURN_AROUND};
movement_t creep_up = {0x18,0x18,300,FORW};
movement_t pass_through = {0x18,0x18,350,FORW};
movement_t backUp  = {0x20,0x20,600,REVERSE};

_Bool move_robot( movement_t new_move , _Bool start_movement );

typedef struct{
	
	uint8_t x;
	uint8_t y;
	
} coordinates;

coordinates currentCoordinates = {0,0};
coordinates stack[71];
uint8_t stackPointer = 0;
void pushStack();
void popStack();
_Bool back_track = false;


struct visitedLinks{
	
	_Bool north:1,
	south: 1,
	east:1 ,
	west:1;
	
};
typedef struct visitedLinks VisitedLinks;
VisitedLinks grid[6][7];

enum robotDirection {north,south,east,west}
robotDirection = north;
enum robotDirection updateDirection(uint8_t currentTurn);

void updatePosition();
#define LEFT 0
#define RIGHT 1
#define FORWARD 3
#define STOP_ROBOT 4
#define FULLTURN 5
#define BACKTRACK 6
uint8_t decideDirection();

void initializeGrid();
void mineFound();
uint8_t calculate_direction_of_previous_coordinates();


#endif /* FINALPROJECT_H_ */