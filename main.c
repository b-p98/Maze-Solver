/* By: Ben Picone and Andrew Benner */
#include "FinalProject.h"
#include "timer1_driver.h"
#include "sensor.h"
#include "left_motor.h"
#include "right_motor.h"


int main(void)
{
		
		configure_timer1();
		configure_sensors();
		configure_left_motor();
		configure_right_motor();
		
		// state variables
		static enum {TURNING_LEFT,TURNING_RIGHT, FULL_TURN, BEGIN, MOVE_FORWARD,ADJUST_FOR_RIGHT_TURN, ADJUST_FOR_LEFT_TURN, ADJUST_FOR_FULL_TURN, END,DECISION,PASS_THROUGH_INTERSECTION,MINE_FOUND,BACK_UP,BACK_TRACK}
		state = BEGIN;
		
		
		initializeGrid();
		
		
		while (1)
		{
			
			
			wait_for_next_timer_period();
			
			enum sensor_state robot_state = read_line_sensor(&line_pos);
			
			// switches based on robots current state - in each state a decision is made based on robot's current light sensor reading and position on grid
			switch(state)
			{
				case BEGIN:
				
					robotDirection = north;
					stack[stackPointer] = currentCoordinates;
				
					steer_robot_forward(0,0);
				
					if (robot_state == LINE_FOUND){
						state = MOVE_FORWARD;
					}
					
					
					break;
				
				
				case MOVE_FORWARD:
				
					move_robot_forward();
					steer_robot_forward(0x27,(.35*line_pos));

					// robot is currently at intersection - doesn't matter what type
					if(robot_state== T_INTERSECTION || robot_state == LEFT_INTERSECTION || robot_state == RIGHT_INTERSECTION){

						if(!backTrackFlag){
							updatePosition();
							pushStack();
						}
						
						state = DECISION;

					}
				
					else if(robot_state == LINE_FOUND){
					
						state = MOVE_FORWARD;
					}
					
					else if( robot_state == LINE_LOST){
						
						state = MINE_FOUND;
					}

				break;
				
				case DECISION:
				
				
					currentCoordinates.x = stack[stackPointer].x;
					currentCoordinates.y = stack[stackPointer].y;
					
					if(decideDirection() == FORWARD ){
						
						
							first_backTracking_turn = true;
							backTrackFlag = false;
						
							state = PASS_THROUGH_INTERSECTION;
						
						}
					
						else if(decideDirection() == RIGHT){
						
							first_backTracking_turn = true;
							backTrackFlag = false;
						
							state = ADJUST_FOR_RIGHT_TURN;
							robotDirection = updateDirection(RIGHT);
						
						}
					
						else if(decideDirection() == LEFT){
						
			
							first_backTracking_turn = true;
							backTrackFlag = false;
						
							state = ADJUST_FOR_LEFT_TURN;
							robotDirection = updateDirection(LEFT);
						
						}
					
						else if(decideDirection() == FULLTURN){
						
						
							first_backTracking_turn = true;
							backTrackFlag = false;
						
							state = ADJUST_FOR_FULL_TURN;
							robotDirection = updateDirection(FULLTURN);
						
						}
					
						else if(decideDirection() == BACKTRACK){
						
							backTrackFlag = true;
							state = BACK_TRACK;
						
						}


				
				break;
				
				case BACK_TRACK:
				
				
					if(calculate_direction_of_previous_coordinates() == FULLTURN ){
						
						state = ADJUST_FOR_FULL_TURN;
						robotDirection = updateDirection(FULLTURN);
						
					}
					
					else if( calculate_direction_of_previous_coordinates() == FORWARD ){
						
						state = PASS_THROUGH_INTERSECTION;
						robotDirection = robotDirection;
						
					}
					
					if(calculate_direction_of_previous_coordinates() == LEFT ){
						
						state = ADJUST_FOR_LEFT_TURN;
						robotDirection = updateDirection(LEFT);
						
					}
					
					else if( calculate_direction_of_previous_coordinates() == RIGHT ){
						
						state = ADJUST_FOR_RIGHT_TURN;
						robotDirection = updateDirection(RIGHT);
						
					}
				
					popStack();
				
				break;
				
				case MINE_FOUND:
			
			
					mineFound();
					state = BACK_UP;
					
				
				break;
				
				case BACK_UP:
				
					backing_up = move_robot(backUp,REVERSE);
					
					if(!backing_up){
						
						state = DECISION;
						
					}
				
				
				break;
				
				case PASS_THROUGH_INTERSECTION:
				
					creeping_up = move_robot(pass_through,true);
					if(!creeping_up){
						
						state = MOVE_FORWARD;
						
					}
				
				break;
					
				
				case ADJUST_FOR_RIGHT_TURN:
				
					creeping_up = move_robot(creep_up,true);
					
					if(!creeping_up){
						
						state = TURNING_RIGHT;
						
					}
				
				break;
			
				
				case ADJUST_FOR_LEFT_TURN:
				
					creeping_up = move_robot(creep_up,true);
					
					if(!creeping_up){
						
						state = TURNING_LEFT;
						
					}
				
				break;
				
				
				case ADJUST_FOR_FULL_TURN:
				
					creeping_up = move_robot(creep_up,true);
					
					if(!creeping_up){
						
						state = FULL_TURN;
						
					}
				
				break;
				
				case FULL_TURN:
			
					turning = move_robot(fullSpin, true);
					
					if (!turning){
						
						state = MOVE_FORWARD;
						
					}
					
					
			
				break;
			
				
			case TURNING_LEFT:
			
				turning = move_robot(left_turn, true);
				if (!turning){

					state = MOVE_FORWARD;
						
				}

			break;
				
				
			case TURNING_RIGHT:
				
				turning = move_robot(right_turn, true);
					
				if (!turning){
						
					state = MOVE_FORWARD;
						
				}

			break;
				
			case END:
				
				steer_robot_forward(0,0);
				
				
			break;
					
				
				
			}

		}

}



uint8_t calculate_direction_of_previous_coordinates(){
	
	uint8_t decision = FORWARD;

	if(robotDirection == north){
		
		if(stack[stackPointer].y - stack[stackPointer - 1].y == 1 && first_backTracking_turn ){
			
			first_backTracking_turn = false;
			decision = FULLTURN;

}

		if(stack[stackPointer].y - stack[stackPointer - 1].y == -1 && !(first_backTracking_turn) ){
			
			first_backTracking_turn = false;
			decision = FORWARD;

		}
		
		else if(stack[stackPointer].x - stack[stackPointer - 1].x == 1 ){
			
			first_backTracking_turn = false;
			decision = LEFT;
			
		}
		
		else if(stack[stackPointer].x - stack[stackPointer - 1].x == -1 ){
			
			first_backTracking_turn = false;
			decision = RIGHT;
			
		}
		
	}
	
	else if(robotDirection == south){
		
		
		if(stack[stackPointer].y - stack[stackPointer - 1].y == -1 && first_backTracking_turn  ){
			
			first_backTracking_turn = false;
			decision = FULLTURN;


		}
		
		if(stack[stackPointer].y - stack[stackPointer - 1].y == 1 && !(first_backTracking_turn) ){
			
			first_backTracking_turn = false;
			decision = FORWARD;


		}
		
		
		else if(stack[stackPointer].x - stack[stackPointer - 1].x == 1 ){
			
			first_backTracking_turn = false;
			decision = RIGHT;
			
		}
		
		else if(stack[stackPointer].x - stack[stackPointer - 1].x == -1 ){
			
			first_backTracking_turn = false;
			decision = LEFT;
			
		}
		
		
	}
	
	else if(robotDirection == east){
	
		
		if(stack[stackPointer].x - stack[stackPointer - 1].x == 1  && first_backTracking_turn){
			
			first_backTracking_turn = false;
			decision = FULLTURN;

		}
		
		
		else if(stack[stackPointer].x - stack[stackPointer - 1].x == -1  && !(first_backTracking_turn) ){
			
			first_backTracking_turn =false;
			decision = FORWARD;

		}
		
		else if(stack[stackPointer].y - stack[stackPointer - 1].y == 1 ){
			
			first_backTracking_turn = false;
			decision = RIGHT;
			
		}
		
		else if(stack[stackPointer].y - stack[stackPointer - 1].y == -1 ){
			
			first_backTracking_turn = false;
			decision = LEFT;
			
		}
		
		
	}
	
	else if(robotDirection == west){
		
		if(stack[stackPointer].x - stack[stackPointer - 1].x == -1 && first_backTracking_turn ){
			
			first_backTracking_turn = false;
			decision = FULLTURN;
			

		}
		
		else if(stack[stackPointer].x - stack[stackPointer - 1].x == 1 && !(first_backTracking_turn) ){
			
			first_backTracking_turn = false;
			decision = FORWARD;
			

		}
		
		
		else if(stack[stackPointer].y - stack[stackPointer - 1].y == 1 ){
			
			first_backTracking_turn = false;
			decision = LEFT;
			
		}
		
		else if(stack[stackPointer].y - stack[stackPointer - 1].y == -1 ){
			
			
			first_backTracking_turn = false;
			decision = RIGHT;
			
		}
		
		
	}
	
	return decision;
	
	
}

uint8_t decideDirection(){
	
	uint8_t decision = FORWARD;

	if(robotDirection == north){
	
		if(grid [currentCoordinates.x][currentCoordinates.y].north == false){
			
			decision = FORWARD;

		}
	
		else if(grid[currentCoordinates.x] [currentCoordinates.y].east == false){
			
			decision = RIGHT;
		
		}
	
		else if(grid[currentCoordinates.x] [currentCoordinates.y].west == false){
			
			decision = LEFT;
			
		}
		
		else if(grid[currentCoordinates.x] [currentCoordinates.y].south == false){
			
			decision = FULLTURN;
			
		}
		
		else{
			
			decision = BACKTRACK;
			
		}

	}
	
	else if(robotDirection == south){
		
		if(grid [currentCoordinates.x][currentCoordinates.y].south == false){
			
			decision = FORWARD;
		}
		
		else if(grid[currentCoordinates.x] [currentCoordinates.y].west == false){
			
			decision = RIGHT;
			
		}
		
		else if(grid[currentCoordinates.x] [currentCoordinates.y].east == false){
			
			decision = LEFT;
			
		}
		
		else if(grid[currentCoordinates.x] [currentCoordinates.y].north == false){
			
			decision = FULLTURN;
			
		}
		
		else{
			
			decision = BACKTRACK;
			
		}
		
	}
	
	else if(robotDirection == east){
		
		if(grid [currentCoordinates.x][currentCoordinates.y].east == false){
			
			decision = FORWARD;
		}
		
		
		else if(grid[currentCoordinates.x] [currentCoordinates.y].south == false){
			
			decision = RIGHT;
			
		}
		
		else if(grid[currentCoordinates.x] [currentCoordinates.y].north == false){
			
			decision = LEFT;
			
		}
		
		else if(grid[currentCoordinates.x] [currentCoordinates.y].west == false){
			
			decision = FULLTURN;
			
		}

		else{
			
			decision = BACKTRACK;
			
		}
		
		
	}
	
	else if(robotDirection == west){
		
		if(grid [currentCoordinates.x][currentCoordinates.y].west == false){
			
			decision = FORWARD;
		}
		
	
		else if(grid[currentCoordinates.x] [currentCoordinates.y].north == false){
			
			decision = RIGHT;
			
		}
		
		else if(grid[currentCoordinates.x] [currentCoordinates.y].south == false){
			
			decision = LEFT;
			
		}
		
		else if(grid[currentCoordinates.x] [currentCoordinates.y].east == false){
			
			decision = FULLTURN;
			
		}
		
		else{
			
			decision = BACKTRACK;
			
		}
		
	}
	
	return decision;
	

}

void mineFound(){
	
	if(robotDirection == north){
		grid[currentCoordinates.x] [currentCoordinates.y].north = true;
		grid[currentCoordinates.x] [(currentCoordinates.y+1)].south = true;
}


else if(robotDirection ==south){
		
		grid[currentCoordinates.x] [currentCoordinates.y].south = true;
		grid[currentCoordinates.x] [(currentCoordinates.y-1)].north = true;
		
	}
	
	else if(robotDirection == east){
		
		grid[currentCoordinates.x] [currentCoordinates.y].east = true;
		grid[(currentCoordinates.x+1)] [currentCoordinates.y].west = true;
		
	}
	
	else if(robotDirection == west){
		
		grid[currentCoordinates.x] [currentCoordinates.y].west = true;
		grid[(currentCoordinates.x-1)] [currentCoordinates.y].east = true;
		
	}

}

void updatePosition(){
	
	
	if(robotDirection == north){
		grid[currentCoordinates.x] [currentCoordinates.y].north = true;
		++currentCoordinates.y;
		grid[currentCoordinates.x] [currentCoordinates.y].south = true;
	}
	
	
	else if(robotDirection == south){
		
		grid[currentCoordinates.x] [currentCoordinates.y].south = true;
		--currentCoordinates.y;
		grid[currentCoordinates.x] [currentCoordinates.y].north = true;
		
	}
	
	else if(robotDirection == east){
		
		grid[currentCoordinates.x] [currentCoordinates.y].east = true;
		++currentCoordinates.x;
		grid[currentCoordinates.x] [currentCoordinates.y].west = true;
		
	}
	
	else if(robotDirection == west){
		
		grid[currentCoordinates.x] [currentCoordinates.y].west = true;
		--currentCoordinates.x;
		grid[currentCoordinates.x] [currentCoordinates.y].east = true;
		
	}

	
}


void pushStack(){
	
	stackPointer++;
	stack[stackPointer] = currentCoordinates;
	
}


void popStack(){
	
	--stackPointer;
	
}


enum robotDirection updateDirection(uint8_t currentTurn){


	if(robotDirection == north){


		if(currentTurn == RIGHT){

			return east;

		}

		else if(currentTurn == LEFT){

			return west;

		}

		else if(currentTurn == FULLTURN){

			return south;

		}



	}

	else if(robotDirection == south){


		if(currentTurn == RIGHT){

			return west;

		}

		else if(currentTurn == LEFT){

			return east;


		}

		else if(currentTurn == FULLTURN){

			return north;

		}


	}

	else if(robotDirection == east){


		if(currentTurn == RIGHT){

			return south;

		}

		else if(currentTurn == LEFT){

			return north;


		}

		else if(currentTurn == FULLTURN){

			return west;

		}


	}

	else if(robotDirection == west){


		if(currentTurn == RIGHT){

			return north;

		}


		else if(currentTurn == LEFT){

			return south;


		}

		else if(currentTurn == FULLTURN){

			return east;

		}


	}
	
	return north;

}


void initializeGrid(){
	
	// western edge
		grid[0][0].west = true;
		grid[0][1].west = true;
		grid[0][2].west = true;
		grid[0][3].west = true;
		grid[0][4].west = true;
		grid[0][5].west = true;
		grid[0][6].west = true;
		
		// eastern edge
		grid[5][0].east = true;
		grid[5][1].east = true;
		grid[5][2].east = true;
		grid[5][3].east = true;
		grid[5][4].east = true;
		grid[5][5].east = true;
		grid[5][6].east = true;
		
		// northern edge
		grid[0][6].north = true;
		grid[1][6].north = true;
		grid[2][6].north = true;
		grid[3][6].north = true;
		grid[4][6].north= true;
		grid[5][6].north = true;
		
		// southern edge
		grid[0][0].south = true;
		grid[1][0].south = true;
		grid[2][0].south = true;
		grid[3][0].south = true;
		grid[4][0].south= true;
		grid[5][0].south= true;
	
	
}


void steer_robot_forward(uint8_t speed, short fract steer){
	
	
	sat unsigned short accum left_duty_cycle; // 0 to 255
	sat unsigned short accum  right_duty_cycle;
	
	left_duty_cycle = speed*(1.075+steer); // adjusted since left motor is slow
	right_duty_cycle = speed*(1-steer);
	
	set_right_motor_duty_cycle(right_duty_cycle);
	set_left_motor_duty_cycle(left_duty_cycle);
	
	return;
	
}


void move_robot_forward(){

	forward_left_motor();
	forward_right_motor();

}


#define LINE_LOST_COUNT 100
#define LEFT_INTERSECTION_COUNT 100
#define RIGHT_INTERSECTION_COUNT 100
#define FLITER .80
enum sensor_state read_line_sensor(short fract * position){
	
	static short fract avg_line_pos = 0;
	static int8_t line_lost_filter = 0;
	static int8_t left_sensor_filter = 0;
	static int8_t right_sensor_fliter = 0;
	
	uint8_t sensor_val = get_line_sensor();
	short fract current_line_pos =
	(sensor_val &(1<<1))? -0.25:0 +
	(sensor_val & (1<<2))? 0:0+
	(sensor_val & (1<<3))? .25:0;

	avg_line_pos = FLITER*avg_line_pos + (1-FLITER)*current_line_pos;


	if(sensor_val && line_lost_filter < LINE_LOST_COUNT){
		++ line_lost_filter;
	}
	
	else if( !sensor_val && line_lost_filter > -LINE_LOST_COUNT) {
		--line_lost_filter;
	}

	if(sensor_val & (1<<0) && left_sensor_filter < LEFT_INTERSECTION_COUNT){
		++ left_sensor_filter;
	}

	else if(!(sensor_val & (1<<0))  && left_sensor_filter > -LEFT_INTERSECTION_COUNT) {
		--left_sensor_filter;
	}
	
	if(sensor_val & (1<<4) && right_sensor_fliter < RIGHT_INTERSECTION_COUNT){
		++ right_sensor_fliter;
	}

	else if(!(sensor_val & (1<<4))  && right_sensor_fliter > -RIGHT_INTERSECTION_COUNT) {
		--right_sensor_fliter;
	}
	
	

	*position = avg_line_pos;

	if(line_lost_filter < 0){
		
		return LINE_LOST;
		
	}
	
	else if(right_sensor_fliter > 0 && left_sensor_filter > 0){
		
		return T_INTERSECTION;
		
	}
	
	else if(right_sensor_fliter > 0){
		
		return RIGHT_INTERSECTION;
		
	}
	
	else if(left_sensor_filter > 0){
		
		return LEFT_INTERSECTION;
		
	}
	
	else if(line_lost_filter >= 0){
		
		return LINE_FOUND;
		
	}
	
	return LINE_LOST;

}


_Bool move_robot( movement_t new_move , _Bool start_movement)
{
	static enum {ST_WAIT_FOR_START, ST_DO_MOVE, ST_DO_STOP} state=ST_WAIT_FOR_START;
	static movement_t movement;
	_Bool is_busy = false;
	

	switch(state)
	{
		case ST_WAIT_FOR_START:
		if (!start_movement)
		{
			is_busy = false;
			state = ST_WAIT_FOR_START;
		}
		else
		{
			movement = new_move;
			is_busy = true;
			state = ST_DO_MOVE;
		}
		
		break;
		
		case ST_DO_MOVE:
		if (movement.time != 0)
		{
			movement.time--;
			is_busy = true;
			state = ST_DO_MOVE;
	}
	else if (movement.time == 0)
		{
			movement.left_duty_cycle = 0;
			movement.right_duty_cycle = 0;
			movement.time = 1000;
			is_busy = true;
			state = ST_DO_STOP;
		}
		
		break;
		
		case ST_DO_STOP:
		if (movement.time != 0)
		{
			movement.time--;
			is_busy = true;
			state = ST_DO_STOP;
		}
		else if (movement.time == 0)
		{

			is_busy = false;
			state = ST_WAIT_FOR_START;
		}
		
		break;
		

	}


	// set duty cycles
	if(is_busy){
		
		set_right_motor_duty_cycle(movement.right_duty_cycle);
		set_left_motor_duty_cycle(movement.left_duty_cycle);
	}
	
	
	// if right turn
	if(movement.type_of_turn == 0  && is_busy){
		forward_left_motor();
		reverse_right_motor();
	}
	
	// left turn
	if(movement.type_of_turn == 1 && is_busy){
		forward_right_motor();
		reverse_left_motor();
	}
	
	
	// turn around
	if(movement.type_of_turn == 2 && is_busy){
		forward_left_motor();
		reverse_right_motor();
	}

	// forward
	if(movement.type_of_turn == 3 && is_busy){
		forward_left_motor();
		forward_right_motor();
	}
	
	// reverse
	if(movement.type_of_turn == 4 && is_busy){
		
		reverse_left_motor();
		reverse_right_motor();
		
	}
	

	//return this FSM return value
	return is_busy;
	
}