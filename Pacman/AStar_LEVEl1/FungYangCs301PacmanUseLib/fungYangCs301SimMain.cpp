//======================================================================
//Author: 
//Mr. Fung Yang
//Senior Technician Engineer Research and Design,
//Robotics and Control system signal processing Labs,
//Department of Electrical, Computer and Software Engineering,
//The University of Auckland.
//
//Written for teaching design course Compsys301 in ECSE department of UOA.
//
//This example program uses the pacman robot simulation library written by Mr. Fung Yang.
//
//Date 2012~2020
//=======================================================================

#include "mainFungGLAppEngin.h" //a must
#include "data.h" //a must
#include "highPerformanceTimer.h"//just to include if timer function is required by user.
#include <vector>
#include <iostream>
#include <string>
#include "aStar.h"
#include <chrono>
#include <ctime>

using namespace std;

//{=================================================
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//these global variables must be defined here with no modification.
float virtualCarLinearSpeed;//can get ands set
float virtualCarAngularSpeed;//can get and set
float currentCarAngle;//can get and set
float currentCarPosCoord_X, currentCarPosCoord_Y;//can get and set

int sensorPopulationAlgorithmID;//can set
float sensorSeparation;//can set
float num_sensors;//can set

vector<int> virtualCarSensorStates; //can get

vector<ghostInfoPack> ghostInfoPackList;// can get
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//}=================================================

highPerformanceTimer myTimer;

//just a helper function
void setVirtualCarSpeed(float linearSpeed, float angularSpeed)
{
	virtualCarLinearSpeed = linearSpeed;
	virtualCarAngularSpeed = angularSpeed;
}

//The Only TWO unctions Students need to modify to add their own sensor guided
//path following control and Map path search calculations.
//{=================================================
float virtualCarLinearSpeed_seed;
float virtualCarAngularSpeed_seed;

// Task 4
//{=================================================
// 2D array of zeros or ones to log where the robot has been
// ones will indicate the robot has been in that cell before
// zeros will indicate the robot has not been in that cell before
// Map is x,y => 18,14 with the origin at the top-left corner and max at the bottom-right corner
int cells_visited[15][19] = { 0 };

int desY = food_list[0][1];
int desX = food_list[0][0];
int counter = 0;
int intersection = 10;
int turn = 0;
int currentDirection = 0;
int turnDirection = 0;
int finish = 0;
int changeFood = 0;
int atDestination = 1;
Pair dest = make_pair(desY, desX);
Pair src;
int found_unvisited = 0;
int potentialIntersection = 0;
int currentStatus = 0;
int normalTurn = 0;

// cells_travelled has total distance travelled by the robot in terms of cells.
// coord_travelled has the same but in terms of coords.
// Note: Probably used for short path algorithm Level 2. 
//		 Reset the travelled variables when reached an intersection (do so after finished with calculating)
int cells_travelled;
int prev_cell_X;
int prev_cell_Y;

float coord_travelled;
float prev_coord_X;
float prev_coord_Y;

int virtualCarInit()
{
	//sensorPopulationAlgorithmID = PLACE_SENSORS_AUTO_SEP;
	sensorPopulationAlgorithmID = PLACE_SENSORS_SEP_USER_DEFINED;
	num_sensors = 7;
	sensorSeparation = 0.08;

	virtualCarLinearSpeed_seed = 1.2;
	virtualCarAngularSpeed_seed = 80;
	currentCarPosCoord_X = cellToCoordX(1);
	currentCarPosCoord_Y = cellToCoordY(1);
	currentCarAngle = -90;

	// initialize to zero
	prev_cell_X = coordToCellX(currentCarPosCoord_X);
	prev_cell_Y = coordToCellY(currentCarPosCoord_Y);
	prev_coord_X = currentCarPosCoord_X;
	prev_coord_Y = currentCarPosCoord_Y;
	coord_travelled = 0;
	cells_travelled = 0;
	src = make_pair(coordToCellY(currentCarPosCoord_Y), coordToCellX(currentCarPosCoord_X));
	aStarSearch(map, src, dest); // Only reevaluate shortest path when robot is at an intersection

	return 1;
}

int virtualCarUpdate()
{

	//{----------------------------------
	//process sensor state information
	float halfTiltRange = (num_sensors - 1.0) / 2.0;
	float tiltSum = 0.0;
	float blackSensorCount = 0.0;
	for (int i = 0; i < num_sensors; i++)
	{
		if (virtualCarSensorStates[i] == 0)
		{
			float tilt = (float)i - halfTiltRange;
			tiltSum += tilt;	
			blackSensorCount += 1.0;
		}
	}
	//}------------------------------------
	// Check time of function elapsed
	auto t1 = std::chrono::high_resolution_clock::now();
	float diff;
	// Record the current cell location the robot is on (Check cells_visted declaration for more info)
	cells_visited[coordToCellY(currentCarPosCoord_Y)][coordToCellX(currentCarPosCoord_X)] = 1;
	// Record the travelled distance in coords
	if (prev_coord_X != currentCarPosCoord_X) { // Diff in coords_X

		if (abs(prev_coord_X) > abs(currentCarPosCoord_X)) {
			diff = abs(prev_coord_X) - abs(currentCarPosCoord_X);
		}
		else {
			diff = abs(currentCarPosCoord_X) - abs(prev_coord_X);
		}
		coord_travelled = coord_travelled + diff;
		prev_coord_X = currentCarPosCoord_X;
	}

	if (prev_coord_Y != currentCarPosCoord_Y) { // Diff in coords_Y

		if (abs(prev_coord_Y) > abs(currentCarPosCoord_Y)) {
			diff = abs(prev_coord_Y) - abs(currentCarPosCoord_Y);
		}
		else {
			diff = abs(currentCarPosCoord_Y) - abs(prev_coord_Y);
		}
		coord_travelled = coord_travelled + diff;
		prev_coord_Y = currentCarPosCoord_Y;
	}

	// Record the travelled distance in cells
	if (prev_cell_X != coordToCellX(currentCarPosCoord_X)) { // Diff in cells_X

		if (prev_cell_X > coordToCellX(currentCarPosCoord_X)) {
			diff = prev_cell_X - coordToCellX(currentCarPosCoord_X);
		}
		else {
			diff = coordToCellX(currentCarPosCoord_X) - prev_cell_X;
		}
		cells_travelled = cells_travelled + diff;
		prev_cell_X = coordToCellX(currentCarPosCoord_X);
	}

	if (prev_cell_Y != coordToCellY(currentCarPosCoord_Y)) { // Diff in cells_Y

		if (prev_cell_Y > coordToCellY(currentCarPosCoord_Y)) {
			diff = prev_cell_Y - coordToCellY(currentCarPosCoord_Y);
		}
		else {
			diff = coordToCellY(currentCarPosCoord_Y) - prev_cell_Y;
		}
		cells_travelled = cells_travelled + diff;
		prev_cell_Y = coordToCellY(currentCarPosCoord_Y);
	}


	// {----------------------------------------------------------- LEVEL ONE

	if (coordToCellY(currentCarPosCoord_Y) == dest.first && coordToCellX(currentCarPosCoord_X) == dest.second) {
		atDestination = 1;
		src = make_pair(coordToCellY(currentCarPosCoord_Y), coordToCellX(currentCarPosCoord_X));
	}

	if (atDestination == 1) {
		for (int i = 1; i < 14; i++) {
			for (int j = 1; j < 18; j++) {
				/*cout << "Cells Visited: "<< cells_visited[i][j] << endl;
				cout << i << "," << j;*/
				if (cells_visited[i][j] == 0) {
					if (map[i][j] == 0) {
						dest = make_pair(i, j);
						atDestination = 0;
						aStarSearch(map, src, dest);
						cout << endl;
						found_unvisited = 1;
					}
					if (found_unvisited == 1) {
						break;
					}
				}
			}
			if (found_unvisited == 1) {
				found_unvisited = 0;
				break;
			}
		}
	}

	// {---------------------------------------------- LEVEL ONE


	// {---------------------------------------------- LEVEL TWO
	//if (coordToCellY(currentCarPosCoord_Y) == desY && coordToCellX(currentCarPosCoord_X) == desX) {
	//	src = make_pair(coordToCellY(currentCarPosCoord_Y), coordToCellX(currentCarPosCoord_X)); // at intersection/dest
	//	if (counter == (sizeof(food_list) / sizeof(food_list[0])) - 1) {
	//		finish = 1;
	//	}
	//	else {
	//		counter++;
	//		desX = food_list[counter][0];
	//		desY = food_list[counter][1];
	//		dest = make_pair(desY, desX);
	//		aStarSearch(map, src, dest); // Only reevaluate shortest path when robot is at the destination/starting point/intersection
	//		changeFood = 1;

	//	}
	//}																							

	//{------------------------------------ LEVEL TWO


	

	if (finish == 0) {

		if (changeFood == 1) {
			changeFood = 0;
			intersection = 1;
			currentStatus = 5;
			potentialIntersection = 1;
			turn = 0;
		}
		else {
			if (blackSensorCount > 5) { // T intersection (bottom of T)
				currentStatus = 1;
				if (turn == 0) {
					potentialIntersection = 1;
				}
			}
			else if (blackSensorCount > 1 && blackSensorCount < 6) { // There is a bend on the road (left/right)
				if (virtualCarSensorStates[6] == 0 && virtualCarSensorStates[3] == 0 && ((virtualCarSensorStates[5] == 0) || (virtualCarSensorStates[4] == 0))) {
					currentStatus = 2; // left turn
					if (turn == 0) {
						potentialIntersection = 1;
					}
				}
				else if (virtualCarSensorStates[0] == 0 && virtualCarSensorStates[3] == 0 && ((virtualCarSensorStates[1] == 0) || (virtualCarSensorStates[2] == 0))) {
					currentStatus = 3; // right turn
					if (turn == 0) {
						potentialIntersection = 1;
					}
				}
				else {
					currentStatus = 0;
				}
			}
			else {
				currentStatus = 0;
			}
		}

		if (potentialIntersection == 1 && turn == 0) {
			if (intersection == 10) { // This happens once the robot reaches a potential intersection
				intersection = currentStatus;
			}
		}

		if (intersection != currentStatus && potentialIntersection == 1 && turn == 0) { // The robot's sensors have moved slightly past the initial detection of the intersection
			turn = 1;
			potentialIntersection = 0;
			if ((currentStatus == 1 && blackSensorCount < 7) || (intersection > 1 && virtualCarSensorStates[3] == 0 && blackSensorCount < 3)) {
				intersection = 1;
			}
			if (currentCarAngle > 45 && currentCarAngle <= 135) {
				currentDirection = 1; // North
			}
			else if (currentCarAngle > 225 && currentCarAngle <= 315) {
				currentDirection = 2; // South
			}
			else if (currentCarAngle > 135 && currentCarAngle <= 225) {
				currentDirection = 3; // West
			}
			else if (currentCarAngle > 350 && currentCarAngle <= 360 || currentCarAngle >= 0 && currentCarAngle <= 10) {
				currentDirection = 4; // East
			}

			if (intersection == 1) { // T intersection
				src = make_pair(coordToCellY(currentCarPosCoord_Y), coordToCellX(currentCarPosCoord_X)); // at intersection/dest
				// {--------------------------   LEVEL ONE

				if (src.first + 1 < 15 && src.first - 1 > 0 && src.second + 1 < 18 && src.second - 1 > 0) {
					if (cells_visited[src.first + 1][src.second] == 0 && map[src.first + 1][src.second] == 0) { // North
						dest = make_pair(src.first + 1, src.second);
					}
					else if (cells_visited[src.first - 1][src.second] == 0 && map[src.first - 1][src.second] == 0) { // South
						dest = make_pair(src.first - 1, src.second);
					}
					else if (cells_visited[src.first][src.second + 1] == 0 && map[src.first][src.second + 1] == 0) { // East
						dest = make_pair(src.first, src.second + 1);
					}
					else if (cells_visited[src.first][src.second - 1] == 0 && map[src.first][src.second - 1] == 0) { // West
						dest = make_pair(src.first, src.second - 1);
					}
				}

				// {-------------------------- LEVEL ONE

				aStarSearch(map, src, dest); // Only reevaluate shortest path when robot is at the destination/starting point/intersection
				cout << endl;
				turnDirection = traceDirection();
				intersection = 10;
			}
			else {
				normalTurn = 1;
			}
			
		}



		if (potentialIntersection == 1 && turn == 0) {
			setVirtualCarSpeed(virtualCarLinearSpeed_seed*2, 0.0);
		}
		else if (normalTurn == 1) {

			if (tiltSum == 0) {
				if (intersection == 2) {
					tiltSum = 2.0;
				}
				else if (intersection == 3) {
					tiltSum = -2.0;
				}
			}
			setVirtualCarSpeed(virtualCarLinearSpeed_seed / 2, virtualCarAngularSpeed_seed * tiltSum);
			if (virtualCarSensorStates[3] == 0) {
				normalTurn = 0;
				turn = 0;
				intersection = 10;
			}
			else {
				return 1;
			}
		}
		else if (turn == 1) {
			// Call function to get direction
			// set local variable to an angle that matches the direction
			if (turnDirection == 1) { // North
				if (currentDirection == 4) { // Facing East
					setVirtualCarSpeed(virtualCarLinearSpeed_seed / 2.5, virtualCarAngularSpeed_seed); // turn left
				}
				else if (currentDirection == 2) { // Facing South
					setVirtualCarSpeed(0.0, virtualCarAngularSpeed_seed); // 180 turn
				}
				else {
					setVirtualCarSpeed(virtualCarLinearSpeed_seed / 2.5, virtualCarAngularSpeed_seed * -1); // turn right
				}
				if (currentCarAngle > 75 && currentCarAngle <= 105) {
					turn = 0;
				}
			}
			else if (turnDirection == 3) { // West
				if (currentDirection == 1) { // Facing North
					setVirtualCarSpeed(virtualCarLinearSpeed_seed / 2.5, virtualCarAngularSpeed_seed); // turn left
				}
				else if (currentDirection == 4) { // Facing East
					setVirtualCarSpeed(0.0, virtualCarAngularSpeed_seed); // 180 turn
				}
				else {
					setVirtualCarSpeed(virtualCarLinearSpeed_seed / 2.5, virtualCarAngularSpeed_seed * -1); // turn right
				}
				if (currentCarAngle > 165 && currentCarAngle <= 185) {
					turn = 0;
				}
			}
			else if (turnDirection == 4) { // East
				if (currentDirection == 2) { // Facing South
					setVirtualCarSpeed(virtualCarLinearSpeed_seed / 2.5, virtualCarAngularSpeed_seed); // turn left
				}
				else if (currentDirection == 3) { // Facing West
					setVirtualCarSpeed(0.0, virtualCarAngularSpeed_seed); // 180 turn
				}
				else {
					setVirtualCarSpeed(virtualCarLinearSpeed_seed / 2.5, virtualCarAngularSpeed_seed * -1); // turn right
				}
				if (currentCarAngle > 345 && currentCarAngle <= 360 || currentCarAngle >= 0 && currentCarAngle <= 15) {
					turn = 0;
				}
			}
			else if (turnDirection == 2) { // South
				if (currentDirection == 3) { // Facing West
					setVirtualCarSpeed(virtualCarLinearSpeed_seed / 2.5, virtualCarAngularSpeed_seed); // turn left
				}
				else if (currentDirection == 1) { // Facing North
					setVirtualCarSpeed(0.0, virtualCarAngularSpeed_seed); // 180 turn
				}
				else {
					setVirtualCarSpeed(virtualCarLinearSpeed_seed / 2.5, virtualCarAngularSpeed_seed * -1); // turn right
				}
				if (currentCarAngle > 255 && currentCarAngle <= 285) {
					turn = 0;
				}
			}
			if (turn == 0) {
				currentStatus = 0;
			}
		} else {
			if (blackSensorCount > 0.0) {
				setVirtualCarSpeed(virtualCarLinearSpeed_seed, virtualCarAngularSpeed_seed * tiltSum);
			}
			else {
				setVirtualCarSpeed(0.0, virtualCarAngularSpeed_seed);
			}
		}


	}
	auto t2 = std::chrono::high_resolution_clock::now();
	auto duration1 = std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1).count();



	//below is optional. just to provid some status report and function test result .
	//You can try to use "printf()" to reimplemet this "cout" c++ section in a c style instead.
	//{--------------------------------------------------------------	
	if (myTimer.getTimer() > 0.5)
	{
		std::cout << "Function Time: " << duration1 << "ns" << endl;
		std::cout << "Duration: " << clock() / double(1000) << "s" << endl;
		cout << "Coordinates Travelled: " << coord_travelled << endl;
		cout << "Cells Travelled: " << cells_travelled << endl;
		myTimer.resetTimer();

		
	}
	//}---------------------------------------------------------------

	return 1;
}
//}=============================================================

int main(int argc, char** argv)
{
	FungGlAppMainFuction(argc, argv);

	return 0;
}
