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
#include <queue>
#include <time.h>
#include <chrono>
#include <ctime>

using namespace std;

//{=================================================
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//these global variables must be defined here with no modification.

// Float Variables
float virtualCarLinearSpeed;//can get ands set
float virtualCarAngularSpeed;//can get and set
float currentCarAngle;//can get and set
float currentCarPosCoord_X, currentCarPosCoord_Y;//can get and set
float sensorSeparation;//can set
float num_sensors;//can set
float detected;

// Bool Variables
bool foodfound = false;
bool pathFound = false;
bool pathway = false;

// Int Arrays
int visited[15][19] = { 0 };
int PathFoundArray[15][19] = { 0 };
int prevRow[15][19] = { 0 };
int prevCol[15][19] = { 0 };
int GetAllFood[15][19] = { 0 };

// Int Variables
int sensorPopulationAlgorithmID;
int flag = 0;
int numIncre = 0;
int startRow = 0;
int startCol = 0;
int pathway_Row = 0;
int pathway_Col = 0;
int pathCounter = 0;
int forwardPath = 0;
int intersection = 0;
int turn = 0;
int currentDirection = 0;
int turnDirection = 0;
int cDflag = 0;
int start = 1;
int turning = 0;
int executeTurn = 0;
int flagMemory = 0;
int normalTurn = 0;
int normalAngle = 0;
int level = 1;

// Vectors
vector<pair<int, int>> path;
vector<int> virtualCarSensorStates; //can get
vector<ghostInfoPack> ghostInfoPackList;// can get

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
//}=================================================

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

	for (int i = 0; i < 15; i++) {
		for (int j = 0; j < 19; j++) {
			visited[i][j] = 1;
			PathFoundArray[i][j] = 1;
			GetAllFood[i][j] = map[i][j];
		}
	}


	for (int i = 0; i < 19; i++) {
		for (int j = 0; j < 15; j++) {
			if (GetAllFood[j][i] == 0) {
				pathway_Row = j;
				pathway_Col = i;
				goto exit;
			}
		}
	}
exit:;
	return 1;
}

void FindNextFood() {
	// Find food in vertical order
	for (int i = 0; i < 19; i++) {
		for (int j = 0; j < 15; j++) {
			if (GetAllFood[j][i] == 0) {
				pathway_Row = j;
				pathway_Col = i;
				goto exit;
			}
		}
	}
exit:;
}

void BFS() {
	int row = 15;
	int column = 19;
	int size = 15 * 19;
	std::queue<int> rowQueue;
	std::queue<int> colQueue;

	int xSearch[4] = { 0, 0, +1, -1 };
	int ySearch[4] = { -1, +1, 0, 0 };
	int rowSearch, colSearch;

	//// Reset and find next food

	if (flagMemory == 5) {
		//GetAllFood[pathway_Row][pathway_Col] = 1;
		FindNextFood();
		foodfound = false;
		pathFound = false;
		startRow = coordToCellY(currentCarPosCoord_Y);
		startCol = coordToCellX(currentCarPosCoord_X);
		path.clear();
		//pathCounter = 0;
		//numIncre++;

		for (int i = 0; i < 15; i++) {
			for (int j = 0; j < 19; j++) {
				visited[i][j] = 1;
				PathFoundArray[i][j] = 1;
				prevRow[i][j] = 0;
				prevCol[i][j] = 0;
			}
		}
	}

	//if (start == 0) {
	while (foodfound == false) {
		int move_count = 0;
		int nodes_left_to_check = 1;
		int nodes_next = 0;

		rowQueue.push(startRow);
		colQueue.push(startCol);
		visited[startRow][startCol] = 0;
		prevRow[startRow][startCol] = startRow; // set the start node's parent node as itself
		prevCol[startRow][startCol] = startCol;
		while (rowQueue.size() > 0) {
			//cout << "while loop" << endl;
			rowSearch = rowQueue.front();
			//cout << "rowSearch = " << rowSearch << endl;
			rowQueue.pop();
			colSearch = colQueue.front();
			//cout << "colSearch = " << colSearch << endl;
			colQueue.pop();

			if (level == 0 && colSearch == pathway_Col && rowSearch == pathway_Row) {
				foodfound = true;
				cout << "Food Found" << endl;
				break;
			}
			else if (level == 1 && colSearch == food_list[numIncre][0] && rowSearch == food_list[numIncre][1]) {
				foodfound = true;
				cout << "Food Found" << endl;
				break;
			}
			// explore neighbours
			for (int i = 0; i < 4; i++) {
				int rowrow = rowSearch + ySearch[i];
				int colcol = colSearch + xSearch[i];

				// skip out of bounds locations
				if (rowrow < 0 || colcol < 0) {
					continue;
				}
				if (rowrow >= row || colcol >= column) {
					continue;
				}
				// skip visited locations or off map locations
				if (visited[rowrow][colcol] == 0) {
					continue;
				}
				if (map[rowrow][colcol]) { // map = 1 , no path
					continue;
				}
				//cout << "ADDING TO QUEUE = " << rowrow << "," << colcol << endl;
				rowQueue.push(rowrow);
				colQueue.push(colcol);
				visited[rowrow][colcol] = 0;
				prevRow[rowrow][colcol] = rowSearch; // attatch as parent node
				prevCol[rowrow][colcol] = colSearch;
				nodes_next++;
			}
			// explore neighbours end
			nodes_left_to_check--;
			if (nodes_left_to_check == 0) {
				nodes_left_to_check = nodes_next;
				nodes_next = 0;
				move_count++;
				//cout << "Move count = " << move_count << endl;
			}
		}
		//}
	}

	if (foodfound == true && pathFound == false) {
		// reconstruct the path
		int pathRow = 0; int pathCol = 0;
		if (level == 0) {
			pathRow = pathway_Row;
			pathCol = pathway_Col;
		}
		else if (level == 1) {
			pathRow = food_list[numIncre][1];
			pathCol = food_list[numIncre][0];
		}

		path.push_back({ pathRow , pathCol }); // end position
		while ((pathRow != startRow) || (pathCol != startCol)) { // loop until reached the starting position
			int pathRowtemp = pathRow; // the previous node
			int pathColtemp = pathCol;
			pathRow = prevRow[pathRow][pathCol]; // next parent node
			pathCol = prevCol[pathRowtemp][pathCol];
			path.push_back({ pathRow, pathCol }); // add the parent node
		}
		reverse(path.begin(), path.end());
		cout << "The path REVERSED is: " << endl;
		for (int i = 0; i < path.size(); i++) {
			cout << path[i].first << " , " << path[i].second << endl;
		}

		for (int i = 0; i < path.size(); i++) {
			PathFoundArray[path[i].first][path[i].second] = 0;
		}
		pathFound = true;
	}

}


int virtualCarUpdate()
{
	// get starting position of the car as soon as it steps on a black line
	if (start == 1 && map[coordToCellY(currentCarPosCoord_Y)][coordToCellX(currentCarPosCoord_X)] == 0) {
		startRow = coordToCellY(currentCarPosCoord_Y);
		startCol = coordToCellX(currentCarPosCoord_X);
		start = 0;
		BFS();
		cout << "Starting position = " << startRow << " , " << startCol << endl;
	}
	//{----------------------------------
	//process sensor state information
	float halfTiltRange = (num_sensors - 1.0) / 2.0;
	float tiltSum = 0.0;
	float blackSensorCount = 0.0;

	for (int i = 0; i < num_sensors; i++)
	{
		if (virtualCarSensorStates[i] == 0) // Check how many sensors are detected on if on road, on track = 0, not on track = 1
		{
			float tilt = (float)i - halfTiltRange;
			tiltSum += tilt; // Negative for left, postiive for right
			blackSensorCount += 1.0; // If we see a path line, increment 

		}
	}

	// Check time of function elapsed
	auto t1 = std::chrono::high_resolution_clock::now();

	/////////////////////////////////////////////GoooDTurns/////////////////////////////////////////////////////////////////////
	if (blackSensorCount >= 6) {
		flag = 1; // T intersection
		if (executeTurn == 0) {
			turning = 1;
		}
	}
	else if (blackSensorCount > 1) {
		if (virtualCarSensorStates[6] == 0 && virtualCarSensorStates[3] == 0 && ((virtualCarSensorStates[5] == 0) || (virtualCarSensorStates[4] == 0))) {
			flag = 2; // turn left
			if (executeTurn == 0) {
				turning = 1;
			}
		}
		else if (virtualCarSensorStates[0] == 0 && virtualCarSensorStates[3] == 0 && ((virtualCarSensorStates[1] == 0) || (virtualCarSensorStates[2] == 0))) {
			flag = 3; // turn right
			if (executeTurn == 0) {
				turning = 1;
			}
		}
		else {
			flag = 0;
		}
	}
	else {
		flag = 0;
	}

	if (turning == 1 && executeTurn == 0) {
		if (flagMemory == 0) {
			flagMemory = flag;
		}
		// catch when it is a T intersection but only left or right turn was flagged
		else if ((flagMemory == 2 || flagMemory == 3) && flag == 1) {
			flagMemory = flag;
		}
		//// T intersection and there is a forward path
		if (flagMemory == 1 && blackSensorCount < 7 && virtualCarSensorStates[3] == 0) {
			cout << " T intersection and foward path " << endl;
			//flagMemory = 0;
			//turning = 0;
		}
		// left/right turn and there is a forward path
		else if (flagMemory > 1 && blackSensorCount < 3 && virtualCarSensorStates[3] == 0) {
			cout << " LRLR and foward path " << endl;
			//flagMemory = 0;
			//turning = 0;
		}
		if (flagMemory != flag) {
			executeTurn = 1;
			turning = 0;
			// T intersection
			if ((flagMemory == 1 && blackSensorCount < 7) || (flagMemory > 1 && blackSensorCount < 3 && virtualCarSensorStates[3] == 0)) {
				// find the current car angle
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
				cout << " CAR CURRENT DIRECTION = " << currentDirection << endl;
				cout << " Execute Turn, CarX = " << coordToCellX(currentCarPosCoord_X) << "CarY" << coordToCellY(currentCarPosCoord_Y) << endl;
				int nextY = 0; int nextX = 0;
				for (int i = 0; i < path.size(); i++) {
					if (level == 1 && GetAllFood[(food_list[numIncre][1])][(food_list[numIncre][0])] == 1) {
						flagMemory = 5; // do 180 degree turn
						numIncre++;
						BFS();
						cout << "BFS SEARCH: Level 1" << endl;
						nextY = path[1].first;
						nextX = path[1].second;
						break;
					}
					if ((coordToCellY(currentCarPosCoord_Y) == path[i].first) && (coordToCellX(currentCarPosCoord_X) == path[i].second) &&
						GetAllFood[coordToCellY(currentCarPosCoord_Y)][coordToCellX(currentCarPosCoord_X)] == 0) {
						nextY = path[i + 1].first;
						nextX = path[i + 1].second;
						cout << "NextX = " << nextX << " NextY = " << nextY << endl;
						break;
					}
					// the correct path is not found
					else if (i == path.size() - 1) {
						flagMemory = 5; // do 180 degree turn
						BFS();
						cout << "BFS SEARCH" << endl;
						nextY = path[1].first;
						nextX = path[1].second;
					}
				}
				if (currentDirection == 1) { // facing North
					if (nextX == coordToCellX(currentCarPosCoord_X) - 1) {
						flagMemory = 2; 	// X direction: new node == prev node - 1 (going left)
					}
					else if (nextX == coordToCellX(currentCarPosCoord_X) + 1) {
						flagMemory = 3; 	// X direction: new node == prev node + 1 (going right)
					}
					else if (nextY == coordToCellY(currentCarPosCoord_Y) - 1) {
						flagMemory = 4; 	// Y direction: new node == prev node - 1 (going up), so go forward
					}
					else if (nextY == coordToCellY(currentCarPosCoord_Y) + 1) {
						flagMemory = 5; 	// Y direction: new node == prev node + 1 (going down), so do 180
					}
				}
				else if (currentDirection == 2) { // facing South
					if (nextX == coordToCellX(currentCarPosCoord_X) - 1) {
						flagMemory = 3; 	// X direction: new node == prev node - 1 (going left), so turn right
					}
					else if (nextX == coordToCellX(currentCarPosCoord_X) + 1) {
						flagMemory = 2; 	// X direction: new node == prev node + 1 (going right), so turn left
					}
					else if (nextY == coordToCellY(currentCarPosCoord_Y) + 1) {
						flagMemory = 4; 	// Y direction: new node == prev node + 1 (going down), so go forward
					}
					else if (nextY == coordToCellY(currentCarPosCoord_Y) - 1) {
						flagMemory = 5; 	// Y direction: new node == prev node - 1 (going up), so do 180
					}
				}
				else if (currentDirection == 3) { // facing West
					if (nextY == coordToCellY(currentCarPosCoord_Y) - 1) {
						flagMemory = 3; 	// Y direction: new node == prev node - 1 (going up), so turn right
					}
					else if (nextY == coordToCellY(currentCarPosCoord_Y) + 1) {
						flagMemory = 2; 	// Y direction: new node == prev node + 1 (going down), so turn left
					}
					else if (nextX == coordToCellX(currentCarPosCoord_X) - 1) {
						flagMemory = 4; 	// X direction: new node == prev node - 1 (going left), so go forward
					}
					else if (nextX == coordToCellX(currentCarPosCoord_X) + 1) {
						flagMemory = 5; 	// X direction: new node == prev node + 1 (going right), so do 180
					}
				}
				else if (currentDirection == 4) { // facing East
					if (nextY == coordToCellY(currentCarPosCoord_Y) - 1) {
						flagMemory = 2; 	// Y direction: new node == prev node - 1 (going up), so turn left
					}
					else if (nextY == coordToCellY(currentCarPosCoord_Y) + 1) {
						flagMemory = 3; 	// Y direction: new node == prev node + 1 (going down), so turn right
					}
					else if (nextX == coordToCellX(currentCarPosCoord_X) + 1) {
						flagMemory = 4; 	// X direction: new node == prev node + 1 (going right), so go forward
					}
					else if (nextX == coordToCellX(currentCarPosCoord_X) - 1) {
						flagMemory = 5; 	// X direction: new node == prev node - 1 (going left), so do 180
					}
				}
			}
			else {
				normalTurn = 1;
				cout << "Normal Turn" << endl;
			}
		}
	}

	// log cell location
	GetAllFood[coordToCellY(currentCarPosCoord_Y)][coordToCellX(currentCarPosCoord_X)] = 1;

	if (turning == 1 && executeTurn == 0) {
		setVirtualCarSpeed(virtualCarLinearSpeed_seed * 2, 0.0);
	}
	else if (normalTurn == 1) {
		if (tiltSum == 0) {
			if (flagMemory == 2) {
				tiltSum = 2.0;
			}
			else if (flagMemory == 3) {
				tiltSum = -2.0;
			}
		}
		setVirtualCarSpeed(virtualCarLinearSpeed_seed / 2, virtualCarAngularSpeed_seed * tiltSum);
		if (virtualCarSensorStates[3] == 0) {
			normalTurn = 0;
			flagMemory = 0;
			executeTurn = 0;
		}
	}
	else if (executeTurn == 1) {
		if (flagMemory == 2) {
			setVirtualCarSpeed(virtualCarLinearSpeed_seed / 6, virtualCarAngularSpeed_seed);
			if (currentDirection == 1) { // Facing North
				if (currentCarAngle > 165 && currentCarAngle <= 185) {
					setVirtualCarSpeed(virtualCarLinearSpeed_seed / 4, virtualCarAngularSpeed_seed * tiltSum);
					cout << " FINISHED TURNING N LEFT" << endl;
					executeTurn = 0;
					flagMemory = 0;
				}
			}
			else if (currentDirection == 2) { // Facing South
				if (currentCarAngle > 345 && currentCarAngle <= 360 || currentCarAngle >= 0 && currentCarAngle <= 15) {
					setVirtualCarSpeed(virtualCarLinearSpeed_seed / 4, virtualCarAngularSpeed_seed * tiltSum);
					cout << " FINISHED TURNING S LEFT" << endl;
					executeTurn = 0;
					flagMemory = 0;
				}
			}
			else if (currentDirection == 3) { // Facing West
				if (currentCarAngle > 255 && currentCarAngle <= 285) {
					setVirtualCarSpeed(virtualCarLinearSpeed_seed / 4, virtualCarAngularSpeed_seed * tiltSum);
					cout << " FINISHED TURNING W LEFT" << endl;
					executeTurn = 0;
					flagMemory = 0;
				}
			}
			else if (currentDirection == 4) { // Facing East
				if (currentCarAngle > 75 && currentCarAngle <= 105) {
					setVirtualCarSpeed(virtualCarLinearSpeed_seed / 4, virtualCarAngularSpeed_seed * tiltSum);
					cout << " FINISHED TURNING E LEFT" << endl;
					executeTurn = 0;
					flagMemory = 0;
				}
			}

		}
		// right turn
		else if (flagMemory == 3 || flagMemory == 1) {
			setVirtualCarSpeed(virtualCarLinearSpeed_seed / 6, virtualCarAngularSpeed_seed * -1);
			if (currentDirection == 1) { // Facing North
				if (currentCarAngle > 345 && currentCarAngle <= 360 || currentCarAngle >= 0 && currentCarAngle <= 15) {
					setVirtualCarSpeed(virtualCarLinearSpeed_seed / 4, virtualCarAngularSpeed_seed * tiltSum);
					cout << " FINISHED TURNING N RIGHT" << endl;
					executeTurn = 0;
					flagMemory = 0;
				}
			}
			else if (currentDirection == 2) { // Facing South
				if (currentCarAngle > 165 && currentCarAngle <= 185) {
					setVirtualCarSpeed(virtualCarLinearSpeed_seed / 4, virtualCarAngularSpeed_seed * tiltSum);
					cout << " FINISHED TURNING S RIGHT" << endl;
					executeTurn = 0;
					flagMemory = 0;
				}
			}
			else if (currentDirection == 3) { // Facing West
				if (currentCarAngle > 75 && currentCarAngle <= 105) {
					setVirtualCarSpeed(virtualCarLinearSpeed_seed / 4, virtualCarAngularSpeed_seed * tiltSum);
					cout << " FINISHED TURNING W RIGHT" << endl;
					executeTurn = 0;
					flagMemory = 0;
				}
			}
			else if (currentDirection == 4) { // Facing East
				if (currentCarAngle > 255 && currentCarAngle <= 285) {
					setVirtualCarSpeed(virtualCarLinearSpeed_seed / 4, virtualCarAngularSpeed_seed * tiltSum);
					cout << " FINISHED TURNING E RIGHT" << endl;
					executeTurn = 0;
					flagMemory = 0;
				}
			}
		}
		else if (flagMemory == 4) {
			setVirtualCarSpeed(virtualCarLinearSpeed_seed, 0.0);
			if (flag == 0) {
				setVirtualCarSpeed(virtualCarLinearSpeed_seed, virtualCarAngularSpeed_seed * tiltSum);
				cout << " FINISHED forward path" << endl;
				executeTurn = 0;
				flagMemory = 0;
			}
		}
		else if (flagMemory == 5) {
			setVirtualCarSpeed(0.0, virtualCarAngularSpeed_seed * -1);
			if (currentDirection == 1) { // Facing North
				if (currentCarAngle > 255 && currentCarAngle <= 285) {
					setVirtualCarSpeed(virtualCarLinearSpeed_seed / 4, virtualCarAngularSpeed_seed * tiltSum);
					cout << " FINISHED TURNING N 180" << endl;
					executeTurn = 0;
					flagMemory = 0;
				}
			}
			else if (currentDirection == 2) { // Facing South
				if (currentCarAngle > 75 && currentCarAngle <= 105) {
					setVirtualCarSpeed(virtualCarLinearSpeed_seed / 4, virtualCarAngularSpeed_seed * tiltSum);
					cout << " FINISHED TURNING S 180" << endl;
					executeTurn = 0;
					flagMemory = 0;
				}
			}
			else if (currentDirection == 3) { // Facing West
				if (currentCarAngle > 345 && currentCarAngle <= 360 || currentCarAngle >= 0 && currentCarAngle <= 15) {
					setVirtualCarSpeed(virtualCarLinearSpeed_seed / 4, virtualCarAngularSpeed_seed * tiltSum);
					cout << " FINISHED TURNING W 180" << endl;
					executeTurn = 0;
					flagMemory = 0;
				}
			}
			else if (currentDirection == 4) { // Facing East
				if (currentCarAngle > 165 && currentCarAngle <= 185) {
					setVirtualCarSpeed(virtualCarLinearSpeed_seed / 4, virtualCarAngularSpeed_seed * tiltSum);
					cout << " FINISHED TURNING E 180" << endl;
					executeTurn = 0;
					flagMemory = 0;
				}
			}
		}
	}
	else {
		if (blackSensorCount > 0.0) {
			setVirtualCarSpeed(virtualCarLinearSpeed_seed, virtualCarAngularSpeed_seed * tiltSum);
		}
		else {
			setVirtualCarSpeed(0.0, virtualCarAngularSpeed_seed);
		}
	}
	auto t2 = std::chrono::high_resolution_clock::now();
	auto duration1 = std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1).count();



	//below is optional. just to provid some status report and function test result .
	//You can try to use "printf()" to reimplemet this "cout" c++ section in a c style instead.
	//{--------------------------------------------------------------	
	if (myTimer.getTimer() > 0.5)
	{
		myTimer.resetTimer();
		cout << "current Cell X, Y = " << coordToCellY(currentCarPosCoord_Y) << " , " << coordToCellX(currentCarPosCoord_X) << endl;
		if (level == 0) {cout << "Destination = " << pathway_Row << " , " << pathway_Col << endl;}
		else if (level == 1) {cout << "Destination = " << food_list[numIncre][1] << " , " << food_list[numIncre][0] << endl;}
		std::cout << "Function Time: " << duration1 << "ns" << endl;
		std::cout << "Duration: " << clock() / double(1000) << "s" << endl;
		std::cout << endl;

		//	for (int i = 0; i < 15; i++) {
		//		for (int j = 0; j < 19; j++) {
		//			cout << PathFoundArray[i][j] << " ";
		//		}
		//		cout << endl;
		//	}
	}

	return 1;
}
//}=============================================================



int main(int argc, char** argv)
{

	FungGlAppMainFuction(argc, argv);

	return 0;
}
