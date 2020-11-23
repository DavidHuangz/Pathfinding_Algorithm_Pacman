#ifndef _ASTAR_H
#define _ASTAR_H 

#define ROW 15 
#define COL 19

int traceDirection(); // getter
typedef std::pair<int, int> Pair;
bool isDestination(int row, int col, Pair dest);
void aStarSearch(int grid[][COL], Pair src, Pair dest);


#endif
