#include <iostream>
#include "../include/frontgoal.h"
#include "../include/wayptrgenerator.h"
using namespace Front;

int main(int argc, char *argv[])
{
	std::vector<std::vector<int>> binaryMatrix = {{0, 0, 0, 0, 1, 1, -1, -1, -1, -1},
												  {0, 0, 0, 0, 1, 1, -1, -1, -1, -1},
													{0, 0, 0, 0, 1, 1, -1, -1, -1, -1},
													{0, 0, 0, 0, 0, 1, -1, -1, -1, -1},
													{1, 0, 0, 0, 0, 0, -1, -1, -1, -1},
													{1, 1, 1, 0, 0, 0, -1, -1, -1, -1},
													{-1, -1, 1, 0, 0, 0, -1, -1, -1, -1},
													{-1, -1, 0, 0, 1, 1, -1, -1, -1, -1},	
													};
    int droneX = 0, droneY = 0;

    FrontGoal goal(binaryMatrix, droneX, droneY);

    // goal.DilateObstacle(1);

	auto next_goal = goal.findGoal();
	
	std::cout<<"x: "<<next_goal.x<<" y: "<<next_goal.y<<" p: "<<next_goal.priority<<std::endl;

    
	auto goalX = next_goal.x;
	auto goalY = next_goal.y;

	Dijkstra Dijkstra(binaryMatrix);

	std::vector<std::pair<int, int>> path = Dijkstra.findShortestPath(droneX, droneY, goalX, goalY);
	
	for (auto wayptr: path){
		std::cout<<" x "<<wayptr.first<<" y "<<wayptr.second<<std::endl;
	}

	// Point p1 = {2,3,1};
	// Point p2 = {1,2,2};
	// if (p2<p1){
	// 	std::cout<<"yes"<<std::endl;
	// }

	return 0;
}