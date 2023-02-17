#include <iostream>
#include <fstream>
#include "../include/frontgoal.h"
#include "../include/wayptrgenerator.h"
#include <memory>
#include <vector>
#include <cstring>
using namespace Front;

int main(int argc, char *argv[])
{
	// 	std::vector<std::vector<int>> binaryMatrix = {{0, 0, 0, 0, 1, 1, -1, -1, -1, -1},
	// 												  {0, 0, 0, 0, 1, 1, -1, -1, -1, -1},
	// 													{0, 0, 0, 0, 1, 1, -1, -1, -1, -1},
	// 													{0, 0, 0, 0, 0, 1, -1, -1, -1, -1},
	// 													{1, 0, 0, 0, 0, 0, -1, -1, -1, -1},
	// 													{1, 1, 1, 0, 0, 0, -1, -1, -1, -1},
	// 													{-1, -1, 1, 0, 0, 0, -1, -1, -1, -1},
	// 													{-1, -1, 0, 0, 1, 1, -1, -1, -1, -1},
	// 													};

	std::ifstream mymap;
	std::string filename = "map1.txt";
	mymap.open(filename.c_str());

	const int height = 135;
	const int width = 196;
	int num = -1;
	std::vector<int> vec;

	if (mymap.is_open())
	{
		std::cout << " in " << std::endl;
		char c;
		while (mymap.get(c))
		{
			if (std::isdigit(c) || c == '-')
			{
				mymap.unget();
				mymap >> num;
				vec.push_back(num);
			}
		}
	}
	else
	{
		std::cout << "Error opening file: " << strerror(errno) << std::endl;
	}

	mymap.close();

	std::cout << "size of this long vector " << vec.size() << std::endl;

	std::vector<std::vector<int>> matrix(height, std::vector<int>(width, 0));

	// std::copy(vec.begin(), vec.end(), matrix[0].begin());
	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < width; j++)
		{
			matrix[i][j] = vec[i * width + j];
		}
	}

	// auto data = std::make_unique<std::vector<int>>(height * width);

	// // Fill the vector with data
	// for (int i = 0; i < height; i++) {
	// 	for (int j = 0; j < width; j++) {
	// 		(*data)[i * width + j] = vec[i * width + j];
	// 	}
	// }

	// std::vector<std::vector<int>> matrix;
	// matrix.reserve(height);
	// for (int i = 0; i < height; i++) {
	// 	matrix.emplace_back(data.get() + i * width, data.get() + (i + 1) * width);
	// }

	// std::cout<<matrix[0].size()<<matrix.size()<<std::endl;

	auto binaryMatrix = matrix;

	int droneX = 1, droneY = 138;

	FrontGoal goal(binaryMatrix, droneX, droneY);

	// goal.DilateObstacle(2);

	auto next_goal = goal.findGoal();

	std::cout << "x: " << next_goal.x << " y: " << next_goal.y << " p: " << next_goal.priority << std::endl;

	auto goalX = next_goal.x;
	auto goalY = next_goal.y;

	Dijkstra Dijkstra(binaryMatrix);

	std::vector<std::pair<int, int>> path = Dijkstra.findShortestPath(droneX, droneY, goalX, goalY);

	for (auto wayptr : path)
	{
		std::cout << " x " << wayptr.first << " y " << wayptr.second << std::endl;
	}

	const int dimension = 3;
    std::vector<std::vector<float>> pos_waypt(dimension, std::vector<float>());

	for (auto pathptr: path){
		pos_waypt[0].push_back(pathptr.first);
		pos_waypt[1].push_back(pathptr.second);
		pos_waypt[2].push_back(7);
	}
	u_int32_t NumberOfWayPtr = path.size();
	std::cout << "NumberOfWayPtr " << NumberOfWayPtr<<std::endl;
    std::vector<std::vector<float>> vel_waypt(dimension, std::vector<float>(NumberOfWayPtr,0));
    std::vector<std::vector<float>> acc_waypt(dimension, std::vector<float>(NumberOfWayPtr,0));

	int count_tmp = 0;
	for (auto vel : vel_waypt){
		for (auto a : vel){
			std::cout<<a<<std::endl;
			count_tmp ++;
		}
	}
	std::cout<<count_tmp<<std::endl;
	// Point p1 = {2,3,1};
	// Point p2 = {1,2,2};
	// if (p2<p1){
	// 	std::cout<<"yes"<<std::endl;
	// }

	return 0;
}
