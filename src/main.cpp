#include <iostream>
#include <fstream>
#include "../include/frontgoal.h"
#include "../include/wayptrgenerator.h"
#include <memory>
#include <vector>
#include <cstring>
using namespace Front;
void writeMatrixToCsvFile(const std::vector<std::vector<int>>& matrix, const std::string& filename);

int main(int argc, char *argv[])
{
	// std::vector<std::vector<int>> binaryMatrix = {{0, 0, 0, 100, 100, 100, -1, -1, -1, -1},
	// 											{0, 0, 0, 0, 0, 100, -1, -1, -1, -1},
	// 											{0, 0, 0, 0, 0, 100, -1, -1, -1, -1},
	// 											{100, 0, 0, 0, 0, 0, -1, -1, -1, -1},
	// 											{100, 100, 0, 0, 0, 0, -1, -1, -1, -1},
	// 											{100, 100, 0, 0, 0, 0, -1, -1, -1, -1},
	// 											{-1, -1, 100, 100, 0, 100, -1, -1, -1, -1},
	// 											{-1, -1, 0, 0, 100, 100, -1, -1, -1, -1},
	// 											};
				

	std::ifstream mymap;
	std::string filename = "map_res0.5.txt";
	mymap.open(filename.c_str());

	const int height = 42;
	const int width = 49;
	int num = -1;
	std::vector<int> vec;

	if (mymap.is_open())
	{
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
	
    writeMatrixToCsvFile(matrix, "matrix.csv");

	auto binaryMatrix = matrix;

	// from now on

	// for (int i = 0; i < binaryMatrix.size(); i++) {
	// 	for (int j = 0; j < binaryMatrix[i].size(); j++) {
	// 	std::cout << binaryMatrix[i][j] << "   ";
	// 	}
	// 	std::cout << std::endl;
	// }

	int droneX = 8, droneY = 16;

	FrontGoal goal(binaryMatrix, droneX, droneY);

	const int radius = 0;
	auto map_processed = goal.DilateObstacle(radius);
	// auto next_goal = goal.findGoal2(map_processed);
	auto next_goal = goal.findGoal4(map_processed, droneX, droneY);


	writeMatrixToCsvFile(map_processed, "matrix1.csv");

	// for (auto row : map_processed) {
	// 	for (int cell : row) {
	// 		std::cout << cell << "    ";
	// 	}
	// 	std::cout << std::endl;
	// }

	std::cout << "x: " << next_goal.x + 1<< " y: " << next_goal.y + 1<< " p: " << next_goal.priority << std::endl;

	auto goalX = next_goal.x;
	auto goalY = next_goal.y;
	// auto goalX = 74;
	// auto goalY = 0;
	Dijkstra Dijkstra(map_processed);

	Dijkstra.VerifyStart(droneX, droneY);

	std::vector<std::pair<int, int>> path = Dijkstra.findShortestPath(droneX, droneY, goalX, goalY);
	
	Dijkstra.VerifyPath(path);

	Dijkstra.CheckIfPathCollision(path, map_processed);

	for (auto wayptr : path)
	{
		std::cout << " x " << wayptr.first+1 << " y " << wayptr.second+1 << std::endl;
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
			// std::cout<<a<<std::endl;
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

void writeMatrixToCsvFile(const std::vector<std::vector<int>>& matrix, const std::string& filename)
{
    std::ofstream file(filename);
    if (file.is_open()) {
        for (const auto& row : matrix) {
            for (size_t i = 0; i < row.size(); i++) {
                file << row[i];
                if (i != row.size() - 1) {
                    file << ",";
                }
            }
            file << "\n";
        }
        file.close();
    }
    else {
        std::cerr << "Unable to open file: " << filename << std::endl;
    }
}