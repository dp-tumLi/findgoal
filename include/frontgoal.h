#ifndef FRONT_GOAL_H
#define FRONT_GOAL_H

#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <algorithm>

namespace Front
{

    struct Point
    {
        int x;
        int y;
        int priority;
    };
    
    bool operator<(const Point &a, const Point &b){
            return a.priority > b.priority;
    }

    class FrontGoal{
    public:
    // constructor 
    FrontGoal(std::vector<std::vector<int>> binaryMatrix, int droneX, int droneY)
        :binaryMatrix_(binaryMatrix), droneX_(droneX), droneY_(droneY)
        {};
    
    // do dilation for all obstacles
    std::vector<std::vector<int>> DilateObstacle(const int DilationRadius){
        int rows = binaryMatrix_.size();
        int columns = binaryMatrix_[0].size();

        // Create a copy of the grid map
        std::vector<std::vector<int>> dilated_grid = binaryMatrix_;

        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < columns; j++) {
                // If the current cell is an obstacle
                if (binaryMatrix_[i][j] == 100) {

                    // This part makes all 8 grids around to be dilated
                    // Mark the neighboring cells as obstacles
                    for (int x = i - DilationRadius; x <= i + DilationRadius; x++) {
                        for (int y = j - DilationRadius; y <= j + DilationRadius; y++) {
                            if (x >= 0 && x < rows && y >= 0 && y < columns) {
                                dilated_grid[x][y] = 100;
                            }
                        }
                    }


                    // This part makes all 4 grids around to be dilated  
                    // if (i > 0) {
                    //     dilated_grid[i - 1][j] = 100;
                    // }
                    // if (i < rows - 1) {
                    //     dilated_grid[i + 1][j] = 100;
                    // }
                    // if (j > 0) {
                    //     dilated_grid[i][j - 1] = 100;
                    // }
                    // if (j < columns - 1) {
                    //     dilated_grid[i][j + 1] = 100;
                    // }
                }
            }
        }
        // test for output of dilated map
        // for (auto row : dilated_grid) {
        //     for (int cell : row) {
        //         std::cout << cell << " ";
        //     }
        //     std::cout << std::endl;
        // }


        return dilated_grid;
    }


    // method for finding the frontest goal at boundary betw. 0 and -1
    Point findGoal(std::vector<std::vector<int>> binaryMatrix){
        // binaryMatrix_ = DilateObstacle(DilationRadius_);
        std::priority_queue<Point> frontier;
        std::vector<std::vector<int> >::iterator row;
        std::vector<int>::iterator col;
        int max_x = binaryMatrix.size()-1;
        int max_y = binaryMatrix[droneX_].size()-1;
        
        for (row = binaryMatrix.begin(); row != binaryMatrix.end(); row ++) {
            for (col = row->begin(); col != row->end(); col ++) {
                if (*col == 0) {

                    int x = row - binaryMatrix.begin();
                    int y = col - row->begin();
                    for (int dx = (x>0? -1:0); dx <= (x<max_x ? 1:0); ++dx) {
                        for (int dy = (y>0? -1:0); dy <= (y<max_y ? 1:0); ++dy) {
                            if ((dx == 0 || dy  == 0) && (dx+dy!=0)){
                                if (binaryMatrix[x+dx][y+dy] == -1) {
                                    int priority = std::abs(dx) + std::abs(dy);
                                    frontier.push({x+dx,y+dy, priority});
                                    std::cout<<" x "<<x+dx<<" y "<<y+dy<<" pri "<<priority<<std::endl;
                                }
                            }
                        }
                    }
                }
            }
        }
        if (!frontier.empty()){
            Point goal = frontier.top();
            frontier.pop();
            return goal;    
        }
        else{
            Point goal = {droneX_,droneY_};
            return goal;
        }
    }  

    Point findGoal2(std::vector<std::vector<int>> binaryMatrix){
    // binaryMatrix_ = DilateObstacle(DilationRadius_);
    std::priority_queue<Point> frontier;
    std::vector<std::vector<int> >::iterator row;
    std::vector<int>::iterator col;
    int max_x = binaryMatrix.size()-1;
    int max_y = binaryMatrix[0].size()-1;
    int max_unknown = 0;
    double max_entropy = 0;
    Point max_point;

    uint32_t temp = 0;
    // Iterate through all potential goal locations
    for (int x = 0; x <= max_x; x++) {
        for (int y = 0; y <= max_y; y++) {
            if (binaryMatrix[x][y] == 0) {
                int num_unknown = 0;
                int num_free = 0;
                double entropy = 0;

                temp ++;
                // Count the number of unknown and free spaces in a 3x3 neighborhood around the current location
                for (int dx = -1; dx <= 1; dx++) {
                    for (int dy = -1; dy <= 1; dy++) {
                        if (x + dx >= 0 && x + dx <= max_x && y + dy >= 0 && y + dy <= max_y) {
                            if (binaryMatrix[x + dx][y + dy] == -1) {
                                num_unknown++;
                            } else if (binaryMatrix[x + dx][y + dy] == 0) {
                                num_free++;
                            }
                        }
                    }
                }

                // Calculate the entropy for the current location
                if (num_unknown > 0 && num_free > 0) {
                    double p_unknown = (double) num_unknown / (num_unknown + num_free);
                    double p_free = (double) num_free / (num_unknown + num_free);
                    entropy = -p_unknown * log2(p_unknown) - p_free * log2(p_free);
                }

                // Update the maximum entropy and maximum point if the current location has a higher entropy
                if (entropy > max_entropy) {
                    max_entropy = entropy;
                    max_point = {x, y};
                    max_unknown = num_unknown;
                    std::cout<<"current max_unknown: "<<x<<" "<<y<<" "<<num_unknown<<" count "<<temp<<std::endl;

                }
            }
        }
    }

    if (max_entropy > 0){
        std::cout<<" max_x "<<max_point.x<<" max_y "<<max_point.y<<" entropy "<<max_entropy<<" unknown "<<max_unknown<<std::endl;
        return max_point;
    }
    else{
        Point goal = {droneX_,droneY_};
        return goal;
    }
}  

bool isBoundary(const std::vector<std::vector<int>>& matrix, int i, int j)
{
    if (matrix[i][j] != -1) {
        return false;
    }
    if (i == 0 || i == matrix.size() - 1 || j == 0 || j == matrix[0].size() - 1) {
        return true;
    }
    if (matrix[i-1][j] == 100 || matrix[i+1][j] == 100 || matrix[i][j-1] == 100 || matrix[i][j+1] == 100) {
        return true;
    }
    return false;
}

Point findGoal3(const std::vector<std::vector<int>>& matrix, int x, int y)
{
    std::vector<std::vector<bool>> visited(matrix.size(), std::vector<bool>(matrix[0].size(), false));
    std::priority_queue<Point> pq;
    pq.push({x, y, 0});
    visited[x][y] = true;
    Point goal = {0, 0, 0};
    int maxLength = 0;
    while (!pq.empty()) {
        Point p = pq.top();
        pq.pop();
        if (isBoundary(matrix, p.x, p.y)) {
            int length = p.priority;
            if (length > maxLength) {
                maxLength = length;
                goal = {p.x, p.y, length};
            }
        }
        if (p.x > 0 && matrix[p.x-1][p.y] != 100 && !visited[p.x-1][p.y]) {
            visited[p.x-1][p.y] = true;
            pq.push({p.x-1, p.y, p.priority + 1});
        }
        if (p.x < matrix.size()-1 && matrix[p.x+1][p.y] != 100 && !visited[p.x+1][p.y]) {
            visited[p.x+1][p.y] = true;
            pq.push({p.x+1, p.y, p.priority + 1});
        }
        if (p.y > 0 && matrix[p.x][p.y-1] != 100 && !visited[p.x][p.y-1]) {
            visited[p.x][p.y-1] = true;
            pq.push({p.x, p.y-1, p.priority + 1});
        }
        if (p.y < matrix[0].size()-1 && matrix[p.x][p.y+1] != 100 && !visited[p.x][p.y+1]) {
            visited[p.x][p.y+1] = true;
            pq.push({p.x, p.y+1, p.priority + 1});
        }
    }
    return goal;
}


Point findGoal4(const std::vector<std::vector<int>>& matrix, int x, int y)
{
    std::vector<std::vector<bool>> visited(matrix.size(), std::vector<bool>(matrix[0].size(), false));
    std::priority_queue<Point> pq;
    pq.push({x, y, 0});
    visited[x][y] = true;

    std::vector<Point> boundaryPoints;

    while (!pq.empty()) {
        Point p = pq.top();
        pq.pop();
        if (isBoundary(matrix, p.x, p.y)) {
            boundaryPoints.push_back(p);
        }
        if (p.x > 0 && matrix[p.x-1][p.y] != 100 && !visited[p.x-1][p.y]) {
            visited[p.x-1][p.y] = true;
            pq.push({p.x-1, p.y, p.priority + 1});
        }
        if (p.x < matrix.size()-1 && matrix[p.x+1][p.y] != 100 && !visited[p.x+1][p.y]) {
            visited[p.x+1][p.y] = true;
            pq.push({p.x+1, p.y, p.priority + 1});
        }
        if (p.y > 0 && matrix[p.x][p.y-1] != 100 && !visited[p.x][p.y-1]) {
            visited[p.x][p.y-1] = true;
            pq.push({p.x, p.y-1, p.priority + 1});
        }
        if (p.y < matrix[0].size()-1 && matrix[p.x][p.y+1] != 100 && !visited[p.x][p.y+1]) {
            visited[p.x][p.y+1] = true;
            pq.push({p.x, p.y+1, p.priority + 1});
        }
    }

    if (boundaryPoints.empty()) {
        // No boundary points found
        return {x, y, 0};
    }

    // Sort boundary points by x-coordinate
    std::sort(boundaryPoints.begin(), boundaryPoints.end(), [](const Point& a, const Point& b) {
        return a.x < b.x;
    });

    // Find the longest segment
    int maxLength = 0;
    int startIndex = 0;
    for (int i = 0; i < boundaryPoints.size() - 1; i++) {
        int length = boundaryPoints[i+1].y - boundaryPoints[i].y;
        if (length > maxLength) {
            maxLength = length;
            startIndex = i;
        }
    }

    // Find the middle point of the longest segment
    int middleY = boundaryPoints[startIndex].y + maxLength / 2;
    return {boundaryPoints[startIndex].x, middleY, maxLength};
}

    private:
        std::vector<std::vector<int>> binaryMatrix_;
        int droneX_, droneY_;
        const int DilationRadius_ = 1;
    };
}
#endif

// FrontGoal(std::vector<std::vector<int>> binaryMatrix_, int M_, int N_, int droneX_, int droneY_);

// Point findGoal();
// Point findGoal(){
//     std::priority_queue<Point> frontier;
//     std::vector<std::vector<int> >::iterator row;
//     std::vector<int>::iterator col;

//     for (row = binaryMatrix_.begin(); row != binaryMatrix_.end(); row ++) {
//         for (col = row->begin(); col != row->end(); col ++) {
//             if (*col == 0) {
//                 int i = row - binaryMatrix_.begin();
//                 int j = col - row->begin();
                
//                 if ((i > 0 && binaryMatrix_[i-1][j] == -1) ||
//                     (i < M_-1 && binaryMatrix_[i+1][j] == -1) ||
//                     (j > 0 && binaryMatrix_[i][j-1] == -1) ||
//                     (j < N_-1 && binaryMatrix_[i][j+1] == -1)) {
//                     int priority = std::abs(i - droneX_) + std::abs(j - droneY_);
//                     frontier.push({i, j, priority});
//                 }
//             }
//         }
//     }
//     Point goal = frontier.top();
//     frontier.pop();

//     return goal;
// }