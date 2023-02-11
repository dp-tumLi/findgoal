#include <iostream>
#include <vector>
#include <list>
#include <algorithm>
#include <iterator>
#include <queue>

std::vector<int> find_neighbors(int x, int y,std::vector<std::vector<int>> binaryMatrix ){
    int max_x = binaryMatrix.size()-1;
    int max_y = binaryMatrix[x].size()-1;
    std::vector<int> list;
    
    for (int dx = (x>0? -1:0); dx <= (x<max_x ? 1:0); ++dx) {
        for (int dy = (y>0? -1:0); dy <= (y<max_y ? 1:0); ++dy) {
            
            if (dx != 0 || dy  != 0){
                if (binaryMatrix[x+dx][y+dy] == -1) {
                    list.push_back(x+dx);
                    list.push_back(y+dy);
                }
            }
        }
    }
    return list;
}

struct Point
{
    int x;
    int y;
    int priority;
};

bool operator<(const Point &a, const Point &b){
        return a.priority > b.priority;
}

Point findGoal(std::vector<std::vector<int>> binaryMatrix,int x, int y){
    std::priority_queue<Point> frontier;
    std::vector<std::vector<int> >::iterator row;
    std::vector<int>::iterator col;
    int max_x = binaryMatrix.size()-1;
    int max_y = binaryMatrix[x].size()-1;
    
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
                            // std::cout<<" x "<<x+dx<<" y "<<y+dy<<" pri "<<priority<<std::endl;
                            }
                        }
                    }
                }
            }
        }
    }
    Point goal = frontier.top();
    frontier.pop();

    return goal;
}    


int main(){

    std::vector<std::vector<int>> binaryMatrix = {  {0, 0, -1,  1, -1},
                                                    {0, 0, 0,  -1, -1},
                                                    {0, 0, 0,  -1, -1},
                                                    {1, 1, 1, 1,-1}};
    
    // std::cout<<binaryMatrix[1][1]<<std::endl;
    int droneX = 0;
    int droneY = 0;
    auto pt = findGoal(binaryMatrix, droneX, droneY);

    std::cout<<" x "<<pt.x<<" y "<<pt.y<<" pri "<<pt.priority<<std::endl;

    // std::vector<int> nb = find_neighbors(droneX, droneY, binaryMatrix);
    // std::copy(nb.begin(),
    //         nb.end(),
    //         std::ostream_iterator<int>(std::cout, "\n"));
}