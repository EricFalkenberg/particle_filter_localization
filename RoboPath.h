#include <vector>


class MyPoint{

public:

    int x;

    int y;

    MyPoint(int, int);

    MyPoint(){x=0; y=0;}

};


std::vector<MyPoint> astar(double start_x, double start_y, double goal_x, double goal_y);

