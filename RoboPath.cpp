/*

roboPath.cpp

MRP

/**/



#include <math.h>

#include <opencv2/opencv.hpp>

#include <unordered_set>

#include <unordered_map>

#include <map>

#include <vector>

#include <functional>

#include <queue>

#include <ostream>

#include <string>



using namespace cv;

using namespace std;



static float CONVFACTOR = 15.7855;



class MyPoint{

public:

    int x;

    int y;

    MyPoint(int, int);

    MyPoint(){x=0; y=0;}

};



class PointComp{

public:

    bool operator() (const MyPoint& lhs, const MyPoint& rhs) const{

        return (lhs.x < rhs.x) || ((lhs.x == rhs.x) && (lhs.y < rhs.y));

    }

};



namespace std{

    template<>

    class hash<MyPoint>{

    public:

        size_t operator()(const MyPoint& p) const{

            return ((51 + hash<int>()(p.x)) * 51 + hash<int>()(p.y));

        }

    };

    inline bool operator == (const MyPoint& lhs, const MyPoint& rhs){

        return (lhs.x == rhs.x) && (lhs.y == rhs.y);

    }

    inline bool operator != (const MyPoint& lhs, const MyPoint& rhs){

        return (lhs.x != rhs.x) || (lhs.y != rhs.y);

    }

}



MyPoint::MyPoint(int xVal, int yVal)

{

    x = xVal;

    y = yVal;

}



class Score{

public:

    float s;

    MyPoint p;

    Score(float, MyPoint);

};



Score::Score(float sVal, MyPoint pt)

{

    s = sVal;

    p = pt;

}



struct LessThanByScore

{

    bool operator()(const Score& lhs, const Score& rhs) const

    {

        return lhs.s < rhs.s;

    }

};



struct MoreThanByScore

{

    bool operator()(const Score& lhs, const Score& rhs) const

    {

        return lhs.s > rhs.s;

    }

};



/*

Method to find the slope between to given points

/**/

float slope(MyPoint p1, MyPoint p2)

{

    float dx = float(p2.x-p1.x);

    if(dx!=0)

    {

        return float(p2.y-p1.y)/dx;

    }

    else

    {

        return FLT_MAX;

    }

}



float distBetween(MyPoint p1, MyPoint p2)

{

    return pow(pow((p2.x-p1.x), 2)+pow((p2.y-p1.y), 2), 0.5);

}



/*

Method that compresses a path by merging nodes with the same slope from

predecessor to child

/**/

vector<MyPoint> path_compression(vector<MyPoint> path, Mat image)

{

    int i = 1;

    for(;;)

    {
        image.at<Vec3b>(Point(path[i].x, path[i].y))[2]=0;

        if(i>=path.size()-1){break;}



        // Keep the segment distane < 90 and make sure slopes are the same

        if(
            (fabs(slope(path[i-1], path[i])-slope(path[i], path[i+1])<.2)
                || (i+2<path.size() 
                && slope(path[i-1], path[i])==slope(path[i+1], path[i+2])))

           and distBetween(path[i-1], path[i+1])<=90)

        {

            path.erase(path.begin()+i);

            continue;

        }

        i+=1;

    }/**/

    for (auto temp : path)

    {
        temp.x = (temp.x-(int)image.cols/2);

        temp.y = ((int)image.rows/2)-temp.y;

        cout << temp.x << ", " << temp.y << endl;

    }
    imshow("result", image);
    waitKey(0);

    return path;

}



/*

Method that returns Manhattan Distance

/**/

float heuristic(MyPoint value, MyPoint goal)

{

    return abs(value.x - goal.x) + abs(value.y - goal.y);

}



vector<MyPoint> reconstruct_path(unordered_map<MyPoint, MyPoint, hash<MyPoint>> cameFrom, MyPoint curr, MyPoint start){

    if (curr != start){

        vector<MyPoint> p = reconstruct_path(cameFrom, cameFrom[curr], start);

        p.push_back(curr);

        return p;

    }

    else{

        vector<MyPoint> result;

        result.insert(result.begin(), curr);

        return result;

    }

}



/*

Method that checks each of the 8 adjacent positions to see if they are valid

returns all of the valid checked positions as neighbors

/**/

vector<MyPoint> neighbors(MyPoint node, Mat image)

{

    vector<MyPoint> nlist;

    //down-left

    if(image.at<Vec3b>(Point(node.x-1, node.y-1)).val[0]==255    

        and image.at<Vec3b>(Point(node.x-2, node.y-2)).val[0]==255

        and image.at<Vec3b>(Point(node.x-2, node.y-1)).val[0]==255

        and image.at<Vec3b>(Point(node.x-2, node.y)).val[0]==255  

        and image.at<Vec3b>(Point(node.x-1, node.y-2)).val[0]==255

        and image.at<Vec3b>(Point(node.x, node.y-2)).val[0]==255)

    {

        nlist.insert(nlist.begin(), MyPoint(node.x-1, node.y-1));

    }

    //left

    if(image.at<Vec3b>(Point(node.x-1, node.y)).val[0]==255     

        and image.at<Vec3b>(Point(node.x-2, node.y)).val[0]==255

        and image.at<Vec3b>(Point(node.x-2, node.y-1)).val[0]==255

        and image.at<Vec3b>(Point(node.x-2, node.y+1)).val[0]==255)

    {

        nlist.insert(nlist.begin(), MyPoint(node.x-1, node.y));

    }

    //up-left

    if(image.at<Vec3b>(Point(node.x-1, node.y+1)).val[0]==255  

        and image.at<Vec3b>(Point(node.x-2, node.y+2)).val[0]==255

        and image.at<Vec3b>(Point(node.x-2, node.y+1)).val[0]==255

        and image.at<Vec3b>(Point(node.x-2, node.y)).val[0]==255  

        and image.at<Vec3b>(Point(node.x-1, node.y+2)).val[0]==255

        and image.at<Vec3b>(Point(node.x, node.y+2)).val[0]==255)

    {

        nlist.insert(nlist.begin(), MyPoint(node.x-1, node.y+1));

    }

    //down

    if(image.at<Vec3b>(Point(node.x, node.y-1)).val[0]==255      

        and image.at<Vec3b>(Point(node.x, node.y-2)).val[0]==255  

        and image.at<Vec3b>(Point(node.x-1, node.y-2)).val[0]==255

        and image.at<Vec3b>(Point(node.x+1, node.y-2)).val[0]==255)

    {

        nlist.insert(nlist.begin(), MyPoint(node.x, node.y-1));

    }

    //down-right

    if(image.at<Vec3b>(Point(node.x+1, node.y-1)).val[0]==255   

        and image.at<Vec3b>(Point(node.x+2, node.y-2)).val[0]==255

        and image.at<Vec3b>(Point(node.x+2, node.y-1)).val[0]==255

        and image.at<Vec3b>(Point(node.x+2, node.y)).val[0]==255  

        and image.at<Vec3b>(Point(node.x+1, node.y-2)).val[0]==255

        and image.at<Vec3b>(Point(node.x, node.y-2)).val[0]==255)

    {

        nlist.insert(nlist.begin(), MyPoint(node.x+1, node.y-1));

    }

    //up-left

    if(image.at<Vec3b>(Point(node.x+1, node.y+1)).val[0]==255    

        and image.at<Vec3b>(Point(node.x+2, node.y+2)).val[0]==255

        and image.at<Vec3b>(Point(node.x+2, node.y+1)).val[0]==255

        and image.at<Vec3b>(Point(node.x+2, node.y)).val[0]==255  

        and image.at<Vec3b>(Point(node.x+1, node.y+2)).val[0]==255

        and image.at<Vec3b>(Point(node.x, node.y+2)).val[0]==255)

    {

        nlist.insert(nlist.begin(), MyPoint(node.x+1, node.y+1));

    }

    //up

    if(image.at<Vec3b>(Point(node.x+1, node.y)).val[0]==255     

        and image.at<Vec3b>(Point(node.x+2, node.y)).val[0]==255  

        and image.at<Vec3b>(Point(node.x+2, node.y+1)).val[0]==255

        and image.at<Vec3b>(Point(node.x+2, node.y-1)).val[0]==255)

    {

        nlist.insert(nlist.begin(), MyPoint(node.x+1, node.y));

    }

    //right

    if(image.at<Vec3b>(Point(node.x, node.y+1)).val[0]==255    

        and image.at<Vec3b>(Point(node.x, node.y+2)).val[0]==255  

        and image.at<Vec3b>(Point(node.x+1, node.y+2)).val[0]==255

        and image.at<Vec3b>(Point(node.x-1, node.y+2)).val[0]==255)

    {

        nlist.insert(nlist.begin(), MyPoint(node.x, node.y+1));

    }



    return nlist;

}





vector<MyPoint> astar(MyPoint start, MyPoint goal)

{

    Mat image;

    image = imread("roboMap.png", CV_LOAD_IMAGE_COLOR);

    if (!image.data)

    {

        cout << "No Image Found" << endl;

        return {};

    }



    start.x+=((int)image.cols/2);

    goal.x+=((int)image.cols/2);

    start.y = ((int)image.rows/2)-start.y;

    goal.y = ((int)image.rows/2)-goal.y;



    auto val = image.at<Vec3b>(Point(start.x, start.y));

    // If the start is inside of a wall, impossible to navigate

    if(val.val[0]!=255)

    {

        cout << "Start appears in wall" << endl;

        return {};

    }

    // If the goal is inside of a wall, impossible to reach

    val = image.at<Vec3b>(Point(goal.x, goal.y));

    if(val.val[0]!=255)

    {

        cout  << "End appears in wall" << endl;

        return {};

    }



    // TODO: declare some variables and stuff

    unordered_set<MyPoint> openset;

    unordered_set<MyPoint> closedset;

    unordered_map<MyPoint, float, hash<MyPoint>> f_score;

    unordered_map<MyPoint, float, hash<MyPoint>> g_score;

    unordered_map<MyPoint, MyPoint, hash<MyPoint>> cameFrom;



    g_score[start] = 0;

    f_score[start] = g_score[start]+heuristic(start, goal);



    priority_queue<Score, vector<Score>, MoreThanByScore> openQueue;

    openQueue.push(Score(f_score[start], start));

    openset.insert(openset.begin(), start);



    while(!openset.empty())

    {

        MyPoint curr = (openQueue.top()).p;

        openQueue.pop();



        auto it = openset.find(curr);

        openset.erase(it);

        closedset.insert(closedset.begin(), curr);

        auto nlist = neighbors(curr, image);



        for (auto n : nlist)

        {

            //cout << n.x << ", " << n.y << endl;

            // Skip if in the closedset

            if (closedset.find(n)!=closedset.end())

            {

                continue;

            }



            auto tentative_g_score = g_score[curr]+distBetween(curr, n);



            if (closedset.find(n)==closedset.end()

                ||

                tentative_g_score<g_score[n])

            {

                cameFrom[n]=curr;

                g_score[n]=tentative_g_score;

                f_score[n]=3.1*g_score[n] + 3*heuristic(n, goal);

                if (openset.find(n)==openset.end())

                {

                    openset.insert(openset.begin(), n);

                    openQueue.push(Score(f_score[n], n));

                }

            }



            if (n == goal)

            {

                //TODO: Compress and return path

                auto ret = reconstruct_path(cameFrom, n, start);

                return path_compression(ret, image);

            }

        }

    }



    return {};

}



int main()

{

    auto shit = astar(MyPoint(int(10.8*CONVFACTOR), int(12.7*CONVFACTOR)), MyPoint(int(-54.5*CONVFACTOR), int(7.6*CONVFACTOR)));

}