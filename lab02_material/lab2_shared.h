// This file contains a struct that holds laser scan data

// Include guard
#ifndef __LAB2_SHARED__
#define __LAB2_SHARED__

// General include files
#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <math.h>

#define PI 3.14159265358979323846

// Include namespaces for convenience in code writing
using namespace std;

// This object stores the (x,y) coordinates of a point
struct Point
{
    double x;
    double y;
    Point() : x(0), y(0)  {}
    Point(const double x, const double y) : x(x), y(y)  {}
    double distance_to(const Point p2) const
    {
        return distance_to(p2.x, p2.y);
    }
    double distance_to(const double x2, const double y2) const
    {
        double dx = x - x2;
        double dy = y - y2;
        return sqrt(dx * dx + dy * dy);
    }
    // This function returns the angle between two points.
    double angle_to(const double x2, const double y2) const
    {
        return atan((y - y2)/(x - x2));
    }
    // This function returns the angle between two points. When theta2 is given,
    // it interprets the point given as the location of the robot and gives the
    // relative angle toward *this.
    double angle_to(const double x2, const double y2, const double theta2) const
    {
        double dtheta = angle_to(x2, y2) - theta2;
        dtheta -= PI * (x2 > x);  // Correct on left half of plane
        dtheta += ((dtheta < -PI) - (dtheta > PI)) * 2 * PI; // Bound on [-PI, PI]
        return dtheta;
    }
};
// This function prints out a Point as [x, y]
ostream& operator<<(ostream& os, const Point& p)
{
    os << "[" << p.x << ", " << p.y << "]";
    return os;
}
// This function reads in a Point given as 2 doubles separated by whitespace
// It is assumed that there is a new line separator at the end
istream& operator>>(istream& s, Point& p)
{
    // initialize components of Point to read in
    double x = 0, y = 0;
    if (!s)     {return s;}     // stream invalid, just return fail state
    s >> x >> y;                // Get data from stream
    p = Point(x, y);            // Everything valid, create DoublePoint
    return s;                   // Return stream
}

// This function implements a go to function based on inputs of where the robot
// destination is and the current pose. It returns the direction and speed to go
// by reference.
int go_to_point(double goal_x, double goal_y,
                double robot_x, double robot_y, double robot_theta,
                double* r_dot, double* theta_dot)
{
    Point goal = Point(goal_x, goal_y); // Turn into point for convenience
    // Get distance and direction to goal
    double dr = goal.distance_to(robot_x, robot_y);
    double dtheta = goal.angle_to(robot_x, robot_y, robot_theta);
    // Don't move if more than 22.5 degrees off, otherwise scale speed with log
    // of angle but cap at dr to prevent overshoot
    *r_dot = (abs(dtheta) < PI/8) * dr * -log(abs(dtheta)) / 10.0;
    *r_dot = (dr < *r_dot) ? dr : *r_dot;
    // Turn angle is simply direction offset
    *theta_dot = dtheta / 2;    // Slow rotation to preven overshoot
    return 0;   // Dummy return value
}

// Read in points from a given filename
vector<Point> read_points(string filename = "goal_points.txt")
{
    ifstream in_file(filename.c_str());
    if (!in_file)
    {
        cerr << "Error: Couldn't open " << filename.c_str() << endl;
        exit(-1);
    }
    // Read in all the points from the feature list
    vector<Point> p_list;
    std::stringstream s;
    s << in_file.rdbuf();
    while (!(s.rdstate() & ios_base::eofbit))
    {
        Point dp;
        s >> dp;
        p_list.push_back(dp);
    }
    p_list.pop_back(); // Account for extra newline in data file
    in_file.close();
    return p_list;
}

#endif // __LAB2_SHARED__
