// General include files
#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <math.h>
// Player related files
#include <libplayerc++/playerc++.h>
#include "cmdline_parsing.h"
#include "common_functions.h"

using namespace PlayerCc;
using namespace std;

#define PI 3.14159265358979323846

// This defines when we are "close enough" to the goal point
#define DIST_EPS    0.25

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
    double angle_to(const double x2, const double y2) const
    {
        return atan((y - y2)/(x - x2));
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
    double dtheta = goal.angle_to(robot_x, robot_y) - robot_theta;
    dtheta -= PI * (robot_x > goal_x);  // Correct on left half of plane
    dtheta += ((dtheta < -PI) - (dtheta > PI)) * 2 * PI; // Bound on [-PI, PI]
    // Don't move if more than 45 degrees off, otherwise scale speed with log of
    // angle but cap at dr to prevent overshoot
    *r_dot = (abs(dtheta) < PI/4) * dr * -log(abs(dtheta)) / 10.0;
    *r_dot = (dr < *r_dot) ? dr : *r_dot;
    // Turn angle is simply direction offset
    *theta_dot = dtheta;
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

int main(int argc, char **argv)
{
    // Calls the command line parser
    parse_args(argc, argv);

    // Get the list of points to go to
    vector<Point> p_list = read_points();
    int num_points = p_list.size();
    int idx = 0;
    // Get first point to go to, or exit if none
    if (num_points == 0)  {return 0;}   // Exit when done
    Point curr_goal = p_list[0];

    try
    {
        // Initialize connection to player //
        PlayerClient robot(gHostname, gPort);
        Position2dProxy pp(&robot, gIndex);
        LaserProxy lp(&robot, gIndex);
        int num_attempts = 20;
        if(!check_robot_connection(robot, pp, 20))
        {
            exit(-2);
        }
        // Start main processing loop
        while(true)
        {
            // Read from the proxies
            robot.Read();
            double robot_x = pp.GetXPos();
            double robot_y = pp.GetYPos();
            double robot_theta = pp.GetYaw();
            // Check if close enough to destination and move to next point if yes
            if (curr_goal.distance_to(robot_x, robot_y) < DIST_EPS) // TODO: SET ARBITRARILY
            {
                idx++;
                if (idx == num_points)  {break;} // Exit when done
                curr_goal = p_list[idx];
            }
            // Move towards current point
            double r_dot, theta_dot;
            go_to_point(curr_goal.x, curr_goal.y, robot_x, robot_y, robot_theta, &r_dot, &theta_dot);
            pp.SetSpeed(r_dot, theta_dot);
        }
    }
    catch(PlayerError e)
    {
        write_error_details_and_exit(argv[0], e);
    }

    return 0;
}

