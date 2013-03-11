// General include files
#include <vector>
// Player related files
#include <libplayerc++/playerc++.h>
#include "cmdline_parsing.h"
#include "common_functions.h"
// Local library files
#include "lab2_shared.h"

using namespace PlayerCc;
using namespace std;

// This defines when we are "close enough" to the goal point
#define DIST_EPS    0.05

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
        // Initialize connection to player
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
            if (curr_goal.distance_to(robot_x, robot_y) < DIST_EPS)
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
