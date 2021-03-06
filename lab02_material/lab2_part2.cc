// General include files
#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include "math.h"
// Player related files
#include <libplayerc++/playerc++.h>
#include "cmdline_parsing.h"
#include "common_functions.h"
// Local library files
#include "lab2_shared.h"

using namespace PlayerCc;
using namespace std;

// This defines when we are "close enough"
#define DIST_EPS        0.05    // Corresponds to 5cm
#define ANGLE_EPS       0.01    // Corresponds to ~0.57 degree

#define X_SIZE          101
#define Y_SIZE          101
#define X_START         -2.5
#define Y_START         -2.5
#define GRID_UNIT_SIZE  0.05
#define LASER_MAX       8

#define FILENAME        "data.txt"

#define PROB_0          0.70
#define PROB_FREE       0.10
#define PROB_OCC        0.90


// Calculate log probablities
static const double PROB_L_0 = log(PROB_0/(1.0 - PROB_0));
static const double PROB_L_FREE = log(PROB_FREE/(1.0 - PROB_FREE));
static const double PROB_L_OCC = log(PROB_OCC/(1.0 - PROB_OCC));

void occupancy_grid_mapping(double **, const Pose &, const vector<LaserData> &);
double inverse_range_sensor_model(const Point &, const Pose &, const vector<LaserData> &);

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
    // Initialize occupancy grid to PROB_L_0
    double** grid = new double*[X_SIZE];
    grid[0] = new double[X_SIZE*Y_SIZE];
    for (unsigned int i = 1; i < X_SIZE; ++i)   {grid[i] = grid[0] + i*Y_SIZE;}
    for (unsigned int i = 0; i < X_SIZE; ++i)
        {for (unsigned int j = 0; j < Y_SIZE; ++j) {grid[i][j] = PROB_L_0;}}

    // Open file to save data in
    ofstream out_file(FILENAME);
    if (!out_file)
    {
        cerr << "Error: Couldn't open " << FILENAME << endl;
        exit(-1);
    }

    try
    {
        // Initialize connection to player
        PlayerClient robot(gHostname, gPort);
        Position2dProxy pp(&robot, gIndex);
        LaserProxy lp(&robot, gIndex);
        int num_attempts = 20;
        if(!check_robot_connection(robot, pp, num_attempts))
        {
            exit(-2);
        }

        // Start main processing loop
        while(true)
        {
            // Read from the proxies
            robot.Read();
            Pose robot_pose = Pose(pp.GetXPos(), pp.GetYPos(), pp.GetYaw());
            // Query the laserproxy to gather the laser scanner data
            unsigned int n = lp.GetCount();
            vector<LaserData> data(n);
            for(unsigned int i=0; i<n; i++)
            {
                data[i] = LaserData(lp.GetRange(i), lp.GetBearing(i));
            }
            // Check if close enough to destination and move to next point if yes
            if (curr_goal.distance_to(robot_pose) < DIST_EPS)
            {
                idx++;
                if (idx == num_points)  {break;} // Exit when done
                curr_goal = p_list[idx];
            }
            // Move towards current point
            
            // Check if too close to a wall, if not, proceed.
            if (!next_to_wall(data, MIN_WALL_DIST))
            {
                double r_dot, theta_dot;
                go_to_point(curr_goal.x, curr_goal.y, robot_pose.x, robot_pose.y,
                            robot_pose.theta, &r_dot, &theta_dot);
                pp.SetSpeed(r_dot, theta_dot);
                // Update the occupancy map
                occupancy_grid_mapping(grid, robot_pose, data);
                // Write out the occupancy map
                for (int i = 0; i < X_SIZE; ++i)
                {
                    for (int j = 0; j < Y_SIZE; ++j)
                    {
                        out_file << grid[i][j] << ", ";
                    }
                    out_file << endl;
                    out_file.flush();
                }
            }
            // Otherwise, stop so we don't hit anything.
            else {
                pp.SetSpeed(0.0, 0.0);
            }
        }
    }
    catch(PlayerError e)
    {
        write_error_details_and_exit(argv[0], e);
    }
    // Clean up
    pp.SetSpeed(0.0, 0.0);
    out_file.close();
    delete[] grid[0];
    delete[] grid;

    return 0;
}

void occupancy_grid_mapping(double **grid, const Pose &state, const vector<LaserData> &laser_data)
{
    // Iterate through all points in the grid
    double x = X_START;
    for (int i = 0; i < X_SIZE; ++i)
    {
        double y = Y_START;
        for (int j = 0; j < Y_SIZE; ++j)
        {
            // Get current point in grid
            Point curr_point = Point(x, y);
            // If the current point is in the perception range of the robot,
            // update its value in the occupancy grid.
            if (abs(curr_point.angle_to(state)) < PI/2)
            {
                grid[i][j] += inverse_range_sensor_model(curr_point, state, laser_data) - PROB_L_0;
            }
            y += GRID_UNIT_SIZE;
        }
        x += GRID_UNIT_SIZE;
    }
}

double inverse_range_sensor_model(const Point &grid_pt, const Pose &state, const vector<LaserData> &laser_data)
{
    // Go through list of laser data
    for (vector<LaserData>::const_iterator it = laser_data.begin(); it != laser_data.end(); it++)
    {
        double grid_pt_theta = grid_pt.angle_to(state);
        // If grid point on same line as laser data
        if (abs((*it).bearing - grid_pt_theta) < ANGLE_EPS)
        {
            double dist_from_robot = grid_pt.distance_to(state);
            // If grid location is the same as range, occupied
            if (abs(dist_from_robot - (*it).range) < DIST_EPS)
            {
                return PROB_L_OCC;
            }
            // If grid location is less than range, empty
            else if (dist_from_robot < (*it).range)
            {
                return PROB_L_FREE;
            }
            // Otherwise unknown
            else
            {
                return PROB_L_0;
            }
        }
    }
    return PROB_L_0;
}