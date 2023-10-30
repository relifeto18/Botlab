#include <planning/motion_planner.hpp>
#include <planning/astar.hpp>
#include <utils/grid_utils.hpp>
#include <common_utils/timestamp.h>
#include <mbot_lcm_msgs/robot_path_t.hpp>
#include <cmath>


MotionPlanner::MotionPlanner(const MotionPlannerParams& params)
: params_(params)
{
    setParams(params);
}


MotionPlanner::MotionPlanner(const MotionPlannerParams& params, const SearchParams& searchParams)
: params_(params)
, searchParams_(searchParams)
{
}


mbot_lcm_msgs::robot_path_t MotionPlanner::planPath(const mbot_lcm_msgs::pose_xyt_t& start,
                                                     const mbot_lcm_msgs::pose_xyt_t& goal,
                                                     const SearchParams& searchParams) const
{
    // If the goal isn't valid, then no path can actually exist
    if(!isValidGoal(goal))
    {
        mbot_lcm_msgs::robot_path_t failedPath;
        failedPath.utime = utime_now();
        failedPath.path_length = 1;
        failedPath.path.push_back(start);

        std::cout << "INFO: path rejected due to invalid goal\n";        

        return failedPath;
    }
    
    // Otherwise, use A* to find the path
    return search_for_path(start, goal, distances_, searchParams);
}


mbot_lcm_msgs::robot_path_t MotionPlanner::planPath(const mbot_lcm_msgs::pose_xyt_t& start,
                                                     const mbot_lcm_msgs::pose_xyt_t& goal) const
{
    return planPath(start, goal, searchParams_);
}


bool MotionPlanner::isValidGoal(const mbot_lcm_msgs::pose_xyt_t& goal) const
{
    float dx = goal.x - prev_goal.x, dy = goal.y - prev_goal.y;
    float distanceFromPrev = std::sqrt(dx * dx + dy * dy);
    
    //if there's more than 1 frontier, don't go to a target that is within a robot diameter of the current pose
    // if(num_frontiers != 1 && distanceFromPrev < 2 * searchParams_.minDistanceToObstacle) return false;

    auto goalCell = global_position_to_grid_cell(Point<double>(goal.x, goal.y), distances_);

    // A valid goal is in the grid
    if(distances_.isCellInGrid(goalCell.x, goalCell.y))
    {
        // And is far enough from obstacles that the robot can physically occupy the space
        // Add an extra cell to account for discretization error and make motion a little safer by not trying to
        // completely snuggle up against the walls in the motion plan

        //std::cout << "DISTANCES = " << distances_(goalCell.x, goalCell.y) << " , radius = " << params_.robotRadius << std::endl;

        return distances_(goalCell.x, goalCell.y) > params_.robotRadius;
    }
    
    // A goal must be in the map for the robot to reach it
    return false;
}

bool MotionPlanner::isValidGoal(const Point<int>& goalCell) const
{
    // A valid goal is in the grid
    if(distances_.isCellInGrid(goalCell.x, goalCell.y))
    {
        // And is far enough from obstacles that the robot can physically occupy the space
        // Add an extra cell to account for discretization error and make motion a little safer by not trying to
        // completely snuggle up against the walls in the motion plan


        return distances_(goalCell.x, goalCell.y) > params_.robotRadius;
    }
    // A goal must be in the map for the robot to reach it
    return false;
}


bool MotionPlanner::isPathSafe(const mbot_lcm_msgs::robot_path_t& path) const
{

    ///////////// TODO: Implement your test for a safe path here //////////////////

    if (path.path.size() < 1) {
        return false;
    }

    //check if each node in path is within min distance
    for (size_t i = 0; i < path.path.size(); ++i)
    {
        mbot_lcm_msgs::pose_xyt_t pose = path.path[i];
        Point<double> pose_point;
        pose_point.x = pose.x;
        pose_point.y = pose.y;
        cell_t cell_pos = global_position_to_grid_cell(pose_point, distances_);

        if (distances_(cell_pos.x, cell_pos.y) < searchParams_.minDistanceToObstacle)
        {
            printf("%f, %f, too close to obstacle!\n", distances_(cell_pos.x, cell_pos.y), searchParams_.minDistanceToObstacle);
            return false;
        }
    }

    return true;


}


void MotionPlanner::setMap(const OccupancyGrid& map)
{
    distances_.setDistances(map);
}


void MotionPlanner::setParams(const MotionPlannerParams& params)
{
    searchParams_.minDistanceToObstacle = 0.14;//params_.robotRadius;//*1.4;
    searchParams_.maxDistanceWithCost = 5.0 * searchParams_.minDistanceToObstacle;
    searchParams_.distanceCostExponent = 1;

    searchParams_.maxIter = 5000;

}
