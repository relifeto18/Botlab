#include <planning/frontiers.hpp>
#include <planning/motion_planner.hpp>
#include <utils/grid_utils.hpp>
#include <common_utils/timestamp.h>
#include <slam/occupancy_grid.hpp>
#include <mbot_lcm_msgs/robot_path_t.hpp>
#include <queue>
#include <set>
#include <cassert>


bool is_frontier_cell(int x, int y, const OccupancyGrid& map);
frontier_t grow_frontier(Point<int> cell, const OccupancyGrid& map, std::set<Point<int>>& visitedFrontiers);
mbot_lcm_msgs::robot_path_t path_to_frontier(const frontier_t& frontier,
                                              const mbot_lcm_msgs::pose_xyt_t& pose,
                                              const OccupancyGrid& map,
                                              const MotionPlanner& planner);
bool nearest_navigable_cell(mbot_lcm_msgs::pose_xyt_t pose,
                                                  Point<float> desiredPosition,
                                                  mbot_lcm_msgs::robot_path_t &plan,
                                                  const OccupancyGrid& map,
                                                  const MotionPlanner& planner);
mbot_lcm_msgs::pose_xyt_t search_to_nearest_free_space(Point<float> position,
                                                        const OccupancyGrid& map,
                                                        const MotionPlanner& planner);
double path_length(const mbot_lcm_msgs::robot_path_t& path);


std::vector<frontier_t> find_map_frontiers(const OccupancyGrid& map, 
                                           const mbot_lcm_msgs::pose_xyt_t& robotPose,
                                           double minFrontierLength)
{
    /*
    * To find frontiers, we use a connected components search in the occupancy grid. Each connected components consists
    * only of cells where is_frontier_cell returns true. We scan the grid until an unvisited frontier cell is
    * encountered, then we grow that frontier until all connected cells are found. We then continue scanning through the
    * grid. This algorithm can also perform very fast blob detection if you change is_frontier_cell to some other check
    * based on pixel color or another condition amongst pixels.
    */
    std::vector<frontier_t> frontiers;
    std::set<Point<int>> visitedCells;
    
    Point<int> robotCell = global_position_to_grid_cell(Point<float>(robotPose.x, robotPose.y), map);
    std::queue<Point<int>> cellQueue;
    cellQueue.push(robotCell);
    visitedCells.insert(robotCell);
  
    // Use a 4-way connected check for expanding through free space.
    const int kNumNeighbors = 4;
    const int xDeltas[] = { -1, 1, 0, 0 };
    const int yDeltas[] = { 0, 0, 1, -1 };
    
    // Do a simple BFS to find all connected free space cells and thus avoid unreachable frontiers
    while(!cellQueue.empty())
    {
        Point<int> nextCell = cellQueue.front();
        cellQueue.pop();
        
        // Check each neighbor to see if it is also a frontier
        for(int n = 0; n < kNumNeighbors; ++n)
        {
            Point<int> neighbor(nextCell.x + xDeltas[n], nextCell.y + yDeltas[n]);
            
            // If the cell has been visited or isn't in the map, then skip it
            if(visitedCells.find(neighbor) != visitedCells.end() || !map.isCellInGrid(neighbor.x, neighbor.y))
            {
                continue;
            }
            // If it is a frontier cell, then grow that frontier
            else if(is_frontier_cell(neighbor.x, neighbor.y, map))
            {
                frontier_t f = grow_frontier(neighbor, map, visitedCells);
                
                // If the frontier is large enough, then add it to the collection of map frontiers
                if(f.cells.size() * map.metersPerCell() >= minFrontierLength)
                {
                    frontiers.push_back(f);
                }
            }
            // If it is a free space cell, then keep growing the frontiers
            else if(map(neighbor.x, neighbor.y) < 0)
            {
                visitedCells.insert(neighbor);
                cellQueue.push(neighbor);
            }
        }
    }    
    return frontiers;
}

struct CompareCentroids
{
    CompareCentroids(mbot_lcm_msgs::pose_xyt_t robotPose) { this->robotPose = robotPose;}
    inline bool operator() (const Point<double>& centr_1, const Point<double>& centr_2)
    {
        // Diff 1
        float diff_1_x = robotPose.x - centr_1.x;
        float diff_1_y = robotPose.y - centr_1.y;
        float diff_1 = diff_1_x * diff_1_x + diff_1_y * diff_1_y;
        // Diff 2
        float diff_2_x = robotPose.x - centr_2.x;
        float diff_2_y = robotPose.y - centr_2.y;
        float diff_2 = diff_2_x * diff_2_x + diff_2_y * diff_2_y;

        return (diff_1 < diff_2);
    }
    mbot_lcm_msgs::pose_xyt_t robotPose;
};

bool cell_visited(Point<int> cell, std::vector<Point<int>> &cells) {
    for (int i = 0; i < cells.size(); ++i) {
        if (cell.x == cells[i].x && cell.y == cells[i].y) {
            return true;
        }
    }
    return false;
}


bool nearest_navigable_cell(mbot_lcm_msgs::pose_xyt_t pose,
                                                  Point<float> desiredPosition,
                                                  mbot_lcm_msgs::robot_path_t &path,
                                                  const OccupancyGrid& map,
                                                  const MotionPlanner& planner)
{
    //do a basic breadth-first-search startint at the desiredPosition
    //return first valid pose with safe path toward it

    mbot_lcm_msgs::pose_xyt_t desiredPose;
    desiredPose.x = desiredPosition.x;
    desiredPose.y = desiredPosition.y;

    std::queue<Point<int>> bfs;
    std::vector<Point<int>> cells;

    const int xDeltas[8] = {1, -1, 0, 0, 1, -1, 1, -1};
    const int yDeltas[8] = {0, 0, 1, -1, 1, -1, -1, 1};

    bfs.push(global_position_to_grid_cell(Point<float>(desiredPosition.x, desiredPosition.y), map));
    cells.push_back(global_position_to_grid_cell(desiredPosition, map));

    size_t max_iter = 5000;
    size_t iter = 0;

    while (!bfs.empty() && iter < max_iter) {
        cell_t test_cell = bfs.front();
        bfs.pop();

        //check if test is valid goal with safe path
        Point<float> test_point = grid_position_to_global_position(test_cell, map);
        mbot_lcm_msgs::pose_xyt_t test_pose;
        test_pose.x = test_point.x;
        test_pose.y = test_point.y;
        if (planner.isValidGoal(test_pose))
        {
            mbot_lcm_msgs::robot_path_t test_path = planner.planPath(pose, test_pose);
            if (planner.isPathSafe(test_path))
            {
                
                path = test_path;
                std::cout << "found path in vicinity" << std::endl;
                return true;
            }
        }

        //add neighbors to queue
        for (int n = 0; n < 8; ++n)
        {
           Point<int> test_cell2(test_cell.x + xDeltas[n], test_cell.y + yDeltas[n]);
            if (!cell_visited(test_cell2, cells)) {
                bfs.push(test_cell2);
                cells.push_back(test_cell2);
            }
        }

        iter++;
    }

    return false;

}


frontier_processing_t plan_path_to_frontier(const std::vector<frontier_t>& frontiers, 
                                            const mbot_lcm_msgs::pose_xyt_t& robotPose,
                                            const OccupancyGrid& map,
                                            const MotionPlanner& planner)
{
    ///////////// TODO: Implement your strategy to select the next frontier to explore here //////////////////
    /*
    * NOTES:
    *   - If there's multiple frontiers, you'll need to decide which to drive to.
    *   - A frontier is a collection of cells, you'll need to decide which one to attempt to drive to.
    *   - The cells along the frontier might not be in the configuration space of the robot, so you won't necessarily
    *       be able to drive straight to a frontier cell, but will need to drive somewhere close.
    */

    // First, choose the frontier to go to
    // Initial alg: find the nearest one

    // Returnable path
    mbot_lcm_msgs::robot_path_t path;
    path.utime = utime_now();
    path.path_length = 1;
    path.path.push_back(robotPose);
    int unreachable_frontiers = 0;

    printf("A\n");

    if (frontiers.size() == 0) {
        std::cout << "No frontiers" << std::endl;
        return frontier_processing_t(path, unreachable_frontiers);
    }
    frontier_t closest_frontier = frontiers[0];

    float min_dist = std::numeric_limits<float>::max();
    for (int f = 0; f < frontiers.size(); ++f) {
        bool unreachable = true;
        for (int c = 0; c < frontiers[f].cells.size(); ++c) {
            auto cell = frontiers[f].cells[c];
            if (map.isCellInGrid(cell.x, cell.y)) {
                unreachable = false;
            }
            float dist = std::sqrt(std::pow(cell.x - robotPose.x, 2) + std::pow(cell.y - robotPose.y, 2));
            if (dist < min_dist) {
                closest_frontier = frontiers[f];
                min_dist = dist;
            }
        }

        // if (unreachable) {
        //     unreachable_frontiers++;
        // }
    }
    printf("B\n");

    auto cell = closest_frontier.cells[closest_frontier.cells.size() / 2];

    mbot_lcm_msgs::robot_path_t test_path;
    bool found_nearby = nearest_navigable_cell(robotPose, cell, test_path, map, planner);
    if (found_nearby) {
        return frontier_processing_t(test_path, unreachable_frontiers);
    }

    printf("C\n");

    
    return frontier_processing_t(path, unreachable_frontiers);
}


bool is_frontier_cell(int x, int y, const OccupancyGrid& map)
{
    // A cell is a frontier if it has log-odds 0 and a neighbor has log-odds < 0
    
    // A cell must be in the grid and must have log-odds 0 to even be considered as a frontier
    if(!map.isCellInGrid(x, y) || (map(x, y) != 0))
    {
        return false;
    }
    
    const int kNumNeighbors = 4;
    const int xDeltas[] = { -1, 1, 0, 0 };
    const int yDeltas[] = { 0, 0, 1, -1 };
    
    for(int n = 0; n < kNumNeighbors; ++n)
    {
        // If any of the neighbors are free, then it's a frontier
        // Note that logOdds returns 0 for out-of-map cells, so no explicit check is needed.
        if(map.logOdds(x + xDeltas[n], y + yDeltas[n]) < 0)
        {
            return true;
        }
    }
    
    return false;
}


frontier_t grow_frontier(Point<int> cell, const OccupancyGrid& map, std::set<Point<int>>& visitedFrontiers)
{
    // Every cell in cellQueue is assumed to be in visitedFrontiers as well
    std::queue<Point<int>> cellQueue;
    cellQueue.push(cell);
    visitedFrontiers.insert(cell);
    
    // Use an 8-way connected search for growing a frontier
    const int kNumNeighbors = 8;
    const int xDeltas[] = { -1, -1, -1, 1, 1, 1, 0, 0 };
    const int yDeltas[] = {  0,  1, -1, 0, 1,-1, 1,-1 };
 
    frontier_t frontier;
    
    // Do a simple BFS to find all connected frontier cells to the starting cell
    while(!cellQueue.empty())
    {
        Point<int> nextCell = cellQueue.front();
        cellQueue.pop();
        
        // The frontier stores the global coordinate of the cells, so convert it first
        frontier.cells.push_back(grid_position_to_global_position(nextCell, map));
        
        // Check each neighbor to see if it is also a frontier
        for(int n = 0; n < kNumNeighbors; ++n)
        {
            Point<int> neighbor(nextCell.x + xDeltas[n], nextCell.y + yDeltas[n]);
            if((visitedFrontiers.find(neighbor) == visitedFrontiers.end()) 
                && (is_frontier_cell(neighbor.x, neighbor.y, map)))
            {
                visitedFrontiers.insert(neighbor);
                cellQueue.push(neighbor);
            }
        }
    }
    
    return frontier;
}

Point<double> find_frontier_centroid(const frontier_t& frontier)
{
    // Using the mid point of the frontier
    Point<double> mid_point;
    int index = (int)(frontier.cells.size() / 2.0);
    // printf("index: %d, size: %d\n", index, frontier.cells.size());
    mid_point = frontier.cells[index];
    printf("Mid point of frontier: (%f,%f)\n", mid_point.x, mid_point.y);

    return mid_point;
}
