#include <slam/mapping.hpp>
#include <utils/grid_utils.hpp>
#include <numeric>
#include <chrono> 
using namespace std::chrono; 

Mapping::Mapping(float maxLaserDistance, int8_t hitOdds, int8_t missOdds)
: kMaxLaserDistance_(maxLaserDistance)
, kHitOdds_(hitOdds)
, kMissOdds_(missOdds)
, initialized_(false)
{
}


void Mapping::updateMap(const mbot_lcm_msgs::lidar_t& scan,
                        const mbot_lcm_msgs::pose_xyt_t& pose,
                        OccupancyGrid& map)
{
    //////////////// TODO: Implement your occupancy grid algorithm here ///////////////////////
    if (!initialized_) {
        previousPose_ = pose;
    }

    MovingLaserScan movingScan(scan, previousPose_, pose);

    for (auto& ray : movingScan) {
        scoreEndpoint(ray, map);
    }

    for (auto& ray : movingScan) {
        scoreRay(ray, map);
    }

    initialized_ = true;
    previousPose_ = pose;
}

void Mapping::scoreEndpoint(const adjusted_ray_t& ray, OccupancyGrid& map)
{
    if (ray.range > kMaxLaserDistance_) {
        return;
    }

    Point<float> f_end = global_position_to_grid_position(
        Point<float>(
            ray.origin.x + ray.range * std::cos(ray.theta),
            ray.origin.y + ray.range * std::sin(ray.theta)
            ), 
        map
        );

    int x = static_cast<int>(f_end.x);
    int y = static_cast<int>(f_end.y);

    if (map.isCellInGrid(x, y)) {
        increaseCellOdds(x, y, map);
    }

//////////////// TODO: Implement your endpoint score ///////////////////////
}

void Mapping::scoreRay(const adjusted_ray_t& ray, OccupancyGrid& map)
{
//////////////// TODO: Implement your ray score ///////////////////////
    if (ray.range > kMaxLaserDistance_) {
        return;
    }

    std::vector<Point<int>> points = bresenham(ray, map);

    for (auto point : points) {
        if (map.isCellInGrid(point.x, point.y)) {
            decreaseCellOdds(point.x, point.y, map);
        }
    }
}

/*
Takes the ray and map, and returns a vector of map cells to check
*/
std::vector<Point<int>> Mapping::bresenham(const adjusted_ray_t& ray, const OccupancyGrid& map)
{
    // Get global positions 
    Point<float> f_end = global_position_to_grid_position(
        Point<float>(
            ray.origin.x + ray.range * std::cos(ray.theta),
            ray.origin.y + ray.range * std::sin(ray.theta)
            ), 
        map
        );

    // Cells
    Point<int> start_cell = global_position_to_grid_cell(ray.origin, map);
    Point<int> end_cell;
    end_cell.x = static_cast<int>(f_end.x);
    end_cell.y = static_cast<int>(f_end.y);
    std::vector<Point<int>> cells_touched;
    //////////////// TODO: Implement Bresenham's Algorithm ////////////////
    int x0 = start_cell.x;
    int y0 = start_cell.y;
    int x1 = end_cell.x;
    int y1 = end_cell.y;

    int dx = std::abs(x1 - x0);
    int dy = std::abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;
    int x = x0;
    int y = y0;

    while (x != x1 || y != y1) 
    {
        if (map.isCellInGrid(x, y))
        {
            cells_touched.push_back(Point<int>(x, y));
        }

        int e2 = 2 * err;
        if (e2 >= -dy)
        {
            err -= dy;
            x += sx;
        }
        if (e2 <= dx)
        {
            err += dx;
            y += sy;
        }
    }

    return cells_touched;
}

std::vector<Point<int>> Mapping::divideAndStepAlongRay(const adjusted_ray_t& ray, const OccupancyGrid& map)
{
    auto end_cell = global_position_to_grid_cell(Point<double>(
        ray.origin.x + ray.range * std::cos(ray.theta),
        ray.origin.y + ray.range * std::sin(ray.theta)
        ), map);
    //////////////// TODO: Implement divide and step ////////////////
    std::vector<Point<int>> cells_touched;
    return cells_touched;
}

void Mapping::increaseCellOdds(int x, int y, OccupancyGrid& map) {
    if (map(x, y) + kHitOdds_ < 127) {
        map(x, y) += kHitOdds_;
    } 
    else {
        map(x, y) = 127;
    }
}
void Mapping::decreaseCellOdds(int x, int y, OccupancyGrid& map) {
    if (map(x, y) - kHitOdds_ > -127) {
        map(x, y) -= kHitOdds_;
    } 
    else {
        map(x, y) = -127;
    }
}
