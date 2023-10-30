#include <slam/sensor_model.hpp>
#include <slam/moving_laser_scan.hpp>
#include <slam/occupancy_grid.hpp>
#include <mbot_lcm_msgs/particle_t.hpp>
#include <utils/grid_utils.hpp>
#include <common_utils/geometric/point.hpp>
SensorModel::SensorModel(void)
:   ray_stride_(5)
{
}

double SensorModel::likelihood(const mbot_lcm_msgs::particle_t& sample, 
                               const mbot_lcm_msgs::lidar_t& scan, 
                               const OccupancyGrid& map)
{
    double scanLikelihood = 0;
    MovingLaserScan movingScan(scan, sample.parent_pose, sample.pose, ray_stride_);

    for (auto &&ray : movingScan)
    {
        Point<double> endpoint(ray.origin.x + ray.range * std::cos(ray.theta), ray.origin.y + ray.range * std::sin(ray.theta));

        auto rayEnd = global_position_to_grid_cell(endpoint, map);
        auto rayCost = map.logOdds(rayEnd.x, rayEnd.y);

        if (rayCost > 0)
        {
            scanLikelihood += rayCost;
        }
    }

    //printf("Likelihood = %f\n", scanLikelihood);

    return scanLikelihood;
}