#include <slam/action_model.hpp>
#include <mbot_lcm_msgs/particle_t.hpp>
#include <common_utils/geometric/angle_functions.hpp>
#include <cassert>
#include <cmath>
#include <iostream>
#include <algorithm>


ActionModel::ActionModel(void)
: k1_(0.02)//0.02
, k2_(0.001)//0.001
, min_dist_(0.0025)
, min_theta_(0.02)
, initialized_(false)
{
    //////////////// TODO: Handle any initialization for your ActionModel /////////////////////////
    std::random_device rd;
    numberGenerator_ = std::mt19937(rd());

}


void ActionModel::resetPrevious(const mbot_lcm_msgs::pose_xyt_t& odometry)
{
    previousPose_ = odometry;
}


bool ActionModel::updateAction(const mbot_lcm_msgs::pose_xyt_t& odometry)
{
    ////////////// TODO: Implement code here to compute a new distribution of the motion of the robot ////////////////
    if (!initialized_)
    {
        previousPose_ = odometry;
        initialized_ = true;
    }
    direction_ = 1;
    float deltaX = odometry.x - previousPose_.x;
    float deltaY = odometry.y - previousPose_.y;
    float deltaTheta = angle_diff(odometry.theta, previousPose_.theta);

    trans_ = std::sqrt(deltaX * deltaX + deltaY * deltaY);
    rot1_ = angle_diff(std::atan2(deltaY, deltaX), previousPose_.theta);
    if (std::abs(rot1_) > M_PI_2)
    {
        rot1_ = angle_diff(M_PI, rot1_);
        direction_ = -1;
    }

    rot2_ = angle_diff(deltaTheta, rot1_);
    bool moved = (deltaX != 0) || (deltaY != 0) || (deltaTheta != 0);

    if (moved)
    {
        rot1Std_ = std::sqrt(k1_ * std::abs(rot1_));
        transStd_ = std::sqrt(k2_ * std::abs(trans_));
        rot2Std_ = std::sqrt(k1_ * std::abs(rot2_));
    }
    utime_ = odometry.utime;
    previousPose_ = odometry;
    trans_ *= direction_;

    return moved;
}

mbot_lcm_msgs::particle_t ActionModel::applyAction(const mbot_lcm_msgs::particle_t& sample)
{
    ////////////// TODO: Implement your code for sampling new poses from the distribution computed in updateAction //////////////////////
    // Make sure you create a new valid particle_t. Don't forget to set the new time and new parent_pose.
    mbot_lcm_msgs::particle_t newSample = sample;
    float sampleRot1 = std::normal_distribution<>(rot1_, rot1Std_)(numberGenerator_);
    float sampleRot2 = std::normal_distribution<>(rot2_, rot2Std_)(numberGenerator_);
    float sampleTrans = std::normal_distribution<>(trans_, transStd_)(numberGenerator_);

    newSample.pose.x += sampleTrans * std::cos(sampleRot1 + sample.pose.theta);
    newSample.pose.y += sampleTrans * std::sin(sampleRot1 + sample.pose.theta);
    newSample.pose.theta = wrap_to_pi(sample.pose.theta + sampleRot1 + sampleRot2);
    newSample.pose.utime = utime_;
    newSample.parent_pose = sample.pose;

    return newSample;
}
