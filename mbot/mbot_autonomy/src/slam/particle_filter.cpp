#include <utils/grid_utils.hpp>
#include <slam/particle_filter.hpp>
#include <slam/occupancy_grid.hpp>
#include <mbot_lcm_msgs/pose_xyt_t.hpp>
#include <mbot_lcm_msgs/particle_t.hpp>
#include <cassert>


ParticleFilter::ParticleFilter(int numParticles)
: kNumParticles_ (numParticles),
  samplingAugmentation(0.5, 0.9, numParticles),
  distribution_quality(1),
  quality_reinvigoration_percentage(0.1)
{
    assert(kNumParticles_ > 1);
    posterior_.resize(kNumParticles_);
}


void ParticleFilter::initializeFilterAtPose(const mbot_lcm_msgs::pose_xyt_t& pose)
{
    ///////////// TODO: Implement your method for initializing the particles in the particle filter /////////////////
    float sampleWeight = 1.0 / kNumParticles_;

    posteriorPose_ = pose;

    for (auto &p : posterior_)
    {
        p.pose.x = pose.x;
        p.pose.y = pose.y;
        p.pose.theta = pose.theta;
        p.pose.utime = pose.utime;
        p.parent_pose = p.pose;

        p.weight = sampleWeight;
    }
}

void ParticleFilter::initializeFilterRandomly(const OccupancyGrid& map)
{
    ///////////// TODO: Implement your method for initializing the particles in the particle filter /////////////////
}

void ParticleFilter::resetOdometry(const mbot_lcm_msgs::pose_xyt_t& odometry)
{
    actionModel_.resetPrevious(odometry);
}


mbot_lcm_msgs::pose_xyt_t ParticleFilter::updateFilter(const mbot_lcm_msgs::pose_xyt_t& odometry,
                                                        const mbot_lcm_msgs::lidar_t& laser,
                                                        const OccupancyGrid& map)
{
    bool hasRobotMoved = actionModel_.updateAction(odometry);

    if (hasRobotMoved)
    {
        auto prior = resamplePosteriorDistribution(); //THIS
        auto proposal = computeProposalDistribution(prior);
        posterior_ = computeNormalizedPosterior(proposal, laser, map); //THIS
        posteriorPose_ = estimatePosteriorPose(posterior_); //THIS
    }

    posteriorPose_.utime = odometry.utime;

    return posteriorPose_;
}

mbot_lcm_msgs::pose_xyt_t ParticleFilter::updateFilterActionOnly(const mbot_lcm_msgs::pose_xyt_t& odometry)
{
    // Only update the particles if motion was detected. If the robot didn't move, then
    // obviously don't do anything.
    bool hasRobotMoved = actionModel_.updateAction(odometry);

    if (hasRobotMoved)
    {
        // auto prior = resamplePosteriorDistribution();
        auto proposal = computeProposalDistribution(posterior_);
        posterior_ = proposal;
    }

    posteriorPose_ = odometry;

    return posteriorPose_;
}



mbot_lcm_msgs::pose_xyt_t ParticleFilter::poseEstimate(void) const
{
    return posteriorPose_;
}


mbot_lcm_msgs::particles_t ParticleFilter::particles(void) const
{
    mbot_lcm_msgs::particles_t particles;
    particles.num_particles = posterior_.size();
    particles.particles = posterior_;
    return particles;
}


ParticleList ParticleFilter::resamplePosteriorDistribution(const OccupancyGrid* map)
{
    //////////// TODO: Implement your algorithm for resampling from the posterior distribution ///////////////////
    ParticleList prior;
    prior.reserve(kNumParticles_);
    float M_inv = 1/(float)kNumParticles_;
    std::random_device rd;
    std::mt19937 generator(rd());
    std::uniform_real_distribution<> dist(0.0, M_inv);
    float r = dist(generator);
    float c = posterior_[0].weight; // Should this be posterior?
    int i = 0;
    for(int m = 0; m < kNumParticles_; ++m){
        float U = r+m*M_inv;
        while (U > c){
            ++i;
            c = c + posterior_[i].weight; // Should this be posterior?
        }
        prior.push_back(posterior_[i]); // Should this be posterior?
    }

    return prior;
}


ParticleList ParticleFilter::computeProposalDistribution(const ParticleList& prior)
{
    //////////// TODO: Implement your algorithm for creating the proposal distribution by sampling from the ActionModel
    ParticleList proposal;
    for (auto &p : prior)
    {
        proposal.push_back(actionModel_.applyAction(p));
    }
    return proposal;
}


ParticleList ParticleFilter::computeNormalizedPosterior(const ParticleList& proposal,
                                                        const mbot_lcm_msgs::lidar_t& laser,
                                                        const OccupancyGrid& map)
{
    /////////// TODO: Implement your algorithm for computing the normalized posterior distribution using the
    ///////////       particles in the proposal distribution
    ParticleList posterior;
    double sumWeight = 0.0;

    double minWeight = 0.001;

    for(auto &p : proposal){
        mbot_lcm_msgs::particle_t weighted = p;
        float weight = sensorModel_.likelihood(weighted, laser, map);
        // if (weight < 0.001) {
        //     weight = minWeight;
        // }
        weighted.weight = weight;
        sumWeight += weighted.weight;
        posterior.push_back(weighted);
    }
    
    //normalization
    for (auto &&p : posterior){
        p.weight /= sumWeight;
    }
    return posterior;
}


mbot_lcm_msgs::pose_xyt_t ParticleFilter::estimatePosteriorPose(const ParticleList& posterior)
{
    //////// TODO: Implement your method for computing the final pose estimate based on the posterior distribution
    mbot_lcm_msgs::pose_xyt_t pose;

    //pose = computeParticlesAverage(posterior);

    ParticleList posteriorSort = posterior;
    std::sort(posteriorSort.begin(), posteriorSort.end(), [](const mbot_lcm_msgs::particle_t & a, const mbot_lcm_msgs::particle_t & b) -> bool
    { 
        return a.weight > b.weight;
    });
    float percentCutoff = 0.2;//top x percent
    int index = static_cast<int>(percentCutoff * kNumParticles_);
    ParticleList particles(posteriorSort.begin(), posteriorSort.begin() + index);
    pose = computeParticlesAverage(particles);


    return pose;
}

mbot_lcm_msgs::pose_xyt_t ParticleFilter::computeParticlesAverage(const ParticleList& particles_to_average)
{
    //////// TODO: Implement your method for computing the average of a pose distribution
    mbot_lcm_msgs::pose_xyt_t avg_pose;

    double xMean = 0;
    double yMean = 0;
    double cosThetaMean = 0;
    double sinThetaMean = 0;
    double weightSum = 0;
    for (int i = 0; i < particles_to_average.size(); i++)
    {
        //const particle_t* p = &posteriorSort[i];
        xMean += particles_to_average[i].weight * particles_to_average[i].pose.x;
        yMean += particles_to_average[i].weight * particles_to_average[i].pose.y;
        cosThetaMean += particles_to_average[i].weight * std::cos(particles_to_average[i].pose.theta);
        sinThetaMean += particles_to_average[i].weight * std::sin(particles_to_average[i].pose.theta);

        weightSum += particles_to_average[i].weight;
    }
    avg_pose.x = xMean / weightSum;
    avg_pose.y = yMean / weightSum;
    avg_pose.theta = std::atan2(sinThetaMean / weightSum, cosThetaMean / weightSum);
    return avg_pose;

}
