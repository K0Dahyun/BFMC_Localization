#include "helper_functions.h"
#include "helper_structs.h"
#include "map.h"

#include <random>
#include <cmath>
#include <limits>
#include <array>
#include <algorithm>
#include <utility>

class ParticleFilter {
public:
    ParticleFilter();
    ~ParticleFilter();

    void init(Pose pose, float init_sigma[3], float sigma[4], float delta_t, int node_threshold, float on_lane_threshold, float stop_velocity_threshold, int num_particles, float front_length);
    Pose updateLocalization(Map* map, std::vector<Crosswalk> cross_walk_areas, Pose pose, Odometry odometry, Lane front_lane, Lane rear_lane, int direction);
    
    Pose movingModel(Pose pose_old, Odometry odometry, int direction);
    void prediction(Odometry odometry, int direction);
    void updateWeights(Map* map, Lane front_lane, Lane rear_lane);
    void resample();
    bool isDeadReckoning(Lane lane, std::vector<Crosswalk> cross_walk_areas);
    bool isOnCrossWalk(std::vector<Crosswalk> cross_walk_areas);
    Pose get_pose();
    Particle pf_moveToFront(Particle pf_center_pose);
    Particle pf_moveToRear(Particle pf_center_pose);
    Pose pose_moveToFront(Pose center_pose);
    float calculate_probability(Map* map, Particle pf_pose, Lane lane);

    bool get_dead_reckoning();
    std::vector<Particle> get_particle();
    std::vector<float> get_weight();

private:
    // Particle
    std::vector<Particle> particles;
    int num_particles;

    // Sigma
    float sigma_x;
    float sigma_y;
    float sigma_theta;
    float sigma_range;

    float delta_t;
    int node_threshold;
    float on_lane_threshold;
    float stop_velocity_threshold;

    float front_length;

    // Dead Recoding
    bool dead_reckoning;

    // Pose
    Pose estimated_pose;

    // Update Weights
    Pose pf_center_pose;
};