#include "particle_filter.h"
#include "helper_structs.h"

#include <limits>
#include <random>

#include <iomanip>

// Create only once the default random engine
static std::default_random_engine gen;

ParticleFilter::ParticleFilter() {
    this->num_particles = 0;

    this->sigma_x = 0.0;
    this->sigma_y = 0.0;
    this->sigma_theta = 0.0;
    this->sigma_range = 0.0;

    this->delta_t = 0.0;
    this->node_threshold = 0;

    this->front_length = 0.0;
    
    this->dead_reckoning = false;
}

ParticleFilter::~ParticleFilter() {
}

void ParticleFilter::init(Pose init_pose, float init_sigma[3], float sigma[4], float delta_t, int node_threshold, float on_lane_threshold, float stop_velocity_threshold, int num_particles, float front_length) {
    this->sigma_x = sigma[0];
    this->sigma_y = sigma[1];
    this->sigma_theta = sigma[2];
    this->sigma_range = sigma[3];

    this->delta_t = delta_t;
    this->node_threshold = node_threshold;
    this->on_lane_threshold = on_lane_threshold;
    this->stop_velocity_threshold = stop_velocity_threshold;
    this->num_particles = num_particles;

    this->front_length = front_length;

    // Create normal Gaussian distribution for p_x, p_y p_theta
    std::normal_distribution<float> dist_x(0, init_sigma[0]);
    std::normal_distribution<float> dist_y(0, init_sigma[1]);
    std::normal_distribution<float> dist_theta(init_pose.theta, init_sigma[2]);

    // Create new this->particles
    for(int i = 0; i < num_particles; ++i) {
        Particle p;
        float dist_rel_x = dist_x(gen);
        float dist_rel_y = dist_y(gen);

        p.x = init_pose.x + (dist_rel_x * cos(init_pose.theta) - dist_rel_y * sin(init_pose.theta));
        p.y = init_pose.y + (dist_rel_x * sin(init_pose.theta) + dist_rel_y * cos(init_pose.theta));
        p.theta = normalize_pi_to_pi(dist_theta(gen));
        p.weight = 1.0;

        this->particles.push_back(p);
    }
}

Pose ParticleFilter::updateLocalization(Map* map, std::vector<Crosswalk> cross_walk_areas, Pose pose, Odometry odometry, Lane front_lane, Lane rear_lane, int direction) {
    prediction(odometry, direction);
    updateWeights(map, front_lane, rear_lane);

    if(!isDeadReckoning(front_lane, cross_walk_areas)) {
        // Particle Filtering 
        this->estimated_pose = get_pose();
    }
    else {
        // Dead Reckoning
        this->dead_reckoning = true;
        this->estimated_pose = movingModel(pose, odometry, direction);
    }
    resample();

    return this->estimated_pose;
}

Pose ParticleFilter::movingModel(Pose pose_old, Odometry odometry, int direction){
    Pose pose_new;

    pose_new.theta = normalize_pi_to_pi(pose_old.theta + odometry.yawrate * delta_t);
    pose_new.x = pose_old.x + (direction * odometry.velocity * cos(pose_old.theta)) * delta_t;
    pose_new.y = pose_old.y + (direction * odometry.velocity * sin(pose_old.theta)) * delta_t;

    return pose_new;
} 

void ParticleFilter::prediction(Odometry odometry, int direction){
    Pose pf_pose_old, pf_pose_new;

    // Update particle
    for(int i = 0; i < num_particles; ++i) {
        pf_pose_old.x = this->particles[i].x;
        pf_pose_old.y = this->particles[i].y;
        pf_pose_old.theta = this->particles[i].theta;

        pf_pose_new = movingModel(pf_pose_old, odometry, direction);
                
        // Initialize normal distributions centered on predicted values
        std::normal_distribution<float> dist_x(0, this->sigma_x);
        std::normal_distribution<float> dist_y(0, this->sigma_y);
        std::normal_distribution<float> dist_theta(pf_pose_new.theta, this->sigma_theta);

        // Update particle with noisy prediction
        float dist_rel_x = dist_x(gen);
        float dist_rel_y = dist_y(gen);
        this->particles[i].x = pf_pose_new.x + (dist_rel_x * cos(pf_pose_new.theta) - dist_rel_y * sin(pf_pose_new.theta));
        this->particles[i].y = pf_pose_new.y + (dist_rel_x * sin(pf_pose_new.theta) + dist_rel_y * cos(pf_pose_new.theta));
        this->particles[i].theta = normalize_pi_to_pi(dist_theta(gen));
    }
}

 void ParticleFilter::updateWeights(Map* map, Lane front_lane, Lane rear_lane) {
    // Set parameter
    float front_probability = 1.0;
    float rear_probability = 1.0;
    float total_weights = 0.0;
     
    for(int i = 0; i < num_particles; ++i) {
        Particle pf_front_pose = pf_moveToFront(particles[i]);
        Particle pf_rear_pose = pf_moveToRear(particles[i]);

        front_probability = calculate_probability(map, pf_front_pose, front_lane);
        rear_probability = calculate_probability(map, pf_rear_pose, rear_lane);

        this->particles[i].weight = front_probability * rear_probability;
        total_weights += this->particles[i].weight;
    }

    // Weight Normalization
    for(int i = 0; i < num_particles; ++i) {
        this->particles[i].weight /= (total_weights + std::numeric_limits<float>::epsilon());
    }
}

void ParticleFilter::resample() {
    std::vector<Particle> resampled_particles;
    std::vector<double> weights;
    int chosen_index = 0;

    for (int i = 0; i < num_particles; ++i) {
        weights.push_back(this->particles[i].weight);
    }
    std::discrete_distribution<int> weight_distribution(weights.begin(), weights.end());

    for (int i = 0; i < num_particles; ++i) {
        chosen_index = weight_distribution(gen);
        resampled_particles.push_back(this->particles[chosen_index]);
    }

    this->particles = resampled_particles;
}

bool ParticleFilter::isDeadReckoning(Lane lane, std::vector<Crosswalk> cross_walk_areas) {
    // Case 1: Lane detection not work
    if (lane.in_range_count < this->node_threshold) {return true;}
    // Case 2: On Stop lane
    else if (lane.range_average < this->on_lane_threshold) {return true;}
    // Case 3: On Cross Walk
    else if (isOnCrossWalk(cross_walk_areas)) {return true;}
    // Case 4: Not Dead Reckoning
    else {return false;}
}

bool ParticleFilter::isOnCrossWalk(std::vector<Crosswalk> cross_walk_areas) {
    Pose previous_pose = pose_moveToFront(this->estimated_pose);

    for(auto it = cross_walk_areas.begin(); it != cross_walk_areas.end(); ++it) {
        Crosswalk cross_walk = *it;
        
        float crosswalk_x_min = cross_walk.x;
        float crosswalk_x_max = cross_walk.x + cross_walk.w;
        float crosswalk_y_min = cross_walk.y;
        float crosswalk_y_max = cross_walk.y + cross_walk.h;

        if (previous_pose.x >= crosswalk_x_min && previous_pose.x <= crosswalk_x_max &&
            previous_pose.y >= crosswalk_y_min && previous_pose.y <= crosswalk_y_max) {
            return true;
        }
    }

    return false;
}

Pose ParticleFilter::get_pose() {
    Pose estimated_pose;

    // Mean of weight
    float weight_sum = 0;
    float vx = 0;
    float vy = 0;

    for (int i = 0; i < num_particles ; i++) {
        estimated_pose.x += this->particles[i].x * this->particles[i].weight;
        estimated_pose.y += this->particles[i].y * this->particles[i].weight;
        vx += cos(this->particles[i].theta) * this->particles[i].weight;
        vy += sin(this->particles[i].theta) * this->particles[i].weight;
        weight_sum += this->particles[i].weight;
    }
    estimated_pose.theta = std::atan2(vy,vx);

    if (weight_sum > 0) {
        estimated_pose.x /= weight_sum;
        estimated_pose.y /= weight_sum;
    }

    estimated_pose.theta = normalize_pi_to_pi(estimated_pose.theta);

    return estimated_pose;
}

Particle ParticleFilter::pf_moveToFront(Particle pf_center_pose) {
    Particle pf_front_pose;
    pf_front_pose.x = pf_center_pose.x + this->front_length * cos(pf_center_pose.theta);
    pf_front_pose.y = pf_center_pose.y + this->front_length * sin(pf_center_pose.theta);
    pf_front_pose.theta = pf_center_pose.theta;

    return pf_front_pose;
}

Particle ParticleFilter::pf_moveToRear(Particle pf_center_pose) {
    Particle pf_rear_pose;
    pf_rear_pose.x = pf_center_pose.x - this->front_length * cos(pf_center_pose.theta);
    pf_rear_pose.y = pf_center_pose.y - this->front_length * sin(pf_center_pose.theta);
    pf_rear_pose.theta = pf_center_pose.theta;

    return pf_rear_pose;
}

Pose ParticleFilter::pose_moveToFront(Pose center_pose) {
    Pose front_pose;
    front_pose.x = center_pose.x + this->front_length * cos(center_pose.theta);
    front_pose.y = center_pose.y + this->front_length * sin(center_pose.theta);
    front_pose.theta = center_pose.theta;

    return front_pose;
}

float ParticleFilter::calculate_probability(Map* map, Particle pf_pose, Lane lane) {
    float pf_range;
    float sensor_theta, sensor_range;
    float diff_range;
    float p, pz;

    for(int i = 0; i < static_cast<int>(lane.thetas.size()); ++i) {
        // Sensor Lane Distance
        sensor_theta = lane.thetas[i];
        sensor_range = lane.ranges[i];

        if (sensor_range < 0) {
            sensor_range = lane.range_max;
        }
        
        // Bresenham raytracing
        pf_range = map_calc_range(map, pf_pose.x, pf_pose.y, pf_pose.theta + sensor_theta, lane.range_max, map->scale);

        diff_range = pf_range - sensor_range;
        pz = gauss_likelihood(diff_range, sigma_range);
        
        p += pz*pz*pz;
    }

    return p;
}



// // // // // For Testing // // // // // // //
std::vector<Particle> ParticleFilter::get_particle() {
    return this->particles;
}

std::vector<float> ParticleFilter::get_weight() {
    std::vector<float> weights; 

    for (int i = 0; i < num_particles ; i++) {
        weights.push_back(this->particles[i].weight);
    }
    return weights;
}

bool ParticleFilter::get_dead_reckoning() {
    return this->dead_reckoning;
}
// // // // // For Testing // // // // // // //
