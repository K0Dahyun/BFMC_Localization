#pragma once

#include <vector>
#include <array>
#include <string>

struct Pose {
    float x;
    float y;
    float theta;

	Pose() {x=0; y=0; theta=0; }
};

struct Lane {
	std::vector<float> thetas;
	std::vector<float> ranges;
	int in_range_count;
	float range_average;
	float range_max;

	Lane() {thetas.clear(); ranges.clear(); in_range_count = 0; range_average = 0.0; range_max = 0.0; }
};

struct Odometry{
	float velocity;
	float yawrate;

	Odometry() {velocity = 0; yawrate = 0;}
};

struct Decision {
	int decision;

	Decision() {decision = 0;}
};

struct Particle {
	float x;
	float y;
	float theta;
	float weight;

	Particle() {x = 0; y = 0; theta = 0; weight = 1;}
};
