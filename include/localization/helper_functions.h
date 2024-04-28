#pragma once

#include <cmath>

inline float normalize_pi_to_pi(float radian) { // -pi ~ pi radian
    while (radian > M_PI) radian -= 2.0 * M_PI;
    while (radian < -M_PI) radian += 2.0 * M_PI;
    return radian;
}

inline float normalize_0_to_2pi(float radian) { // 0 ~ 2pi radian
    while (radian > 2.0 * M_PI) radian -= 2.0 * M_PI;
    while (radian < 0.0) radian += 2.0 * M_PI;
    return radian;
}

inline float dist(float x1, float y1, float x2, float y2) {
    return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

inline float gauss_likelihood(float x, float sigma) {
    return exp (-(x * x) / (2 * sigma * sigma));
}

inline float rad_to_deg(float radian) {
    return radian * 180 / M_PI;
}

inline float deg_to_rad(float degree) {
    return degree * M_PI / 180;
}
