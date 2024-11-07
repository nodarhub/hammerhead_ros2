#pragma once

#include <cmath>

struct PointFilter {
    bool isValid(const float *const xyz) const {
        return not std::isinf(xyz[0]) and not std::isinf(xyz[1]) and not std::isinf(xyz[2]);
    }

    bool inRange(const float *const xyz) const {
        const auto x = -xyz[0];
        const auto y = -xyz[1];
        const auto z = -xyz[2];
        return not(std::isinf(x)  //
                   or std::isinf(y) or y < y_min or y > y_max  //
                   or std::isinf(z) or z < z_min or z > z_max);
    }

    float z_min = 8.0;
    float z_max = 500.0;
    float y_min = -50.0;
    float y_max = 50.0;
};