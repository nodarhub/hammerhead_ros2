#pragma once

#include <cmath>

struct PointFilter {
    bool isValid(const float *const xyz) const {
        return not std::isinf(xyz[0]) and not std::isinf(xyz[1]) and not std::isinf(xyz[2]);
    }
};