#pragma once
#include <optional>
#include <vector>
#include <cstdint>

namespace v2x {

struct Agent {
    uint32_t id;
    double px, py;     // m
    double vx, vy;     // m/s
    double radius;     // bounding circle, m
};

// Time until the bounding circles of a and b first touch, assuming constant
// velocity. nullopt if they never collide (diverging or missing each other).
std::optional<double> timeToCollision(const Agent& a, const Agent& b);

struct Conflict {
    uint32_t id_a, id_b;
    double ttc;
};

// All pairs with TTC below threshold, sorted most-urgent first.
std::vector<Conflict> detectConflicts(const std::vector<Agent>& agents,
                                      double ttc_threshold);

} // namespace v2x
