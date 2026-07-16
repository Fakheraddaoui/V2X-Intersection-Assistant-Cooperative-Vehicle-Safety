#include "v2x_conflict_detector/ttc.hpp"
#include <cmath>
#include <algorithm>

namespace v2x {

std::optional<double> timeToCollision(const Agent& a, const Agent& b)
{
    // Relative kinematics: solve |p + v t| = R for smallest t >= 0
    const double px = b.px - a.px, py = b.py - a.py;
    const double vx = b.vx - a.vx, vy = b.vy - a.vy;
    const double R  = a.radius + b.radius;

    const double dist2 = px * px + py * py;
    if (dist2 <= R * R) return 0.0;              // already overlapping

    const double A = vx * vx + vy * vy;
    if (A < 1e-12) return std::nullopt;          // no relative motion

    const double B = 2.0 * (px * vx + py * vy);
    const double C = dist2 - R * R;
    const double disc = B * B - 4.0 * A * C;
    if (disc < 0.0) return std::nullopt;         // paths miss

    const double t = (-B - std::sqrt(disc)) / (2.0 * A);
    if (t < 0.0) return std::nullopt;            // diverging
    return t;
}

std::vector<Conflict> detectConflicts(const std::vector<Agent>& agents,
                                      double ttc_threshold)
{
    std::vector<Conflict> out;
    for (std::size_t i = 0; i < agents.size(); i++)
        for (std::size_t j = i + 1; j < agents.size(); j++)
            if (auto t = timeToCollision(agents[i], agents[j]); t && *t < ttc_threshold)
                out.push_back({agents[i].id, agents[j].id, *t});

    std::sort(out.begin(), out.end(),
              [](const Conflict& a, const Conflict& b) { return a.ttc < b.ttc; });
    return out;
}

} // namespace v2x
