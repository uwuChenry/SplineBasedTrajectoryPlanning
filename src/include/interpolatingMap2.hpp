#pragma once

#include <map>
#include "structs.hpp"

/*
class InterpolatingTrajectoryPoint {
    public:

    void insert(const double& key, const TrajectoryPoint& value) {
        m_container.insert(std::make_pair(key, value));
    }


    void insert(double&& key, TrajectoryPoint&& value) {
        m_container.insert(std::make_pair(key, value));
    }
 
    TrajectoryPoint operator[](const double& key) const {
        using const_iterator = typename std::map<double, TrajectoryPoint>::const_iterator;
        // Get iterator to upper bound key-value pair for the given key
        const_iterator upper = m_container.upper_bound(key);
        // If key > largest key in table, return value for largest table key
        if (upper == m_container.end()) {
          return (--upper)->second;
        }
        // If key <= smallest key in table, return value for smallest table key
        if (upper == m_container.begin()) {
          return upper->second;
        }
        // Get iterator to lower bound key-value pair
        const_iterator lower = upper;
        --lower;
        // Perform linear interpolation between lower and upper bound
        const double delta = (key - lower->first) / (upper->first - lower->first);
        return {
            delta * upper->second.vel + (1.0 - delta) * lower->second.vel,
            delta * upper->second.accel + (1.0 - delta) * lower->second.accel,
            delta * upper->second.position + (1.0 - delta) * lower->second.position,
            delta * upper->second.curvature + (1.0 - delta) * lower->second.curvature};
    }


    void clear() { m_container.clear(); }

    private:
    std::map<double, TrajectoryPoint> m_container;
}; */
