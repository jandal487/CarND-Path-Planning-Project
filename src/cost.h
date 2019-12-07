#ifndef COST_H
#define COST_H

#include "vehicle.h"

using namespace std;

float calculate_cost(const vector<Vehicle> & trajectory, float dist);

float goal_distance_cost(const Vehicle & vehicle, float dist);

float safety_cost(const Vehicle & vehicle, float dist);

float comfort_cost(const Vehicle & vehicle, float dist);

#endif  // COST_H
