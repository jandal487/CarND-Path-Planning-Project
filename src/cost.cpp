#include "cost.h"
#include "vehicle.h"
#include <functional>
#include <iterator>
#include <map>
#include <math.h>

const float REACH_GOAL = 1;
const float SAFETY = 0.01;
const float COMFORT = 1;


float goal_distance_cost(const Vehicle & vehicle, float dist) {
	
	// Cost increases based on distance of intended lane (for planning a lane 
	//   change) and final lane of trajectory.
    // Cost of being out of goal lane also becomes larger as vehicle approaches 
    //   goal distance.
	float cost;
    float distance = dist - vehicle.s;
    if (distance > 0) {
        cost = distance;
    } else {
        cost = 1;
    }
    return cost;
}

float safety_cost(const Vehicle & vehicle, float dist) {
	
	// Cost for trajectories with intended lane and final lane with traffic slower than vehicle's target speed.
    float cost;
    float delta = vehicle.target_speed-vehicle.v_s;
    if (delta > 0) {
        cost = 1 - exp(-delta);
    } else {
        cost = 100000;
    }
    return cost;
}

float comfort_cost(const Vehicle & vehicle, float dist) {

    float cost = 0;
    if (vehicle.state.compare("LCR")==0 || vehicle.state.compare("LCL")==0){
		cost = 1;
	}
	
    return cost;
}

float calculate_cost(const vector<Vehicle> & trajectory, float dist) {

	// Calculate weighted cost as total cost of trajectory
	Vehicle v_new = trajectory[1];
    float cost = 0.0;

    // Vector of cost functions
    vector<function<float(const Vehicle &, float dist)>> cf_list = {goal_distance_cost, safety_cost, comfort_cost};
    vector<float> weight_list = {REACH_GOAL, SAFETY, COMFORT};

    for (int i = 0; i < cf_list.size(); i++) {
        float new_cost = weight_list[i]*cf_list[i](v_new, dist);
        cost += new_cost;
    }

    return cost;
}


