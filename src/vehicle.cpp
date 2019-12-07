#include <algorithm>
#include <iostream>
#include "vehicle.h"
#include "road.h"
#include <cmath>
#include <math.h>
#include <map>
#include <string>
#include <iterator>
#include "cost.h"

// Initializes Vehicle
Vehicle::Vehicle(){}

// Initializes custom Vehicle object
Vehicle::Vehicle(int lane, float s,float d, float v_s, float a_s, string state, int target_lane) {

    this->lane = lane;
    this->s = s;
    this->d = d;
    this->v_s = v_s;
    this->a_s = a_s;
    this->state = state;
    this->target_lane = target_lane;
}

// Destructor
Vehicle::~Vehicle() {}

// Vehile actions
vector<Vehicle> Vehicle::choose_next_state(map<int, vector<Vehicle>> predictions, float time_window) {

	/**
	* Here you can implement the transition_function code from the Behavior 
	*   Planning Pseudocode classroom concept.
	*
	* @param A predictions map. This is a map of vehicle id keys with predicted
	*   vehicle trajectories as values. Trajectories are a vector of Vehicle 
	*   objects representing the vehicle at the current timestep and one timestep
	*   in the future.
	* @output The best (lowest cost) trajectory corresponding to the next ego 
	*   vehicle state.
	*
	* Functions that will be useful:
	* 1. successor_states - Uses the current state to return a vector of possible
	*    successor states for the finite state machine.
	* 2. generate_trajectory - Returns a vector of Vehicle objects representing 
	*    a vehicle trajectory, given a state and predictions. Note that 
	*    trajectory vectors might have size 0 if no possible trajectory exists 
	*    for the state. 
	* 3. calculate_cost - Included from cost.cpp, computes the cost for a trajectory.
	*
	*/
	
    vector<string> states = successor_states();
    float cost;
    vector<float> costs;
    vector<vector<Vehicle>> final_trajectories;
    float max_dist = this->goal_s;
    for (vector<string>::iterator it = states.begin(); it != states.end(); ++it) {
        vector<Vehicle> trajectory = generate_trajectory(*it, predictions, time_window);
        if (trajectory.size() > 1) {
            cost = calculate_cost(trajectory,max_dist);
            costs.push_back(cost);
            final_trajectories.push_back(trajectory);
        }
    }
    vector<float>::iterator best_cost = min_element(begin(costs), end(costs));
    int best_idx = distance(begin(costs), best_cost);
	
    return final_trajectories[best_idx];
}

vector<string> Vehicle::successor_states() {
    
	// Provides the possible next states given the current state for the FSM 
    //   discussed in the course, with the exception that lane changes happen 
    //   instantaneously, so LCL and LCR can only transition back to KL.
    vector<string> states;
    states.push_back("KL");
    string state = this->state;
    if(state.compare("KL") == 0 && this->lane < 2){
        states.push_back("LCR");
    }
    if(state.compare("KL") == 0 && this->lane > 0){
    	states.push_back("LCL");
    }
	
    //If state is "LCL" or "LCR", then just return "KL"
    return states;
}

vector<Vehicle> Vehicle::generate_trajectory(string state, map<int, vector<Vehicle>> predictions, float time_window) {
    
	// Given a possible next state, generate the appropriate trajectory to realize
    //   the next state.
    vector<Vehicle> trajectory;
    if (state.compare("CS") == 0) {
        trajectory = constant_speed_trajectory(time_window);
    } else if (state.compare("KL") == 0) {
        trajectory = keep_lane_trajectory(predictions,time_window);
    } else if (state.compare("LCL") == 0 || state.compare("LCR") == 0) {
        trajectory = lane_change_trajectory(state, predictions,time_window);
    } else if (state.compare("PLCL") == 0 || state.compare("PLCR") == 0) {
        trajectory = prep_lane_change_trajectory(state, predictions,time_window);
    }
	
    return trajectory;
}

vector<float> Vehicle::get_kinematics(map<int, vector<Vehicle>> predictions, int lane, float time_window) {
	
	// Gets next timestep kinematics (position, velocity, acceleration) 
	//   for a given lane. Tries to choose the maximum velocity and acceleration, 
	//   given other vehicle positions and accel/velocity constraints.
    float max_pos_vel = this->max_acceleration*time_window + this->v_s;
    float max_neg_vel = -this->max_acceleration*time_window + this->v_s;
    float new_position;
    float new_velocity;
    float new_accel;
    Vehicle vehicle_ahead;
    Vehicle vehicle_behind;
    vector<double>  ahead = get_vehicle_ahead(predictions, vehicle_ahead,lane);
    if (ahead[0] && max_pos_vel>vehicle_ahead.v_s) {
		//must travel at the speed of traffic, regardless of preferred buffer
        new_velocity = max(vehicle_ahead.v_s,max_neg_vel); 
        new_position = this->s + new_velocity*time_window - 0.5*this->max_acceleration*time_window*time_window;
    } else {
        new_velocity = min(max_pos_vel, this->target_speed);
        new_position = this->s + new_velocity*time_window + 0.5*this->max_acceleration*time_window*time_window;
    }

    new_accel = (new_velocity - this->v_s)/time_window; //Equation: (v_1 - v_0)/t = acceleration
	
    return{new_position, new_velocity, new_accel,ahead[0], ahead[1]};
}

vector<Vehicle> Vehicle::constant_speed_trajectory(float time_window) {
    
	// Generate a constant speed trajectory.
    float next_pos_s = s_position_at(time_window);
    vector<Vehicle> trajectory = {Vehicle(this->lane, this->s,this->d, this->v_s,0,this->state,this->target_lane),
                                  Vehicle(this->lane, next_pos_s, this->d, this->v_s,0, this->state,this->target_lane)};
    return trajectory;
}

vector<Vehicle> Vehicle::keep_lane_trajectory(map<int, vector<Vehicle>> predictions, float time_window) {
    
	// Generate a keep lane trajectory.
    vector<Vehicle> trajectory = {Vehicle(this->lane, this->s,this->d, this->v_s, this->a_s, this->state, this->target_lane)};
    vector<float> kinematics = get_kinematics(predictions, this->lane, time_window);
    float new_s = kinematics[0];
    float new_v = kinematics[1];
    float new_a = kinematics[2];
    trajectory.push_back(Vehicle(this->lane, new_s,this->lane*4+2, new_v, new_a,"KL",this->lane));
	
    return trajectory;
}

vector<Vehicle> Vehicle::prep_lane_change_trajectory(string state, map<int, vector<Vehicle>> predictions, float time_window) {
	
	// Generate a trajectory preparing for a lane change.
    float new_s;
    float new_v;
    float new_a;
    Vehicle vehicle_behind;
    int new_lane = this->lane + lane_direction[state];
    vector<Vehicle> trajectory = {Vehicle(this->lane, this->s,this->d, this->v_s, this->a_s,this->state,this->target_lane)};
    vector<float> curr_lane_new_kinematics = get_kinematics(predictions, this->lane,time_window);

    if (get_vehicle_behind(predictions, vehicle_behind,lane)[0]) {
        //Keep speed of current lane so as not to collide with car behind.
        new_s = curr_lane_new_kinematics[0];
        new_v = curr_lane_new_kinematics[1];
        new_a = curr_lane_new_kinematics[2];

    } else {
        vector<float> best_kinematics;
        vector<float> next_lane_new_kinematics = get_kinematics(predictions, new_lane,time_window);
        //Choose kinematics with lowest velocity.
        if (next_lane_new_kinematics[1] < curr_lane_new_kinematics[1]) {
            best_kinematics = next_lane_new_kinematics;
        } else {
            best_kinematics = curr_lane_new_kinematics;
        }
        new_s = best_kinematics[0];
        new_v = best_kinematics[1];
        new_a = best_kinematics[2];
    }

    trajectory.push_back(Vehicle(this->lane, new_s, this->d, new_v, new_a, state, this->target_lane));
    
	return trajectory;
}

vector<Vehicle> Vehicle::lane_change_trajectory(string state, map<int, vector<Vehicle>> predictions, float time_window) {
	
	// Generate a lane change trajectory.
    int new_lane = this->lane + lane_direction[state];
    float future_s = this->s_position_at(time_window);
    bool car_ahead = false;
    bool car_behind = false;
    vector<Vehicle> trajectory;
    Vehicle next_lane_vehicle;
	
    // Check if a lane change is possible (check if another vehicle occupies that spot).
    for (map<int, vector<Vehicle>>::iterator it = predictions.begin(); it != predictions.end(); ++it) {
        next_lane_vehicle = it->second[0];
        car_ahead = (next_lane_vehicle.s - future_s) < 5 && next_lane_vehicle.s > future_s && next_lane_vehicle.lane == new_lane;
        if (car_ahead) break;
    }
    for (map<int, vector<Vehicle>>::iterator it = predictions.begin(); it != predictions.end(); ++it) {
        next_lane_vehicle = it->second[0];
        car_behind = (future_s-next_lane_vehicle.s) < 5 && next_lane_vehicle.s < future_s && next_lane_vehicle.lane == new_lane;
        if (car_behind) break;
    }
    if(car_behind || car_ahead){
    	return trajectory;
    }
    trajectory.push_back(Vehicle(this->lane, this->s, this->d, this->v_s, this->a_s, this->state, this->target_lane));
    vector<float> kinematics = get_kinematics(predictions, new_lane,time_window);
    float new_d = new_lane*4+2;
    trajectory.push_back(Vehicle(new_lane, kinematics[0],new_d, kinematics[1], kinematics[2], state,new_lane));
    
	return trajectory;
}

void Vehicle::s_increment(float dt = 1) {
	this->s = s_position_at(dt);
}

float Vehicle::s_position_at(float t) {
    return this->s + this->v_s*t + this->max_acceleration*t*t/2.0;
}

float Vehicle::s_speed_at(float t) {
    return this->v_s + this->a_s*t;
}


vector<double> Vehicle::get_vehicle_behind(map<int, vector<Vehicle>> predictions, Vehicle & rVehicle, int lane) {
    
	// Returns a true if a vehicle is found behind the current vehicle, false 
    //   otherwise. The passed reference rVehicle is updated if a vehicle is found.
    int max_s = -1;
    bool found_vehicle = false;
    Vehicle temp_vehicle;
    double delta_s = this->s-max_s;
    for (map<int, vector<Vehicle>>::iterator it = predictions.begin(); it != predictions.end(); ++it) {
        temp_vehicle = it->second[0];
        int v_id=it->first;
        if (temp_vehicle.lane == this->lane && temp_vehicle.s < this->s && temp_vehicle.s > max_s) {
            max_s = temp_vehicle.s;
            rVehicle = temp_vehicle;
            found_vehicle = true;
        }
    }
    return {found_vehicle};
}

vector<double> Vehicle::get_vehicle_ahead(map<int, vector<Vehicle>> predictions, Vehicle & rVehicle, int lane) {

	// Returns a true if a vehicle is found ahead of the current vehicle, false 
    //   otherwise. The passed reference rVehicle is updated if a vehicle is found.
    int min_s = this->goal_s;
    bool found_vehicle = false;
    float vehicle_speed = 0;
    Vehicle temp_vehicle;
    double delta_s = min_s-this->s;
    for (map<int, vector<Vehicle>>::iterator it = predictions.begin(); it != predictions.end(); ++it) {
    	int v_id = it->first;
        temp_vehicle = it->second[0];
        delta_s = temp_vehicle.s - this->s;
        vehicle_speed = temp_vehicle.v_s;
    	if(v_id  != -1 && temp_vehicle.lane == lane && delta_s > 0 && delta_s<30){
            rVehicle = temp_vehicle;
            found_vehicle = true;
            break;
    	}
    }
    return {found_vehicle, delta_s, vehicle_speed};
}

vector<Vehicle> Vehicle::generate_predictions(float horizon) {
    // Generates predictions for non-ego vehicles to be used in trajectory 
    //   generation for the ego vehicle.
	vector<Vehicle> predictions;
    for(int i = 0; i < horizon; i++) {
      float next_s = s_position_at(i);

      float next_v_s = 0;
      if (i < horizon-1) {
        next_v_s = s_position_at(i+1) - next_s;
      }

      float next_a_s = 0;
      if (i < horizon-2) {
        next_a_s = s_speed_at(i+1) - next_v_s;
      }
      predictions.push_back(Vehicle(this->lane, next_s,this->d, next_v_s, next_a_s, this->state, this->target_lane));
  	}
    return predictions;

}

void Vehicle::realize_next_state(vector<Vehicle> trajectory) {
	
    // Sets state and kinematics for ego vehicle using the last state of the trajectory.
    Vehicle next_state = trajectory[1];
    this->target_lane=next_state.target_lane;
    this->state = next_state.state;
    this->lane = next_state.lane;
    this->s = next_state.s;
    this->d = next_state.d;
    this->v_s = next_state.v_s;
    this->a_s = next_state.a_s;
}

void Vehicle::configure(vector<float> road_data) {
	
    // Called by simulator before simulation begins. Sets various parameters which
    //  will impact the ego vehicle.
    target_speed = road_data[0];
    lanes_available = road_data[1];
    goal_s = road_data[2];
    max_acceleration = road_data[3];
}
