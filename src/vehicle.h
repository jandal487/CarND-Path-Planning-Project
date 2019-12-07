#ifndef VEHICLE_H
#define VEHICLE_H

#include <iostream>
#include <random>
#include <vector>
#include <map>
#include <string>

using namespace std;

class Vehicle {
	public:
		// Vehicle attributes
		map<string, int> lane_direction = {{"LCL", -1}, {"LCR", 1}};
		struct collider {
			bool collision ; // collision check
			int  time; // time of collision
		};
		int L = 1;
		int preferred_buffer = 6; // impacts "keep lane" behavior.
		int lane;
		float s;
		float v_s;
		float a_s;
		float d;
		float target_speed;
		int lanes_available;
		float max_acceleration;
		int target_lane = 0;
		int goal_s;
		string state;

		// Default Constructor
		Vehicle();
	  
		// Set Constructor
		Vehicle(int lane, float s, float d, float v_s, float a_s, string state="CS", int target_lane=1);

		// Destructor
		virtual ~Vehicle();

		// Vehicle actions
		vector<Vehicle> choose_next_state(map<int, vector<Vehicle>> predictions, float time_window);
		vector<string> successor_states();
		vector<Vehicle> generate_trajectory(string state, map<int, vector<Vehicle>> predictions, float time_window);
		vector<float> get_kinematics(map<int, vector<Vehicle>> predictions, int lane, float time_window);
		vector<Vehicle> constant_speed_trajectory(float time_window);
		vector<Vehicle> keep_lane_trajectory(map<int, vector<Vehicle>> predictions, float time_window);
		vector<Vehicle> lane_change_trajectory(string state, map<int, vector<Vehicle>> predictions,float time_window);
		vector<Vehicle> prep_lane_change_trajectory(string state, map<int, vector<Vehicle>> predictions, float time_window);
		void s_increment(float dt);
		float s_position_at(float t);
		float s_speed_at(float t);
		vector<double> get_vehicle_behind(map<int, vector<Vehicle>> predictions, Vehicle & rVehicle, int lane);
		vector<double> get_vehicle_ahead(map<int, vector<Vehicle>> predictions, Vehicle & rVehicle, int lane);
		vector<Vehicle> generate_predictions(float horizon);
		void realize_next_state(vector<Vehicle> trajectory);
		void configure(vector<float> road_data);

};
#endif  // VEHICLE_H
