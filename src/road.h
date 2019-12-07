#ifndef ROAD_H
#define ROAD_H

#include <iostream>
#include <random>
#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>
#include <set>
#include <map>
#include <string>
#include <iterator>
#include "vehicle.h"

using namespace std;

class Road {
	public:
		// Road attributes
		int ego_key = -1;
		int num_lanes;
		vector<float> lane_speeds;
		float speed_limit;
		int lane_width;
		map<int, Vehicle> vehicles;
		int vehicles_added = 0;
		float time_horizon;
		float mph_convert;

		// Constructor
		Road(float speed_limit, vector<float> lane_speeds, int lane_width, float time_horizon, float mph_convert);

		// Destructor
		virtual ~Road();

		// Road actions
		Vehicle get_ego();
		void populate_traffic(vector<vector<double>> sf_data, vector<double> car_data);
		void advance();
		void add_ego(int lane_num, float s, float d, float v, float a, int state_of_car, int target_lane, vector<float> config_data);
		void cull();
		vector<double> JMT(vector< double> start, vector <double> end, double T);

};
#endif  // ROAD_H
