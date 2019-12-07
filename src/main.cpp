#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include <algorithm>
#include "json.hpp"
#include "spline.h"
#include "road.h"
#include "vehicle.h"

// for convenience
using json = nlohmann::json;
using std::string;
using std::vector;

// For converting back and forth between radians and degrees.
constexpr double pi() { 
	return M_PI; 
}
double deg2rad(double x) { 
	return x * pi() / 180; 
}
double rad2deg(double x) { 
	return x * 180 / pi(); 
}

string hasData(string s) {
	
	// Checks if the SocketIO event has JSON data.
	// If there is data the JSON object in string format will be returned,
	// else the empty string "" will be returned.
	auto found_null = s.find("null");
	auto b1 = s.find_first_of("[");
	auto b2 = s.find_first_of("}");
	if (found_null != string::npos) {
		return "";
	} else if (b1 != string::npos && b2 != string::npos) {
		return s.substr(b1, b2 - b1 + 2);
	}
	
	return "";
}

double distance(double x1, double y1, double x2, double y2) {
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y) {

	double closestLen = 100000; //large number
	int closestWaypoint = 0;
	for(int i = 0; i < maps_x.size(); i++) {
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}
	}

	return closestWaypoint;
}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y) {

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);
	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];
	double heading = atan2((map_y-y),(map_x-x));
	double angle = fabs(theta-heading);
	angle = min(2*pi() - angle, angle);

	if(angle > pi()/4) {
		closestWaypoint++;
		if (closestWaypoint == maps_x.size()) {
			closestWaypoint = 0;
		}
	}

  return closestWaypoint;
}

vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y) {
	
	// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
	int next_wp = NextWaypoint(x, y, theta, maps_x, maps_y);
	int prev_wp;
	prev_wp = next_wp - 1;
	if(next_wp == 0) {
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp] - maps_x[prev_wp];
	double n_y = maps_y[next_wp] - maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	// see if d value is positive or negative by comparing it to a center point
	double center_x = 1000 - maps_x[prev_wp];
	double center_y = 2000 - maps_y[prev_wp];
	double centerToPos = distance(center_x, center_y, x_x, x_y);
	double centerToRef = distance(center_x, center_y, proj_x, proj_y);

	if(centerToPos <= centerToRef) {
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++) {
		frenet_s += distance(maps_x[i], maps_y[i], maps_x[i+1], maps_y[i+1]);
	}

	frenet_s += distance(0, 0, proj_x, proj_y);

	return {frenet_s, frenet_d};
}

vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y) {
	
	// Transform from Frenet s,d coordinates to Cartesian x,y
	int prev_wp = -1;
	while( (s > maps_s[prev_wp+1]) && (prev_wp < (int)(maps_s.size()-1)) ) {
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2] - maps_y[prev_wp]), (maps_x[wp2] - maps_x[prev_wp]));
	
	// the x,y,s along the segment
	double seg_s = (s - maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp] + seg_s*cos(heading);
	double seg_y = maps_y[prev_wp] + seg_s*sin(heading);
	double perp_heading = heading-pi()/2;
	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};
}


// Initialize road parameters
double REF_VEL = 49.0;
double GOAL_S = 6945.554;
vector<float> LANE_SPEEDS = {REF_VEL, REF_VEL, REF_VEL};
int LANE_WIDTH = 4;
int MAX_ACCEL = 1;
float TIME_HORIZON = 2;
float MPH_CONVERT = 0.447;
int NUM_LANES = LANE_SPEEDS.size();
double acc = 0;
int TARGET_LANE = 1;
int car_state = 0; // 0=="KL", 1=="LCR", -1=="LCL"
Road road = Road(REF_VEL, LANE_SPEEDS, LANE_WIDTH, TIME_HORIZON, MPH_CONVERT);
vector<float> ego_config = {REF_VEL*MPH_CONVERT, NUM_LANES, GOAL_S, MAX_ACCEL};

int main() {
	uWS::Hub h;

	// Load up map values for waypoint's x,y,s and d normalized normal vectors
	vector<double> map_waypoints_x;
	vector<double> map_waypoints_y;
	vector<double> map_waypoints_s;
	vector<double> map_waypoints_dx;
	vector<double> map_waypoints_dy;

	// Waypoint map to read from
	string map_file_ = "../data/highway_map.csv";

	std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

	string line;
	while (getline(in_map_, line)) {
	std::istringstream iss(line);
	double x;
	double y;
	float s;
	float d_x;
	float d_y;
	iss >> x;
	iss >> y;
	iss >> s;
	iss >> d_x;
	iss >> d_y;
	map_waypoints_x.push_back(x);
	map_waypoints_y.push_back(y);
	map_waypoints_s.push_back(s);
	map_waypoints_dx.push_back(d_x);
	map_waypoints_dy.push_back(d_y);
	}

	road.add_ego2(1,0,6,0,0,0,1,ego_config);
	
	h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
			   &map_waypoints_dx,&map_waypoints_dy]
			  (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
			   uWS::OpCode opCode) {
		// "42" at the start of the message means there's a websocket message event.
		// The 4 signifies a websocket message
		// The 2 signifies a websocket event
		if (length && length > 2 && data[0] == '4' && data[1] == '2') {
			auto s = hasData(data);

			if (s != "") {
				auto j = json::parse(s);			
				string event = j[0].get<string>();
			
				if (event == "telemetry") {
					// j[1] is the data JSON object
				  
					// Main car's localization Data
					double car_x = j[1]["x"];
					double car_y = j[1]["y"];
					double car_s = j[1]["s"];
					double car_d = j[1]["d"];
					double car_yaw = j[1]["yaw"];
					double car_speed = j[1]["speed"];

					// Previous path data given to the Planner
					auto previous_path_x = j[1]["previous_path_x"];
					auto previous_path_y = j[1]["previous_path_y"];
				  
					// Previous path's end s and d values 
					double end_path_s = j[1]["end_path_s"];
					double end_path_d = j[1]["end_path_d"];

					// Sensor Fusion Data, a list of all other cars on the same side 
					//   of the road.
					auto sensor_fusion = j[1]["sensor_fusion"];

					json msgJson;

					vector<double> next_x_vals;
					vector<double> next_y_vals;
					
					/**
					* TODO: define a path made up of (x,y) points that the car will visit
					*   sequentially every .02 seconds
					*/
					
					// Update car state: s position, d position, lane, speed, acceleration,
					vector<double> car_data = {car_x, car_y, car_s, car_d, car_speed*MPH_CONVERT, acc, car_state, TARGET_LANE};
					road.populate_traffic(sensor_fusion, car_data);
					road.advance();
					Vehicle new_pos = road.get_ego();
					double new_s = new_pos.s;
					double new_d = new_pos.d;
					double new_v_s = new_pos.v_s;
					acc = new_pos.a_s;
					if(new_pos.state=="KL"){
						car_state=0;
					}
					if(new_pos.state=="LCR"){
						car_state=1;
					}
					if(new_pos.state=="LCL"){
						car_state=-1;
					}
					if(new_v_s > 0){ // Speed update
						REF_VEL = new_v_s/MPH_CONVERT;
					}
					TARGET_LANE = new_pos.target_lane;
					double delta_d = new_d-car_d;
					cout<<"new_s: "<<new_s<<endl;

					// Transform to x,y coordinates
					int prev_size = previous_path_x.size();
					vector<double> ptsx;
					vector<double> ptsy;
					double ref_x = car_x;
					double ref_y = car_y;
					double ref_yaw = deg2rad(car_yaw);
					if(prev_size < 2){
						double prev_car_x = car_x - cos(ref_yaw);
						double prev_car_y = car_y - sin(ref_yaw);
						ptsx.push_back(prev_car_x);
						ptsx.push_back(car_x);
						ptsy.push_back(prev_car_y);
						ptsy.push_back(car_y);
					}else{
						ref_x = previous_path_x[prev_size-1];
						ref_y = previous_path_y[prev_size-1];
						double prev_car_x = previous_path_x[prev_size-2];
						double prev_car_y = previous_path_y[prev_size-2];
						ref_yaw = atan2(ref_y-prev_car_y, ref_x-prev_car_x);
						ptsx.push_back(prev_car_x);
						ptsx.push_back(ref_x);
						ptsy.push_back(prev_car_y);
						ptsy.push_back(ref_y);
					}

					vector<double> next_wp0;
					vector<double> next_wp1;
					vector<double> next_wp2;
					next_wp0 = getXY(car_s+60, new_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
					next_wp1 = getXY(car_s+80, new_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
					next_wp2 = getXY(car_s+90, new_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);

					ptsx.push_back(next_wp0[0]);
					ptsx.push_back(next_wp1[0]);
					ptsx.push_back(next_wp2[0]);
					ptsy.push_back(next_wp0[1]);
					ptsy.push_back(next_wp1[1]);
					ptsy.push_back(next_wp2[1]);

					for(int i=0; i<ptsx.size();i++){
						double shift_x = ptsx[i]-ref_x;
						double shift_y = ptsy[i]-ref_y;
						ptsx[i] = shift_x*cos(0-ref_yaw) - shift_y*sin(0-ref_yaw);
						ptsy[i] = shift_x*sin(0-ref_yaw) + shift_y*cos(0-ref_yaw);
					}
					for(int i=0;i<previous_path_x.size();i++){
						next_x_vals.push_back(previous_path_x[i]);
						next_y_vals.push_back(previous_path_y[i]);
					}

					tk::spline s;
					s.set_points(ptsx,ptsy);

					double target_x = 30.0;
					double target_y = s(target_x);
					double target_dist = sqrt(target_x*target_x + target_y*target_y);
					double x_add_on = 0;

					for(int i=1;i<=50-previous_path_x.size();i++){
						double N = target_dist/(0.02*REF_VEL*MPH_CONVERT);
						double x_point = x_add_on + target_x/N;
						double y_point = s(x_point);
						x_add_on = x_point;
						double x_ref = x_point;
						double y_ref = y_point;
						x_point = x_ref*cos(ref_yaw)-y_ref*sin(ref_yaw);
						y_point = x_ref*sin(ref_yaw)+y_ref*cos(ref_yaw);
						x_point += ref_x;
						y_point += ref_y;
						next_x_vals.push_back(x_point);
						next_y_vals.push_back(y_point);
					}
					
					msgJson["next_x"] = next_x_vals;
					msgJson["next_y"] = next_y_vals;

					auto msg = "42[\"control\","+ msgJson.dump()+"]";

					ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
				}  // end "telemetry" if
			} else {
				// Manual driving
				std::string msg = "42[\"manual\",{}]";
				ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
			}
		}  // end websocket if
	}); // end h.onMessage

	h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
	std::cout << "Connected!!!" << std::endl;
	});

	h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
						 char *message, size_t length) {
	ws.close();
	std::cout << "Disconnected" << std::endl;
	});

	int port = 4567;
	if (h.listen(port)) {
	std::cout << "Listening to port " << port << std::endl;
	} else {
	std::cerr << "Failed to listen to port" << std::endl;
	return -1;
	}

	h.run();
}