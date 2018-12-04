#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include <functional>
#include "Eigen-3.3/Eigen/Dense"
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "helper.h"
#include "spline.h"
#include "cost.h"

using namespace std;

// for convenience
using json = nlohmann::json;
using Eigen::MatrixXd;
using Eigen::VectorXd;



// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
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


vector<double> getXYspline(double s, double d, double max_s, tk::spline &spline_x, tk::spline &spline_y, tk::spline &spline_dx, tk::spline &spline_dy){
  
    // (s, d)->(x, y) transform to get precise (x, y) by means of spline fuction
   
  s = fmod(s, max_s);
  double x = spline_x(s) + d * spline_dx(s);
  double y = spline_y(s) + d * spline_dy(s);
  return {x,y};
}



int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  // about 9800 point are interpolated to get precise (s, d) <-> (x, y) transform
  string map_file_ = "../data/precise_highway_map.csv";
  

  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;
  // double max_s = 6943.83;
  
  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
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


  // set initial value
  int lane = 1;
  double ref_vel = 0.0; // [mph]
  vector<vector<double>> previous_path_s = {{},{},{}};
  vector<vector<double>> previous_path_d = {{},{},{}};
  
  tk::spline spline_x;
  spline_x.set_points(map_waypoints_s, map_waypoints_x);
  tk::spline spline_y;
  spline_y.set_points(map_waypoints_s, map_waypoints_y);
  tk::spline spline_dx;
  spline_dx.set_points(map_waypoints_s, map_waypoints_dx);
  tk::spline spline_dy;
  spline_dy.set_points(map_waypoints_s, map_waypoints_dy);
 
  
  
  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy, &max_s, &lane, &ref_vel,
	       &spline_x, &spline_y, &spline_dx, &spline_dy, &previous_path_s, &previous_path_d](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
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

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];

          	json msgJson;
		
          	vector<double> next_x_vals;
          	vector<double> next_y_vals;
		vector<vector<double>> next_path_s;
		vector<vector<double>> next_path_d;
		
          	
		double car_ds      = 0;
		double car_dds     = 0;
		double car_dd      = 0;
		double car_ddd     = 0;
		  
		int prev_size = previous_path_x.size();
		cout << "prev_size: " <<  prev_size << endl;
		cout << "present_car_s: " << car_s << endl;
		
		// Jerk minimize trajectory
		double dt          = 0.02;
		int n_keep = min(prev_size, 20);
		int n_path = 50;
		int last_point = 0;
		int n_consumed = n_path - prev_size;
		
		cout << "n_keep: " << n_keep << endl;

		vector<double> s_start;
		vector<double> s_end;
		vector<double> d_start;
		vector<double> d_end;
		
		// add last n_keep path points to the next trajectory 
		if(prev_size < 1){
		  s_start = {car_s, 0, 0};
		  d_start = {car_d, 0, 0};
		  }else{
		  for(unsigned int i=0; i< n_keep; i++){
		    next_x_vals.push_back(previous_path_x[i]);
		    next_y_vals.push_back(previous_path_y[i]);
		    next_path_s.push_back(previous_path_s[n_consumed+i]);
		    next_path_d.push_back(previous_path_d[n_consumed+i]);
		  }
		  last_point = n_keep -1;
		  s_start = next_path_s[last_point];
		  car_s   = next_path_s[last_point][0];
		  car_ds  = next_path_s[last_point][1];
		  car_dds = next_path_s[last_point][2];
		  
		  d_start = next_path_d[last_point];
		  car_d   = next_path_d[last_point][0];
		  car_dd  = next_path_d[last_point][1];
		  car_ddd = next_path_d[last_point][2];
		}

		
		// sensor fusion
		vector<vector<double>> near_sur_cars = surroundingCarState(sensor_fusion, car_s,   20,   20, n_keep, dt);
		vector<vector<double>> far_sur_cars  = surroundingCarState(sensor_fusion, car_s, 1000, 1000, n_keep, dt);
		// vector<vector<double>> sur_car_states =surroundingCarState(sensor_fusion, car_s, 50, 22, 5);
		// vector<vector<double>> sur_car_states2 =surroundingCarState(sensor_fusion, car_s, 50, 1000, 1000);

		
		
		cout << "------near surrounding cars-------" << endl;
		printSurroundingCarState(near_sur_cars);
		cout << "------far surrounding cars--------" << endl;
		printSurroundingCarState(far_sur_cars);

		// find the fastest lane
		double v_max = far_sur_cars[2][lane];
		int pref_lane = lane;
		  
		for(int i=0;i<3; i++){
		  if(i!=lane){
		      if(far_sur_cars[2][i] > v_max){
			pref_lane = i;
			v_max = far_sur_cars[2][i];
		      }
		    }
		}
		cout << "preferable lane is " <<  pref_lane << "!" << endl;
		

		/*
		double my_lane_vel = my_lane_velocity(sensor_fusion, car_s, lane, prev_size);
		cout << "my_lane_vel: " << my_lane_vel << endl;

		// Set velocity  for test
		if(car_s + 22 > near_sur_cars[0][lane]){
		  ref_vel -= 0.2237 * 1.2;
		    }else if(ref_vel > my_lane_vel){ 
		// }else if(ref_vel > my_lane_vel+10){ 
		  ref_vel -= 0.2237;
		}else if(ref_vel < 46.0){
		// }else if(ref_vel < 80.0){
		  ref_vel += 0.2237;
		}
		 
		// easy lane finder for test
		if(lane != pref_lane){
		  if((lane == 1) && (near_sur_cars[2][pref_lane] > 999) && (near_sur_cars[2][pref_lane+3] > 999)){
		    lane = pref_lane;
		  }else{
		    if ((near_sur_cars[2][1] > 999) && (near_sur_cars[2][1+3] > 999)){
		      lane = 1;
		    }
		  }
		}
		*/
		
		// Generate coefficients of polynominal trajectories
		double T = 2;
		double accel_step = 0.2237 * 0.5;
		int n_accel = 2;
		vector<trajectory> possible_trajectories = generateTrajectories(s_start, d_start, lane, ref_vel, T, n_accel, accel_step);
		


		// Choose the optimal trajectory using cost function and sensor fusion data
		
		double weight_drivability      = 1000; // feasibility
		double weight_safety           = 100;  // collision
		double weight_leagality        = 10;   // speed limit
		double weight_lane_efficiency  = 1;    // the d difference between present lane and the highest speed lane
		double weight_speed_efficiency = 1;     // the speed difference between 50.0

		vector<double> weights = {weight_drivability, weight_safety, weight_leagality, weight_lane_efficiency, weight_speed_efficiency};
		
	        
		// check drivability
		// acceptable state (minus value is safety margin for coordinate transform) 
		double acceptable_vel  = mph2ms(50.0 - 3.5);
		double acceptable_acc  = 10.0 - 0.4;
		double acceptable_jerk = 10.0 - 0.5;

		double min_cost = 9999999;
		int i_min = 0;
		for(int i=0; i<possible_trajectories.size(); i++){
		  vector<double> eval_coeffs_s = possible_trajectories[i].coeffs_s;
		  vector<double> eval_coeffs_d = possible_trajectories[i].coeffs_d;
		  
		  double cost_drivability      = costDrivability(eval_coeffs_s, eval_coeffs_d, acceptable_acc, acceptable_jerk, n_path, dt);
		  double cost_safety           = costSafety(eval_coeffs_s,  eval_coeffs_d, near_sur_cars, n_path, dt);
		  double cost_leagality        = costLeagality(eval_coeffs_s, eval_coeffs_d, acceptable_vel, n_path, dt);
		  double cost_lane_efficiency  = costLaneEfficiency(eval_coeffs_d, pref_lane, n_path, dt);
		  double cost_speed_efficiency = costSpeedEfficiency(eval_coeffs_s, n_path, dt);
		  vector<double> costs = {cost_drivability, cost_safety, cost_leagality, cost_lane_efficiency, cost_speed_efficiency}; 
		  possible_trajectories[i].total_cost = weightedsumCosts(costs, weights);

		  if(possible_trajectories[i].total_cost <min_cost){
		    
		    min_cost = possible_trajectories[i].total_cost;
		    i_min = i;
		  }
		  
		  cout << "cost" << i << ": ";
		  for(int j=0; j<5; j++){
		    cout << costs[j] << " ";
		  }
		  cout << possible_trajectories[i].total_cost << endl;
		}


		trajectory opt_trajectory = possible_trajectories[i_min];
		
		vector<double> coeffs_s = opt_trajectory.coeffs_s;
		vector<double> coeffs_d = opt_trajectory.coeffs_d;
		lane    = opt_trajectory.lane;
		cout << "lane: " << lane << endl;
		ref_vel = opt_trajectory.target_vel; 
		cout << "ref_vel: " << ref_vel << endl;

		/*
		cout << "possible coeffs s" << endl;
		for(int i=0; i<possible_trajectories.size(); i++){
		  for(int j=0; j<6;j++){
		  cout << possible_trajectories[i].coeffs_s[j] << " ";
		  }
		  cout << "\n";
		}
		cout << "possible coeffs d" << endl;
		for(int i=0; i<possible_trajectories.size(); i++){
		  for(int j=0; j<6;j++){
		  cout << possible_trajectories[i].coeffs_d[j] << " ";
		  }
		  cout << "\n";
		}
		*/

		
		/*
		// easy lane controller for test
		cout << "target velocity[m/s]" << mph2ms(ref_vel) << endl;
		double test_next_d = lane_d(lane);
		s_end = {car_s + 2.0*mph2ms(ref_vel)*T, mph2ms(ref_vel), 0};
		d_end = {car_d, 0, 0};

		if (ref_vel > 10){
		    d_end[0] = test_next_d;
		    s_end[0] = car_s + mph2ms(ref_vel)*T;
		  }

		vector<double> coeffs_s = JMT(s_start, s_end, T);
		vector<double> coeffs_d = JMT(d_start, d_end, T);
		*/
		

		// add next SD points including each velocity and accelaration
		for(unsigned int i=1; i<= n_path-n_keep+4; i++){
		  double pointS = fmod(polyeval(coeffs_s, dt*i), max_s);
		  double pointD = polyeval(coeffs_d, dt*i);
		  next_path_s.push_back({pointS, 0, 0});
		  next_path_d.push_back({pointD, 0, 0});
		}
		
		for(unsigned int i=1; i<= n_path-n_keep+2; i++){
		  double tmp_ds = fmod(next_path_s[last_point+i+1][0] + max_s - next_path_s[last_point+i-1][0], max_s) / (2.0*dt);
		  double tmp_dd = (next_path_d[last_point+i+1][0] - next_path_d[last_point+i-1][0]) / (2.0*dt);
		  next_path_s[last_point+i][1] = tmp_ds;
		  next_path_d[last_point+i][1] = tmp_dd;
		}
			
		for(unsigned int i=1; i<= n_path-n_keep; i++){
		  double tmp_dds = (next_path_s[last_point+i+1][1] - next_path_s[last_point+i-1][1]) / (2.0*dt);
		  double tmp_ddd = (next_path_d[last_point+i+1][1] - next_path_d[last_point+i-1][1]) / (2.0*dt);
		  next_path_s[last_point+i][2] = tmp_dds;
		  next_path_d[last_point+i][2] = tmp_ddd;
		}

		// add next XY points
		for(unsigned int i=0; i< n_path	 - n_keep; i++){
		  vector<double> pointXY =  getXYspline(next_path_s[last_point+i][0], next_path_d[last_point+i][0], max_s, spline_x, spline_y, spline_dx, spline_dy);
		  next_x_vals.push_back(pointXY[0]);
		  next_y_vals.push_back(pointXY[1]);
		}

		previous_path_s = next_path_s;
		previous_path_d = next_path_d;
		
		  
		// end TODO 
		msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
          	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

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
