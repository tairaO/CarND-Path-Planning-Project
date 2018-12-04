#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include <iomanip>
#include <string>
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
  /*
    (s, d)->(x, y) transform to get precise (x, y) by means of spline fuction
  */
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
  double max_s = 6945.554;   // for my interpolated map way point 
  // double max_s = 6943.83; // udacity provided
  
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
  int lane       = 1;   // [0, 1, 2]
  double ref_vel = 0.0; // [mph]
  vector<vector<double>> previous_path_s = {{},{},{}};
  vector<vector<double>> previous_path_d = {{},{},{}};

  // Calculating x, y,normal vector (x,y) in d coordinate when s in Frenet coordinate is given
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

		/*****************************************************************************
		 *  Initialization and get last path information
		 ****************************************************************************/
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
		
		double dt          = 0.02;
		int n_keep = min(prev_size, 15);
		int n_path = 50;
		int last_point = 0;
		int n_consumed = n_path - prev_size;
		
		// (position, velocity, accelaration) in start and end condition
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

		
		/*****************************************************************************
		 *  Sensor fusion
		 ****************************************************************************/
		
		vector<vector<double>> near_sur_cars = surroundingCarState(sensor_fusion, car_s,  30,  30, prev_size, dt);
		vector<vector<double>> far_sur_cars  = surroundingCarState(sensor_fusion, car_s, 100, 100, prev_size, dt);
	        
		cout << "------near surrounding cars-------" << endl;
		printSurroundingCarState(near_sur_cars);
		cout << "------far surrounding cars--------" << endl;
		printSurroundingCarState(far_sur_cars);

		 
		// find the fastest lane
		double v_max = far_sur_cars[2][lane];
		int pref_lane = lane;
		
		vector<int>  better_lane_order = {1, 0, 2}; // lane 1 is the best because it has more options
		for(int i=0;i<3; i++){
		  int check_lane = better_lane_order[i]; 
		  if(check_lane!=lane){
		      if(far_sur_cars[2][check_lane] > v_max){
			pref_lane = check_lane;
			v_max = far_sur_cars[2][check_lane];
		      }
		    }
		}
		cout << "preferable lane is " <<  pref_lane << "!" << endl;
		
		/*****************************************************************************
		 *  Trajectory generation
		 ****************************************************************************/
		

		// Generate coefficients of polynominal trajectories
		double T = 2;
		double accel_step = 0.2237;
		int n_accel = 2;

		vector<trajectory> possible_trajectories = generateTrajectories(s_start, d_start, lane, ref_vel, T, n_accel, accel_step);

		/*****************************************************************************
		 * Optimal path selection with cost function
		 ****************************************************************************/

		double acceptable_vel  = mph2ms(50.0 - 4.0);
		double acceptable_acc  = 10.0 - 2.0;
		double acceptable_jerk = 10.0 - 0.5;

		// weights for each cost function
		double weight_max_acc          = 1000; // acceralation
		double weight_safety           = 10;    // collision
		double weight_speed            = 1;    // speed limit and efficiency
		double weight_comfort          = 0;    // jerk
		double weight_lane_efficiency  = 1;    // the d difference between present lane and the highest speed lane
		double weight_sum_accel        = 0;
		double weight_sum_steer        = 0;

		vector<double> weights = {weight_max_acc, weight_safety, weight_speed, weight_comfort, weight_lane_efficiency, weight_sum_accel, weight_sum_steer};
	  
		// compute cost for each possible trajectories and select the minimum cost trajectory
		double min_cost = 9999999;
		int i_min = 0;

		for(int i=0; i<possible_trajectories.size(); i++){
		  vector<double> eval_coeffs_s = possible_trajectories[i].coeffs_s;
		  vector<double> eval_coeffs_d = possible_trajectories[i].coeffs_d;
		  
		  double cost_max_acc          = costMaxAcc(eval_coeffs_s, eval_coeffs_d, acceptable_acc, n_path, dt);
		  double cost_safety           = costSafety(eval_coeffs_s,  eval_coeffs_d, near_sur_cars, n_path, dt);
		  double cost_speed            = costSpeed(eval_coeffs_s, eval_coeffs_d, acceptable_vel, n_path, dt);
		  double cost_comfort          = costComfort(eval_coeffs_s, eval_coeffs_d, acceptable_jerk, n_path, dt); // this function does not work well
		  double cost_lane_efficiency  = costLaneEfficiency(eval_coeffs_d, pref_lane, n_path, dt);
		  double cost_sum_accel        = costSpeed(eval_coeffs_s, eval_coeffs_d, acceptable_vel, n_path, dt);
		  double cost_sum_steer        = costSumSteer(eval_coeffs_s, eval_coeffs_d, n_path, dt);

		  vector<double> costs = {cost_max_acc, cost_safety, cost_speed, cost_comfort, cost_lane_efficiency, cost_sum_accel, cost_sum_steer}; 
		  possible_trajectories[i].total_cost = weightedsumCosts(costs, weights);

		  if(possible_trajectories[i].total_cost <min_cost){
		    min_cost = possible_trajectories[i].total_cost;
		    i_min = i;
		  }
		}

		// minimum cost trajectory
		trajectory opt_trajectory = possible_trajectories[i_min];
		
		vector<double> coeffs_s = opt_trajectory.coeffs_s;
		vector<double> coeffs_d = opt_trajectory.coeffs_d;

		// print out optimal cost
		double opt_cost_max_acc          = costMaxAcc(coeffs_s, coeffs_d, acceptable_acc, n_path, dt);
		double opt_cost_safety           = costSafety(coeffs_s,  coeffs_d, near_sur_cars, n_path, dt);
		double opt_cost_speed            = costSpeed(coeffs_s, coeffs_d, acceptable_vel, n_path, dt);
		double opt_cost_comfort          = 0;
		double opt_cost_lane_efficiency  = costLaneEfficiency(coeffs_d, pref_lane, n_path, dt);
		double opt_cost_sum_accel        = costSpeed(coeffs_s, coeffs_d, acceptable_vel, n_path, dt);
		double opt_cost_sum_steer        = costSumSteer(coeffs_s, coeffs_d, n_path, dt);

		vector<double> costs = {opt_cost_max_acc, opt_cost_safety, opt_cost_speed, opt_cost_comfort, opt_cost_lane_efficiency, opt_cost_sum_accel, opt_cost_sum_steer}; 

		int j_max_cost = 0;
		double max_cost = 0;
		vector<string> cost_names = {"max_acc", "safety",  "speed", "comfort", "lane_eff",  "sum_acc", "sum_steer"};
		cout << "weighted cost: " << endl;
		for(int j=0; j<7; j++){
		    double weighted_cost = weights[j]*costs[j];
		    cout << setprecision(3) << weighted_cost << " ";
		    if(weighted_cost > max_cost){
		      max_cost = weighted_cost;
		      j_max_cost = j;	
		    }
		}
		cout << "\n" << endl;
		cout << "max_cost is *"<< cost_names[j_max_cost] << "*!!!!! total cost is " << opt_trajectory.total_cost << endl;

		// update lane and reference speed
		lane    = opt_trajectory.lane;
		ref_vel = opt_trajectory.target_vel; 
		
		
		/*****************************************************************************
		 * Frenet -> Cartesian Coordinate transform
		 ****************************************************************************/

		// compute path point in *Frenet* coordinate with coefficients of optimal path 
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

		// Coordinate transform from Frenet to Cartesian
		for(unsigned int i=0; i< n_path	 - n_keep; i++){
		  vector<double> pointXY =  getXYspline(next_path_s[last_point+i][0], next_path_d[last_point+i][0], max_s, spline_x, spline_y, spline_dx, spline_dy);
		  next_x_vals.push_back(pointXY[0]);
		  next_y_vals.push_back(pointXY[1]);
		}

		previous_path_s = next_path_s;
		previous_path_d = next_path_d;
		
		cout << "----------------------------------------------------------\n" << endl;
		
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
