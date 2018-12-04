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
  s = fmod(s, max_s);
  double x = spline_x(s) + d * spline_dx(s);
  double y = spline_y(s) + d * spline_dy(s);
  return {x,y};
}

// added by taira
vector<double> getFrenetSpeed(double x, double y, double prev_x, double prev_y, const vector<double> &maps_x, const vector<double> &maps_y, const vector<double> &maps_dx, const vector<double> &maps_dy){
  /* 
     Compute speed in Frenet coordinate
     added by taira 2018//11/24
  */

  int close_wp = ClosestWaypoint(x, y, maps_x, maps_y);

  double vx = (x - prev_x)/0.02;
  double vy = (y - prev_y)/0.02;
  double dx = maps_dx[close_wp];
  double dy = maps_dy[close_wp];

  double vd = vx*dx + vy*dy;
  double vs = sqrt(vx*vx+vy*vy -vd*vd); // vs*vs + vd*vd = vx*vx + vy*vy

  return {vs, vd};
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
  string map_file_ = "../data/precise_highway_map.csv";
  // string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

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
 
  
  
  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy, &max_s, &lane, &ref_vel, &spline_x, &spline_y, &spline_dx, &spline_dy, &previous_path_s, &previous_path_d](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
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

          	// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
		double car_s_jmt   = car_s;
		double car_ds      = 0;
		double car_dds     = 0;
		double car_d_jmt   = car_d;
		double car_dd      = 0;
		double car_ddd     = 0;
		  
		// cout << "car_x: " << car_x << endl;
		// cout << "car_y: " << car_y << endl;
		int prev_size = previous_path_x.size();
		cout << "prev_size: " <<  prev_size << endl;
		
	
				
		
		 // walkthrough code
		/*
		if(prev_size > 0){
		  car_s = end_path_s;
		}
		bool too_close = false;
		for(unsigned int i=0; i < sensor_fusion.size(); i++){
		  float d = sensor_fusion[i][6];
		  double d_min = lane_d(lane)-2;
		  double d_max = lane_d(lane)+2;

		  if ((d < d_max) && (d > d_min)){
		    double vx = sensor_fusion[i][3];
		    double vy = sensor_fusion[i][4];
		    double check_speed = sqrt(vx*vx + vy*vy);
		    double check_car_s = sensor_fusion[i][5];

		    check_car_s += (double)prev_size*0.02*check_speed;

		    if((check_car_s > car_s) && (check_car_s -car_s < 30)){
		      too_close = true;
		      if(lane ==1){
			lane = 0;
		      }
		    }
		  }
		}
		
		if(too_close && (ref_vel > my_lane_vel-0.1)){
		  ref_vel -= 0.2237;
		}else if(ref_vel < 49.5){
		  ref_vel += 0.2237;
		}
		*/

		  /*

		double tmp_lane = lane;
		
		if(lane != pref_lane){
		  tmp_lane = lane-1;
		  cout << "check left!" << endl;
		  if(tmp_lane >= 0){ 
		    if((sur_car_states[1][tmp_lane] > 999) && (sur_car_states[1][tmp_lane+3] > 999)){
		      lane = tmp_lane;
		    }else{
		      tmp_lane = lane+1;
		      cout << "check right!" << endl;
		      if(tmp_lane <= 2){ 
			if((sur_car_states[1][tmp_lane] > 999) && (sur_car_states[1][tmp_lane+3] > 999)){
			  lane = tmp_lane;
			}
		      }
		    }
		  }else{
		    tmp_lane = lane+1;
		    cout << "check right!" << endl;
		    if(tmp_lane <= 2){ 
		      if((sur_car_states[1][tmp_lane] > 999) && (sur_car_states[1][tmp_lane+3] > 999)){
			lane = tmp_lane;
		      }
		    }	
		  }
		}
		

		vector<double> ptsx;
		vector<double> ptsy;
		
		double ref_x = car_x;
		double ref_y = car_y;
		double ref_yaw = deg2rad(car_yaw);

		if (prev_size < 2){
		  double prev_car_x = car_x - cos(car_yaw);
		  double prev_car_y = car_y - sin(car_yaw);
		  
		  ptsx.push_back(prev_car_x);
		  ptsx.push_back(car_x);
		  ptsy.push_back(prev_car_y);
		  ptsy.push_back(car_y);
		  
		}else{
		  ref_x = previous_path_x[prev_size-1];
		  ref_y = previous_path_y[prev_size-1];
		  
		  double prev_ref_x = previous_path_x[prev_size-2];
		  double prev_ref_y = previous_path_y[prev_size-2];
		  ref_yaw = atan2(ref_y-prev_ref_y, ref_x-prev_ref_x);

		  ptsx.push_back(prev_ref_x);
		  ptsx.push_back(ref_x);

		  ptsy.push_back(prev_ref_y);
		  ptsy.push_back(ref_y);

		}
		
		double next_d = lane_d(lane);

		
		vector<double> next_wp0 = getXYspline(car_s+30, next_d, max_s, spline_x, spline_y, spline_dx, spline_dy);
		vector<double> next_wp1 = getXYspline(car_s+60, next_d, max_s, spline_x, spline_y, spline_dx, spline_dy);
		vector<double> next_wp2 = getXYspline(car_s+90, next_d, max_s, spline_x, spline_y, spline_dx, spline_dy);
		
		
		// vector<double> next_wp0 = getXY(car_s+30, next_d, map_waypoints_s, map_waypoints_x,  map_waypoints_y);
		// vector<double> next_wp1 = getXY(car_s+60, next_d, map_waypoints_s, map_waypoints_x,  map_waypoints_y);
		// vector<double> next_wp2 = getXY(car_s+90, next_d, map_waypoints_s, map_waypoints_x,  map_waypoints_y);
		
		ptsx.push_back(next_wp0[0]);
		ptsx.push_back(next_wp1[0]);
		ptsx.push_back(next_wp2[0]);
		ptsy.push_back(next_wp0[1]);
		ptsy.push_back(next_wp1[1]);
		ptsy.push_back(next_wp2[1]);


		// global coordinate to car coordinate
		for (unsigned int i =0; i< ptsx.size(); i++){
		  double shift_x = ptsx[i] - ref_x;
		  double shift_y = ptsy[i] - ref_y;

		  ptsx[i] = shift_x * cos(0-ref_yaw) - shift_y*sin(0-ref_yaw);
		  ptsy[i] = shift_x * sin(0-ref_yaw) + shift_y*cos(0-ref_yaw);
		}
		
		
		tk::spline s;
		s.set_points(ptsx, ptsy);

		for(unsigned int i=0; i< prev_size; i++){
		  next_x_vals.push_back(previous_path_x[i]);
		  next_y_vals.push_back(previous_path_y[i]);
		}
		
		double target_x = 30.0;
		double target_y = s(target_x);
		double target_dist = distance(0.0, 0.0, target_x, target_y);
	       
		double x_add_on = 0;
		
		for(unsigned int i=1; i<= 50-prev_size; i++){
		  double N = target_dist/(0.02*mph2ms(ref_vel));
		  double x_point = x_add_on + target_x/N;
		  double y_point = s(x_point);

		  x_add_on = x_point;

		  double x_point_prev = x_point;
		  double y_point_prev = y_point;

		  //car coordinate to global coordinate 
		  x_point = x_point_prev*cos(ref_yaw) - y_point_prev*sin(ref_yaw) + ref_x;
		  y_point = x_point_prev*sin(ref_yaw) + y_point_prev*cos(ref_yaw) + ref_y;
		  
		  next_x_vals.push_back(x_point);
		  next_y_vals.push_back(y_point);
		}
		
		*/

		
		
		// Jerk minimize trajectory
		// Set velocity  !Potential imprvovement: this programa only consider path(s,d), not consider the time

		/*
		// if(car_s_jmt + 22 > sur_car_states[0][lane]){
		if(false){
		  ref_vel -= 0.2237 * 1.2;
		    }else if(ref_vel > my_lane_vel){ 
		  ref_vel -= 0.2237;
		}else if(ref_vel < 49.5){
		  ref_vel += 0.2237;
		  // ref_vel += 0.2237;
		}
		*/

		/*
		double tmp_lane = lane;
		if(lane != pref_lane){
		  tmp_lane = lane-1;
		  cout << "check left!" << endl;
		  if(tmp_lane >= 0){ 
		    if((sur_car_states[1][tmp_lane] > 999) && (sur_car_states[1][tmp_lane+3] > 999)){
		      lane = tmp_lane;
		    }else{
		      tmp_lane = lane+1;
		      cout << "check right!" << endl;
		      if(tmp_lane <= 2){ 
			if((sur_car_states[1][tmp_lane] > 999) && (sur_car_states[1][tmp_lane+3] > 999)){
			  lane = tmp_lane;
			}
		      }
		    }
		  }else{
		    tmp_lane = lane+1;
		    cout << "check right!" << endl;
		    if(tmp_lane <= 2){ 
		      if((sur_car_states[1][tmp_lane] > 999) && (sur_car_states[1][tmp_lane+3] > 999)){
			lane = tmp_lane;
		      }
		    }	
		  }
		}
		*/
		/*
		bool too_close = false;
		for(unsigned int i=0; i < sensor_fusion.size(); i++){
		  float d = sensor_fusion[i][6];
		  double d_min = lane_d(lane)-2;
		  double d_max = lane_d(lane)+2;

		  if ((d < d_max) && (d > d_min)){
		    double vx = sensor_fusion[i][3];
		    double vy = sensor_fusion[i][4];
		    double check_speed = sqrt(vx*vx + vy*vy);
		    double check_car_s = sensor_fusion[i][5];

		    check_car_s += (double)prev_size*0.02*check_speed;

		    if((check_car_s > car_s) && (check_car_s -car_s < 30)){
		      too_close = true;
		      
		      if(lane == 1){
			lane = 0;
		       
		      }
		    }
		  }
		}
		*/

		int n_keep = min(prev_size, 20);
		int n_path = 50;
		int last_point = 0;
		int n_consumed = n_path - prev_size;
		cout << "n_keep: " << n_keep << endl;

		vector<double> s_start;
		vector<double> s_end;
		vector<double> d_start;
		vector<double> d_end;
		
		
		/*
		cout << " prevous s" << endl;
		for(int i=50 - prev_size; i  < previous_path_s[0].size();i++){
		  cout << previous_path_s[0][i] <<endl;
		}
		cout << " prevous d" << endl;
		for(int i=50 - prev_size; i  < previous_path_s[0].size();i++){
		  cout << previous_path_d[0][i] <<endl;
		}
		*/

		

		vector<vector<double>> next_path_s;
		vector<vector<double>> next_path_d;
		
		if(prev_size < 2){
		  s_start = {car_s_jmt, 0, 0};
		  d_start = {car_d_jmt, 0, 0};
		  }else{
		  for(unsigned int i=0; i< n_keep; i++){
		    next_x_vals.push_back(previous_path_x[i]);
		    next_y_vals.push_back(previous_path_y[i]);
		    next_path_s.push_back(previous_path_s[n_consumed+i]);
		    next_path_d.push_back(previous_path_d[n_consumed+i]);
		  }
		  last_point = n_keep -1;
		  s_start   = next_path_s[last_point];
		  car_s_jmt = next_path_s[last_point][0];
		  car_ds    = next_path_s[last_point][1];
		  car_dds   = next_path_s[last_point][2];
		  
		  d_start   = next_path_d[last_point];
		  car_d_jmt = next_path_d[last_point][0];
		  car_dd    = next_path_d[last_point][1];
		  car_ddd   = next_path_d[last_point][2];
		}

		
		double my_lane_vel = my_lane_velocity(sensor_fusion, car_s_jmt, lane, prev_size);
		cout << "my_lane_vel: " << my_lane_vel << endl;
		
		vector<vector<double>> sur_car_states =surroundingCarState(sensor_fusion, car_s_jmt, 50, 22, 5);

		cout << "----------------" << endl; 
		cout << "|"; 
		for(int i=0; i<3; i++){
		  if(sur_car_states[1][i] < 99999){
		    cout << "O|";
		  }else{
		    cout << "-|";
		  }
		}
		cout << "\n";
		cout << "|"; 
		for(int i=0; i<3; i++){
		  if(sur_car_states[1][i+3] < 99999){
		    cout << "O|";
		  }else{
		    cout << "-|";
		  }
		}
		cout << "\n";
		cout << "----------------" << endl; 

		for(int i=0; i< 6; i++){
		  cout <<sur_car_states[1][i] << " ";
		}
		cout << "\n";

		vector<vector<double>> sur_car_states2 =surroundingCarState(sensor_fusion, car_s, 50, 1000, 1000);

		cout << "2--------------2" << endl; 
		cout << "|"; 
		for(int i=0; i<3; i++){
		  if(sur_car_states2[1][i] < 99999){
		    cout << "O|";
		  }else{
		    cout << "-|";
		  }
		}
		cout << "\n";
		cout << "|"; 
		for(int i=0; i<3; i++){
		  if(sur_car_states2[1][i+3] < 99999){
		    cout << "O|";
		  }else{
		    cout << "-|";
		  }
		}
		cout << "\n";
		cout << "----------------" << endl; 

		for(int i=0; i< 6; i++){
		  cout <<sur_car_states2[1][i] << " ";
		}
		cout << "\n";

		double v_max = sur_car_states2[1][lane];
		int pref_lane = lane;
		  
		for(int i=0;i<3; i++){
		  if(i!=lane){
		      if(sur_car_states2[1][i] > v_max){
			pref_lane = i;
			v_max = sur_car_states2[1][i];
		      }
		    }
		}
		cout << "preferable lane is " <<  pref_lane << "!" << endl;


		// Set velocity  !Potential imprvovement: this programa only consider path(s,d), not consider the time
		if(car_s + 22 > sur_car_states[0][lane]){
		  ref_vel -= 0.2237 * 1.2;
		    }else if(ref_vel > my_lane_vel){ 
		  ref_vel -= 0.2237;
		}else if(ref_vel < 46.0){
		  ref_vel += 0.2237;
		}
	

		 
		// easy lane finder
		if(lane != pref_lane){
		  if((lane == 1) && (sur_car_states[1][pref_lane] > 999) && (sur_car_states[1][pref_lane+3] > 999)){
		    lane = pref_lane;
		  }else{
		    if ((sur_car_states[1][1] > 999) && (sur_car_states[1][1+3] > 999)){
		      lane = 1;
		    }
		  }
		}
		
		
		double T = 2;
		double next_d = lane_d(lane);
		double target_vel = ref_vel;
		// double target_vel = 22;
		cout << "target_vel[m/s]" << mph2ms(target_vel) << endl;
		
		
		s_end = {car_s_jmt + 2.0*mph2ms(target_vel)*T, mph2ms(target_vel), 0};
		d_end = {car_d_jmt, 0, 0};

		if (target_vel > 10) // mph
		  {
		    d_end[0] = next_d;
		    s_end[0] = car_s_jmt + mph2ms(target_vel)*T;
		  }

		vector<double> coeffs_s = JMT(s_start, s_end, T);
		vector<double> coeffs_d = JMT(d_start, d_end, T);


		for(unsigned int i=1; i<= n_path-n_keep+4; i++){
		  double pointS = polyeval(coeffs_s, 0.02*i);
		  double pointD = polyeval(coeffs_d, 0.02*i);
		  next_path_s.push_back({pointS, 0, 0});
		  next_path_d.push_back({pointD, 0, 0});
		}
		
		for(unsigned int i=1; i<= n_path-n_keep+2; i++){
		  double tmp_ds = fmod(next_path_s[last_point+i+1][0] + max_s - next_path_s[last_point+i-1][0], max_s) / 0.04;
		  double tmp_dd = (next_path_d[last_point+i+1][0] - next_path_d[last_point+i-1][0]) / 0.04;
		  next_path_s[last_point+i][1] = tmp_ds;
		  next_path_d[last_point+i][1] = tmp_dd;
		}
			
		for(unsigned int i=1; i<= n_path-n_keep; i++){
		  double tmp_dds = (next_path_s[last_point+i+1][1] - next_path_s[last_point+i-1][1]) / 0.04;
		  double tmp_ddd = (next_path_d[last_point+i+1][1] - next_path_d[last_point+i-1][1]) / 0.04;
		  next_path_s[last_point+i][2] = tmp_dds;
		  next_path_d[last_point+i][2] = tmp_ddd;
		}
		
		for(unsigned int i=0; i< n_path	 - n_keep; i++){
		  vector<double> pointXY =  getXYspline(next_path_s[last_point+i][0], next_path_d[last_point+i][0], max_s, spline_x, spline_y, spline_dx, spline_dy);
		  next_x_vals.push_back(pointXY[0]);
		  next_y_vals.push_back(pointXY[1]);
		}

		previous_path_s = next_path_s;
		previous_path_d = next_path_d;
		
		/*
		cout << "next_x" << endl;
		for(int i=0; i< next_x_vals.size(); i++){
		  cout << next_x_vals[i] << endl;
		}
		cout << "next_y" << endl;
		
		for(int i=0; i< next_x_vals.size(); i++){
		  cout << next_y_vals[i] << endl;
		}		
		*/
		
		  
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
