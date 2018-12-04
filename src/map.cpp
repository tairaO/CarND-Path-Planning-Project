#include <fstream>
#include <math.h>
#include <iostream>
#include <vector>
#include "helper.h"
#include "spline.h"

using namespace std;


Map::Map(void){
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
  
  spline_x.set_points(map_waypoints_s, map_waypoints_x);
  spline_x.set_points(map_waypoints_s, map_waypoints_y);
  spline_x.set_points(map_waypoints_s, map_waypoints_dx);
  spline_x.set_points(map_waypoints_s, map_waypoints_dy);

}
