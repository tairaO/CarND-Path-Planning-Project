#ifndef HELPER_H
#define HELPER_H
#include <vector>
#include "spline.h"

using namespace std;

struct trajectory{
  vector<double>  coeffs_s;
  vector<double>  coeffs_d;
  int lane;
  double target_vel;
  double total_cost;

};

// unit transform
constexpr double pi();
double deg2rad(double x);
double rad2deg(double x);
double lane_d(int lane_num);
double mph2ms(double v_mph);

double distance(double x1, double y1, double x2, double y2);

// Frenet transform
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y);
int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y);

// vector<double> getXYspline(double s, double d, double max_s, const tk::spline &spline_x, const tk::spline &spline_y, const tk::spline &spline_dx, const tk::spline &spline_dy);

// Jerk minimizing trajectories
vector<double> JMT(vector< double> start, vector <double> end, double T);
double polyeval(vector<double> coeffs, double t);
vector<double> polydiff_coeffs(vector<double> coeffs, int n_diff);

// sensor fusion
double my_lane_velocity(vector<vector<double>> sensor_fusion, double car_s,  int lane, int prev_size);
vector<vector<double>> surroundingCarState(vector<vector<double>> sensor_fusion, double car_s, double front_dist, double behind_dist, int n_keep, double dt);
void printSurroundingCarState(vector<vector<double>> sur_car_states);

// generate trajectory
vector<trajectory> generateTrajectories(vector<double> s_start, vector<double> d_start, int lane, double ref_vel, double T, int n_accel, double accel_step);


#endif
