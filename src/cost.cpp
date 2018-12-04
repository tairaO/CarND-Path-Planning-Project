#include <math.h>
#include <iostream>
#include <vector>
#include <cmath>
#include "helper.h"

using namespace std;


double costMaxAcc(vector<double> coeffs_s, vector<double> coeffs_d, double acceptable_acc, int n_path, double dt){
  /*
    penalizes the highest accleration in the trajectory. This checks the feasibility, therefore this is a binary cost function.
   */
  vector<double> dds_coeffs  = polydiff_coeffs(coeffs_s,  2);
  vector<double> ddd_coeffs  = polydiff_coeffs(coeffs_d,  2);
  double max_acc_squared  = 0;
  double acceptable_acc_squared  = acceptable_acc  * acceptable_acc;
  for(int ti = 1; ti < n_path; ti++){
    double tmp_acc_squared   = pow(polyeval(dds_coeffs,  ti*dt), 2) + pow(polyeval(ddd_coeffs,   ti*dt), 2);
      if(tmp_acc_squared > max_acc_squared){
      max_acc_squared = tmp_acc_squared;
    }
  }

  // cout << "max_acc: " << max_acc_squared << endl;
  bool acceptable = (max_acc_squared < acceptable_acc_squared);
  if(acceptable){
    return 0.0;
  }else{
    return 1.0;
    }
}



double costSafety(vector<double> coeffs_s, vector<double> coeffs_d, vector<vector<double>> near_sur_cars , int n_path, double dt){
  /*
    penalizes thedistance between nerest car in the same lane.
   */
  int n_decimate = 1;
  double min_dist = 30;
  double normalizer = 30;
  double lane_width = 4;
  int lane_num  = 3;
  
  for(int ti = 1; ti < n_path/n_decimate; ti++){
    double car_s = polyeval(coeffs_s, ti*n_decimate*dt);
    double car_d = polyeval(coeffs_d, ti*n_decimate*dt);      
    for(int ci=0; ci<near_sur_cars[1].size(); ci++){   
      if(abs(car_d - near_sur_cars[1][ci]) < 0.9 * lane_width){
	double check_car_s = (near_sur_cars[0][ci] + near_sur_cars[1][ci]*ti*n_decimate*dt);
	double dist_s = abs(car_s - check_car_s);
	if (dist_s < min_dist){
	  min_dist = dist_s;
	}
      }
    }
      	
  }
  // cout << "min_dist: " << min_dist << endl;

  return 1 - pow(min_dist/normalizer, 2);
}



double costSpeed(vector<double> coeffs_s, vector<double> coeffs_d, double acceptable_vel, int n_path, double dt){
  /*
    penalize the speed lower than 46 mph, and higher than 50 mph. This checks leagality and efficiency.
   */
  vector<double> ds_coeffs      = polydiff_coeffs(coeffs_s,  1);
  vector<double> dd_coeffs      = polydiff_coeffs(coeffs_d,  1);
  double leagal_vel_squared     = 50.0*50.0;
  double acceptable_vel_squared = acceptable_vel*acceptable_vel;
  double max_vel_squared  = 0;
  for(int ti = 1; ti < n_path; ti++){
    double tmp_ds = polyeval(ds_coeffs, ti*dt);
    if (tmp_ds < 0){
      return 1;
    }
    double tmp_vel_squared  = pow(tmp_ds, 2) + pow(polyeval(dd_coeffs, ti*dt), 2);
    if(tmp_vel_squared > max_vel_squared){
      max_vel_squared = tmp_vel_squared;
    }
  }

  bool illeagal    = (max_vel_squared > leagal_vel_squared);
  bool over_target = (max_vel_squared > acceptable_vel_squared);
  if(illeagal){
    return 1.0;
  }else if(over_target){
    return (max_vel_squared-acceptable_vel_squared)/(leagal_vel_squared - acceptable_vel_squared);
  }else{
    return (acceptable_vel_squared - max_vel_squared)/acceptable_vel_squared;
  }
}

double costComfort(vector<double> coeffs_s, vector<double> coeffs_d, double acceptable_jerk, int n_path, double dt){
  /*
    check comfort (not in use)
   */
  vector<double> ddds_coeffs = polydiff_coeffs(coeffs_s, 3);
  double acceptable_jerk_squared = acceptable_jerk * acceptable_jerk;
  double max_jerk_squared = 0;
  for(int ti = 1; ti < n_path; ti++){
    double tmp_ddds = polyeval(ddds_coeffs, ti*dt);
    double tmp_jerk_squared = pow(polyeval(ddds_coeffs, ti*dt), 2);
    if(tmp_jerk_squared > max_jerk_squared){
      max_jerk_squared = tmp_jerk_squared;
    }
  }
  // cout << "jerk: " << acceptable_jerk_squared << " " << max_jerk_squared << endl;
  return 1.0 - pow((acceptable_jerk_squared - max_jerk_squared)/acceptable_jerk_squared, 2);
}


double costLaneEfficiency(vector<double> coeffs_d, int pref_lane, int n_path, double dt){
  /*
    penarizes the Frenet d distance between current position and preferable lane.
   */
  double pref_d = lane_d(pref_lane);
  double eval_d = polyeval(coeffs_d, n_path*dt);
  double max_d  = 4 * 3; // lane width * lane number
  return abs(pref_d - eval_d) / max_d;
}

double costSpeedEfficiency(vector<double> coeffs_s, int n_path, double dt){
  /*
    penalize low speed (not in use)
   */
  vector<double> ds_coeffs   = polydiff_coeffs(coeffs_s,  1);
  double max_vel = mph2ms(50.0);
  double eval_vel = polyeval(ds_coeffs, n_path*dt);

  return (max_vel - eval_vel)/max_vel;
}

double costSumAccel(vector<double> coeffs_s, double acceptable_acc, int n_path, double dt){
  /*
    penalize the sum of acceleration (not in use)
   */
  int n_decimate    = 5;
  double sum_accel  = 0;
  double normalizer = (n_path/n_decimate-1) *acceptable_acc;
  vector<double> dds_coeffs = polydiff_coeffs(coeffs_s, 2);
  for(int i=1; i < n_path/n_decimate; i++){
    sum_accel += polyeval(dds_coeffs, n_decimate*i*dt);
  }
  
  return sum_accel / normalizer;
}

double costSumSteer(vector<double> coeffs_s, vector<double> coeffs_d, int n_path, double dt){
  /*
    penalize the sum of steering (not in use)
   */
  int n_decimate    = 5;
  double lane_width = 4;
  double max_steer = deg2rad(45);
  double normalizer = (n_path/n_decimate-1) * max_steer ;
  vector<double> ds_coeffs = polydiff_coeffs(coeffs_s, 1);
  vector<double> ddd_coeffs = polydiff_coeffs(coeffs_d, 2);

  double sum_steer_squared  = 0;
  for(int i=1; i < n_path/n_decimate; i++){
    
    double steer_angle = 0;
    double tmp_ds = polyeval(ds_coeffs, n_decimate*i*dt);
    if (tmp_ds < 2.0){
      steer_angle = 0;
    }else{
      steer_angle = polyeval(ddd_coeffs, n_decimate*i*dt) / tmp_ds;// when angle is small, sin(x) = x
    }
    sum_steer_squared += pow(steer_angle, 2);
  }
  
  return sum_steer_squared / pow(normalizer, 2);
}



double weightedsumCosts(vector<double> costs, vector<double> weights){
  /*
    Compute weighted sum of the cost
   */
  double total_cost;
  for(int i=0; i<costs.size(); i++){
    total_cost += weights[i]* costs[i];
  }
  return total_cost;
}

