#ifndef COST_H
#define COST_H
#include <vector>

// cost functions
double costMaxAcc(vector<double> coeffs_s, vector<double> coeffs_d, double acceptable_acc, int n_path, double dt);
double costSafety(vector<double> coeffs_s, vector<double> coeffs_d, vector<vector<double>> near_sur_cars , int n_path, double dt);
double costSpeed(vector<double> coeffs_s, vector<double> coeffs_d, double acceptable_vel, int n_path, double dt);
double costComfort(vector<double> coeffs_s, vector<double> coeffs_d, double acceptable_jerk, int n_path, double dt);
double costLaneEfficiency(vector<double> coeffs_d, int pref_lane, int n_path, double dt);
double costSpeedEfficiency(vector<double> coeffs_s, int n_path, double dt);
double costSumAccel(vector<double> coeffs_s, double acceptable_acc, int n_path, double dt);
double costSumSteer(vector<double> coeffs_s, vector<double> coeffs_d, int n_path, double dt);

// methods for cost functions
double weightedsumCosts(vector<double> costs, vector<double> weights);

#endif
