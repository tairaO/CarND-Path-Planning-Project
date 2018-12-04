#include <fstream>
#include <math.h>
#include <iostream>
#include <vector>
#include "Eigen-3.3/Eigen/Dense"
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helper.h"
#include "spline.h"

using namespace std;

// for convenience
using Eigen::MatrixXd;
using Eigen::VectorXd;



// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// ! Lane number 
double lane_d(int lane_num){
  if (lane_num == 2){
    return 2 + 4*lane_num -0.2; // for compensating spline error
  }else{
    return 2 + 4*lane_num;
  }
}
double mph2ms(double v_mph){ return v_mph/2.237;}

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}


// this is modified version for 10000 waypoints
// !room of improvement: this is optimized for 10000 points. If the number change, it is not work well. I should have implemented search method like binary search tree.
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

  double tmp_closestLen = 100000; //large number
  double closestLen = 100000; //large number
  int tmp_closestWaypoint = 0;
  int closestWaypoint = 0;
  int n = maps_x.size();
  for(int i = 0; i < n/100; i++)
    {
      double map_x = maps_x[i*100];
      double map_y = maps_y[i*100];
      double dist = distance(x,y,map_x,map_y);
      if(dist < closestLen)
	{
	  tmp_closestLen = dist;
	  tmp_closestWaypoint = i;
	}

    }
  int positive_i;
  for(int i = tmp_closestWaypoint-105; i < tmp_closestWaypoint+105; i++)
    { 
      if(i < 0){
	positive_i = n+i;
      }else{
	positive_i =i;
      }
      double map_x = maps_x[positive_i];
      double map_y = maps_y[positive_i];
      double dist = distance(x,y,map_x,map_y);
      if(dist < closestLen)
	{
	  closestLen = dist;
	  closestWaypoint = positive_i;
	}

    }

  return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
  int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);
  // cout << "closestWaypoint: " << closestWaypoint << endl;
  double map_x = maps_x[closestWaypoint];
  double map_y = maps_y[closestWaypoint];
	
  double heading = atan2((map_y-y),(map_x-x));

  double angle = fabs(theta-heading);
  angle = min(2*pi() - angle, angle);
  if(angle > pi()/4)
  {
    closestWaypoint++;
  if (closestWaypoint == maps_x.size())
  {
    closestWaypoint = 0;
  }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);
	// cout << "next_x: " << maps_x[next_wp] << endl;
	// cout << "next_y: " << maps_y[next_wp] << endl;
	// cout << "next_wp: " << next_wp << endl;
	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];
	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;
	double frenet_d = distance(x_x,x_y,proj_x,proj_y);
	//see if d value is positive or negative by comparing it to a center point
	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);
	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}



// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}

/*
vector<double> getXYspline(double s, double d, double max_s, const tk::spline &spline_x, const tk::spline &spline_y, const tk::spline &spline_dx, const tk::spline &spline_dy){
  s = fmod(s, max_s);
  double x = spline_x(s) + d * spline_dx(s);
  double y = spline_y(s) + d * spline_dy(s);
  return {x,y};
}
*/

vector<double> JMT(vector< double> start, vector <double> end, double T)
{
    /*
    Calculate the Jerk Minimizing Trajectory that connects the initial state
    to the final state in time T.
    */
    
    vector<double> coeffs;
    
    coeffs.push_back(start[0]);
    coeffs.push_back(start[1]);
    coeffs.push_back(start[2]/2.0);
    
    MatrixXd A(3, 3);
    A <<     pow(T,3),      pow(T,4),     pow(T, 5),
         3.0*pow(T,2),  4.0*pow(T,3),  5.0*pow(T,4),
                6.0*T, 12.0*pow(T,2), 20.0*pow(T,3);
    MatrixXd b(3, 1);
    b << end[0] - (start[0] + start[1]*T + 1.0/2.0 * start[2]*pow(T,2)),
         end[1] - (start[1] + start[2]*T),
         end[2] -  start[2];
    
    MatrixXd coeffs2 = A.inverse() * b;
    coeffs.push_back(coeffs2(0, 0));
    coeffs.push_back(coeffs2(1, 0));
    coeffs.push_back(coeffs2(2, 0));
    
    return coeffs;
}


double polyeval(vector<double> coeffs, double t){
  /*
    compute polynominals from coefficients
  */
  double sum = 0;
  int n = coeffs.size();
  for(int i=0; i<n ; i++){
    sum += coeffs[i] * pow(t, i);
  }
  return sum;
}

vector<double> polydiff_coeffs(vector<double> coeffs, int n_diff){
  /*
    compute coefficients of differential polynominal 
  */
  
  int n = coeffs.size();
  if (n < n_diff){
    return {};
  }

  vector<double> diff_coeffs;
  for(int i=n_diff; i<n; i++){
    double coeff_diff = 1;
    for(int j=0; j<n_diff ; j++){
      coeff_diff *= i-j;
    }
      
    diff_coeffs.push_back(coeff_diff * coeffs[i]);
  }
  return diff_coeffs;
}


double my_lane_velocity(vector<vector<double>> sensor_fusion, double car_s,  int lane, int prev_size){
  /*
    get my lane velocity if there is no front car, leagal speed is returned (not in use)
  */ 
  double my_lane_vel = 49.5; // [mph]
  my_lane_vel /= 2.237;
  double min_car_s = 999999;
  for(unsigned int i=0; i < sensor_fusion.size(); i++){
    double d = sensor_fusion[i][6];
    double d_min = lane_d(lane)-2;
    double d_max = lane_d(lane)+2;

    if ((d < d_max) && (d > d_min)){
      double vx = sensor_fusion[i][3];
      double vy = sensor_fusion[i][4];
      double check_speed = sqrt(vx*vx + vy*vy);
      double check_car_s = sensor_fusion[i][5];
      
      check_car_s += (double)prev_size*0.02*check_speed;

      if((check_car_s > car_s) && (check_car_s -car_s < 30)){
	my_lane_vel = check_speed;
	}
      }
    }
		  
  return my_lane_vel*2.237;
}



vector<vector<double>> surroundingCarState(vector<vector<double>> sensor_fusion, double car_s, double front_dist, double behind_dist, int prev_size, double dt){
  /*
    obtain position in Frenet and speed of the nearest surrounding car front and behind in each lane and speed at 0.02*prev_size seconds later (the constant velocity model)
  */  
  vector<double> sur_cars_vel(6, 999999);// {lane0-front, lane1-front, lane2-front, lane0-behind, lane1-behind, lane2-behind}
  vector<double> sur_cars_s(6, 999999);
  vector<double> sur_cars_d(6, 999999);
  int lane_num = 3;
    
  for(int i=0; i < lane_num; i++){
    double d_min = lane_d(i)-2;
    double d_max = lane_d(i)+2;
    double s_max = -99999999;
    double s_min =  99999999;

    for(int j=0; j< sensor_fusion.size();j++){  
      double d = sensor_fusion[j][6];
      if ((d < d_max) && (d > d_min)){
	double vx = sensor_fusion[j][3];
	double vy = sensor_fusion[j][4];
	double check_speed = sqrt(vx*vx + vy*vy);
	double check_car_s = sensor_fusion[j][5];
      
	check_car_s += (double)prev_size*dt*check_speed;

	// check front car in front_dist
	if((check_car_s >= car_s) && (check_car_s -car_s < front_dist) && (check_car_s < s_min)){
	  s_min =  check_car_s;
	  sur_cars_vel[i] = check_speed;
	  sur_cars_s[i]   = check_car_s;
	  sur_cars_d[i]   = d;
	  
	}
	// check behind cars in behind_dist
	if((check_car_s < car_s) && (car_s - check_car_s < behind_dist) && (check_car_s > s_max)){
	  s_max =  check_car_s;
	  sur_cars_vel[i+lane_num] = check_speed;
	  sur_cars_s[i+lane_num]   = check_car_s;
	  sur_cars_d[i+lane_num]   = d;
	}
      }
    }
  }  
  
  return {sur_cars_s, sur_cars_d, sur_cars_vel}; 
}

void printSurroundingCarState(vector<vector<double>> sur_car_states){
  /*
    output surrounding cars' state
  */
  int lane_num = 3;
  
  cout << "----------------" << endl; 
  cout << "|"; 
  for(int i=0; i<lane_num; i++){
    if(sur_car_states[1][i] < 99999){
      cout << "O|";
    }else{
      cout << "-|";
    }
  }
  cout << "\n";
  cout << "|"; 
  for(int i=0; i<lane_num; i++){
    if(sur_car_states[2][i+lane_num] < 99999){
      cout << "O|";
    }else{
      cout << "-|";
    }
  }
  cout << "\n";
  cout << "----------------" << endl; 

  for(int i=0; i< lane_num*2; i++){
    cout <<sur_car_states[2][i] << " ";
  }
  cout << endl;
}


vector<trajectory> generateTrajectories(vector<double> s_start, vector<double> d_start, int lane, double ref_vel, double T, int n_accel, double accel_step){
  /* 
     generate candidates trajectories
   */
  vector<trajectory> possible_trajectories;
  double car_s = s_start[0];
  double car_d = d_start[0];
  vector<double> s_end;
  vector<double> d_end;
  
  
  if (ref_vel < 10){ // special setup for low speed
    
    for(int vi=-n_accel; vi <= n_accel; vi++){
      possible_trajectories.push_back(trajectory());
      int n = possible_trajectories.size();
      possible_trajectories[n-1].target_vel = ref_vel + vi*accel_step;
      possible_trajectories[n-1].lane       = lane;
      s_end = {car_s + 2.0*mph2ms(possible_trajectories[n-1].target_vel)*T, mph2ms(possible_trajectories[n-1].target_vel), 0};
      d_end = {car_d, 0, 0};
      possible_trajectories[n-1].coeffs_s = JMT(s_start, s_end, T);
      possible_trajectories[n-1].coeffs_d = JMT(d_start, d_end, T);
      
    }
  }else{
    vector<vector<double>> tmp_keep_coeffs_s;
    vector<vector<double>> tmp_keep_coeffs_d;
    vector<double> tmp_keep_target_vel;
    vector<int> tmp_keep_lane;
    for(int vi=-n_accel; vi <= n_accel; vi++){
      double target_vel = ref_vel + vi*accel_step;
      s_end = {car_s + mph2ms(target_vel)*T, mph2ms(target_vel), 0};
      vector<double> coeffs_s = JMT(s_start, s_end, T);
      tmp_keep_target_vel.push_back(target_vel);
      tmp_keep_coeffs_s.push_back(coeffs_s);
    }

    for(int li=0; li<3; li++){
      if(abs(lane-li) < 2){ // two lane change is prohibited because in real car situations, it is dangerous
	double target_d = lane_d(li);
	d_end = {target_d, 0, 0};
	vector<double> coeffs_d = JMT(d_start, d_end, T);
	tmp_keep_lane.push_back(li);
	tmp_keep_coeffs_d.push_back(coeffs_d);
      }
    }
		
    for(int si=0; si<tmp_keep_coeffs_s.size(); si++){
      for(int di=0; di<tmp_keep_coeffs_d.size(); di++){
	possible_trajectories.push_back(trajectory());
	int n = possible_trajectories.size();
        possible_trajectories[n-1].target_vel = tmp_keep_target_vel[si];
        possible_trajectories[n-1].lane       = tmp_keep_lane[di];
        possible_trajectories[n-1].coeffs_s   = tmp_keep_coeffs_s[si];
        possible_trajectories[n-1].coeffs_d   = tmp_keep_coeffs_d[di];
      }
    }
  }

  return possible_trajectories;
}


