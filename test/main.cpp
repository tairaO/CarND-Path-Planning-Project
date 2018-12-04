#include <fstream>
#include <math.h>
#include <iostream>
#include <thread>
#include <vector>

using namespace std;



double polydifferential(vector<double> coeffs, double t, int n_diff){
  double sum = 0;
  int n = coeffs.size();
  if (n < n_diff){
    return 0;
  }
  
  for(int i=n_diff; i<n; i++){
    double coeff_diff = 1;
    for(int j=0; j<n_diff ; j++){
      coeff_diff *= i-j;
    }
      
    sum += coeff_diff * coeffs[i] * pow(t, i-n_diff);
  }
  return sum;
}

vector<double> polydiff_coeffs(vector<double> coeffs, int n_diff){
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

double polyeval(vector<double> coeffs, double t){

  double sum = 0;
  int n = coeffs.size();
  for(int i=0; i<n ; i++){
    sum += coeffs[i] * pow(t, i);
  }
  return sum;
}


int main(){

  vector<double> coeffs = {1, 1, 1, 1};
  double t = 0.5;
  
  for(int i=1; i<5; i++){
    cout << i << " " << polydifferential(coeffs, t, i) << endl;
  }

  vector<double> coeffs2;
  coeffs2 = polydiff_coeffs(coeffs, 4);

  cout<< polyeval(coeffs2, 0.5) << endl;
  
  return 0;
}
