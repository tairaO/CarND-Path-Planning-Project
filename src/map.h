
#include <vector>
#include "spline.h"

using namespace std;

class Map {
public:
  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;


  /**
   * Constructor
   */
  Map();

  /**
   * Destructor
   */
  virtual ~Map();

  tk::spline spline_x;
  tk::spline spline_y;
  tk::spline spline_dx;
  tk::spline spline_dy;


  vector<double> getXYspline(double s, double d);

};
  


