#ifndef COMMON_H
#define COMMON_H

#include <vector>
#include <math.h>

using namespace std;

struct LocalizationState {
  double x;
  double y;
  double s;
  double d;
  double yaw;
  double speed;
};

vector<double> transform_to_ref_coord(
	double x, double y, double ref_x, double ref_y, double ref_yaw) {
  double shift_x = x - ref_x;
  double shift_y = y - ref_y;
  double new_x = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
  double new_y = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
  return {new_x, new_y};
}

vector<double> transform_from_ref_coord(
	double x, double y, double ref_x, double ref_y, double ref_yaw) {
  double new_x = x * cos(ref_yaw) - y * sin(ref_yaw);
  double new_y = x * sin(ref_yaw) + y * cos(ref_yaw);
  return {new_x + ref_x, new_y + ref_y};
}

#endif