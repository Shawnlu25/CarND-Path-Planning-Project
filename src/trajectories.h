#ifndef TRAJECTORIES_H
#define TRAJECTORIES_H

#include <vector>
#include "common.h"

using namespace std;

namespace trajectories {

struct Trajectory {
  vector<double> pts_x;
  vector<double> pts_y;
};

struct TrajectoryEndRef {
  double x;
  double y;
  double yaw;
  double prev_x;
  double prev_y;  
  double s;
  double d;
  double speed;
  double accel;
};

struct Behavior {
  int target_lane_id;
  int target_leading_vehicle_id;
  double target_speed;
  double distance_to_target_vehicle;
};

class Constants {
public:
  static const double timestep;   // in seconds
  static const int max_step;

  static const double max_jerk;   // m/s^3
  static const double max_accel;  // m/s^2
  static const double max_speed;  // m/s
  static const double approach_speed_delta;

  static const int max_lane;      // max lane number

  static const double vehicle_radius;  // m
  static const double vehicle_buffer;  // m
  static const double safe_buffer;
};

class SplineTrajectoryGenerator {
private:
  MapWayPoints map_waypoints;

  Trajectory transform_traj_to_ref_coord(const TrajectoryEndRef &end_ref_data, const Trajectory &traj);
  Trajectory sample_points_on_lane(const TrajectoryEndRef &end_ref_data, int lane_num, double from_s);
  Trajectory generate_keep_lane_trajectory(const TrajectoryEndRef &end_ref_data, double target_speed, double max_s, int lane_num);

public:
  SplineTrajectoryGenerator(MapWayPoints &map_waypoints);

  TrajectoryEndRef get_traj_end_ref_data(const LocalizationState &localization,
      const Trajectory &previous_traj);
  Trajectory generate_trajectory(const Trajectory &previous_traj, 
      const Behavior &behavior, const LocalizationState &localization, 
      const vector<vector<double>> &sensor_fusions);

};

}

#endif