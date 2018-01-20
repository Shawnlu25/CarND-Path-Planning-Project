#ifndef TRAJECTORIES_H
#define TRAJECTORIES_H

#include <vector>
#include "spline.h"

using namespace std;

namespace Trajectory {

struct TrajectoryPts {
  vector<double> xpts;
  vector<double> ypts;
};

struct 

TrajectoryPts keep_lane_trajectory(
  const vector<vector<double>> &sensor_fusions, 
  );


}

#endif