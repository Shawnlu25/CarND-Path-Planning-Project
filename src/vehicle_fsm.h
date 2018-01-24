#ifndef VEHICLE_FSM_H
#define VEHICLE_FSM_H

#include <string>
#include <map>
#include <vector>
#include "trajectories.h"
#include "predictions.h"

using namespace trajectories;
using namespace predictions;
using namespace std;

namespace vehicle_fsm{
class StateCostFunctions {
public:
  static double forward_lane_speed_cost(const TrajectoryEndRef &traj_end_ref, 
      const vector<vector<double>> &sensor_fusions, SensorFusionPredictor &predictor, 
      Behavior behavior, double timespan);
  static double forward_lane_distance_cost(const TrajectoryEndRef &traj_end_ref, 
      const vector<vector<double>> &sensor_fusions, SensorFusionPredictor &predictor, 
      Behavior behavior, double timespan);  

};

class VehicleFSM {
  /**
  Provides an interface for vehicle behavior planning, the current
  supported states are : KL, LCL, LCR, PLCL, PLCR
  */
public:
  enum State {
    KL,
    PLCL,
    LCL,
    PLCR,
    LCR
  };

private:
  static map<State, int> lane_direction;    // lane direction, see vehicle_fsm.cpp for more detail 
   
  SensorFusionPredictor *predictor_p;
  State cur_state;

  vector<State> successor_states();
  Behavior get_behavior_for_state(State state, 
    const TrajectoryEndRef &traj_end_ref, 
    const vector<vector<double>> &sensor_fusions, double timespan);

public:
  // constructor
  VehicleFSM(SensorFusionPredictor *predictor);

  // desctructor
  virtual ~VehicleFSM();

  // Retrieve new behavior
  Behavior get_behavior(const TrajectoryEndRef &traj_end_ref, 
      const vector<vector<double>> &sensor_fusions, double timespan);
};

}
#endif