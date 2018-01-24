#include "vehicle_fsm.h"
#include "common.h"
#include <cassert>
#include <cmath>
#include <iostream>

#define TOTAL_LENGTH 6945.5

using namespace predictions;
using namespace trajectories;
using namespace vehicle_fsm;
using namespace std;

double StateCostFunctions::forward_lane_speed_cost(const TrajectoryEndRef &traj_end_ref, 
    const vector<vector<double>> &sensor_fusions, SensorFusionPredictor &predictor, 
    Behavior behavior, double timespan) {
  if (behavior.target_lane_id < 0 || behavior.target_lane_id > 2) {
    return 1000.0;
  }
  const vector<double> front_vehicle_sf = 
    predictor.get_front_vehicle_sf_in_time(sensor_fusions, behavior.target_lane_id, traj_end_ref.s, timespan);
  if (front_vehicle_sf.size() <= 0) {
    return 0.0;
  }
  double vx = front_vehicle_sf[SensorFusionIndex::VX];
  double vy = front_vehicle_sf[SensorFusionIndex::VY];
  return abs(sqrt(vx*vx + vy*vy) / Constants::max_speed);
}

double StateCostFunctions::forward_lane_distance_cost(const TrajectoryEndRef &traj_end_ref, 
    const vector<vector<double>> &sensor_fusions, SensorFusionPredictor &predictor, 
    Behavior behavior, double timespan) {
  if (behavior.target_lane_id < 0 || behavior.target_lane_id > 2) {
    return 1000.0;
  }
  const vector<double> front_vehicle_sf = 
    predictor.get_front_vehicle_sf_in_time(sensor_fusions, behavior.target_lane_id, traj_end_ref.s, timespan);
  if (front_vehicle_sf.size() == 0) {
    return 0.0;
  }
  double distance = front_vehicle_sf[SensorFusionIndex::S] - traj_end_ref.s;
  if (front_vehicle_sf[SensorFusionIndex::S] - traj_end_ref.s < 0) {
    distance += TOTAL_LENGTH;
  }
  if (distance - Constants::max_speed * 10.0 < 0) {
    return 0.0;
  }
  return 1.0 - (distance - Constants::max_speed * 10.0) / Constants::max_speed * 10.0;
}

// Lane Direction Definition
map<VehicleFSM::State, int> VehicleFSM::lane_direction = {
  {VehicleFSM::State::PLCL, -1}, {VehicleFSM::State::LCL, -1}, 
  {VehicleFSM::State::PLCR, 1}, {VehicleFSM::State::LCR, -1}
};

// Destructor
VehicleFSM::~VehicleFSM(){}

// Constructor
VehicleFSM::VehicleFSM(SensorFusionPredictor *predictor){
  this->predictor_p = predictor;
  this->cur_state = VehicleFSM::State::KL;
}

vector<VehicleFSM::State> VehicleFSM::successor_states() {
  vector<VehicleFSM::State> next_states;

  next_states.push_back(VehicleFSM::State::KL);
  if (this->cur_state == VehicleFSM::State::KL) { 
  	next_states.push_back(VehicleFSM::State::PLCL);  	
  	next_states.push_back(VehicleFSM::State::PLCR);
  } else if (this->cur_state == VehicleFSM::State::PLCL) {
  	next_states.push_back(VehicleFSM::State::PLCL);
  	next_states.push_back(VehicleFSM::State::LCL);
  } else if (this->cur_state == VehicleFSM::State::PLCR) {
  	next_states.push_back(VehicleFSM::State::PLCR);
  	next_states.push_back(VehicleFSM::State::LCR);
  }

  return next_states;
}

Behavior VehicleFSM::get_behavior_for_state(VehicleFSM::State state, 
    const TrajectoryEndRef &traj_end_ref, 
    const vector<vector<double>> &sensor_fusions, double timespan) {
  Behavior behavior = {current_lane_from_d(traj_end_ref.d), -1, Constants::max_speed, -1};
  const vector<double> front_vehicle_sf = 
    this->predictor_p->get_front_vehicle_sf_in_time(sensor_fusions, current_lane_from_d(traj_end_ref.d), traj_end_ref.s, timespan);
  if (front_vehicle_sf.size() != 0) {
    double fv_vx = front_vehicle_sf[SensorFusionIndex::VX];
    double fv_vy = front_vehicle_sf[SensorFusionIndex::VY];
    behavior.target_speed = sqrt(fv_vx * fv_vx + fv_vy * fv_vy);
    behavior.target_leading_vehicle_id = front_vehicle_sf[SensorFusionIndex::ID];
    if (front_vehicle_sf[SensorFusionIndex::S] - traj_end_ref.s < 0) {
      behavior.distance_to_target_vehicle = front_vehicle_sf[SensorFusionIndex::S] - traj_end_ref.s + 6945.5;
    } else {
      behavior.distance_to_target_vehicle = front_vehicle_sf[SensorFusionIndex::S] - traj_end_ref.s;             
    }            
  }
  if (state == VehicleFSM::State::LCL) {
    behavior.target_lane_id = max(current_lane_from_d(traj_end_ref.d) - 1, 0);
  } else if (state == VehicleFSM::State::LCR) {
    behavior.target_lane_id = min(current_lane_from_d(traj_end_ref.d) + 1, 2);
  }

  /*cout << "FSM|State: " << state << "|lane_id: " << behavior.target_lane_id <<
          "|vehi_id: " << behavior.target_leading_vehicle_id <<
          "|speed: " << behavior.target_speed <<
          "|distance: " << behavior.distance_to_target_vehicle << endl;*/

  return behavior;
}

Behavior VehicleFSM::get_behavior(const TrajectoryEndRef &traj_end_ref, 
    const vector<vector<double>> &sensor_fusions, double timespan){
  vector<vector<double>> current = 
    this->predictor_p->get_near_vehicle_sf_in_time(sensor_fusions, current_lane_from_d(traj_end_ref.d), traj_end_ref.s, timespan);  
  vector<vector<double>> left = 
    this->predictor_p->get_near_vehicle_sf_in_time(sensor_fusions, current_lane_from_d(traj_end_ref.d)-1, traj_end_ref.s, timespan);
  vector<vector<double>> right = 
    this->predictor_p->get_near_vehicle_sf_in_time(sensor_fusions, current_lane_from_d(traj_end_ref.d)+1, traj_end_ref.s, timespan);
  bool can_left_turn = true;
  bool can_right_turn = true;
  bool can_turn = false;
  if (current.size() > 0) {
    can_turn = true;
  }
  if (left.size() > 0 || current_lane_from_d(traj_end_ref.d) <= 0) {
    can_left_turn = false;
  }
  if (right.size() > 0 || current_lane_from_d(traj_end_ref.d) >= 2) {
    can_right_turn = false;
  }

  Behavior kl_b = get_behavior_for_state(VehicleFSM::State::KL, traj_end_ref, sensor_fusions, timespan);
  Behavior l_b = get_behavior_for_state(VehicleFSM::State::LCL, traj_end_ref, sensor_fusions, timespan);  
  Behavior r_b = get_behavior_for_state(VehicleFSM::State::LCR, traj_end_ref, sensor_fusions, timespan);    

  //cout << "LEFT - " << (can_left_turn ? "T" : "F") << "-" << left.size() 
  //     << " RIGHT - " << (can_right_turn ? "T" : "F")  << "-" << right.size()<< endl;
  //if (StateCostFunctions::forward_lane_speed_cost(traj_end_ref, sensor_fusions, *(this->predictor_p), kl_b, timespan) > 0.05){
    if (!can_turn) {
      cout << "KEEP" << endl;
      return kl_b;
    }
    if (can_left_turn) {
      cout << "LEFT" << endl;
      cur_state = VehicleFSM::State::LCL;
      return l_b;
    } 
    if (can_right_turn){
      cur_state = VehicleFSM::State::LCR;      
      cout << "RIGHT" << endl;
      return r_b;
    }
  //}

  cout << "MAX" << endl;
  return kl_b;
}