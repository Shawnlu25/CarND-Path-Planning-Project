#include "vehicle_fsm.h"
#include <cassert>

// Lane Direction Definition
map<VehicleFSM::State, int> VehicleFSM::lane_direction = {
  {VehicleFSM::State::PLCL, -1}, {VehicleFSM::State::LCL, -1}, 
  {VehicleFSM::State::PLCR, 1}, {VehicleFSM::State::LCR, -1}
};

// Destructor
VehicleFSM::~VehicleFSM(){}

// Constructor
VehicleFSM::VehicleFSM(const double &max_speed, const double &max_accel, const int &max_lane){
  this->max_speed = max_speed;
  this->max_accel = max_accel;
  this->max_lane  = max_lane;
  this->cur_state = VehicleFSM::State::KL;
}

bool VehicleFSM::is_state_initialized() {
  return this->state_initialized;
}

void VehicleFSM::initialize_state(const int &cur_lane) {
  assert(cur_lane >= 0 && cur_lane <= this->max_lane);

  this->cur_lane = cur_lane;
  this->state_initialized = true;
}

vector<VehicleFSM::State> VehicleFSM::successor_states() {
  vector<VehicleFSM::State> next_states;

  next_states.push_back(VehicleFSM::State::KL);
  if (this->cur_state == VehicleFSM::State::KL) { 
  	if (this->cur_lane != 0) {
  	  next_states.push_back(VehicleFSM::State::PLCL);
  	}
  	if (this->cur_lane != this->max_lane) {
  	  next_states.push_back(VehicleFSM::State::PLCR);
  	}
  } else if (this->cur_state == VehicleFSM::State::PLCL) {
  	if (this->cur_lane != 0) {
  	  next_states.push_back(VehicleFSM::State::PLCL);
  	  next_states.push_back(VehicleFSM::State::LCL);
  	}
  } else if (this->cur_state == VehicleFSM::State::PLCR) {
  	if (this->cur_lane != this->max_lane) {
  	  next_states.push_back(VehicleFSM::State::PLCR);
  	  next_states.push_back(VehicleFSM::State::LCR);
  	}
  }

  return next_states;
}
