#include "vehicle_fsm.h"
#include <cassert>

// Lane Direction Definition
map<string, int> VehicleFSM::lane_direction = {{"PLCL", -1}, {"LCL", -1}, {"PLCR", 1}, {"PLCR", -1}};

// Destructor
VehicleFSM::~VehicleFSM(){}

// Constructor
VehicleFSM::VehicleFSM(const double &max_speed, const double &max_accel, const int &max_lane){
  this->max_speed = max_speed;
  this->max_accel = max_accel;
  this->max_lane  = max_lane;
}

bool VehicleFSM::is_state_initialized() {
  return this->state_initialized;
}

void VehicleFSM::initialize_state(const int &cur_lane) {
  assert(cur_lane >= 0 && cur_lane <= this->max_lane);

  this->cur_lane = cur_lane;
  this->state_initialized = true;
}

vector<string> VehicleFSM::successor_states() {
  vector<string> next_states;

  next_states.push_back("KL");
  if (this->cur_state.compare("KL") == 0) { 
  	if (this->cur_lane != 0) {
  	  next_states.push_back("PLCL");
  	}
  	if (this->cur_lane != this->max_lane) {
  	  next_states.push_back("PLCR");
  	}
  } else if (this->cur_state.compare("PLCL") == 0) {
  	if (this->cur_lane != 0) {
  	  next_states.push_back("PLCL");
  	  next_states.push_back("LCL");
  	}
  } else if (this->cur_state.compare("PLCR") == 0) {
  	if (this->cur_lane != this->max_lane) {
  	  next_states.push_back("PLCR");
  	  next_states.push_back("LCR");
  	}
  }

  return next_states;
}
