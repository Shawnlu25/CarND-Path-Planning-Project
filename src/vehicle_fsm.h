#ifndef VEHICLE_FSM_H
#define VEHICLE_FSM_H

#include <string>
#include <map>
#include <vector>

using namespace std;

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
  	static map<State, int> lane_direction;    // lane direction, see vehicle_fsm.cpp for more detail

  	State cur_state;            // current fsm state
  	int cur_lane;               // current lane
  	double cur_speed;           // current speed
  	double cur_accel;
  	int cur_target_lane;        // vehicle's current lane
  	double cur_target_speed;    // current target speed

   	int max_lane;               // max lane number that is drivable
  	double max_speed;           // max speed of the vehicle
  	double max_accel;           // max acceleration of the vehicle
  	double max_jerk;            // max jerk of the vehicle

  	// constructor
  	VehicleFSM(const double &max_speed, const double &max_accel, const double &max_jerk, 
  		       const int &max_lane);

  	// desctructor
  	virtual ~VehicleFSM();

  	// check / initialize current state of FSM, including the initial vehicle state
  	bool is_state_initialized();
  	void initialize_state(const int &cur_lane, const double &cur_speed, const double &cur_accel);

  private:
  	bool state_initialized = false;

  	vector<State> successor_states();
};

#endif