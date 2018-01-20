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
  	static map<string, int> lane_direction;    // lane direction, see vehicle_fsm.cpp for more detail

  	string cur_state;     // current fsm state
  	int cur_lane;         // vehicle's current lane
  	int max_lane;         // max lane number that is drivable
  	double max_speed;     // max speed of the vehicle
  	double max_accel;     // max acceleration of the vehicle

  	// constructor
  	VehicleFSM(const double &max_speed, const double &max_accel, const int &max_lane);

  	// desctructor
  	virtual ~VehicleFSM();

  	// 
  	bool is_state_initialized();
  	void initialize_state(const int &cur_lane);

  private:
  	bool state_initialized = false;

  	vector<string> successor_states();
};

#endif