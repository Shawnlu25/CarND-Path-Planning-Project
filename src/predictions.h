#ifndef PREDICTIONS_H
#define PREDICTIONS_H

#include <vector>
#include <math.h>
#include "common.h"

namespace predictions {

enum SensorFusionIndex {
	ID = 0,
	X = 1,
	Y = 2, 
	VX = 3,
	VY = 4,
	S = 5,
	D = 6
};

class SensorFusionPredictor {

private:
	MapWayPoints map_waypoints;

public:
	const vector<double> get_vehicle_data_by_id(const vector<vector<double>> &sensor_fusion, int id);
	const vector<double> get_front_vehicle_sf_in_time(const vector<vector<double>> &sensor_fusions, int lane, double min_s, double timespan);
	const vector<vector<double>> get_near_vehicle_sf_in_time(const vector<vector<double>> &sensor_fusions, int lane, double min_s, double timespan);	
	const vector<double> get_vehicle_kinematics_in_time(const vector<double> &sensor_fusion, double timespan);

	SensorFusionPredictor(const MapWayPoints &map_waypoints) {this->map_waypoints = map_waypoints;}
};

const vector<double> get_vehicle_sf_by_id(const vector<vector<double>> &sensor_fusions, int id);

}

#endif