#include "predictions.h"
#include <cstddef>
#include <iostream>
using namespace predictions;

#define TOTAL_LENGTH 6945.5
#define ROUND_OVER_BUFFER 800.0

const vector<double> SensorFusionPredictor::get_vehicle_data_by_id(const vector<vector<double>> &sensor_fusion, int id) {
	for (int i = 0; i < sensor_fusion.size(); i++) {
		if (sensor_fusion[i][0] == id) {
			return sensor_fusion[i];
		}
	}
	return {};
}

const vector<double> *SensorFusionPredictor::get_future_front_vehicle_data(const vector<vector<double>> &sensor_fusion, const vector<double> &ref_data, double timespan) {
	const vector<double> *result = NULL;
	double min_s = 9999.0;	
	for (int i = 0; i < sensor_fusion.size(); i++) {
		vector<double> frenet = this->vehicle_frenet_state_in_time(sensor_fusion[i], timespan);
		if (current_lane_from_d(frenet[3]) == current_lane_from_d(ref_data[4])) {
			//cout << frenet[3] << "|" << ref_data[4] << "|" << min_s <<"|" << frenet[0]<< endl;
			if (ref_data[3] < frenet[0] || (frenet[0] < 1000 && ref_data[3] > 5500 && frenet[0] + 6945.5 > ref_data[3])){
				if (min_s > frenet[0]) {
					min_s = frenet[0];
					result = &(sensor_fusion[i]);
				}
			}
		}
	}
	return result;
}

const vector<double> SensorFusionPredictor::get_front_vehicle_sf_in_time(const vector<vector<double>> &sensor_fusions, int lane, double min_s, double timespan) {
	const vector<double> *result = NULL;
	double next_vehicle_s = min_s + TOTAL_LENGTH * 2;

	for (int i = 0; i < sensor_fusions.size(); i++) {
		const vector<double> future_kinematics = get_vehicle_kinematics_in_time(sensor_fusions[i], timespan);

		// Check if the vehicle's future lane is in specified lane
		int future_lane = current_lane_from_d(future_kinematics[SensorFusionIndex::D]);
		if (future_lane != lane) {
			continue;
		} 

		// Check s round over
		double future_s = future_kinematics[SensorFusionIndex::S];
		if (min_s <= ROUND_OVER_BUFFER && future_kinematics[SensorFusionIndex::S] >= TOTAL_LENGTH - ROUND_OVER_BUFFER) {
			future_s -= TOTAL_LENGTH;
		} else if (min_s >= TOTAL_LENGTH - ROUND_OVER_BUFFER && future_kinematics[SensorFusionIndex::S] <= ROUND_OVER_BUFFER) {
			future_s += TOTAL_LENGTH;
		}

		//cout << "PREDICTION|" << future_kinematics[0] << "|S|" << min_s << "|"<< future_kinematics[5] << "|D|" << future_kinematics[6];

		if (future_s > min_s && future_s < next_vehicle_s) {
			result = &(sensor_fusions[i]);
			next_vehicle_s = future_s;
		}
	}
	if (result == NULL) {
		return {};
	}
	return *result;
}

const vector<double> SensorFusionPredictor::get_vehicle_kinematics_in_time(const vector<double> &sensor_fusion, double timespan) {
	double x = sensor_fusion[SensorFusionIndex::X];
	double y = sensor_fusion[SensorFusionIndex::Y];
	double vx = sensor_fusion[SensorFusionIndex::VX];
	double vy = sensor_fusion[SensorFusionIndex::VY];	
	double s = sensor_fusion[SensorFusionIndex::S];
	double d = sensor_fusion[SensorFusionIndex::D];

	double v = sqrt(vx * vx + vy * vy);
	vector<double> next_frenet = getFrenet(x + vx*timespan, y + vy*timespan, atan2(vy, vx), 
		this->map_waypoints.x, this->map_waypoints.y);

	return {sensor_fusion[SensorFusionIndex::ID], x + vx * timespan, y + vy * timespan, vx, vy, next_frenet[0], next_frenet[1]};
}

vector<double> SensorFusionPredictor::vehicle_frenet_state_in_time(const vector<double> &sensor_fusion, double timespan) {
	double x = sensor_fusion[1];
	double y = sensor_fusion[2];
	double vx = sensor_fusion[3];
	double vy = sensor_fusion[4];
	double s = sensor_fusion[5];
	double d = sensor_fusion[6];

	double next_x = x + vx;
	double next_y = y + vy;

	vector<double> next_frenet = getFrenet(next_x, next_y, atan2(next_y - y, next_x - x), 
			this->map_waypoints.x, this->map_waypoints.y);
	double vs = (next_frenet[0] - s);
	double vd = (next_frenet[1] - d);
	return {s + timespan*vs, vs, 0.0, d, vd + timespan*vd, 0.0};
}

const vector<double> predictions::get_vehicle_sf_by_id(const vector<vector<double>> &sensor_fusions, int id) {
	for (int i = 0; i < sensor_fusions.size(); i++) {
		if (sensor_fusions[i][0] == id) {
			return sensor_fusions[i];
		}
	}
	return {};
}