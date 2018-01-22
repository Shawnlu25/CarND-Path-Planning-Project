#include <cmath>
#include "ptg.h"
#include "predictions.h"

using namespace ptg;
using namespace std;
using namespace predictions;
using Eigen::MatrixXd;
using Eigen::VectorXd;

// Constants
const double Constants::timestep = 0.02;

const int Constants::num_samples = 20;

const double Constants::max_jerk = 9.0;
const double Constants::max_accel = 9.5;
const double Constants::max_speed = 22.2;

const double Constants::vehicle_radius = 1.6;

// Cost Functions
double CostFunctions::logistic(double x) {
	return 2.0 / (1 + exp(-x)) - 1.0;
}

double CostFunctions::time_diff_cost(const Trajectory &traj, int target_vehicle_id, 
		const vector<double> &delta, int T, const vector<vector<double>> &sensor_fusion,
		predictions::SensorFusionPredictor &predictor) {
	return CostFunctions::logistic((double)(abs(T - traj.T) / T));
}

double CostFunctions::s_diff_cost(const Trajectory &traj, int target_vehicle_id, 
		const vector<double> &delta, int T, const vector<vector<double>> &sensor_fusion,
		predictions::SensorFusionPredictor &predictor) {
	const vector<double> *target_sensor_fusion = predictor.get_vehicle_data_by_id(sensor_fusion, target_vehicle_id);
	vector<double> target_position;
	target_position.push_back(delta[0]);
	return CostFunctions::logistic((double)(abs(T - traj.T) / T));
}


// JMT
vector<double> ptg::JMT(vector<double> start, vector<double> end, int T) {

	double time_duration   = Constants::timestep * T;
	double time_duration_2 = time_duration * time_duration;
	double time_duration_3 = time_duration_2 * time_duration;
	double time_duration_4 = time_duration_2 * time_duration_2;	
	double time_duration_5 = time_duration_3 * time_duration_2;		

	MatrixXd A = MatrixXd(3, 3);
	A << time_duration_3, time_duration_4, time_duration_5,
		 3*time_duration_2, 4*time_duration_3, 5*time_duration_4,
		 6*time_duration, 12*time_duration_2, 20*time_duration_3;

	MatrixXd B = MatrixXd(3, 1);
	B << end[0]-(start[0] + start[1] * time_duration + 0.5 * start[2]*time_duration_2),
		 end[1]-(start[1] + start[2] * time_duration),
		 end[2]-start[2];

	MatrixXd Ai = A.inverse();

	MatrixXd C = Ai * B;

	vector<double> result = {start[0], start[1], 0.5 * start[2]};
	for (int i = 0; i < C.size(); i++) {
		result.push_back(C.data()[i]);
	}

	return result;
}

double ptg::get_position(vector<double> coeff, int T) {
	double time_duration = Constants::timestep * T;
	double t_coeff = 1.0;	
	double result = 0.0;
	for (int i = 0; i < coeff.size(); i++) {
		result += t_coeff * coeff[i];
		t_coeff *= time_duration;
	}
	return result;
}

double ptg::get_velocity(vector<double> coeff, int T) {
	double time_duration = Constants::timestep * T;
	double t_coeff = 1.0;	
	double result = 0.0;
	for (int i = 1; i < coeff.size(); i++) {
		result += i * t_coeff * coeff[i];
		t_coeff *= time_duration;
	}
	return result;
}

double ptg::get_accel(vector<double> coeff, int T) {
	double time_duration = Constants::timestep * T;
	double t_coeff = 1.0;	
	double result = 0.0;
	for (int i = 2; i < coeff.size(); i++) {
		result += i * (i-1) * t_coeff * coeff[i];
		t_coeff *= time_duration;
	}
	return result;
}