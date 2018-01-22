#ifndef PTG_H
#define PTG_H

#include <vector>
#include "Eigen-3.3/Eigen/Dense"
#include "predictions.h"

using namespace std;
using namespace predictions;
using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace ptg {

struct Trajectory {
	vector<double> s_coefficients;
	vector<double> d_coefficients;
	int T;
};

class Constants {
public:
	static const double timestep;   // in seconds
	static const int num_samples;

	static const double max_jerk;   // m/s^3
	static const double max_accel;  // m/s^2
	static const double max_speed;  // m/s

	static const double vehicle_radius;  //m
};


class CostFunctions {
public:
	static double time_diff_cost(const Trajectory &traj, int target_vehicle_id, 
		const vector<double> &delta, int T, const vector<vector<double>> &sensor_fusion,
		SensorFusionPredictor &predictor);
	static double s_diff_cost(const Trajectory &traj, int target_vehicle_id, 
		const vector<double> &delta, int T, const vector<vector<double>> &sensor_fusion,
		SensorFusionPredictor &predictor);
private:
	static double logistic(double x);
};

/**
JMT

Calculate the Jerk Minimizing Trajectory given the initial state to the 
final state in time T

INPUTS

start - the vehicles start location given as a length three array 
   corresponding to initial values of [s, s_dot, s_double_dot]
end   - the desired end state for the vehicle.
T     - the duration, in number of TIME_STEP

OUTPUT
an array of length 6, each corresponding to a coefficient in the polynomial
s(t) =  a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5
*/
vector<double> JMT(vector<double> start, vector<double> end, int T);

double get_position(vector<double> coeff, int T);
double get_velocity(vector<double> coeff, int T);
double get_accel(vector<double> coeff, int T);
}

#endif