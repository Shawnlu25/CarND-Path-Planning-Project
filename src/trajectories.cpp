#include "trajectories.h"
#include "spline.h"
#include "predictions.h"

#include <iostream>
#include <cmath>

using namespace predictions;
using namespace trajectories;

// Constants
const double Constants::timestep = 0.02;
const int Constants::max_step = 40;

const double Constants::max_jerk = 2.0;
const double Constants::max_accel = 4.0;
const double Constants::max_speed = 22.2;
const double Constants::approach_speed_delta = 1.0;

const int Constants::max_lane = 2;

const double Constants::vehicle_radius = 2.0;
const double Constants::vehicle_buffer = Constants::vehicle_radius * 8;
const double Constants::safe_buffer = Constants::vehicle_radius * 18;

SplineTrajectoryGenerator::SplineTrajectoryGenerator(MapWayPoints &map_waypoints) {
	this->map_waypoints = map_waypoints;
}

TrajectoryEndRef SplineTrajectoryGenerator::get_traj_end_ref_data(const LocalizationState &localization,
      	const Trajectory &previous_traj) {
	int prev_size = previous_traj.pts_x.size();
	if (prev_size < 2) {
		vector<double> ref_frenet = 
			getFrenet(localization.x, localization.y, deg2rad(localization.yaw), map_waypoints.x, map_waypoints.y);
		double yaw = deg2rad(localization.yaw);
		return {localization.x, localization.y, yaw, 
			    localization.x - cos(yaw), localization.y - sin(yaw),
				ref_frenet[0], ref_frenet[1], 0.0, 0.0};
	} 

	double ref_x = previous_traj.pts_x[prev_size-1];
	double ref_y = previous_traj.pts_y[prev_size-1];
	double prev_ref_x = previous_traj.pts_x[prev_size-2];
	double prev_ref_y = previous_traj.pts_y[prev_size-2];
	double ref_yaw = atan2(ref_y - prev_ref_y, ref_x - prev_ref_x);
	vector<double> ref_frenet = 
		getFrenet(ref_x, ref_y, ref_yaw, map_waypoints.x, map_waypoints.y);			
	double distance = sqrt((ref_x - prev_ref_x) * (ref_x - prev_ref_x) 
							+ (ref_y - prev_ref_y) * (ref_y - prev_ref_y));
	double ref_speed = distance / Constants::timestep;

	if (prev_size < 3) {
		return {ref_x, ref_y, ref_yaw, prev_ref_x, prev_ref_y,
			ref_frenet[0], ref_frenet[1], ref_speed, 0.0};		
	}

	double prev_ref_x2 = previous_traj.pts_x[prev_size-3];
	double prev_ref_y2 = previous_traj.pts_y[prev_size-3];
	double distance2 = sqrt((prev_ref_x - prev_ref_x2) * (prev_ref_x - prev_ref_x2) 
							+ (prev_ref_y - prev_ref_y2) * (prev_ref_y - prev_ref_y2));
	double prev_speed = distance2 / Constants::timestep;
	//cout << ref_x << "|"<< ref_y << "|"<< prev_ref_x << "|" <<prev_ref_y << "|"<< distance << "|" << distance2 << "|" << ref_speed << "|" << prev_speed << "|"<< (ref_speed - prev_speed) / Constants::timestep <<endl;
	return {ref_x, ref_y, ref_yaw, prev_ref_x, prev_ref_y, 
			ref_frenet[0], ref_frenet[1], ref_speed, (ref_speed - prev_speed) / Constants::timestep};
}

Trajectory SplineTrajectoryGenerator::sample_points_on_lane(const TrajectoryEndRef &end_ref_data, int lane_num, double from_s) {
	Trajectory result = {{}, {}};
	for (int i = 1; i < 6; i++) {
		vector<double> next_wp = getXY(from_s + Constants::max_speed * i * 1.3 , lane_num * 4.0 + 2, 
			map_waypoints.s, map_waypoints.x, map_waypoints.y);
		result.pts_x.push_back(next_wp[0]);
		result.pts_y.push_back(next_wp[1]);		
	}
	return result;
}

Trajectory SplineTrajectoryGenerator::transform_traj_to_ref_coord(const TrajectoryEndRef &end_ref_data, const Trajectory &traj) {
	Trajectory result = {{}, {}};
	for (int i = 0; i < traj.pts_x.size(); i++) {
		vector<double> pt = transform_to_ref_coord(traj.pts_x[i], traj.pts_y[i], end_ref_data.x, end_ref_data.y, end_ref_data.yaw);
		result.pts_x.push_back(pt[0]);
		result.pts_y.push_back(pt[1]);
	}
	return result;
}

Trajectory SplineTrajectoryGenerator::generate_keep_lane_trajectory(const TrajectoryEndRef &end_ref_data, 
		double target_speed, double max_s, int lane_num) {
	int ref_lane = lane_num;//current_lane_from_d(end_ref_data.d);

	Trajectory sampled_future_traj = sample_points_on_lane(end_ref_data, ref_lane, end_ref_data.s);
	Trajectory spline_points = {{}, {}};
	spline_points.pts_x.push_back(end_ref_data.prev_x);
	spline_points.pts_y.push_back(end_ref_data.prev_y);	
	spline_points.pts_x.push_back(end_ref_data.x);
	spline_points.pts_y.push_back(end_ref_data.y);		

	for (int i = 0 ; i < sampled_future_traj.pts_x.size(); i++) {
		spline_points.pts_x.push_back(sampled_future_traj.pts_x[i]);
		spline_points.pts_y.push_back(sampled_future_traj.pts_y[i]);		
	}

	spline_points = this->transform_traj_to_ref_coord(end_ref_data, spline_points);

	tk::spline sp;
	sp.set_points(spline_points.pts_x, spline_points.pts_y);
	Trajectory result = {{}, {}};

	double cur_speed = end_ref_data.speed;
	double cur_accel = end_ref_data.accel;
	double cur_x = 0;	
	double cur_target_speed = target_speed;
	if (max_s > 0) {
		cur_target_speed = min(target_speed + Constants::approach_speed_delta, Constants::max_speed);
	}
	//cout << "SPEED|ACCEL|" << cur_speed << "|" << cur_accel << endl;

	int step = Constants::max_step;
	if (lane_num != current_lane_from_d(end_ref_data.d)) {
		step *= 2;
	}
	for (int i = 1 ; i <= step; i++) {
		if (max_s >= 0) {
			if (cur_x + cur_speed * Constants::timestep > max_s ) {
				cur_target_speed = target_speed - Constants::approach_speed_delta;
			} else if (cur_x + cur_speed * Constants::timestep > max_s - (Constants::safe_buffer - Constants::vehicle_buffer)){
				cur_target_speed = min(target_speed + Constants::approach_speed_delta, Constants::max_speed);				
			} else {
				cur_target_speed = Constants::max_speed;
			}
		} else {
			cur_target_speed = target_speed - Constants::approach_speed_delta;
		}
		//cout << "SPEED|ACCEL|" << cur_speed << "|" << cur_accel << "|" << cur_target_speed << "|" << max_s << endl;
		if (cur_speed <= cur_target_speed) {
			cur_speed = min(cur_speed + Constants::max_accel * Constants::timestep, cur_target_speed);
		} else {
			cur_speed = max(cur_speed - Constants::max_accel * Constants::timestep, cur_target_speed);
		}
		double x_pt = cur_x + cur_speed * Constants::timestep;
			//+ .5 *cur_accel * Constants::timestep * Constants::timestep;
		double y_pt = sp(x_pt);
		//cout << "ACCE|" <<cur_speed << "|" << cur_accel << "|" <<x_pt << "|" << y_pt << "|"<< i << endl;		
		cur_x = x_pt;
		vector<double> generated_pt = 
			transform_from_ref_coord(x_pt, y_pt, end_ref_data.x, end_ref_data.y, end_ref_data.yaw);
		result.pts_x.push_back(generated_pt[0]);
		result.pts_y.push_back(generated_pt[1]);		
	}
	return result;
}

Trajectory SplineTrajectoryGenerator::generate_trajectory(const Trajectory &previous_traj, 
			const Behavior &behavior, const LocalizationState &localization, 
			const vector<vector<double>> &sensor_fusions) {

	// duration = how far to look ahead in terms of time
	double duration = Constants::timestep * (Constants::max_step - previous_traj.pts_x.size());

	/*// Target vehicle predictions, if available
	vector<double> target_vehicle_sf;
	double target_vehicle_speed;
	double target_vehicle_future_x;
	double target_vehicle_future_y;

	if (behavior.target_leading_vehicle_id >= 0) {
		target_vehicle_sf = get_vehicle_sf_by_id(sensor_fusions, behavior.target_leading_vehicle_id);
		double x = target_vehicle_sf[SensorFusionIndex::X];
		double y = target_vehicle_sf[SensorFusionIndex::Y];		
		double vx = target_vehicle_sf[SensorFusionIndex::VX];
		double vy = target_vehicle_sf[SensorFusionIndex::VY];		
		target_vehicle_speed = sqrt(vx*vx + vy*vy);
		target_vehicle_future_x = x + vx * duration;
		target_vehicle_future_y = y + vy * duration;
	}*/

	// Reference point data
	TrajectoryEndRef traj_end_ref = this->get_traj_end_ref_data(localization, previous_traj);
	int ref_lane = current_lane_from_d(traj_end_ref.d);
	// Re-using populated trajectory
	Trajectory result_trajectory = {{}, {}};
	for (int i = 0; i < previous_traj.pts_x.size(); i++) {
		result_trajectory.pts_x.push_back(previous_traj.pts_x[i]);
		result_trajectory.pts_y.push_back(previous_traj.pts_y[i]);		
	}

	// Generating new trajectory 
	Trajectory new_trajectory = { {}, {}};

	// Different trajectory for different behavior
	if (behavior.target_lane_id == ref_lane && behavior.target_leading_vehicle_id < 0) {
		new_trajectory = this->generate_keep_lane_trajectory(traj_end_ref, Constants::max_speed, 1000, ref_lane);
	} else if (behavior.target_lane_id == ref_lane && behavior.target_leading_vehicle_id >= 0) {
		double max_s = behavior.distance_to_target_vehicle - Constants::vehicle_buffer;
		new_trajectory = this->generate_keep_lane_trajectory(traj_end_ref, behavior.target_speed-Constants::approach_speed_delta, max_s, ref_lane);		
	} else if (behavior.target_lane_id == ref_lane && behavior.target_speed > 0) {
		new_trajectory = this->generate_keep_lane_trajectory(traj_end_ref, behavior.target_speed, 1000, ref_lane);
	} else if (behavior.target_lane_id != ref_lane) {
		cout << "CHANGE LANE TO " << behavior.target_lane_id << endl;
		new_trajectory = this->generate_keep_lane_trajectory(traj_end_ref, behavior.target_speed, 10, behavior.target_lane_id);
	}

	// Append new trajectory
	for (int i = 0; i < new_trajectory.pts_x.size(); i++) {	
		if (result_trajectory.pts_x.size() >= Constants::max_step && behavior.target_lane_id == ref_lane) {
			break;
		}
		result_trajectory.pts_x.push_back(new_trajectory.pts_x[i]);
		result_trajectory.pts_y.push_back(new_trajectory.pts_y[i]);
	}

	return result_trajectory;
}
