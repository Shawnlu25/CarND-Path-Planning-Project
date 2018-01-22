#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"
#include "vehicle_fsm.h"
#include "predictions.h"
#include "trajectories.h"

#include "common.h"

using namespace trajectories;
using namespace predictions;
using namespace std;

// for convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
  	double x;
  	double y;
  	float s;
  	float d_x;
  	float d_y;
  	iss >> x;
  	iss >> y;
  	iss >> s;
  	iss >> d_x;
  	iss >> d_y;
  	map_waypoints_x.push_back(x);
  	map_waypoints_y.push_back(y);
  	map_waypoints_s.push_back(s);
  	map_waypoints_dx.push_back(d_x);
  	map_waypoints_dy.push_back(d_y);
  }

  MapWayPoints map_waypoints = {
    map_waypoints_x,
    map_waypoints_y,
    map_waypoints_s,
    map_waypoints_dx,
    map_waypoints_dy
  };

  SplineTrajectoryGenerator traj_gen = SplineTrajectoryGenerator(map_waypoints);
  VehicleFSM vehicle_fsm = VehicleFSM(22.2, 8.0, 8.0, 2);
  SensorFusionPredictor predictor = SensorFusionPredictor(map_waypoints);

  h.onMessage([&traj_gen, &predictor, &vehicle_fsm, &map_waypoints,&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy]
             (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
              uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
        	// Main car's localization Data
        	double car_x = j[1]["x"];
        	double car_y = j[1]["y"];
        	double car_s = j[1]["s"];
        	double car_d = j[1]["d"];
        	double car_yaw = j[1]["yaw"];
        	double car_speed = j[1]["speed"];
          LocalizationState cur_localization = {
            car_x, car_y, car_s, car_d, car_yaw, car_speed
          };

        	// Previous path data given to the Planner
        	auto previous_path_x = j[1]["previous_path_x"];
        	auto previous_path_y = j[1]["previous_path_y"];
        	// Previous path's end s and d values 
        	double end_path_s = j[1]["end_path_s"];
        	double end_path_d = j[1]["end_path_d"];
          
          GeneratedWayPoints previous_path = {
            previous_path_x, previous_path_y, end_path_s, end_path_d
          };

        	// Sensor Fusion Data, a list of all other cars on the same side of the road.
        	auto sensor_fusion = j[1]["sensor_fusion"];

        	json msgJson;

          // Initialize FSM if not done so
          if (!vehicle_fsm.is_state_initialized()) {
            vehicle_fsm.initialize_state(current_lane_from_d(car_d), car_speed, 0);
          }



          Behavior behavior = {1, -1, -1, 0.0};
          vector<double> ref_data = get_path_end_ref_data(cur_localization, previous_path);
          double ref_x = ref_data[0];
          double ref_y = ref_data[1];
          double ref_yaw = ref_data[2];

          double ref_s = 0.0;
          double ref_d = 0.0;
          if (end_path_s == 0 && end_path_d == 0) {
            vector<double> ref_frenet = getFrenet(ref_x, ref_y, ref_yaw, map_waypoints.x, map_waypoints.y);
            ref_s = ref_frenet[0];
            ref_d = ref_frenet[1];
          } else {
            ref_s = end_path_s;
            ref_d = end_path_d;
          }
          
          //cout << "REF_D|" << ref_d << "|" << current_lane_from_d(ref_d) << endl;
          vector<double> front_vehicle_sf = predictor.get_front_vehicle_sf_in_time(
            sensor_fusion, current_lane_from_d(ref_d), ref_s, previous_path_x.size()*0.02);

          if (front_vehicle_sf.size() != 0) {
            double fv_vx = front_vehicle_sf[SensorFusionIndex::VX];
            double fv_vy = front_vehicle_sf[SensorFusionIndex::VY];
            behavior.target_speed = sqrt(fv_vx * fv_vx + fv_vy * fv_vy);
            behavior.target_leading_vehicle_id = front_vehicle_sf[SensorFusionIndex::ID];
            if (front_vehicle_sf[SensorFusionIndex::S] - ref_s < 0) {
              behavior.distance_to_target_vehicle = front_vehicle_sf[SensorFusionIndex::S] - ref_s + 6945.5;
            } else {
              behavior.distance_to_target_vehicle = front_vehicle_sf[SensorFusionIndex::S] - ref_s;               
            }            
            cout << "TARGET|" << front_vehicle_sf[SensorFusionIndex::ID] <<"|" << behavior.target_speed <<"|" << behavior.distance_to_target_vehicle<< endl;                        
          }else {
            cout << "TARGET|" << -1 <<"|" << behavior.target_speed << endl;                                    
          }

          Trajectory traj = traj_gen.generate_trajectory({previous_path_x, previous_path_y}, behavior,
            cur_localization, sensor_fusion);

        	msgJson["next_x"] = traj.pts_x;//next_x_vals;
        	msgJson["next_y"] = traj.pts_y;//next_y_vals;

        	auto msg = "42[\"control\","+ msgJson.dump()+"]";

        	//this_thread::sleep_for(chrono::milliseconds(1000));
        	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
