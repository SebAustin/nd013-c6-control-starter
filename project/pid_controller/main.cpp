/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: September 20, 2020
 *      Author: Munir Jojo-Verge
 				Aaron Brown
 **********************************************/

/**
 * @file main.cpp
 **/

#include <string>
#include <array>
#include <cfloat>
#include <chrono>
#include <cmath>
#include <iostream>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <tuple>
#include <vector>
#include <iostream>
#include <fstream>
#include <typeinfo>

#include "json.hpp"
#include <carla/client/ActorBlueprint.h>
#include <carla/client/BlueprintLibrary.h>
#include <carla/client/Client.h>
#include <carla/client/Map.h>
#include <carla/client/Sensor.h>
#include <carla/client/TimeoutException.h>
#include <carla/client/World.h>
#include <carla/geom/Transform.h>
#include <carla/image/ImageIO.h>
#include <carla/image/ImageView.h>
#include <carla/sensor/data/Image.h>
#include "Eigen/QR"
#include "behavior_planner_FSM.h"
#include "motion_planner.h"
#include "planning_params.h"
#include "utils.h"
#include "pid_controller.h"
#include "control_errors.h"
#include "pid_gains.h"

#include <limits>
#include <iostream>
#include <fstream>
#include <uWS/uWS.h>
#include <math.h>
#include <vector>
#include <cmath>
#include <time.h>

using namespace std;
using json = nlohmann::json;

#define _USE_MATH_DEFINES

string hasData(string s) {
  auto found_null = s.find("null");
    auto b1 = s.find_first_of("{");
    auto b2 = s.find_first_of("}");
    if (found_null != string::npos) {
      return "";
    }
    else if (b1 != string::npos && b2 != string::npos) {
      return s.substr(b1, b2 - b1 + 1);
    }
    return "";
}


template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

double angle_between_points(double x1, double y1, double x2, double y2){
  return atan2(y2-y1, x2-x1);
}

BehaviorPlannerFSM behavior_planner(
      P_LOOKAHEAD_TIME, P_LOOKAHEAD_MIN, P_LOOKAHEAD_MAX, P_SPEED_LIMIT,
      P_STOP_THRESHOLD_SPEED, P_REQ_STOPPED_TIME, P_REACTION_TIME,
      P_MAX_ACCEL, P_STOP_LINE_BUFFER);

// Decalre and initialized the Motion Planner and all its class requirements
MotionPlanner motion_planner(P_NUM_PATHS, P_GOAL_OFFSET, P_ERR_TOLERANCE);

bool have_obst = false;
vector<State> obstacles;

void path_planner(vector<double>& x_points, vector<double>& y_points, vector<double>& v_points, double yaw, double velocity, State goal, bool is_junction, string tl_state, vector< vector<double> >& spirals_x, vector< vector<double> >& spirals_y, vector< vector<double> >& spirals_v, vector<int>& best_spirals){

  State ego_state;

  ego_state.location.x = x_points[x_points.size()-1];
  ego_state.location.y = y_points[y_points.size()-1];
  ego_state.velocity.x = velocity;

  if( x_points.size() > 1 ){
  	ego_state.rotation.yaw = angle_between_points(x_points[x_points.size()-2], y_points[y_points.size()-2], x_points[x_points.size()-1], y_points[y_points.size()-1]);
  	ego_state.velocity.x = v_points[v_points.size()-1];
  	if(velocity < 0.01)
  		ego_state.rotation.yaw = yaw;

  }

  Maneuver behavior = behavior_planner.get_active_maneuver();

  goal = behavior_planner.state_transition(ego_state, goal, is_junction, tl_state);

  if(behavior == STOPPED){

  	int max_points = 20;
  	double point_x = x_points[x_points.size()-1];
  	double point_y = y_points[x_points.size()-1];
  	while( x_points.size() < max_points ){
  	  x_points.push_back(point_x);
  	  y_points.push_back(point_y);
  	  v_points.push_back(0);

  	}
  	return;
  }

  auto goal_set = motion_planner.generate_offset_goals(goal);

  auto spirals = motion_planner.generate_spirals(ego_state, goal_set);

  auto desired_speed = utils::magnitude(goal.velocity);

  State lead_car_state;  // = to the vehicle ahead...

  if(spirals.size() == 0){
  	cout << "Error: No spirals generated " << endl;
  	return;
  }

  for(int i = 0; i < spirals.size(); i++){

    auto trajectory = motion_planner._velocity_profile_generator.generate_trajectory( spirals[i], desired_speed, ego_state,
                                                                                    lead_car_state, behavior);

    vector<double> spiral_x;
    vector<double> spiral_y;
    vector<double> spiral_v;
    for(int j = 0; j < trajectory.size(); j++){
      double point_x = trajectory[j].path_point.x;
      double point_y = trajectory[j].path_point.y;
      double velocity = trajectory[j].v;
      spiral_x.push_back(point_x);
      spiral_y.push_back(point_y);
      spiral_v.push_back(velocity);
    }

    spirals_x.push_back(spiral_x);
    spirals_y.push_back(spiral_y);
    spirals_v.push_back(spiral_v);

  }

  best_spirals = motion_planner.get_best_spiral_idx(spirals, obstacles, goal);
  int best_spiral_idx = -1;

  if(best_spirals.size() > 0)
  	best_spiral_idx = best_spirals[best_spirals.size()-1];

  int index = 0;
  int max_points = 20;
  int add_points = spirals_x[best_spiral_idx].size();
  while( x_points.size() < max_points && index < add_points ){
    double point_x = spirals_x[best_spiral_idx][index];
    double point_y = spirals_y[best_spiral_idx][index];
    double velocity = spirals_v[best_spiral_idx][index];
    index++;
    x_points.push_back(point_x);
    y_points.push_back(point_y);
    v_points.push_back(velocity);
  }


}

void set_obst(vector<double> x_points, vector<double> y_points, vector<State>& obstacles, bool& obst_flag){

	for( int i = 0; i < x_points.size(); i++){
		State obstacle;
		obstacle.location.x = x_points[i];
		obstacle.location.y = y_points[i];
		obstacles.push_back(obstacle);
	}
	obst_flag = true;
}

int main ()
{
  cout << "starting server" << endl;
  uWS::Hub h;

  double new_delta_time;
  int i = 0;

  fstream file_steer;
  file_steer.open("steer_pid_data.txt", std::ofstream::out | std::ofstream::trunc);
  file_steer.close();
  fstream file_throttle;
  file_throttle.open("throttle_pid_data.txt", std::ofstream::out | std::ofstream::trunc);
  file_throttle.close();

  time_t prev_timer;
  time_t timer;
  time(&prev_timer);

  // Default gains: {Kp, Ki, Kd}.
  // STARTING VALUES ONLY: not yet tuned in the simulator.
  pid_gains::ControllerGains default_gains;
  default_gains.steer = {0.3, 0.0, 0.0};     // heading controller
  default_gains.throttle = {0.2, 0.0, 0.0};  // speed controller

  // During tuning the gains can be changed without rebuilding: if a file
  // named pid_gains.txt exists in the directory the program is started from
  // (project/), its values replace the defaults above. See pid_gains.h for
  // the format. A malformed file stops the program, so a run is never
  // recorded under gains it did not use.
  const std::string kGainsFile = "pid_gains.txt";
  const pid_gains::ParseResult loaded_gains = pid_gains::LoadGains(kGainsFile, default_gains);
  if (!loaded_gains.ok) {
    cerr << "Error: " << loaded_gains.error << endl;
    return 1;
  }
  const pid_gains::ControllerGains gains = loaded_gains.gains;
  cout << "PID gains (" << (loaded_gains.from_file ? kGainsFile : "built-in defaults") << "):" << endl
       << pid_gains::FormatGains(gains);

  // The steer command must stay inside [-1.2, 1.2].
  const double kSteerOutputMax = 1.2;
  const double kSteerOutputMin = -1.2;
  // The throttle command must stay inside [-1, 1]; a negative value is
  // applied as braking further down.
  const double kThrottleOutputMax = 1.0;
  const double kThrottleOutputMin = -1.0;

  // initialize pid steer
  PID pid_steer = PID();
  pid_steer.Init(gains.steer.kp, gains.steer.ki, gains.steer.kd, kSteerOutputMax, kSteerOutputMin);

  // initialize pid throttle
  PID pid_throttle = PID();
  pid_throttle.Init(gains.throttle.kp, gains.throttle.ki, gains.throttle.kd, kThrottleOutputMax, kThrottleOutputMin);

  h.onMessage([&pid_steer, &pid_throttle, &new_delta_time, &timer, &prev_timer, &i, &prev_timer](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode)
  {
        auto s = hasData(data);

        if (s != "") {

          auto data = json::parse(s);

          // create file to save values
          fstream file_steer;
          file_steer.open("steer_pid_data.txt");
          fstream file_throttle;
          file_throttle.open("throttle_pid_data.txt");

          vector<double> x_points = data["traj_x"];
          vector<double> y_points = data["traj_y"];
          vector<double> v_points = data["traj_v"];
          double yaw = data["yaw"];
          double velocity = data["velocity"];
          double sim_time = data["time"];
          double waypoint_x = data["waypoint_x"];
          double waypoint_y = data["waypoint_y"];
          double waypoint_t = data["waypoint_t"];
          bool is_junction = data["waypoint_j"];
          string tl_state = data["tl_state"];

          double x_position = data["location_x"];
          double y_position = data["location_y"];
          double z_position = data["location_z"];

          if(!have_obst){
          	vector<double> x_obst = data["obst_x"];
          	vector<double> y_obst = data["obst_y"];
          	set_obst(x_obst, y_obst, obstacles, have_obst);
          }

          State goal;
          goal.location.x = waypoint_x;
          goal.location.y = waypoint_y;
          goal.rotation.yaw = waypoint_t;

          vector< vector<double> > spirals_x;
          vector< vector<double> > spirals_y;
          vector< vector<double> > spirals_v;
          vector<int> best_spirals;

          path_planner(x_points, y_points, v_points, yaw, velocity, goal, is_junction, tl_state, spirals_x, spirals_y, spirals_v, best_spirals);

          // Save time and compute delta time
          time(&timer);
          new_delta_time = difftime(timer, prev_timer);
          prev_timer = timer;

          ////////////////////////////////////////
          // Steering control
          ////////////////////////////////////////

          // Update the delta time with the previous command
          pid_steer.UpdateDeltaTime(new_delta_time);

          // Compute steer error
          //
          // The steer error is a heading error: the angle between where the
          // car should be pointing and where it is pointing (yaw).
          //  - "Where it should be pointing" is the direction from the car's
          //    current position to the last point of the planned trajectory,
          //    the same point whose planned speed drives the throttle.
          //  - The error is desired minus actual, so a target to the left
          //    (counter-clockwise) gives a positive error and, with positive
          //    gains, a positive steer command.
          //  - The difference is wrapped into [-pi, pi] so the car always
          //    turns the short way round.
          //  - If the planner returned no points, or the car is already on
          //    the target point, there is no direction to aim at and the
          //    error is 0.
          double error_steer = 0.0;
          if (!x_points.empty() && !y_points.empty()) {
            error_steer = control_errors::HeadingError(
                x_position, y_position, yaw, x_points.back(), y_points.back());
          }

          double steer_output;

          // Compute control to apply
          pid_steer.UpdateError(error_steer);
          steer_output = pid_steer.TotalError();

          // Save data
          file_steer.seekg(std::ios::beg);
          for(int j=0; j < i - 1; ++j) {
              file_steer.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
          }
          file_steer  << i ;
          file_steer  << " " << error_steer;
          file_steer  << " " << steer_output << endl;

          ////////////////////////////////////////
          // Throttle control
          ////////////////////////////////////////

          // Update the delta time with the previous command
          pid_throttle.UpdateDeltaTime(new_delta_time);

          // Compute error of speed
          //
          // The throttle error is a speed error: the speed the planner wants
          // minus the speed the car has.
          //  - The desired speed is the planned speed at the last point of
          //    the trajectory (the last element of v_points).
          //  - The error is desired minus actual, so a car that is too slow
          //    gets a positive error and a positive (accelerating) command,
          //    and a car that is too fast gets a negative command, which is
          //    turned into braking below.
          //  - If the planner returned no points, the error is 0.
          double error_throttle = 0.0;
          if (!v_points.empty()) {
            error_throttle = control_errors::SpeedError(v_points.back(), velocity);
          }

          double throttle_output;
          double brake_output;

          // Compute control to apply
          pid_throttle.UpdateError(error_throttle);
          double throttle = pid_throttle.TotalError();

          // Adapt the negative throttle to break
          if (throttle > 0.0) {
            throttle_output = throttle;
            brake_output = 0;
          } else {
            throttle_output = 0;
            brake_output = -throttle;
          }

          // Save data
          file_throttle.seekg(std::ios::beg);
          for(int j=0; j < i - 1; ++j){
              file_throttle.ignore(std::numeric_limits<std::streamsize>::max(),'\n');
          }
          file_throttle  << i ;
          file_throttle  << " " << error_throttle;
          file_throttle  << " " << brake_output;
          file_throttle  << " " << throttle_output << endl;


          // Send control
          json msgJson;
          msgJson["brake"] = brake_output;
          msgJson["throttle"] = throttle_output;
          msgJson["steer"] = steer_output;

          msgJson["trajectory_x"] = x_points;
          msgJson["trajectory_y"] = y_points;
          msgJson["trajectory_v"] = v_points;
          msgJson["spirals_x"] = spirals_x;
          msgJson["spirals_y"] = spirals_y;
          msgJson["spirals_v"] = spirals_v;
          msgJson["spiral_idx"] = best_spirals;
          msgJson["active_maneuver"] = behavior_planner.get_active_maneuver();

          //  min point threshold before doing the update
          // for high update rate use 19 for slow update rate use 4
          msgJson["update_point_thresh"] = 16;

          auto msg = msgJson.dump();

          i = i + 1;
          file_steer.close();
          file_throttle.close();

      ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

    }

  });


  h.onConnection([](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req)
  {
      cout << "Connected!!!" << endl;
    });


  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length)
    {
      ws.close();
      cout << "Disconnected" << endl;
    });

  int port = 4567;
  if (h.listen("0.0.0.0", port))
    {
      cout << "Listening to port " << port << endl;
      h.run();
    }
  else
    {
      cerr << "Failed to listen to port" << endl;
      return -1;
    }


}
