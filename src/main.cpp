#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"
#include "high_way_deicder.h"
// for convenience
using nlohmann::json;
using std::string;
using std::vector;

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

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);
  if(!in_map_.is_open()) {
    std::cerr << "Error in open the map in the path : " << map_file_;
    return -1;
  }

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
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
  double speed_limit_from_map = 50; //mph
  //aim line
  double lane = 1;

  //initial speed
  double ref_speed = 0; //mph


  h.onMessage([&speed_limit_from_map, &ref_speed, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy, &lane]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {

    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
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
          double car_speed = j[1]["speed"];//mph

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          int pre_size = previous_path_x.size();
          if(pre_size > 0) {
            car_s = end_path_s;
          }

          bool need_slow_down = false;
          HighWayDecider high_way_decider(car_s, car_d, car_speed,
                                          pre_size, sensor_fusion, speed_limit_from_map);
          need_slow_down = high_way_decider.hasBlockingByOthers();
          ChangeLineType change_line_type = high_way_decider.changeLineDecider();
          int current_lane = (floor)(car_d / 4.);
          double aim_lane;
          if(change_line_type == ChangeLineType::Left && lane > 0)
            aim_lane = current_lane - 1;
          else if(change_line_type == ChangeLineType::Right && lane < 2)
            aim_lane = current_lane + 1;
          else
            aim_lane = current_lane;

          if(need_slow_down && change_line_type == ChangeLineType::None)
            ref_speed -= 0.224; //mph
          else if(change_line_type != ChangeLineType::None && ref_speed > 40.)
            ref_speed -= 0.224; //mph
          else if(ref_speed < 49.5)
            ref_speed += 0.224;
          std::cout << "ref_speed/mph: " << ref_speed <<std::endl;

          //the points for spline
          vector<double> pts_x, pts_y;

          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          if(pre_size < 2) {
            double pre_car_x = car_x - cos(car_yaw);
            double pre_car_y = car_y - sin(car_yaw);
            pts_x.push_back(pre_car_x);
            pts_y.push_back(pre_car_y);

            pts_x.push_back(car_x);
            pts_y.push_back(car_y);
          } else {
            ref_x = previous_path_x[pre_size - 1];
            ref_y = previous_path_y[pre_size - 1];

            double ref_x_prev = previous_path_x[pre_size - 2];
            double ref_y_prev = previous_path_y[pre_size - 2];
            ref_yaw = atan2(ref_y - ref_y_prev,
                ref_x - ref_x_prev);

            pts_x.push_back(ref_x_prev);
            pts_y.push_back(ref_y_prev);

            pts_x.push_back(ref_x);
            pts_y.push_back(ref_y);
          }
          vector<double> next_ref0 =
              getXY(car_s + 30., 2 + 4*aim_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_ref1 =
              getXY(car_s + 60., 2 + 4*aim_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_ref2 =
              getXY(car_s + 90., 2 + 4*aim_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);

          pts_x.push_back(next_ref0[0]);
          pts_x.push_back(next_ref1[0]);
          pts_x.push_back(next_ref2[0]);

          pts_y.push_back(next_ref0[1]);
          pts_y.push_back(next_ref1[1]);
          pts_y.push_back(next_ref2[1]);

          //transfrom t0 local
          for(uint i = 0; i < pts_x.size(); ++i) {
            double shift_x = pts_x[i] - ref_x;
            double shift_y = pts_y[i] - ref_y;

            pts_x[i] = (shift_x * cos(0-ref_yaw) - shift_y * sin(0-ref_yaw));
            pts_y[i] = (shift_x * sin(0-ref_yaw) + shift_y * cos(0-ref_yaw));
          }
          //now let's create the spline from pts.
          std::cout << "pts'size: " << pts_x.size() << std::endl;
          tk::spline splines;
          splines.set_points(pts_x, pts_y);
          vector<double> next_x_vals;
          vector<double> next_y_vals;
          for(int i = 0; i < previous_path_x.size(); ++i) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }
          //interplot the spline with the inspire speed
          double target_x = 30.;
          double target_y = splines(target_x);
          double target_distance = sqrt(target_x * target_x + target_y * target_y);

          double x_add_on = 0.;
          for(int i = 0; i <= 50 - previous_path_x.size(); ++i) {
            double N = (target_distance / (0.02 * ref_speed/2.24));
            double x_point = x_add_on + target_x/N;
            double y_point = splines(x_point);

            x_add_on = x_point;
            double x_ref = x_point,
                y_ref = y_point;

            //transform to global
            x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
            y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

            x_point += ref_x;
            y_point += ref_y;

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
          // only keep line hardly.
//          double dist_inc = 0.5;
//          for (int i = 0; i < 50; ++i) {
//            double next_s = car_s + dist_inc * (i + 1);
//            //the zero is in the middle yellow line
//            //the line'width is 4.0
//            double next_d = 6.0;
//            vector<double> xy_cartesian =
//                getXY(next_s, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
//            next_x_vals.push_back(xy_cartesian[0]);
//            next_y_vals.push_back(xy_cartesian[1]);
//          }

          json msgJson;
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";
          std::cout << "send msgs by uWs and use josn" << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

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
