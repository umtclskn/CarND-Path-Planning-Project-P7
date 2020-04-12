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

// for convenience
using nlohmann::json;
using std::string;
using std::vector;
using namespace std;

int main() {
  uWS::Hub h;

  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  string map_file_ = "../data/highway_map.csv";
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



  //start in lane 1;
  int lane = 1;
  int lane_change_wp = 0;

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy,&lane,&lane_change_wp](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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
          //double car_x = j[1]["x"];

            double car_x = j[1]["x"];
            double car_y = j[1]["y"];
            double car_s = j[1]["s"];
            double car_d = j[1]["d"];
            double car_yaw = j[1]["yaw"];
            double car_speed = j[1]["speed"];
            auto previous_path_x = j[1]["previous_path_x"];
            auto previous_path_y = j[1]["previous_path_y"];
            double end_path_s = j[1]["end_path_s"];
            double end_path_d = j[1]["end_path_d"];
            vector<vector<double>> sensor_fusion = j[1]["sensor_fusion"];

            double ref_vel = 49.5; //mph

            int prev_size = previous_path_x.size();

            int next_wp = -1;
            double ref_x = car_x;
            double ref_y = car_y;
            double ref_yaw = deg2rad(car_yaw);

            if(prev_size < 2)
            {
                next_wp = NextWaypoint(ref_x, ref_y, ref_yaw, map_waypoints_x,map_waypoints_y,map_waypoints_dx,map_waypoints_dy);
            }
            else
            {
                ref_x = previous_path_x[prev_size-1];
                double ref_x_prev = previous_path_x[prev_size-2];
                ref_y = previous_path_y[prev_size-1];
                double ref_y_prev = previous_path_y[prev_size-2];
                ref_yaw = atan2(ref_y-ref_y_prev,ref_x-ref_x_prev);
                next_wp = NextWaypoint(ref_x,ref_y,ref_yaw,map_waypoints_x,map_waypoints_y,map_waypoints_dx,map_waypoints_dy);

                car_s = end_path_s;

                car_speed = (sqrt((ref_x-ref_x_prev)*(ref_x-ref_x_prev)+(ref_y-ref_y_prev)*(ref_y-ref_y_prev))/.02)*2.237;
            }

            //find ref_v to use
            double closestDist_s = 100000;
            bool change_lanes = false;
            for(int i = 0; i < sensor_fusion.size(); i++)
            {
                //car is in my lane
                float d = sensor_fusion[i][6];
                if(d < (2+4*lane+2) && d > (2+4*lane-2) )
                {
                    double vx = sensor_fusion[i][3];
                    double vy = sensor_fusion[i][4];
                    double check_speed = sqrt(vx*vx+vy*vy);
                    double check_car_s = sensor_fusion[i][5];
                    check_car_s+=((double)prev_size*.02*check_speed);
                    //check s values greater than mine and s gap
                    if((check_car_s > car_s) && ((check_car_s-car_s) < 30) && ((check_car_s-car_s) < closestDist_s ) )
                    {

                        closestDist_s = (check_car_s - car_s);

                        if((check_car_s-car_s) > 20)
                        {

                            //match that cars speed
                            ref_vel = check_speed*2.237;
                            change_lanes = true;
                        }
                        else
                        {
                            //go slightly slower than the cars speed
                            ref_vel = check_speed*2.237-5;
                            change_lanes = true;

                        }
                    }


                }
            }

            //try to change lanes if too close to car in front
            if(change_lanes && ((next_wp-lane_change_wp)%map_waypoints_x.size() > 2))
            {
                bool changed_lanes = false;
                //first try to change to left lane
                if(lane != 0 && !changed_lanes)
                {
                    bool lane_safe = true;
                    for(int i = 0; i < sensor_fusion.size(); i++)
                    {
                        //car is in left lane
                        float d = sensor_fusion[i][6];
                        if(d < (2+4*(lane-1)+2) && d > (2+4*(lane-1)-2) )
                        {
                            double vx = sensor_fusion[i][3];
                            double vy = sensor_fusion[i][4];
                            double check_speed = sqrt(vx*vx+vy*vy);

                            double check_car_s = sensor_fusion[i][5];
                            check_car_s+=((double)prev_size*.02*check_speed);
                            double dist_s = check_car_s-car_s;
                            if(dist_s < 20 && dist_s > -20)
                            {
                                lane_safe = false;
                            }
                        }
                    }
                    if(lane_safe)
                    {
                        changed_lanes = true;
                        lane -= 1;
                        lane_change_wp = next_wp;
                    }
                }
                //next try to change to right lane
                if(lane != 2 && !changed_lanes)
                {
                    bool lane_safe = true;
                    for(int i = 0; i < sensor_fusion.size(); i++)
                    {
                        //car is in right lane
                        float d = sensor_fusion[i][6];
                        if(d < (2+4*(lane+1)+2) && d > (2+4*(lane+1)-2) )
                        {
                            double vx = sensor_fusion[i][3];
                            double vy = sensor_fusion[i][4];
                            double check_speed = sqrt(vx*vx+vy*vy);

                            double check_car_s = sensor_fusion[i][5];
                            check_car_s+=((double)prev_size*.02*check_speed);
                            double dist_s = check_car_s-car_s;
                            if(dist_s < 20 && dist_s > -10)
                            {
                                lane_safe = false;
                            }
                        }
                    }
                    if(lane_safe)
                    {
                        changed_lanes = true;
                        lane += 1;
                        lane_change_wp = next_wp;
                    }

                }

            }


            vector<double> ptsx;
            vector<double> ptsy;

            if(prev_size < 2)
            {
                double prev_car_x = car_x - cos(car_yaw);
                double prev_car_y = car_y - sin(car_yaw);

                ptsx.push_back(prev_car_x);
                ptsx.push_back(car_x);

                ptsy.push_back(prev_car_y);
                ptsy.push_back(car_y);
            }
            else
            {
                ptsx.push_back(previous_path_x[prev_size-2]);
                ptsx.push_back(previous_path_x[prev_size-1]);

                ptsy.push_back(previous_path_y[prev_size-2]);
                ptsy.push_back(previous_path_y[prev_size-1]);


            }

            vector<double> next_wp0 = getXY(car_s+30,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
            vector<double> next_wp1 = getXY(car_s+60,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
            vector<double> next_wp2 = getXY(car_s+90,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);

            ptsx.push_back(next_wp0[0]);
            ptsx.push_back(next_wp1[0]);
            ptsx.push_back(next_wp2[0]);

            ptsy.push_back(next_wp0[1]);
            ptsy.push_back(next_wp1[1]);
            ptsy.push_back(next_wp2[1]);


            for (int i = 0; i < ptsx.size(); i++ )
            {

                //shift car reference angle to 0 degrees
                double shift_x = ptsx[i]-ref_x;
                double shift_y = ptsy[i]-ref_y;

                ptsx[i] = (shift_x *cos(0-ref_yaw)-shift_y*sin(0-ref_yaw));
                ptsy[i] = (shift_x *sin(0-ref_yaw)+shift_y*cos(0-ref_yaw));

            }


            tk::spline s;


            s.set_points(ptsx,ptsy);

            vector<double> next_x_vals;
            vector<double> next_y_vals;

            for(int i = 0; i < previous_path_x.size(); i++)
            {
                next_x_vals.push_back(previous_path_x[i]);
                next_y_vals.push_back(previous_path_y[i]);
            }

            double target_x = 30.0;
            double target_y = s(target_x);
            double target_dist = sqrt((target_x)*(target_x)+(target_y)*(target_y));

            double x_add_on = 0;

            for (int i = 1; i <= 50-previous_path_x.size(); i++) {

                if(ref_vel > car_speed)
                {
                    car_speed+=.224;
                }
                else if(ref_vel < car_speed)
                {
                    car_speed-=.224;
                }


                double N = (target_dist/(.02*car_speed/2.24));
                double x_point = x_add_on+(target_x)/N;
                double y_point = s(x_point);

                x_add_on = x_point;

                double x_ref = x_point;
                double y_ref = y_point;

                x_point = (x_ref *cos(ref_yaw)-y_ref*sin(ref_yaw));
                y_point = (x_ref *sin(ref_yaw)+y_ref*cos(ref_yaw));

                x_point += ref_x;
                y_point += ref_y;


                next_x_vals.push_back(x_point);
                next_y_vals.push_back(y_point);
            }

            json msgJson;
            msgJson["next_x"] = next_x_vals;
            msgJson["next_y"] = next_y_vals;

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
