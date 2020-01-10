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

#define TIMER_BREAK     90

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
  
  int lane = 1;
  
  double ref_vel = 0; // mph
  
  double timer_change_lane = 0;
  double timer_speed_adjust = 0;
  
  bool first_time = true;
  
  double dynamic_driving = 0.0f; // Dynamic driving will fail this project but provide a more human/agile approach

  h.onMessage([&ref_vel, &map_waypoints_x, &map_waypoints_y, &map_waypoints_s,
               &map_waypoints_dx, &map_waypoints_dy, &lane, &timer_change_lane, &first_time, &dynamic_driving, &timer_speed_adjust]
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
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
          
          /* -------------------------- Variant 4 - keep lane smooth -------------------------- */
          
          // List of widely spaced x,y points, to fit a spline
          vector<double> ptsx;
          vector<double> ptsy;

          int prev_size = previous_path_x.size(); // last path it was following
          
          /* We are starting with speed 0 - if we attempt to change right, there might be a car there */
          if (first_time == true) {
            first_time = false;
            timer_change_lane = TIMER_BREAK * 2.5;
            prev_size = 0;
          }
          
          /* ---------------- Variant 5 - smooth start and check cars in front ---------------- */
          if(prev_size > 0) {
            car_s = end_path_s;
          }
          
          double SPEED_EXCEED = 15.0;
          
          bool slightly_close = false; // if car ahead within 45 m, change lane without slowing down - elegant change
          bool too_close = false;      // slow down and change lane
          bool way_too_close = false;
          bool collision_warn_too_close = false;
          bool too_far = false;
          bool  allow_left_change = true;
          bool allow_right_change = true;
          bool danger_too_close = false;
          double target_speed = 49.48 + (dynamic_driving * SPEED_EXCEED);
          int far_count = 0;
          int cars_in_lane_count = 0;
          bool my_car_faster = false;
          double min_dist = 9999.9;
          double min_pos = 9999.9;
          double min_speed = 9999.9;
          double min_indx = -1;
          
          // Do not allow swerving across multiple lanes - pause for a while
          if (timer_change_lane > 0) {
            timer_change_lane--;
            allow_left_change = false;
            allow_right_change = false;
          }
          
          if (timer_speed_adjust > 0) 
            timer_speed_adjust--;

          // find ref_v to use
          for (auto i = 0; i < sensor_fusion.size(); i++) { 
            
            float d = sensor_fusion[i][6];

            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            double check_speed = sqrt(vx*vx + vy*vy); // Error in speed calculation...
            double check_car_s = sensor_fusion[i][5];
            double COLLISION_AVOIDANCE_DISTANCE = 0.7;
            
            check_car_s += ((double) prev_size * 0.02 * check_speed); // if useing prev pts can project s value out one step ahead
            check_speed *= 2.24; // meters per second to miles per hour
            
            /* Special situations - occur every once in a while, sometimes rarely, and the system fails, for robustness, should deal with them */
            /* Special situation - car changes lane and we change in the same lane -> collision */
            /* When a car merges into our lane right in front of us we won't detect it in our lane until it's too late for collision avoidance */

            // ---------------------------------------------------- LEFT ----------------------------------------------------------------
            // Check if there is a car to our left and distance is too small (30 m) so to disallow lane change
            if (lane > 0) // Can only shift lanes left if not at the edge lane
            {
              if (d < (2 + 4 * (lane - 1) + 2 + COLLISION_AVOIDANCE_DISTANCE) && d > (2 + 4 * (lane - 1) - 2 - COLLISION_AVOIDANCE_DISTANCE)) {
                  // If car ahead
                  if ((check_car_s > car_s) && (fabs(check_car_s - car_s) < (30 - (dynamic_driving * 15)))) {
                    allow_left_change = false;
                  }
                  /* If car behind */
                  if ((check_car_s < car_s) && (fabs(check_car_s - car_s) < (35  - (dynamic_driving * 20))) && (ref_vel < check_speed + 0.25) /* + speed differential */) {
                    allow_left_change = false;
                    //std::cout << "Right lane car, <35m behind, faster than us, allow_left_change = false\n";
                  }
                  /* Even in the above case, don't change INTO a car in the right lane, respect them */
                  if ((check_car_s < car_s) && (fabs(check_car_s - car_s) < (8 - (dynamic_driving * 2))) /* + speed differential */) {
                    allow_left_change = false;
                    //std::cout << "Right lane car, <10m behind, allow_left_change = false\n";
                  }
              }
            }
            else
              allow_left_change = false;
            
            // ---------------------------------------------------- RIGHT ----------------------------------------------------------------
            // Check car to our right and distance too small - if we are clear in all directions, prefer the rightmost lane (law in most countries!) 
            //   and leave left lanes for faster traffic
            if (lane < 2) { // Can only shift if not in rightmost lane
              if (d < (2 + 4 * (lane + 1) + 2 + COLLISION_AVOIDANCE_DISTANCE) && d > (2 + 4 * (lane + 1) - 2 - COLLISION_AVOIDANCE_DISTANCE)) {
                  //std::cout << "Right lane car, (" << check_speed << " and our " << ref_vel << ").\n";
                  /* If car ahead */
                  if ((check_car_s > car_s) && (fabs(check_car_s - car_s) < (30 - (dynamic_driving * 15))) /* + speed differential */) {
                    allow_right_change = false;
                  }
                  
                  /* If car behind */
                  if ((check_car_s < car_s) && (fabs(check_car_s - car_s) < (35  - (dynamic_driving * 20))) && (ref_vel < check_speed + 0.25) /* + speed differential */) {
                    allow_right_change = false;
                    //std::cout << "Right lane car, <35m behind, faster than us, ALLOW_RIGHT_CHANGE = false\n";
                  }
                  /* Even in the above case, don't change INTO a car in the right lane, respect them */
                  if ((check_car_s < car_s) && (fabs(check_car_s - car_s) < (8 - (dynamic_driving * 2))) /* + speed differential */) {
                    allow_right_change = false;
                    //std::cout << "Right lane car, <10m behind, ALLOW_RIGHT_CHANGE = false\n";
                  }
                  
                  /* Should ideally take speed into consideration for the distance required between cars for safe change */
                  // If the car in the right lane has lower speed than us and distance < a quantity, do not change lanes
                  if ((check_car_s > car_s) && (fabs(check_car_s - car_s) < (90  - (dynamic_driving * 45))) && (ref_vel > check_speed - 0.25)) {
                    allow_right_change = false;
                    //std::cout << "Right lane car, <80m ahead, slower than us (" << check_speed << " < our " << ref_vel << " ), ALLOW_RIGHT_CHANGE = false\n";
                  }
              }
            }
            else
              allow_right_change = false;

            // ---------------------------------------------------- THIS LANE ----------------------------------------------------------------
            // Car is in our lane            
            if (d < (2 + 4 * lane + 2 + COLLISION_AVOIDANCE_DISTANCE) && d > (2 + 4 * lane - 2 - COLLISION_AVOIDANCE_DISTANCE)) {
            
              // Car ahead and is closest
              if ((fabs(check_car_s - car_s) < min_dist) && (check_car_s > car_s)) {
                min_dist = fabs(check_car_s - car_s);
                min_pos = check_car_s;
                min_speed = check_speed;
                min_indx = i;
              }

              cars_in_lane_count++;
              
              // check svalues greater than mine and s gap
              // car is in front of us, our car in the future is < 30 m from the other car)
              if ((check_car_s > car_s) && ((check_car_s - car_s) < (60 - (dynamic_driving * 25)))) {
                 slightly_close = true;
              }
              if ((check_car_s > car_s) && ((check_car_s - car_s) < (30 - (dynamic_driving * 15)))) {
                 // do some logic here, lower reference velocity so we dont crash into car in front, could also flag to try to change lanes.
                 too_close = true;
              }
              if ((check_car_s > car_s) && ((check_car_s - car_s) < (15 - (dynamic_driving * 5)))) {
                 way_too_close = true;
              }
              if ((check_car_s > car_s) && ((check_car_s - car_s) < (10 - (dynamic_driving * 2)))) {
                 danger_too_close = true;
                 allow_right_change = false;
                 allow_left_change = false;
                 // Disallow changes, collision needs to be averted first and distance taken
              }
              if ((check_car_s > car_s) && ((check_car_s - car_s) < 8)) {
                 collision_warn_too_close = true;
                 allow_right_change = false;
                 allow_left_change = false;
                 // Disallow changes, collision needs to be averted first and distance taken
              }
              if ((check_car_s > car_s) && ((check_car_s - car_s) > 70)) {
                 far_count++;
              }
            } // car in our lane
          } // End for

          /* ------------------- Take flags and alter behaviour - change lanes, break, accelerate --------------------- */
          if ((cars_in_lane_count == far_count) || (cars_in_lane_count == 0))
            too_far = true;

          /* Get closest car to us which is in our lane */
          double check_car_s = 0;
          double dst = -1;
          double vx = 0;
          double vy = 0;
          double check_speed = ref_vel - 0.01;
          if (min_indx != -1) {
            check_car_s = min_pos;
            dst = min_dist;
            check_speed = min_speed;
            
            //std::cout << "Our lane car, (" << check_speed << " and our " << ref_vel << "), at d = " << min_dist << ".\n";
            
          }
          
          /* If too close allow lane changes! */
          if (collision_warn_too_close || danger_too_close || way_too_close || too_close || slightly_close)  {
            /* When kinda getting near, prefer changing left if possible, otherwise right */
            if (allow_left_change) {
              //std::cout << "Something is too close, changing lane left. \n";

              lane--;
              timer_change_lane = TIMER_BREAK * (1.0 - dynamic_driving);
              // Impulse shift if close to vehicles
              if (ref_vel < 49 + (dynamic_driving * SPEED_EXCEED)) 
                ref_vel += 0.28 * (dynamic_driving + 1);
            }
            else {
              if (allow_right_change) {
                //std::cout << "Something is too close, changing lane right. \n";
                
                lane++;
                timer_change_lane = TIMER_BREAK * (1.0 - dynamic_driving);
                // Impulse shift if close to vehicles
                if (ref_vel < 49 + (dynamic_driving * SPEED_EXCEED)) 
                  ref_vel += 0.28 * (dynamic_driving + 1);
              }
            }
            /* We are close, but we can accelerate*/
            if (ref_vel < target_speed) {
              if (ref_vel < check_speed + 0.25) {
                if (timer_speed_adjust == 0) {
                  ref_vel += 0.5 + (dynamic_driving / 1.2f) - ((100 - dst) / 200); // incr 2.5m/s
                  //std::cout << "We are too close to a car(" << check_speed << "), and our speed(" << ref_vel << ") is less than maximum.\n";
                  //std::cout << "We are close, but speeds allow us accelerating to " << ref_vel << "m/s.\n";
                  timer_speed_adjust = 10;
                  if (slightly_close)
                    timer_speed_adjust = 0;
                }
              }
            }
          }
          
          /* If too close also break! */
          if (collision_warn_too_close) {
            //std::cout << "There is a car < 8m ahead - emergency break!\n";
            // Emergency brake should be adaptive - every meter closer to this generates a harder break
            ref_vel -= (0.5 + (dynamic_driving / 0.9f)) * ((dst != -1)*(8 - (check_car_s - car_s)));
          }
          else if (danger_too_close) {
            /* If distance is closing - then break, otherwise don't */
            ref_vel -= 0.45 + (dynamic_driving / 1.1f);
            //std::cout << "There is a car < 10m ahead - hard break!\n";
          }
          else if (way_too_close) {
            /* If distance is closing - then break, otherwise don't */
            if (ref_vel > check_speed - 0.25) {
              ref_vel -= 0.294 + (dynamic_driving / 1.2f); // 0.224 = 5m/s less
              //std::cout << "There is a car < 15m ahead and it's slower than us - decrease speed.\n";
            }
            //else std::cout << "There is a car < 15m ahead and it's faster than us.\n";
          }
          else if (too_close) {
            /* If distance is closing - then break, otherwise don't */
            if (ref_vel > check_speed - 0.25) {
              //std::cout << "There is a car < 30m ahead and it's slower than us - decrease speed.\n";
              ref_vel -= 0.244 + (dynamic_driving / 1.25f); // 2.5 m/s less
            }
            //else std::cout << "There is a car < 30m ahead and it's faster than us.\n";
          }
          else if (slightly_close) {
            /* This is only executed when we are only slightly close */
            //std::cout << "There is a car < 60m ahead.\n";
          }
          else {
            /* Clear road ahead, prefer right-most lane if not in dynamic driving */
            if (allow_right_change && (dynamic_driving < 0.51f)) {
              //std::cout << "Nothing close - changing lane right.\n";
              lane++;
              timer_change_lane = TIMER_BREAK;
            }
            
            /* Adjust speed*/
            if (ref_vel < target_speed) {
              if (too_far) {
                ref_vel += 0.55 + (dynamic_driving / 0.5f);
                //std::cout << "Nothing close, accelerating vigurously.\n";
              }
              else {
                ref_vel += 0.23 + (dynamic_driving / 1.5f); // incr 2.5m/s
                //std::cout << "Nothing close, accelerating mildly.\n";
              }
            }
            else {
            }
          }
          
          // Sanity check
          if (ref_vel <= 0)
            ref_vel = 0.1;
            
          if (ref_vel > 49.5 + (dynamic_driving * SPEED_EXCEED)) {
              /* Slow down, we have likely given too big of an impulse on a lane change */
              ref_vel -= 0.11;
          }

          /* -------------------------- Variant 4 - keep lane smooth -------------------------- */

          // Reference x, y, yaw states
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          // If previous size is almost empty, use the car as starting reference

          if (prev_size < 2) {
            // Use two points that make the path tangent to the car
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);

            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);

            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);
          }
          // use the previous path's end point as a starting reference
          else {
            ref_x = previous_path_x[prev_size - 1];
            ref_y = previous_path_y[prev_size - 1];

            double ref_x_prev = previous_path_x[prev_size - 2];
            double ref_y_prev = previous_path_y[prev_size - 2];
            ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);

            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);
          }

          // in frenet add evenly 30m spaced points ahead of the starting reference
          vector<double> next_wp0 = getXY(car_s +  30, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s +  60, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s +  90, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp3 = getXY(car_s + 120, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);
          ptsx.push_back(next_wp3[0]);

          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);
          ptsy.push_back(next_wp3[1]);
          
          /* Ensure that when curvature around points is high we don't cut the curb -> add some offset to corner point based on curvature/angle */
          /* Take each ptsx and ptsy from 1 to last-1 and calculate angles. add offset proportional to angle */
          
          for (int i = 0; i < ptsx.size(); i++) {
            // shift car reference angle to 0 degrees // local car coordinates transformation
            double shift_x = ptsx[i] - ref_x;
            double shift_y = ptsy[i] - ref_y;

            ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
            ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
          }

          // Create a spline
          tk::spline s;

          // set (x,y) points on the spline
          s.set_points(ptsx, ptsy); // Only the anchor points

          // start with all of the previous path points from the last time
          //for (auto i = 0; i < previous_path_x.size(); i++) {
          for (auto i = 0; i < prev_size; i++) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          // calculate how to break up spline points so that we travel at our desired reference velocity
          double target_x = 30.0;
          double target_y = s(target_x); // whats the y
          double target_dist = sqrt((target_x) * (target_x) + (target_y) * (target_y));

          double x_add_on = 0;

          // fill up the rest of our path planner after filling it with previous points, here we will always output 50 pts
          double pts = 20;
          if (dynamic_driving > 0)
            pts = 8;
          
          //for (auto i = 1; i <= pts - previous_path_x.size(); i++) {
          for (auto i = 1; i <= pts - prev_size; i++) {
            
            double N = (target_dist / (0.02 * ref_vel / 2.24)); // mph to m/s
            double x_point = x_add_on + (target_x) / N;
            double y_point = s(x_point);
            
            x_add_on = x_point;

            double x_ref = x_point;
            double y_ref = y_point;

            // rotate back to normal after rotating it earlier
            x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
            y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

            x_point += ref_x;
            y_point += ref_y;

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }

          // End TODO
          
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

  h.onConnection([&h, &ref_vel, &first_time, &timer_change_lane, &timer_speed_adjust](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
    
    // Reset speed so we dont break accel/jerk on reconnect
    ref_vel = 0;
    first_time = true;
    timer_change_lane = TIMER_BREAK * 2.5;
    timer_speed_adjust = 0;
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
