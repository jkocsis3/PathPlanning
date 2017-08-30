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

using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

//In the original file, the below variables were in the h.onmessage, I had to move them out because they reset each time a message was recieved
//have a reference velocity to target
double ref_vel = 0.0;//49.5;
//start in lane 1
int lane = 1;

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

//Calculating euclidian distance between 2 points.
double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

//get the closets waypoint on the hyghway
int ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

//Gets the next way point.  Differs from ClosestWaypoint in if there is a closer waypoint, but it is not 
//along the angle, it will get the right waypoint.  Dont want it getting the waypoint behind you.
int NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2( (map_y-y),(map_x-x) );

	double angle = abs(theta-heading);

	if(angle > pi()/4)
	{
		closestWaypoint++;
	}

	return closestWaypoint;

}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
//Theta helkps for transformation, the list of X and Y waypoints are calculated at the beginning.
//x, y, and theta are whatever we calculate for the maps x and maps y. 
vector<double> getFrenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}


int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;


  // Waypoint map to read from.
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

  
  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    
    

    //gets teh exact position of the vehicle in the simulator
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

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];

            //comes from the previous path size. This can help in transitions as it is the last path the car was following.
            int prev_size = previous_path_x.size();

            if(prev_size > 2)
            {
              car_s = end_path_s;
            }

            bool too_close = false;
            bool change_lanes = false;
            //these will let us know if we are OK to change lanes. 
            bool left_clear = true;
            bool right_clear = true;
            //need to go through the sensor fusion list to see if a car is ahead of us
            for(int i = 0; i < sensor_fusion.size(); i++)
            {
              //get the position of the vehicle
              float d = sensor_fusion[i][6];
             
              //check the speed of the car, calculate the velocity and the angle
              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];
              //check speed is useful to determine where the car will be in the future.
              double check_speed = sqrt(vx*vx+vy+vy);
              double check_car_s = sensor_fusion[i][5];
              //if using previous points can project s value out
              check_car_s+=((double)prev_size*.02*check_speed);
              bool lane_open = true;
              //Check for 30m in front and 15 m behind car
              if((check_car_s > (car_s - 15)) && ((check_car_s-car_s) < 30))
              {
                //If the car is close enough to us, then determine if it is blocking us.
                //this will mark what lane the detected vehicle is in 
                int object_lane = 0;
                //Determine the lane the object is in
                if(4 < d && d < 8)
                {
                  object_lane = 1;
                }
                else if(8.1 < d && d < 12)
                {
                  object_lane = 2;
                }

                //now we need to determine if the car is blocking the left or right, 
                //while also excluding lanes more than 1 lane out.
                if(object_lane > lane && (object_lane < lane + 2))
                {   
                  // std::cout<<"Blocking the right"<<std::endl;               
                  right_clear = false;
                }
                else if (object_lane < lane && (object_lane > lane - 2))
                {
                  // std::cout<<"Blocking the left"<<std::endl;
                  left_clear = false;
                }                
              }

              //this will tell us if a car is in our lane. the +-2 allows us to see a car that is off center in a lane
              if(d < (2+4*lane+2) && (d>(2+4*lane-2)))
              {
                //check s values greater than mine and the s gap
                //if our car is within 30 meters, then slow down
                if((check_car_s > car_s) && ((check_car_s-car_s) < 30))
                {
                  //lower the reference v so we dont hit the car ahead. Could also change lanes.
                  too_close = true;
                  change_lanes = true;
                }
              }
            }

            //we want to favor lane changes to the left as you should always pass on the left if open.
            if(change_lanes)
            {
              // std::cout<<"Change Lanes Triggered"<<std::endl;
              if(left_clear && lane != 0)
              {
                lane = lane - 1;
                // std::cout<<"Left Clear"<<std::endl;
              }
              else if(right_clear && lane != 2)
              {
                lane = lane + 1;
                // std::cout<<"Right Clear"<<std::endl;
              }
            }  
            //If we are not changing lanes due to obstacle, then we want to get back to center. 
            else
            {
              if(lane < 1 && right_clear)
              {
                lane = lane + 1;
              }
              else if(lane > 1 && left_clear)
              {
                lane = lane -1;
              }
            }
           
            //This will allow for a smooth acceleration and decelleration
            if(too_close)
            {
              ref_vel -=.224;
            }
            else if(ref_vel < 49.5)
            {
              ref_vel += .224;
            }

            
            
            //use widely spaced waypoints, spaced at 30m
            //later these are interpolated with a spline and filled in
            vector<double> ptsx;
            vector<double> ptsy;

            //reference the x,y,yaw stated
            //either we reference the starting points as where the car is or at the previous paths and point
            double ref_x = car_x;
            double ref_y = car_y;
            double ref_yaw = deg2rad(car_yaw);

            //if the previous size is almost empty, use the car as a starting reference
            if(prev_size < 2)
            {
              double prev_car_x = car_x - cos(car_yaw);
              double prev_car_y = car_y - sin(car_yaw);
              //like going back in time, and getting the last points/vector
              ptsx.push_back(prev_car_x);
              ptsx.push_back(car_x);
              ptsy.push_back(prev_car_y);
              ptsy.push_back(car_y);
            }
            //use the previous path as a reference
            else
            {
              //redefine the reference state as previous path end point
              ref_x = previous_path_x[prev_size-1];
              ref_y = previous_path_y[prev_size-1];
              double ref_x_prev = previous_path_x[prev_size-2];
              double ref_y_prev = previous_path_y[prev_size-2];
              ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

              //two points that make the path tangent to the previous path end point
              ptsx.push_back(ref_x_prev);
              ptsx.push_back(ref_x);
              ptsy.push_back(ref_y_prev);
              ptsy.push_back(ref_y);

            }

            //in frenet add evenly 30m spaced points ahead of thye starting reference
            vector<double> next_wp_0 = getXY(car_s + 30, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> next_wp_1 = getXY(car_s + 60, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> next_wp_2 = getXY(car_s + 90, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

            ptsx.push_back(next_wp_0[0]);
            ptsx.push_back(next_wp_1[0]);
            ptsx.push_back(next_wp_2[0]);

            ptsy.push_back(next_wp_0[1]);
            ptsy.push_back(next_wp_1[1]);
            ptsy.push_back(next_wp_2[1]);
            //shifting the points to the vehicle coordinate system.
            for(int i = 0; i <ptsx.size(); i++)
            {
              //shift car reference angle 0 degrees
              double shift_x = ptsx[i] - ref_x;
              double shift_y = ptsy[i] - ref_y;

              ptsx[i] = (shift_x * cos(0-ref_yaw) - shift_y*sin(0 - ref_yaw));
              ptsy[i] = (shift_x * sin(0-ref_yaw) + shift_y*cos(0 - ref_yaw));
            }

            //create a spline
            tk::spline s;

            //these are the 5 anchor points, not the path itself.  the next points are the future path
            s.set_points(ptsx, ptsy);

          	vector<double> next_x_vals;
          	vector<double> next_y_vals;

            //start with all of the previous path points from last time
            for(int i = 0; i < previous_path_x.size(); i++)
            {
              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);
            }
            //calculate how to break up the spline points so we travel at our desired reference velocity
            
            //
            //this is the horizon
            double target_x = 30.0;
            //gets the y for a given x from spline
            double target_y = s(target_x);
            //this is doing the distance calculation of: num points * .02 * velocity = distance.
            double target_distance = sqrt((target_x)*(target_x)+(target_y)*(target_y));

            double x_add_on = 0;

            //previous_path_x is not the full path, it is what is left after the car passed x amount of points.
            //fill the rest of the path planner after filling it wil previous points. this will always output 50 points
            for(int i = 0; i < 50 - previous_path_x.size(); i++)
            {
              //dividing by 2.24 because we need to go from MPH to meters per second
              double N = (target_distance/(.02 * ref_vel/2.24));
              double x_point = x_add_on + (target_x)/N;
              double y_point = s(x_point);

              x_add_on = x_point;

              double x_ref = x_point;
              double y_ref = y_point;

              //rotate back to global coordinates from car coordinates.
              x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
              y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

              x_point += ref_x;
              y_point += ref_y;

              next_x_vals.push_back(x_point);
              next_y_vals.push_back(y_point);

            }
            json msgJson;           

          	// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
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
















































































