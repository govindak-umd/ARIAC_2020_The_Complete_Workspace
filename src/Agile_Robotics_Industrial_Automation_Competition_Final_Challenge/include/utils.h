/**
 * @file utils.cpp
 * @author Pradeep Gopal, Govind Ajith Kumar, Rajesh NS, Cheng, Dakota Abernathy
 * @copyright MIT License
 * @brief Header of the Utils class
 * This class contains all the different utilities such as template structs that are needed by other classes
 */

/**
 *MIT License
 *Copyright (c) 2020 Pradeep Gopal, Govind Ajith Kumar, Rajesh NS, Cheng, Dakota Abernathy
 *Permission is hereby granted, free of charge, to any person obtaining a copy
 *of this software and associated documentation files (the "Software"), to deal
 *in the Software without restriction, including without limitation the rights
 *to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *copies of the Software, and to permit persons to whom the Software is
 *furnished to do so, subject to the following conditions:
 *The above copyright notice and this permission notice shall be included in all
 *copies or substantial portions of the Software.
 *THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *SOFTWARE.
 */

#ifndef UTILS_H
#define UTILS_H
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <unordered_map>
#include <string>
#include <ros/ros.h>
#include <nist_gear/VacuumGripperState.h>
#include <nist_gear/Shipment.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

/**
 * Struct for Orders
 */
typedef struct Order order;

/**
 * Struct for Shipments
 */
typedef struct Shipment shipment;

/**
 * Struct for Products
 */
typedef struct Product product;

/**
 * PI Constant for ease of application
 */
const double PI = 3.141592;

/**
 * Maximum number of cameras
 */
const int MAX_NUMBER_OF_CAMERAS = 18;

/**
 * Maxumum number of attemps for picking up
 */
const int MAX_PICKING_ATTEMPTS = 3;

/**
 * Offset in Z-axis above the target
 */
const double ABOVE_TARGET = 0.2;

/**
 * Timeout for picking the part by the gantry robot
 */
const double PICK_TIMEOUT = 4.0;

/**
 * Timeout for retreiving the part by the gantry robot
 */
const double RETRIEVE_TIMEOUT = 2.0;

/**
 * Conveyor belt speed in metres per second
 */
const double BELT_SPEED = 0.2;

/**
 * Height of the gripper attached to the gantry robot
 */
const double GRIPPER_HEIGHT = 0.01;

/**
 * A small offset to assure that the gripper has been
 * attached firmly
 */
const double EPSILON = 0.008;

/**
 * a constant for bin height
 */
const double BIN_HEIGHT = 0.724;

/**
 * a constant for tray height
 */
const double TRAY_HEIGHT = 0.755;

/**
 * a constant for rail height
 */
const double RAIL_HEIGHT = 0.95;

/**
 * Planning time declared for the move group
 */
const double PLANNING_TIME = 20;

/**
 * Maximum number of times the exchanges can be
 * attempted for flipping the pulley parts
 */
const int MAX_EXCHANGE_ATTEMPTS = 6;

/**
 * Array for the action state, declared externally
 */
extern std::string action_state_name[];

/**
 * Hash map for model height, declared externally
 */
extern std::unordered_map<std::string, double> model_height;

/**
 * @brief      Various possible Statuses of the parts 
 */
enum PartStates {
  FREE,
  BOOKED,
  UNREACHABLE,
  ON_TRAY,
  GRIPPED,
  GOING_HOME,
  REMOVE_FROM_TRAY,
  LOST
};

/**
 * Struct that contains the waypoints of the gantry
 * robot, arms and the agvs
 */
typedef struct PresetLocation {
  std::vector<double> gantry;
  std::vector<double> left_arm;
  std::vector<double> right_arm;
} start, bin3, bin16, bin13, shelf5, agv2, agv2_drop, shelf5, waypoint_1,
    waypoint_2, waypoint_3, waypoint_4, pose_change, flip_target, agv2_flip;

/**
 * Part struct that contains the 
 * model type, its pose, frame, and information 
 * to check if it is faulty or not, and 
 * if it has been picked or not 
 */
typedef struct Part {
  std::string type;  // model type
  geometry_msgs::Pose pose;  // model pose (in frame)
  geometry_msgs::Pose initial_pose;
  geometry_msgs::Pose save_pose;
  std::string frame;  // model frame (e.g., "logical_camera_1_frame")
  ros::Time time_stamp;
  std::string id;
  PartStates state;  // model state (enum PartStates)
  bool faulty;
  bool picked;
} part;

/**
 * Quaternion struct
 */
typedef struct Quaternion {
  double w, x, y, z;
} Quat;

/**
 * Main struct containing the order,
 * shipment and product information
 */
typedef struct master_struct {
  std::string type;
  geometry_msgs::Pose place_part_pose;
  std::string order_id;
  std::string shipment_type;
  std::string agv_id;
  bool delivered = false;
} master_struct;

/**
 * Pick and place struct that contains the
 * pose for pick and placing the parts 
 */
typedef struct pick_and_place {
  std::string type;
  geometry_msgs::Pose pickup_part_pose;
  geometry_msgs::Pose place_part_pose;
} pick_and_place;

/**
 * Position struct that contains the 
 * gantry position information
 */
typedef struct Position {
  std::vector<double> gantry;
  std::vector<double> left;
  std::vector<double> right;
} position;

/**
 * Shipment struct that holds the
 * shipment type, agv ID, order and products
 */
typedef struct Shipment {
  std::string shipment_type;
  std::string agv_id;
  std::vector<nist_gear::Product> products;
  order *parent_order;
} shipment;

/**
 * Product Struct that contains the pose,
 * agv ID, tray information and high priority order
 * information
 */
typedef struct Product {
  std::string type;
  geometry_msgs::Pose pose;
  part p;
  geometry_msgs::Pose actual_pose;
  std::string actual_pose_frame;
  std::string agv_id;
  std::string tray;
  std::string arm_name;
  std::string cache_id;
  shipment *parent_shipment;
  bool high_priority;
  int correction_attempts;
  int service_attempts;

  Product();  // contructor
} product;

/**
 * Order struct that contains the order ID, and the
 * vector of shipments
 */
typedef struct Order {
  std::string order_id;
  std::vector<nist_gear::Shipment> shipments;
} order;

/**
 * Struct that contains the major stats of the simulation
 */
typedef struct Stats {
  double total_time = 0.0;
  double fail_time = 0.0;
  int calls = 0;
  int fails = 0;
} stats;

#endif
