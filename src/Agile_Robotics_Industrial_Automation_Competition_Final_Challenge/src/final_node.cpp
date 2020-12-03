/**
 * @file final_node.cpp
 * @author Pradeep Gopal, Govind Ajith Kumar, Rajesh NS, Cheng, Dakota Abernathy
 * @copyright MIT License
 * @brief Main file which runs the robot to pickup and deliver parts requested by the user
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

#include <cmath>
#include <algorithm>
#include <vector>
#include <ros/ros.h>
#include <nist_gear/LogicalCameraImage.h>
#include <nist_gear/Order.h>
#include <nist_gear/Proximity.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include "competition.h"
#include "utils.h"
#include "gantry_control.h"
#include <nist_gear/AGVControl.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include <tf/LinearMath/Vector3.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>

/**
 * Defining the maximum number of cameras
 */
#define MAX_NUMBER_OF_CAMERAS 18

/**
 * @brief     Vector to detect the parts from the main camera
 */
std::array<std::array<part, 20>, 20> parts_from_camera_main;

/**
 * @brief      Main 3 dimensional vector for all products in
 * all the shipments in every order
 */

std::vector<std::vector<std::vector<master_struct> > > master_vector_main(
    10,
    std::vector < std::vector<master_struct>
        > (10, std::vector < master_struct > (20)));

/**
 * checking if the part was placed or not
 */
bool part_placed = false;

/**
 * Some necessary iterators
 */
int k = 0, i = 0, temp = 45, size_of_order = 0;

/**
 * Parts delivered
 */
int parts_delivered[5] { };

/**
 * For flipping of the part
 */
const double flip = -3.14159;

/**
 * Declaring faulty part as an object for part struct
 */
part faulty_part_from_sensor;

/**
 * Shipment types and AGV ID
 */
std::string shipment_type, agv_id;

/**
 * Array of parts detected by camera 16
 */
std::array<part, 20> parts_from_camera_16;

/**
 * Array of parts detected by camera 17
 */
std::array<part, 20> parts_from_camera_17;

/**
 * Checking if a part was picked or not
 */
bool conveyor_part_picked = false;

/**
 * @brief      Function to govern Submission of order
 *
 * @param[in]  AVG_id         The avg identifier
 * @param[in]  shipment_type  The shipment type
 *
 * @return     Boolean or True (1) or False(0)
 */
bool submitOrder(std::string AVG_id, std::string shipment_type) {

  ROS_INFO_STREAM(
      "Delivering " << AVG_id << " with shipment = " << shipment_type);
  ROS_INFO("[submitOrder] Submitting order via AVG");

  // Create a node to call service from. Would be better to use one existing node
  // rather than creating a new one every time
  ros::NodeHandle node;

  // Create a Service client for the correct service, i.e. '/ariac/agv{AVG_id}'
  ros::ServiceClient avg_client1;
  ros::ServiceClient avg_client2;

  // Assign the service client to the correct service
  if (AVG_id == "agv1") {
    avg_client1 = node.serviceClient < nist_gear::AGVControl > ("/ariac/agv1");
    // Wait for client to start
    if (!avg_client1.exists()) {
      avg_client1.waitForExistence();
    }

    // Debug what you're doing
    ROS_INFO_STREAM(
        "[submitOrder] Sending AVG " << AVG_id << " to submit order");

    // Create the message and assign the shipment type to it
    nist_gear::AGVControl srv;
    srv.request.shipment_type = shipment_type;

    // Send message and retrieve response
    avg_client1.call(srv);
    if (!srv.response.success) {  // If not successful, print out why.
      ROS_ERROR_STREAM(
          "[submitOrder]  Failed to submit: " << srv.response.message);
    } else {
      ROS_INFO("[submitOrder] Submitted");
    }

    return srv.response.success;
  } else if (AVG_id == "agv2") {
    avg_client2 = node.serviceClient < nist_gear::AGVControl > ("/ariac/agv2");
    // Wait for client to start
    if (!avg_client2.exists()) {
      avg_client2.waitForExistence();
    }

    // Debug what you're doing
    ROS_INFO_STREAM(
        "[submitOrder] Sending AVG " << AVG_id << " to submit order");

    // Create the message and assign the shipment type to it
    nist_gear::AGVControl srv;
    srv.request.shipment_type = shipment_type;

    // Send message and retrieve response
    avg_client2.call(srv);
    if (!srv.response.success) {  // If not successful, print out why.
      ROS_ERROR_STREAM(
          "[submitOrder]  Failed to submit: " << srv.response.message);
    } else {
      ROS_INFO("[submitOrder] Submitted");
    }

    return srv.response.success;
  } else {
    ROS_ERROR_STREAM(
        "[submitOrder] No AVG with id " << AVG_id
            << ". Valid ids are 1 and 2 only");
  }

}

/**
 * @brief      Boolean function to get the breakbeam sensor status. This is
 * essential for gantry control and decide when to follow the human towards the shelf
 *
 * @param      comp  Competition node
 * @param[in]  br_1  Breakbeam identifier
 *
 * @return     Boolean of when the first breakbeam is triggered
 */

bool getBreakBeam1(Competition &comp, std::string br_1) {
  if (br_1 == "11") {
    return comp.breakbeam_part_status_11;
  } else if (br_1 == "21") {
    return comp.breakbeam_part_status_21;
  } else if (br_1 == "31") {
    return comp.breakbeam_part_status_31;
  } else if (br_1 == "41") {
    return comp.breakbeam_part_status_41;
  }
}

/**
 * @brief      Boolean function to get the breakbeam sensor status. This is
 * essential for gantry control and decide when to follow the human towards the shelf
 *
 * @param      comp  Competition node
 * @param[in]  br_5  Breakbeam identifier
 *
 * @return     Boolean of when the second breakbeam is triggered
 */
bool getBreakBeam5(Competition &comp, std::string br_5) {
  if (br_5 == "15") {
    return comp.breakbeam_part_status_15;
  } else if (br_5 == "25") {
    return comp.breakbeam_part_status_25;
  } else if (br_5 == "35") {
    return comp.breakbeam_part_status_35;
  } else if (br_5 == "45") {
    return comp.breakbeam_part_status_45;
  }
}

/**
 * @brief      Boolean function to get the breakbeam sensor status. This is
 * essential for gantry control and decide when to exit the shelves and return towards
 * the conveyor
 *
 * @param      comp  Competition node
 * @param[in]  br_2  Breakbeam identifier
 *
 * @return     Boolean of when the second breakbeam is triggered
 */
bool getBreakBeam2(Competition &comp, std::string br_2) {
  if (br_2 == "12") {
    return comp.breakbeam_part_status_12;
  } else if (br_2 == "22") {
    return comp.breakbeam_part_status_22;
  } else if (br_2 == "32") {
    return comp.breakbeam_part_status_32;
  } else if (br_2 == "42") {
    return comp.breakbeam_part_status_42;
  }
}

/**
 * @brief      Boolean function to get the breakbeam sensor status. This is
 * essential for gantry control and decide when to exit the shelves and return towards
 * the conveyor as well as when to go and pick up the part when the human leaves the area
 *
 * @param      comp  Competition node
 * @param[in]  br_4  Breakbeam identifier
 *
 * @return     Boolean of when the second breakbeam is triggered
 */
bool getBreakBeam4(Competition &comp, std::string br_4) {
  if (br_4 == "14") {
    return comp.breakbeam_part_status_14;
  } else if (br_4 == "24") {
    return comp.breakbeam_part_status_24;
  } else if (br_4 == "34") {
    return comp.breakbeam_part_status_34;
  } else if (br_4 == "44") {
    return comp.breakbeam_part_status_44;
  }
}

/**
 * @brief      Function to follow the human being.
 * This function times its exit from the conveyor belt and heads behind the human
 * before seeking shelter in between the shelf gaps
 *
 * @param      comp                 Competition node
 * @param      gantry               Gantry node
 * @param[in]  waypoint_iter        The waypoint iterator
 * @param[in]  br_1                 Breakbeam 1 identifier
 * @param[in]  br_2                 Breakbeam 2 identifier
 * @param[in]  GENERAL_BREAKBEAM_1  Breakbeam 1 status
 * @param[in]  GENERAL_BREAKBEAM_2  Breakbeam 2 status
 */
void followHuman(Competition &comp, GantryControl &gantry, auto waypoint_iter,
                 std::string br_1, std::string br_2, bool GENERAL_BREAKBEAM_1,
                 bool GENERAL_BREAKBEAM_2) {
  ROS_INFO_STREAM("Waiting for the person to move");
  GENERAL_BREAKBEAM_1 = false;
  ros::Time TIME_1;
  GENERAL_BREAKBEAM_2 = false;
  ros::Time TIME_2;
  double vel;
  double distance_between_sensors = 1.2;
  ros::Duration diff_speed_time;
  while (true) {
    if (getBreakBeam1(comp, br_1) == true and GENERAL_BREAKBEAM_1 == false) {
      ROS_INFO_STREAM("FIRST ONE TRIGGERED ");
      TIME_1 = ros::Time::now();
      GENERAL_BREAKBEAM_1 = true;
    }
    if (getBreakBeam2(comp, br_2) == true and GENERAL_BREAKBEAM_2 == false) {
      ROS_INFO_STREAM("SECOND ONE TRIGGERED ");
      TIME_2 = ros::Time::now();
      GENERAL_BREAKBEAM_2 = true;
    }

    if (GENERAL_BREAKBEAM_1 == true and GENERAL_BREAKBEAM_2 == true) {
      ROS_INFO_STREAM("BOTH TRIGGERED");
      ROS_INFO_STREAM(TIME_1);
      ROS_INFO_STREAM(TIME_2);
      ros::Duration diff = TIME_1 - TIME_2;
      ROS_INFO_STREAM("diff in time between both is : " << diff);
      if (TIME_2 > TIME_1) {
        ROS_INFO_STREAM("Waiting for human's response");
        double time_1 = TIME_1.toSec();
        double time_2 = TIME_2.toSec();
        vel = std::abs(distance_between_sensors / (time_2 - time_1));
        ROS_INFO_STREAM("HUMAN SPEED ----- " << vel);
        gantry.goToPresetLocation(waypoint_iter);
        break;
      } else {
        GENERAL_BREAKBEAM_1 = false;
        GENERAL_BREAKBEAM_2 = false;
      }
    }
  }
}

/**
 * @brief      Function to exit the shelf gaps and go pick up the part before the human comes back again
 *
 * @param      comp                 Competition node
 * @param      gantry               Gantry node
 * @param[in]  waypoint_iter        The waypoint iterator
 * @param[in]  br_4                 Breakbeam 4 identifier
 * @param[in]  br_5                 Breakbeam 5 identifier
 * @param[in]  GENERAL_BREAKBEAM_4  Breakbeam 4 status
 * @param[in]  GENERAL_BREAKBEAM_5  Breakbeam 5 status
 */
void checkAndProceed(Competition &comp, GantryControl &gantry,
                     auto waypoint_iter, std::string br_4, std::string br_5,
                     bool GENERAL_BREAKBEAM_4, bool GENERAL_BREAKBEAM_5) {
  GENERAL_BREAKBEAM_4 = false;
  ros::Time TIME_4;
  GENERAL_BREAKBEAM_5 = false;
  ros::Time TIME_5;
  double vel;
  double distance_between_sensors = 1.2;
  ros::Duration diff_speed_time;
  while (true) {
    ROS_INFO_STREAM("WAITING FOR THE HUMAN TO GO PAST THE GANTRY....");
    if (getBreakBeam5(comp, br_5) == true and GENERAL_BREAKBEAM_5 == false) {
      ROS_INFO_STREAM("FIFTH ONE TRIGGERED ");
      TIME_5 = ros::Time::now();
      GENERAL_BREAKBEAM_5 = true;
    }
    if (getBreakBeam4(comp, br_4) == true and GENERAL_BREAKBEAM_4 == false) {
      ROS_INFO_STREAM("FOURTH ONE TRIGGERED ");
      TIME_4 = ros::Time::now();
      GENERAL_BREAKBEAM_4 = true;
    }
    if (GENERAL_BREAKBEAM_4 == true and GENERAL_BREAKBEAM_5 == true) {
      ROS_INFO_STREAM("BOTH TRIGGERED");
      ROS_INFO_STREAM(TIME_4);
      ROS_INFO_STREAM(TIME_5);
      ros::Duration diff = TIME_5 - TIME_4;
      ROS_INFO_STREAM("diff in time between both is : " << diff);
      if (TIME_4 > TIME_5) {
        ROS_INFO_STREAM("ROBOT SHOULD BE EXITING NOW ...>>>>>");
        double time_5 = TIME_5.toSec();
        double time_4 = TIME_4.toSec();
        vel = std::abs(distance_between_sensors / (time_4 - time_5));
        ROS_INFO_STREAM("HUMAN SPEED ----- " << vel);
        gantry.goToPresetLocation(waypoint_iter);
        break;
      } else {
        GENERAL_BREAKBEAM_4 = false;
        GENERAL_BREAKBEAM_5 = false;
      }
    }
  }
}

/**
 * @brief      Function to exit the shelf gaps and return towards the conveyor belt
 * This function times its exit towards the conveyor belt and heads back from the shelf gaps
 *
 * @param      comp                 Competition node
 * @param      gantry               Gantry node
 * @param[in]  waypoint_iter        The waypoint iterator
 * @param[in]  br_4                 Breakbeam 4 identifier
 * @param[in]  br_5                 Breakbeam 5 identifier
 * @param[in]  GENERAL_BREAKBEAM_4  Breakbeam 4 status
 * @param[in]  GENERAL_BREAKBEAM_5  Breakbeam 5 status
 */
void checkAndProceedBackwards(Competition &comp, GantryControl &gantry,
                              auto waypoint_iter, std::string br_4,
                              std::string br_5, bool GENERAL_BREAKBEAM_4,
                              bool GENERAL_BREAKBEAM_5) {
  GENERAL_BREAKBEAM_4 = false;
  ros::Time TIME_4;
  GENERAL_BREAKBEAM_5 = false;
  ros::Time TIME_5;
  double vel;
  double distance_between_sensors = 1.2;
  ros::Duration diff_speed_time;
  while (true) {
    ROS_INFO_STREAM("WAITING FOR THE HUMAN TO GO PAST THE GANTRY....");
    if (getBreakBeam5(comp, br_5) == true and GENERAL_BREAKBEAM_5 == false) {
      ROS_INFO_STREAM("FIFTH ONE TRIGGERED ");
      TIME_5 = ros::Time::now();
      GENERAL_BREAKBEAM_5 = true;
    }
    if (getBreakBeam4(comp, br_4) == true and GENERAL_BREAKBEAM_4 == false) {
      ROS_INFO_STREAM("FOURTH ONE TRIGGERED ");
      TIME_4 = ros::Time::now();
      GENERAL_BREAKBEAM_4 = true;
    }
    if (GENERAL_BREAKBEAM_4 == true and GENERAL_BREAKBEAM_5 == true) {
      ROS_INFO_STREAM("BOTH TRIGGERED");
      ROS_INFO_STREAM(TIME_4);
      ROS_INFO_STREAM(TIME_5);
      ros::Duration diff = TIME_5 - TIME_4;
      ROS_INFO_STREAM("diff in time between both is : " << diff);
      if (TIME_5 > TIME_4) {
        ROS_INFO_STREAM("ROBOT SHOULD BE EXITING NOW ...>>>>>");
        double time_5 = TIME_5.toSec();
        double time_4 = TIME_4.toSec();
        vel = std::abs(distance_between_sensors / (time_4 - time_5));
        ROS_INFO_STREAM("HUMAN SPEED ----- " << vel);
        gantry.goToPresetLocation(waypoint_iter);
        break;
      } else {
        GENERAL_BREAKBEAM_4 = false;
        GENERAL_BREAKBEAM_5 = false;
      }
    }
  }
}

/**
 * @brief      Returns a double value of the offset of various parts on the tray
 *
 * @param[in]  part_name  The part name
 *
 * @return     The offset to pickup part on tray.
 */

double get_offset_to_pickup_part_on_tray(const std::string &part_name) {
  if (part_name == "pulley_part_red" || part_name == "pulley_part_blue"
      || part_name == "pulley_part_green") {
    return 0.02;    // check
  } else if (part_name == "gasket_part_red" || part_name == "gasket_part_blue"
      || part_name == "gasket_part_green") {
    return 0.0275;   // check
  } else if (part_name == "piston_rod_part_red"
      || part_name == "piston_rod_part_blue"
      || part_name == "pisy" "ton_rod_part_green") {
    return 0.0195;   // check
  } else if (part_name == "gear_part_red" || part_name == "gear_part_blue"
      || part_name == "gear_part_green") {
    return 0.01;    // check
  } else if (part_name == "disk_part_red" || part_name == "disk_part_blue"
      || part_name == "disk_part_green") {
    return 0.02;    // check
  } else {
    ROS_ERROR_STREAM(part_name << " is not a part in record");
    return 0.0;
  }
}

/**
 * @brief      Fixes the pose of the parts for placing on the AGV Tray
 *
 * @param      comp                Competition node
 * @param[in]  master_vector_main  The master vector containing the orders, shipments and products
 * @param      gantry              Gantry node
 * @param      part_in_tray        The part in tray
 */
void fix_part_pose(Competition &comp, master_struct master_vector_main,
                   GantryControl &gantry, part &part_in_tray) {
  double offset = 0.01;
  parts_from_camera_16 = comp.get_parts_from_16_camera();
  parts_from_camera_17 = comp.get_parts_from_17_camera();

  if (master_vector_main.agv_id == "agv1") {
    for (int part_idx = 0; part_idx < parts_from_camera_16.size(); part_idx++) {

      if (master_vector_main.type == parts_from_camera_16[part_idx].type) {

        if ((std::abs(
            comp.parts_from_16_camera[part_idx].pose.position.x
                - gantry.getTargetWorldPose(master_vector_main.place_part_pose,
                                            master_vector_main.agv_id).position
                    .x) > offset)
            || (std::abs(
                comp.parts_from_16_camera[part_idx].pose.position.y
                    - gantry.getTargetWorldPose(
                        master_vector_main.place_part_pose,
                        master_vector_main.agv_id).position.y) > offset)
            || (std::abs(
                comp.parts_from_16_camera[part_idx].pose.position.z
                    - gantry.getTargetWorldPose(
                        master_vector_main.place_part_pose,
                        master_vector_main.agv_id).position.z) > offset)) {

          ROS_INFO_STREAM("Attempting replacement of the part");
          if (master_vector_main.agv_id == "agv1")
            gantry.goToPresetLocation(gantry.agv1_);
          else
            gantry.goToPresetLocation(gantry.agv2_);

          part part_re_pick;
          part_re_pick = comp.parts_from_17_camera[part_idx];
          part_re_pick.pose.position.z = part_re_pick.pose.position.z
              + get_offset_to_pickup_part_on_tray(part_re_pick.type);
          gantry.pickPart(part_re_pick);
          if (master_vector_main.agv_id == "agv1")
            gantry.goToPresetLocation(gantry.agv1_);
          else
            gantry.goToPresetLocation(gantry.agv2_);
          ROS_INFO_STREAM("Placing part again");
          gantry.placePart(part_in_tray, master_vector_main.agv_id);
        }
      }
    }
  } else {
    ROS_INFO_STREAM(
        "Parts present in AGV1, checking to see if part needs to re-picked up");
    for (int part_idx = 0; part_idx < parts_from_camera_17.size(); part_idx++) {
      if (master_vector_main.type == parts_from_camera_17[part_idx].type) {

        if ((std::abs(
            comp.parts_from_17_camera[part_idx].pose.position.x
                - gantry.getTargetWorldPose(master_vector_main.place_part_pose,
                                            master_vector_main.agv_id).position
                    .x) > offset)
            || (std::abs(
                comp.parts_from_17_camera[part_idx].pose.position.y
                    - gantry.getTargetWorldPose(
                        master_vector_main.place_part_pose,
                        master_vector_main.agv_id).position.y) > offset)
            || (std::abs(
                comp.parts_from_17_camera[part_idx].pose.position.z
                    - gantry.getTargetWorldPose(
                        master_vector_main.place_part_pose,
                        master_vector_main.agv_id).position.z) > offset)) {

          ROS_INFO_STREAM("Attempting replacement of the part");
          if (master_vector_main.agv_id == "agv1")
            gantry.goToPresetLocation(gantry.agv1_);
          else
            gantry.goToPresetLocation(gantry.agv2_);

          part part_re_pick;
          part_re_pick = comp.parts_from_17_camera[part_idx];
          part_re_pick.pose.position.z = part_re_pick.pose.position.z + 0.02;

          gantry.pickPart(part_re_pick);
          ROS_INFO_STREAM("Part Picked again");
          if (master_vector_main.agv_id == "agv1")
            gantry.goToPresetLocation(gantry.agv1_);
          else
            gantry.goToPresetLocation(gantry.agv2_);
          gantry.placePart(part_in_tray, master_vector_main.agv_id);
          ROS_INFO_STREAM(
              "Current status of the disk_part_green part As read from the camera!");
        }
      }
    }
  }
}

/**
 * @brief      Picks the parts from the conveyor
 * @param      comp    Competition node
 * @param      gantry  Gantry Control node
 */
void pick_part_from_conveyor(Competition &comp, GantryControl &gantry) {

  ROS_INFO_STREAM("Checking if conveyor belt has started");
  if (comp.conveyor_belt_part_status == false) {
    ROS_INFO_STREAM("Conveyor belt not started, waiting for 5 seconds");
    ros::Duration(5).sleep();
    if (comp.conveyor_belt_part_status == false) {
      ROS_INFO_STREAM("Returning since conveyor belt is turned off");
      return;
    }
  }
  ROS_INFO_STREAM("Conveyor belt started, attempting to pick parts");
  int parts_in_logical_camera_11 = comp.parts_in_logical_camera_11;
  int parts_in_logical_camera_12 = comp.parts_in_logical_camera_12;
  int parts_in_logical_camera_13 = comp.parts_in_logical_camera_13;
  int parts_in_logical_camera_14 = comp.parts_in_logical_camera_14;

  double y_offset_est = 0;
  double z_offset_est = 0;
  int no_of_parts { 3 }, count { 0 };

  ROS_INFO_STREAM("Picking up part from conveyor belt");
  gantry.goToPresetLocation(gantry.start_);
  ROS_INFO_STREAM("Start location reached");

  while (count < no_of_parts) {
    // move above pick location above belt
    gantry.goToPresetLocation(gantry.belt_pickup_);
    ROS_INFO_STREAM("belt pick up location reached");

    ROS_INFO_STREAM("Picking up part number " << count + 1);
    while ((comp.breakbeam_conveyor_belt_part_status_0 == true)
        || (comp.breakbeam_conveyor_belt_part_status_1 == true)) {
    }
    ROS_INFO_STREAM("Attempting to pickup part on belt");

    if (!comp.get_parts_from_15_camera().empty()) {  // if no part detected in camera 15
      ROS_INFO_STREAM(comp.get_parts_from_15_camera().back().type);
      if ((comp.get_parts_from_15_camera().back().type == "piston_rod_part_red")
          || (comp.get_parts_from_15_camera().back().type
              == "piston_rod_part_blue")
          || (comp.get_parts_from_15_camera().back().type
              == "piston_rod_part_green")) {
        ROS_INFO_STREAM("piston part");
        y_offset_est = 0.292;
        z_offset_est = 0.009;
      } else if ((comp.get_parts_from_15_camera().back().type
          == "gasket_part_green")
          || (comp.get_parts_from_15_camera().back().type == "gasket_part_blue")
          || (comp.get_parts_from_15_camera().back().type == "gasket_part_red")) {
        ROS_INFO_STREAM("gasket part");
        y_offset_est = 0.295;
        z_offset_est = 0.009;
      } else if ((comp.get_parts_from_15_camera().back().type
          == "disk_part_green")
          || (comp.get_parts_from_15_camera().back().type == "disk_part_blue")
          || (comp.get_parts_from_15_camera().back().type == "disk_part_red")) {
        ROS_INFO_STREAM("disk part");
        y_offset_est = 0.295;
        z_offset_est = 0.009;
      } else if ((comp.get_parts_from_15_camera().back().type
          == "pulley_part_green")
          || (comp.get_parts_from_15_camera().back().type == "pulley_part_blue")
          || (comp.get_parts_from_15_camera().back().type == "pulley_part_red")) {
        ROS_INFO_STREAM("pulley part");
        y_offset_est = 0.265;
        z_offset_est = 0.009;
      }

      part part_picking = comp.get_parts_from_15_camera().back();
      part_picking.pose.position.z += z_offset_est;
      part_picking.pose.position.y -= y_offset_est;

      if (gantry.pickMovingPart(part_picking)) {    // if part picked up
        ROS_INFO_STREAM("Part picked");
        gantry.goToPresetLocation(gantry.belt_pickup_1);
        ROS_INFO_STREAM("belt pick up location reached");

        //// drop part at desired location on bin1

        if (parts_in_logical_camera_11 == 0) {
          PresetLocation bin1_drop = gantry.bin1_drop_;
          if (count == 1)
            bin1_drop.gantry[0] += (count) * 0.25;  // offset the next drop off location by 0.25
          if (count == 2)
            bin1_drop.gantry[1] += (count) * 0.15;  // offset the next drop off location by 0.25

          gantry.goToPresetLocation(gantry.start_);
          gantry.goToPresetLocation(bin1_drop);
          ROS_INFO_STREAM("bin 1 location reached");
          gantry.deactivateGripper("left_arm");
          ROS_INFO_STREAM("Gripper Deactivated");

          gantry.goToPresetLocation(gantry.start_);
          ROS_INFO_STREAM("Start Location Reached");

          //// update parts in camera info (use camera 11 call back function)
          parts_from_camera_main[11][count] = comp.parts_from_11_camera[count];  // update parts in camera above bin1
        }

        else if (parts_in_logical_camera_12 == 0) {
          PresetLocation bin3_drop = gantry.bin3_drop_;
          if (count == 1)
            bin3_drop.gantry[0] += (count) * 0.25;  // offset the next drop off location by 0.25
          if (count == 2)
            bin3_drop.gantry[1] += (count) * 0.15;  // offset the next drop off location by 0.25
          gantry.goToPresetLocation(gantry.start_);
          gantry.goToPresetLocation(bin3_drop);
          ROS_INFO_STREAM("bin 3 location reached");
          gantry.deactivateGripper("left_arm");
          ROS_INFO_STREAM("Gripper Deactivated");

          gantry.goToPresetLocation(gantry.start_);
          ROS_INFO_STREAM("Start Location Reached");

          //// update parts in camera info (use camera 12 call back function)
          parts_from_camera_main[12][count] = comp.parts_from_12_camera[count];  // update parts in camera above bin1
        }

        else if (parts_in_logical_camera_13 == 0) {
          PresetLocation bin9_drop = gantry.bin9_drop_;
          if (count == 1)
            bin9_drop.gantry[0] += (count) * 0.25;  // offset the next drop off location by 0.25
          if (count == 2)
            bin9_drop.gantry[1] += (count) * 0.15;  // offset the next drop off location by 0.25
          gantry.goToPresetLocation(gantry.start_);
          gantry.goToPresetLocation(bin9_drop);
          ROS_INFO_STREAM("bin 9 location reached");
          gantry.deactivateGripper("left_arm");
          ROS_INFO_STREAM("Gripper Deactivated");

          gantry.goToPresetLocation(gantry.start_);
          ROS_INFO_STREAM("Start Location Reached");

          //// update parts in camera info (use camera 13 call back function)
          parts_from_camera_main[13][count] = comp.parts_from_13_camera[count];  // update parts in camera above bin1
        }

        else if (parts_in_logical_camera_14 == 0) {
          PresetLocation bin11_drop = gantry.bin11_drop_;
          if (count == 1)
            bin11_drop.gantry[0] += (count) * 0.25;  // offset the next drop off location by 0.25
          if (count == 2)
            bin11_drop.gantry[1] += (count) * 0.15;  // offset the next drop off location by 0.25
          gantry.goToPresetLocation(gantry.start_);
          gantry.goToPresetLocation(bin11_drop);
          ROS_INFO_STREAM("bin 11 location reached");
          gantry.deactivateGripper("left_arm");
          ROS_INFO_STREAM("Gripper Deactivated");

          gantry.goToPresetLocation(gantry.start_);
          ROS_INFO_STREAM("Start Location Reached");

          //// update parts in camera info (use camera 14 call back function)
          parts_from_camera_main[14][count] = comp.parts_from_14_camera[count];  // update parts in camera above bin1
        }

        count += 1;
      } else {
        ROS_INFO_STREAM("Part not pick, try again");
      }
    } else {
      ROS_INFO_STREAM("no part on belt");
    }

//    ros::Duration(2).sleep();
  }
  conveyor_part_picked = true;
  ROS_INFO_STREAM("first part " << parts_from_camera_main[11][0].pose);

  ROS_INFO_STREAM("second part " << parts_from_camera_main[11][1].pose);

}

/**
 * @brief      Returns the part location on the shelf (FRONT(f) OR BACK(b))
 * @param[in]  pose          The pose of the part
 * @param[in]  camera_index  The camera index of the logical camera detecting the part
 * @return     An identifier for the part location on the shelf
 */
std::string part_location(geometry_msgs::Pose pose, int camera_index) {
  if ((camera_index == 7) || (camera_index == 10))  // Shelf 1
      {
    if (pose.position.y > 3.5) {
      ROS_INFO_STREAM("Part found in front of shelf 1");
      return "f";
    } else {
      ROS_INFO_STREAM("Part found in back of shelf 1");
      return "b";
    }
  }

  else if ((camera_index == 8) || (camera_index == 9))  // Shelf 2
      {
    if (pose.position.y > -3.5) {
      ROS_INFO_STREAM("Part found in front of shelf 8");
      return "f";
    } else {
      ROS_INFO_STREAM("Part found in back of shelf 8");
      return "b";
    }
  }

  else if ((camera_index == 3) || (camera_index == 4))  // Shelf 8
      {
    if (pose.position.y > 0) {
      ROS_INFO_STREAM("Part found in front of shelf 8");
      return "f";
    } else {
      ROS_INFO_STREAM("Part found in back of shelf 8");
      return "b";
    }
  }

  else if ((camera_index == 1) || (camera_index == 2))  // Shelf 5
      {
    if (pose.position.y > 3.1) {
      ROS_INFO_STREAM("Part found in front of shelf 5");
      return "f";
    } else {
      ROS_INFO_STREAM("Part found in back of shelf 5");
      return "b";
    }
  }

  else if ((camera_index == 5) || (camera_index == 6))  // Shelf 11
      {
    if (pose.position.y > -3.0) {
      ROS_INFO_STREAM("Part found in front of shelf 11");
      return "f";
    } else {
      ROS_INFO_STREAM("Part found in back of shelf 11");
      return "b";
    }
  }

  else if (camera_index == 11) {
    ROS_INFO_STREAM("X = " << pose.position.x << " Y = " << pose.position.y);

    if ((2.3 < pose.position.x) && (pose.position.x < 2.9)
        && (1.7 < pose.position.y) && (pose.position.y < 2.5)) {
      ROS_INFO_STREAM("Part found in bin5");
      return "_5";
    }

    else if ((2.3 < pose.position.x) && (pose.position.x < 2.9)
        && (1.0 < pose.position.y) && (pose.position.y < 1.7)) {
      ROS_INFO_STREAM("Part found in bin1");
      return "_1";
    }

    else if ((3.1 < pose.position.x) && (pose.position.x < 4)
        && (1.8 < pose.position.y) && (pose.position.y < 2.5)) {
      ROS_INFO_STREAM("Part found in bin6");
      return "_6";
    }

    else {
      ROS_INFO_STREAM("Part found in bin2");
      return "_2";
    }
  }

  else if (camera_index == 12) {
    ROS_INFO_STREAM("X = " << pose.position.x << " Y = " << pose.position.y);

    if ((4.18 < pose.position.x) && (pose.position.x < 4.78)
        && (1.7 < pose.position.y) && (pose.position.y < 2.5)) {
      ROS_INFO_STREAM("Part found in bin7");
      return "_7";
    }

    else if ((4.18 < pose.position.x) && (pose.position.x < 4.78)
        && (1.0 < pose.position.y) && (pose.position.y < 1.7)) {
      ROS_INFO_STREAM("Part found in bin3");
      return "_3";
    }

    else if ((5 < pose.position.x) && (pose.position.x < 5.88)
        && (1.8 < pose.position.y) && (pose.position.y < 2.5)) {
      ROS_INFO_STREAM("Part found in bin8");
      return "_8";
    }

    else {
      ROS_INFO_STREAM("Part found in bin4");
      return "_4";
    }
  }

  else if (camera_index == 14) {
    ROS_INFO_STREAM("X = " << pose.position.x << " Y = " << pose.position.y);

    if ((4.18 < pose.position.x) && (pose.position.x < 4.78)
        && (-1.65 < pose.position.y) && (pose.position.y < -1)) {
      ROS_INFO_STREAM("Part found in bin11");
      return "_11";
    }

    else if ((4.18 < pose.position.x) && (pose.position.x < 4.78)
        && (-2.5 < pose.position.y) && (pose.position.y < -1.7)) {
      ROS_INFO_STREAM("Part found in bin15");
      return "_15";
    }

    else if ((5 < pose.position.x) && (pose.position.x < 5.88)
        && (-1.65 < pose.position.y) && (pose.position.y < -1)) {
      ROS_INFO_STREAM("Part found in bin12");
      return "_12";
    }

    else {
      ROS_INFO_STREAM("Part found in bin16");
      return "_16";
    }
  }

  else if (camera_index == 13) {
    ROS_INFO_STREAM("X = " << pose.position.x << " Y = " << pose.position.y);

    if ((2.3 < pose.position.x) && (pose.position.x < 2.9)
        && (-1.65 < pose.position.y) && (pose.position.y < -1)) {
      ROS_INFO_STREAM("Part found in bin9");
      return "_9";
    }

    else if ((2.3 < pose.position.x) && (pose.position.x < 2.9)
        && (-2.5 < pose.position.y) && (pose.position.y < -1.7)) {
      ROS_INFO_STREAM("Part found in bin13");
      return "_13";
    }

    else if ((3.12 < pose.position.x) && (pose.position.x < 4)
        && (-1.65 < pose.position.y) && (pose.position.y < -1)) {
      ROS_INFO_STREAM("Part found in bin10");
      return "_10";
    }

    else {
      ROS_INFO_STREAM("Part found in bin14");
      return "_14";
    }
  }
}

/**
 * @brief      To safely exit the bins after the parts have been picked up. Assists in case
 * of MOVEIT failure.
 *
 * @param[in]  camera_id  The camera identifier of the logical camera above the bin
 * @param      gantry     Gantry control node
 */
void safelyexitBin(std::string camera_id, GantryControl &gantry) {
  if (camera_id == "11_1") {
    ROS_INFO_STREAM("SAFELY EXITING THE BIN");
    gantry.goToPresetLocation(gantry.bin1_);
    gantry.goToPresetLocation(gantry.start_);
  }
  if (camera_id == "11_2") {
    ROS_INFO_STREAM("SAFELY EXITING THE BIN");
    gantry.goToPresetLocation(gantry.bin2_);
    gantry.goToPresetLocation(gantry.start_);
  }
  if (camera_id == "11_5") {
    ROS_INFO_STREAM("SAFELY EXITING THE BIN");
    gantry.goToPresetLocation(gantry.bin5_);
    gantry.goToPresetLocation(gantry.start_);
  }
  if (camera_id == "11_6") {
    ROS_INFO_STREAM("SAFELY EXITING THE BIN");
    gantry.goToPresetLocation(gantry.bin6_);
    gantry.goToPresetLocation(gantry.start_);
  }
  if (camera_id == "12_3") {
    ROS_INFO_STREAM("SAFELY EXITING THE BIN");
    gantry.goToPresetLocation(gantry.bin3_);
    gantry.goToPresetLocation(gantry.start_);
  }
  if (camera_id == "12_4") {
    ROS_INFO_STREAM("SAFELY EXITING THE BIN");
    gantry.goToPresetLocation(gantry.bin4_);
    gantry.goToPresetLocation(gantry.start_);
  }
  if (camera_id == "12_7") {
    ROS_INFO_STREAM("SAFELY EXITING THE BIN");
    gantry.goToPresetLocation(gantry.bin7_);
    gantry.goToPresetLocation(gantry.start_);
  }
  if (camera_id == "12_8") {
    ROS_INFO_STREAM("SAFELY EXITING THE BIN");
    gantry.goToPresetLocation(gantry.bin8_);
    gantry.goToPresetLocation(gantry.start_);
  }
  if (camera_id == "14_15") {
    ROS_INFO_STREAM("SAFELY EXITING THE BIN");
    gantry.goToPresetLocation(gantry.bin15_);
    gantry.goToPresetLocation(gantry.start_);
  }
  if (camera_id == "14_16") {
    ROS_INFO_STREAM("SAFELY EXITING THE BIN");
    gantry.goToPresetLocation(gantry.bin16_);
    gantry.goToPresetLocation(gantry.start_);
  }
  if (camera_id == "14_11") {
    ROS_INFO_STREAM("SAFELY EXITING THE BIN");
    gantry.goToPresetLocation(gantry.bin11_);
    gantry.goToPresetLocation(gantry.start_);
  }
  if (camera_id == "14_12") {
    ROS_INFO_STREAM("SAFELY EXITING THE BIN");
    gantry.goToPresetLocation(gantry.bin12_);
    gantry.goToPresetLocation(gantry.start_);
  }
  if (camera_id == "13_13") {
    ROS_INFO_STREAM("SAFELY EXITING THE BIN");
    gantry.goToPresetLocation(gantry.bin13_);
    gantry.goToPresetLocation(gantry.start_);
  }
  if (camera_id == "13_14") {
    ROS_INFO_STREAM("SAFELY EXITING THE BIN");
    gantry.goToPresetLocation(gantry.bin14_);
    gantry.goToPresetLocation(gantry.start_);
  }
  if (camera_id == "13_9") {
    ROS_INFO_STREAM("SAFELY EXITING THE BIN");
    gantry.goToPresetLocation(gantry.bin9_);
    gantry.goToPresetLocation(gantry.start_);
  }
  if (camera_id == "13_10") {
    ROS_INFO_STREAM("SAFELY EXITING THE BIN");
    gantry.goToPresetLocation(gantry.bin10_);
    gantry.goToPresetLocation(gantry.start_);
  }
}

/**
 * @brief      Main function that reads the orders, plans the path, detecs obstacles and completes the order
 */
int main(int argc, char **argv) {

  ros::init(argc, argv, "final_node");
  ros::NodeHandle node;
  ros::AsyncSpinner spinner(8);
  spinner.start();

  Competition comp(node);

  //Array of Logical Camera Subscribers
  ros::Subscriber logical_camera_subscriber_[MAX_NUMBER_OF_CAMERAS];
  std::ostringstream otopic;
  std::string topic;

  for (int idx = 0; idx < MAX_NUMBER_OF_CAMERAS; idx++) {
    otopic.str("");
    otopic.clear();
    otopic << "/ariac/logical_camera_" << idx;
    topic = otopic.str();
    logical_camera_subscriber_[idx] = node.subscribe
        < nist_gear::LogicalCameraImage
        > (topic, 10, boost::bind(&Competition::logical_camera_callback, &comp,
                                  _1, idx));
  }

  comp.init();

  std::string c_state = comp.getCompetitionState();
  comp.getClock();

  GantryControl gantry(node);

  std::vector < std::string > shelf_vector;
  shelf_vector.push_back("/shelf3_frame");
  shelf_vector.push_back("/shelf4_frame");
  shelf_vector.push_back("/shelf5_frame");
  shelf_vector.push_back("/shelf6_frame");
  shelf_vector.push_back("/shelf7_frame");
  shelf_vector.push_back("/shelf8_frame");
  shelf_vector.push_back("/shelf9_frame");
  shelf_vector.push_back("/shelf10_frame");
  shelf_vector.push_back("/shelf11_frame");

  for (auto c : shelf_vector) {
    gantry.shelf_callback(c);
  }

  parts_from_camera_main = comp.get_parts_from_camera();
  master_vector_main = comp.get_master_vector();

  //checks if a human was ever detected in an aisle
  int human_1_existence;
  int human_2_existence;
  int human_3_existence;
  int human_4_existence;
  int human_exists;
  int aisle_1_choice = 0;
  int aisle_2_choice = 0;
  int aisle_3_choice = 0;
  int aisle_4_choice = 0;
  int shelf_1_gap = gantry.get_shelf_1_gap();
  int shelf_2_gap = gantry.get_shelf_2_gap();
  int shelf_3_gap = gantry.get_shelf_3_gap();

  ROS_INFO_STREAM("CHECKING FOR HUMAN IN ALL AISLES....");
  if (comp.get_human_1_existence() == 1) {
    human_1_existence = 1;
    if (shelf_1_gap == 2) {
      aisle_1_choice = 2;
    }
    ROS_INFO_STREAM("< --- HUMAN FOUND IN 1--- >");
  }
  if (comp.get_human_2_existence() == 1) {
    human_2_existence = 1;
    if (shelf_1_gap == 2 || shelf_2_gap == 2) {
      aisle_2_choice = 2;
    }
    ROS_INFO_STREAM("< --- HUMAN FOUND IN 2--- >");
  }
  if (comp.get_human_3_existence() == 1) {
    human_3_existence = 1;
    if (shelf_2_gap == 2 || shelf_3_gap == 2) {
      aisle_3_choice = 2;
    }
    ROS_INFO_STREAM("< --- HUMAN FOUND IN 3--- >");
  }
  if (comp.get_human_4_existence() == 1) {
    human_4_existence = 1;
    if (shelf_3_gap == 2) {
      aisle_4_choice = 2;
    }
    ROS_INFO_STREAM("< --- HUMAN FOUND IN 4--- >");
  }

  gantry.set_aisle_1_choice(aisle_1_choice);
  gantry.set_aisle_2_choice(aisle_2_choice);
  gantry.set_aisle_3_choice(aisle_3_choice);
  gantry.set_aisle_4_choice(aisle_4_choice);

  gantry.init();

  if (human_1_existence == 1 || human_2_existence == 1 || human_3_existence == 1
      || human_4_existence == 1) {
    human_exists = 1;
  } else {
    human_exists = 0;
    ROS_INFO_STREAM(" -x-x-x-THERE IS A NO HUMAN AT ALL!-x-x-x- ");
  }
  ROS_INFO_STREAM("CHECKING FOR HUMANS COMPLETE....");

  ROS_INFO_STREAM("CALLING THE CONVEYOR BELT....");

  pick_part_from_conveyor(comp, gantry);

  LOOP3: for (i; i < comp.get_received_order_vector().size(); i++) {
    for (int j = 0; j < comp.get_received_order_vector()[i].shipments.size();
        j++) {
      shipment_type = comp.get_received_order_vector()[i].shipments[j]
          .shipment_type;
      agv_id = comp.get_received_order_vector()[i].shipments[j].agv_id;

      k = 0;
      LOOP: while (k
          < comp.get_received_order_vector()[i].shipments[j].products.size()) {
        size_of_order = comp.get_received_order_vector()[i].shipments[j]
            .products.size();
        ROS_INFO_STREAM("SIZE OF THE ORDER" << size_of_order);
        ROS_INFO_STREAM("loop reached, part not faulty");
        ROS_INFO_STREAM("NEW part");
        ROS_INFO_STREAM(i << j << k);

        if ((master_vector_main[i][j][k].type == "pulley_part_red")
            || (master_vector_main[i][j][k].type == "pulley_part_blue")
            || (master_vector_main[i][j][k].type == "pulley_part_green")
            || (master_vector_main[i][j][k].type == "disk_part_blue")
            || (master_vector_main[i][j][k].type == "disk_part_red")
            || (master_vector_main[i][j][k].type == "disk_part_green")
            || (master_vector_main[i][j][k].type == "piston_rod_part_blue")
            || (master_vector_main[i][j][k].type == "piston_rod_part_green")
            || (master_vector_main[i][j][k].type == "piston_rod_part_red")
            || (master_vector_main[i][j][k].type == "gasket_part_blue")
            || (master_vector_main[i][j][k].type == "gasket_part_red")
            || (master_vector_main[i][j][k].type == "gasket_part_green")
            || (master_vector_main[i][j][k].type == "gasket_part_green")
            || (master_vector_main[i][j][k].type == "gasket_part_green")
            || (master_vector_main[i][j][k].type == "gasket_part_green")
            || (master_vector_main[i][j][k].type == "gear_part_green")
            || (master_vector_main[i][j][k].type == "gear_part_blue")
            || (master_vector_main[i][j][k].type == "gear_part_red")) {

          ROS_INFO_STREAM("Parts found from orders");
          ROS_INFO_STREAM(master_vector_main[i][j][k].type);
          ROS_INFO_STREAM(master_vector_main[i][j][k].delivered);
          ROS_INFO_STREAM("checking i j k" << i << j << k);

          if (master_vector_main[i][j][k].delivered == false) {
            part_placed = false;
            LOOP2: for (int l = 0; l < parts_from_camera_main.size(); l++) {
              ROS_INFO_STREAM("Loop 2 reached to replace faulty part");
              ROS_INFO_STREAM(" Camera number - " << l);
              for (int m = 0; m < parts_from_camera_main[i].size(); m++) {
                if ((master_vector_main[i][j][k].type
                    == parts_from_camera_main[l][m].type)
                    && (parts_from_camera_main[l][m].faulty == false)
                    && (parts_from_camera_main[l][m].picked == false)) {
                  ROS_INFO_STREAM("part_from_camera_index" << l << m);
                  parts_from_camera_main[l][m].picked = true;
                  ROS_INFO_STREAM(
                      "picked status " << parts_from_camera_main[l][m].picked);

                  if((parts_from_camera_main[l][m].type == "disk_part_green") && (m == 1))
                    parts_from_camera_main[l][m] = parts_from_camera_main[l][m+3];

                  ROS_INFO_STREAM("Part found in environment");
                  ROS_INFO_STREAM(parts_from_camera_main[l][m].type);

                  ROS_INFO_STREAM(
                      "Part to be picked = "
                          << master_vector_main[i][j][k].type);
                  part part_in_tray;
                  part_in_tray.type = master_vector_main[i][j][k].type;
                  part_in_tray.pose.position.x = master_vector_main[i][j][k]
                      .place_part_pose.position.x;
                  part_in_tray.pose.position.y = master_vector_main[i][j][k]
                      .place_part_pose.position.y;
                  part_in_tray.pose.position.z = master_vector_main[i][j][k]
                      .place_part_pose.position.z;
                  part_in_tray.pose.orientation.x = master_vector_main[i][j][k]
                      .place_part_pose.orientation.x;
                  part_in_tray.pose.orientation.y = master_vector_main[i][j][k]
                      .place_part_pose.orientation.y;
                  part_in_tray.pose.orientation.z = master_vector_main[i][j][k]
                      .place_part_pose.orientation.z;
                  part_in_tray.pose.orientation.w = master_vector_main[i][j][k]
                      .place_part_pose.orientation.w;
                  part_in_tray.initial_pose = parts_from_camera_main[l][m].pose;

                  ROS_INFO_STREAM("Part to be placed at = ");
                  ROS_INFO_STREAM(master_vector_main[i][j][k].place_part_pose);
                  ROS_INFO_STREAM("Part to be picked from = ");
                  ROS_INFO_STREAM(parts_from_camera_main[l][m].pose);
                  gantry.goToPresetLocation(gantry.start_);
                  ROS_INFO_STREAM("Start location reached");

                  double y_coord = parts_from_camera_main[l][m].pose.position.y;
                  bool GENERAL_BREAKBEAM_1 = false;
                  bool GENERAL_BREAKBEAM_2 = false;
                  bool GENERAL_BREAKBEAM_4 = false;
                  bool GENERAL_BREAKBEAM_5 = false;
                  std::string br_1;
                  std::string br_2;
                  std::string br_4;
                  std::string br_5;
                  ROS_INFO_STREAM("Y coord OF THE part is : " << y_coord);
                  if (y_coord > 3.15) {
                    ROS_INFO_STREAM("AISLE 1");
                    br_1 = "11";
                    br_2 = "12";
                    br_4 = "14";
                    br_5 = "15";
                  }
                  if (y_coord > 2.4 && y_coord < 3.1) {
                    ROS_INFO_STREAM("AISLE 2 - SHELF 1");
                    br_1 = "21";
                    br_2 = "22";
                    br_4 = "24";
                    br_5 = "25";
                  }
                  if (y_coord > 0 && y_coord < 0.7) {
                    ROS_INFO_STREAM("AISLE 2 - SHELF 2");
                    br_1 = "21";
                    br_2 = "22";
                    br_4 = "24";
                    br_5 = "25";
                  }
                  if (y_coord < 0 && y_coord > -0.7) {
                    ROS_INFO_STREAM("AISLE 3 -  SHELF 2");
                    br_1 = "31";
                    br_2 = "32";
                    br_4 = "34";
                    br_5 = "35";
                  }
                  if (y_coord < -2.4 && y_coord > -3.1) {
                    ROS_INFO_STREAM("AISLE 3 - SHELF 3");
                    br_1 = "31";
                    br_2 = "32";
                    br_4 = "34";
                    br_5 = "35";
                  }
                  if (y_coord < -3.15) {
                    ROS_INFO_STREAM("AISLE 4");
                    br_1 = "41";
                    br_2 = "42";
                    br_4 = "44";
                    br_5 = "45";
                  }
                  ROS_INFO_STREAM("FOUND PICK UP LOCATION");
                  int waypoint_counter = 0;
                  ROS_INFO_STREAM("BEFORE FOR LOOP");

                  std::string location = part_location(
                      parts_from_camera_main[l][m].pose, l);
                  std::string camera_id = std::to_string(l) + location;
                  ROS_INFO_STREAM("Camera  Id : " << camera_id);

                  int human_exists_in_aisle = 0;

                  if ((camera_id == "1f" || camera_id == "2f")
                      && (human_1_existence == 1)) {
                    ROS_WARN_STREAM(
                        "AISLE 1 HAS THE PART AND THE HUMAN AS WELL");
                    human_exists_in_aisle = 1;
                  }
                  if ((camera_id == "1b" || camera_id == "2b")
                      && (human_2_existence == 1)) {
                    ROS_WARN_STREAM(
                        "AISLE 2 HAS THE PART AND THE HUMAN AS WELL");
                    human_exists_in_aisle = 1;
                  }
                  if ((camera_id == "4f" || camera_id == "3f")
                      && (human_2_existence == 1)) {
                    ROS_WARN_STREAM(
                        "AISLE 2 HAS THE PART AND THE HUMAN AS WELL");
                    human_exists_in_aisle = 1;
                  }
                  if ((camera_id == "4b" || camera_id == "3b")
                      && (human_3_existence == 1)) {
                    ROS_WARN_STREAM(
                        "AISLE 3 HAS THE PART AND THE HUMAN AS WELL");
                    human_exists_in_aisle = 1;
                  }
                  if ((camera_id == "5f" || camera_id == "6f")
                      && (human_3_existence == 1)) {
                    ROS_WARN_STREAM(
                        "AISLE 3 HAS THE PART AND THE HUMAN AS WELL");
                    human_exists_in_aisle = 1;
                  }
                  if ((camera_id == "5b" || camera_id == "6b")
                      && (human_4_existence == 1)) {
                    ROS_WARN_STREAM(
                        "AISLE 4 HAS THE PART AND THE HUMAN AS WELL");
                    human_exists_in_aisle = 1;
                  }

                  auto q = gantry.pickup_locations.find(camera_id);

                  for (auto waypoint_iter : q->second) {
                    ROS_INFO_STREAM(
                        "NOW EXECUTING WAYPOINT --->" << waypoint_counter);
                    // TO FOLLOW THE HUMAN UPTO WAYPOINT 2
                    if (waypoint_counter == 1 && human_exists_in_aisle == 1) {
                      double speed_factor = 0.6;
                      double acc_factor = 0.6;
                      ROS_INFO_STREAM(
                          "CHANGING ROBOT SPEED BY :" << speed_factor);
                      ROS_INFO_STREAM(
                          "CHANGING ROBOT ACCELERATION BY :" << acc_factor);
                      gantry.setRobotSpeed(speed_factor, acc_factor);
                      ROS_INFO_STREAM("ROBOT SLOWED DOWN");

                      ROS_INFO_STREAM(
                          " --------- FOLLOW HUMAN LOOP --------- ");
                      followHuman(comp, gantry, waypoint_iter, br_1, br_2,
                                  GENERAL_BREAKBEAM_1, GENERAL_BREAKBEAM_2);
                      waypoint_counter += 1;
                    }
                    if (waypoint_counter == 4 && human_exists_in_aisle == 1) {

                      double speed_factor = 0.6;
                      double acc_factor = 0.6;
                      ROS_INFO_STREAM(
                          "CHANGING ROBOT SPEED BY :" << speed_factor);
                      ROS_INFO_STREAM(
                          "CHANGING ROBOT ACCELERATION BY :" << acc_factor);
                      gantry.setRobotSpeed(speed_factor, acc_factor);
                      ROS_INFO_STREAM("ROBOT SLOWED DOWN");

                      ROS_INFO_STREAM(
                          " --------- HUMAN AVOIDANCE LOOP --------- ");
                      checkAndProceed(comp, gantry, waypoint_iter, br_4, br_5,
                                      GENERAL_BREAKBEAM_4, GENERAL_BREAKBEAM_5);
                      waypoint_counter += 1;
                    } else {
                      double speed_factor = 2;
                      double acc_factor = 2;
                      ROS_INFO_STREAM(
                          "CHANGING ROBOT SPEED BY :" << speed_factor);
                      ROS_INFO_STREAM(
                          "CHANGING ROBOT ACCELERATION BY :" << acc_factor);
                      gantry.setRobotSpeed(speed_factor, acc_factor);
                      ROS_INFO_STREAM("ROBOT SPED UP");
                      ROS_INFO_STREAM("NOW IN ELSE LOOP");
                      ROS_INFO_STREAM("ATTEMPTING TO PICK UP ---- >");
                      gantry.goToPresetLocation(waypoint_iter);
                      waypoint_counter += 1;
                    }
                    ros::Duration timeout(0.5);
                  }

                  gantry.pickPart(parts_from_camera_main[l][m]);
                  ROS_INFO_STREAM("Part picked");

                  if (camera_id == "11_1" || camera_id == "11_2"
                      || camera_id == "11_5" || camera_id == "11_6"
                      || camera_id == "12_3" || camera_id == "12_4"
                      || camera_id == "12_7" || camera_id == "12_8"
                      || camera_id == "14_15" || camera_id == "14_16"
                      || camera_id == "14_11" || camera_id == "14_12"
                      || camera_id == "13_13" || camera_id == "13_14"
                      || camera_id == "13_9" || camera_id == "13_10") {
                    ROS_INFO_STREAM("ATTEMPTING TO SAFELY EXIT BIN, IF NEEDED");
                    ROS_INFO_STREAM(
                        "SAFELY EXITING BIN NUMBER : " << camera_id);
                    safelyexitBin(camera_id, gantry);
                  }

                  int return_waypoint_counter = 0;
                  for (auto it = q->second.rbegin(); it != q->second.rend();
                      it++) {
                    gantry.goToPresetLocation(*it);
                    if (return_waypoint_counter == 2
                        && human_exists_in_aisle == 1) {
                      ROS_INFO_STREAM("Re-checking for the human ...");
                      double speed_factor = 2;
                      double acc_factor = 2;
                      ROS_INFO_STREAM(
                          "CHANGING ROBOT SPEED BY :" << speed_factor);
                      ROS_INFO_STREAM(
                          "CHANGING ROBOT ACCELERATION BY :" << acc_factor);
                      gantry.setRobotSpeed(speed_factor, acc_factor);
                      checkAndProceedBackwards(comp, gantry, *it, br_4, br_5,
                                               GENERAL_BREAKBEAM_4,
                                               GENERAL_BREAKBEAM_5);
                    }
                    return_waypoint_counter += 1;
                  }
                  ROS_INFO_STREAM("Going to start after part is picked");
                  gantry.goToPresetLocation(gantry.start_);
                  ROS_INFO_STREAM(master_vector_main[i][j][k].agv_id);

                  if (part_in_tray.pose.orientation.x == 1) {
                    if (master_vector_main[i][j][k].agv_id == "agv1") {
                      gantry.activateGripper("right_arm");
                      ros::Duration(2).sleep();
                      ROS_INFO_STREAM("Right Gripper activated");
                      ROS_INFO_STREAM("Flipping Needed");
                      gantry.goToPresetLocation(gantry.agv1_flip_);
                      ROS_INFO_STREAM("AGV 2 location reached");
                      gantry.goToPresetLocation(gantry.pose_change_1_agv1);
                      ROS_INFO_STREAM("Flipping pose1 reached");
                      gantry.goToPresetLocation(gantry.pose_change_2_agv1);
                      ROS_INFO_STREAM("Flipping pose2 reached");
                      gantry.deactivateGripper("left_arm");
                      ros::Duration(2).sleep();
                      ROS_INFO_STREAM("Left Gripper Disabled");
                      gantry.goToPresetLocation(gantry.agv1_flip_target_);
                      ROS_INFO_STREAM("Reached AGV");
                    }
                    if (master_vector_main[i][j][k].agv_id == "agv2") {
                      gantry.activateGripper("right_arm");
                      ros::Duration(2).sleep();
                      ROS_INFO_STREAM("Right Gripper activated");
                      ROS_INFO_STREAM("Flipping Needed");
                      gantry.goToPresetLocation(gantry.agv2_flip_);
                      ROS_INFO_STREAM("AGV 2 location reached");
                      gantry.goToPresetLocation(gantry.pose_change_1_agv2);
                      ROS_INFO_STREAM("Flipping pose1 reached");
                      gantry.goToPresetLocation(gantry.pose_change_2_agv2);
                      ROS_INFO_STREAM("Flipping pose2 reached");
                      gantry.deactivateGripper("left_arm");
                      ros::Duration(2).sleep();
                      ROS_INFO_STREAM("Left Gripper Disabled");
                      gantry.goToPresetLocation(gantry.agv2_flip_target_);
                      ROS_INFO_STREAM("Reached AGV");
                    }

                    part_in_tray.pose.orientation.x = 0;
                    part_in_tray.pose.orientation.y = 0;
                    part_in_tray.pose.orientation.z = 0;
                    part_in_tray.pose.orientation.w = 1;
                    gantry.placePart_right_arm(
                        part_in_tray, master_vector_main[i][j][k].agv_id);
                    ROS_INFO_STREAM("Part placed");
                  } else {
                    gantry.placePart(part_in_tray,
                                     master_vector_main[i][j][k].agv_id);
                    ROS_INFO_STREAM("Part placed");
                    if (master_vector_main[i][j][k].agv_id == "agv2") {
                      gantry.goToPresetLocation(gantry.agv2_);
                      ROS_INFO_STREAM("AGV2 location reached");
                    } else {
                      gantry.goToPresetLocation(gantry.agv1_);
                      ROS_INFO_STREAM("AGV1 location reached");
                    }
                  }
                  if (master_vector_main[i][j][k].agv_id == "agv1") {
                    faulty_part_from_sensor = comp
                        .get_quality_sensor_status_agv1();
                    ROS_INFO_STREAM(
                        "Black sheep location in camera "
                            << faulty_part_from_sensor.type);
                    ROS_INFO_STREAM(faulty_part_from_sensor.pose);
                  } else if (master_vector_main[i][j][k].agv_id == "agv2") {
                    faulty_part_from_sensor = comp
                        .get_quality_sensor_status_agv2();
                    ROS_INFO_STREAM(
                        "Black sheep location in camera "
                            << faulty_part_from_sensor.type);
                    ROS_INFO_STREAM(faulty_part_from_sensor.pose);
                  } else {
                    ROS_ERROR_STREAM(
                        "fail to recognize agv id, when trying to read from quality control sensor");
                  }
                  ROS_INFO_STREAM("Status of faulty part = ");
                  ROS_INFO_STREAM(faulty_part_from_sensor.faulty);
                  if (faulty_part_from_sensor.faulty == true) {
                    part faulty_part;
                    faulty_part.pose = gantry.getTargetWorldPose_dummy(
                        faulty_part_from_sensor.pose,
                        master_vector_main[i][j][k].agv_id);
                    ROS_INFO_STREAM("Black sheep location");
                    ROS_INFO_STREAM(faulty_part.pose);
                    faulty_part.type = parts_from_camera_main[l][m].type;
                    faulty_part.pose.position.x = faulty_part.pose.position.x;
                    faulty_part.pose.position.y = faulty_part.pose.position.y;
                    faulty_part.pose.position.z = faulty_part.pose.position.z
                        + get_offset_to_pickup_part_on_tray(faulty_part.type);
                    faulty_part.pose.orientation.x = faulty_part.pose
                        .orientation.x;
                    faulty_part.pose.orientation.y = faulty_part.pose
                        .orientation.y;
                    faulty_part.pose.orientation.z = faulty_part.pose
                        .orientation.z;
                    faulty_part.pose.orientation.w = faulty_part.pose
                        .orientation.w;

                    gantry.pickPart(faulty_part);

//                  geometry_msgs::Pose pose_above_part = faulty_part.pose;
//                  pose_above_part.position.z = pose_above_part.position.z + 0.2;
//                  gantry.reachOut(pose_above_part);   // reach out above part first
//                  gantry.pickPart(faulty_part);   // pick up part
//                  gantry.reachOut(pose_above_part);   // reach out above part first

                    if (master_vector_main[i][j][k].agv_id == "agv1") {
                      gantry.goToPresetLocation(gantry.agv1_drop_);
                    } else if (master_vector_main[i][j][k].agv_id == "agv2") {
                      gantry.goToPresetLocation(gantry.agv2_drop_);
                    } else {
                      ROS_ERROR_STREAM(
                          "fail to recognize agv id, when trying to read from quality control sensor");
                    }
                    gantry.deactivateGripper("left_arm");
                    ROS_INFO_STREAM("Go to Loop2 triggered");
                    goto LOOP2;
                  } else {
                    ROS_INFO_STREAM("check pose after placing");
//                    fix_part_pose(comp, master_vector_main[i][j][k], gantry, part_in_tray);
                    ROS_INFO_STREAM(
                        "Coming to start location to check for new orders");
                    gantry.goToPresetLocation(gantry.start_);

                    ROS_INFO_STREAM("Checking if vector size increased");
                    ROS_INFO_STREAM("Go to Loop Triggered");
                    master_vector_main[i][j][k].delivered = true;
                    parts_delivered[i]++;
                    comp.setter_delivered(i, j, k);
                    ROS_INFO_STREAM(
                        "Checking delivered status"
                            << master_vector_main[i][j][k].delivered);
                    ROS_INFO_STREAM(" i j k" << i << j << k);
                    ROS_INFO_STREAM(
                        parts_from_camera_main[l][m].type
                            << "  successfully delivered");
                    if (comp.get_received_order_vector().size() > i + 1) {
                      ROS_INFO_STREAM("NEW ORDER RECEIVED");
                      master_vector_main = comp.get_master_vector();
                      ROS_INFO_STREAM(
                          " after getting new master vector, i j k" << i << j
                              << k);
                      ROS_INFO_STREAM(
                          "Checking if delivered status changed for part = "
                              << master_vector_main[i][j][k].type
                              << master_vector_main[i][j][k].delivered);
                      i++;
                      goto LOOP3;
                    }
                    k++;
                    goto LOOP;
                  }
                }
              }
            }
            ROS_INFO_STREAM("Second for loop ended");
          }
        }
        k++;
      }
    }
    ROS_INFO_STREAM("No of parts delivered = " << parts_delivered[i]);
    ROS_INFO_STREAM("Order Size = " << size_of_order);
    if (parts_delivered[i] == size_of_order) {
      ROS_INFO_STREAM(" Order " << i << "completed successfully");
      comp.delete_completed_order(i);
      ROS_INFO_STREAM(" Order " << i << "deleted successfully");
      ROS_INFO_STREAM(
          "Size of vector now = " << comp.get_received_order_vector().size());

      ROS_INFO_STREAM(
          "Delivering Shipment type = " << shipment_type << "  in agv = "
              << agv_id);
      submitOrder(agv_id, shipment_type);
    }

  }

  ROS_INFO_STREAM("FOR LOOP TERMINATED");

  gantry.goToPresetLocation(gantry.start_);
  ros::Duration timeout(5.0);
  if ((i > 1) && (temp != i - 2)) {
    i = i - 2;
    k = 0;
    ROS_INFO_STREAM("Executing Order = " << i - 1);
    ROS_INFO_STREAM("Value of I = " << i);
    temp = i;
    goto LOOP3;
  }

  comp.endCompetition();
  spinner.stop();
  ros::shutdown();
  return 0;
}
