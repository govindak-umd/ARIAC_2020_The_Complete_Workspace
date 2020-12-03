/**
 * @file gantry_control.cpp
 * @author Pradeep Gopal, Govind Ajith Kumar, Rajesh NS, Cheng, Dakota Abernathy
 * @copyright MIT License
 * @brief Implementing the gantry control class
 * This class takes care all the robot related operations in the environment
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

#include "gantry_control.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_listener.h> //for shelves gap
#include <tf/LinearMath/Vector3.h>
#include "competition.h"

//shelf vector
std::vector<std::vector<double>> shelf_vector(9, std::vector<double>(3));

/**
 * @brief Function which converts RPY to Quaternion
 * @param roll
 * @param pitch
 * @param yaw
 * @return Quaternion
 */
Quat GantryControl::ToQuaternion(double roll, double pitch, double yaw)  // yaw (Z), pitch (Y), roll (X)
                                 {
  // Abbreviations for the various angular functions
  double cy = cos(yaw * 0.5);
  double sy = sin(yaw * 0.5);
  double cp = cos(pitch * 0.5);
  double sp = sin(pitch * 0.5);
  double cr = cos(roll * 0.5);
  double sr = sin(roll * 0.5);

  Quat q;
  q.w = cr * cp * cy + sr * sp * sy;
  q.x = sr * cp * cy - cr * sp * sy;
  q.y = cr * sp * cy + sr * cp * sy;
  q.z = cr * cp * sy - sr * sp * cy;

  return q;
}

/**
 * @brief method to reachout for faulty part in tray
 * @param pose_end_effector
 */
void GantryControl::reachOut(geometry_msgs::Pose pose_end_effector) {
    left_arm_group_.setPoseTarget(pose_end_effector);
    left_arm_group_.move();
}

/**
 * @brief Constructor for the class
 * @param ROS node
 */
GantryControl::GantryControl(ros::NodeHandle &node)
    :
    node_("/ariac/gantry"),
    planning_group_("/ariac/gantry/robot_description"),
    full_robot_options_("Full_Robot", planning_group_, node_),
    left_arm_options_("Left_Arm", planning_group_, node_),
    right_arm_options_("Right_Arm", planning_group_, node_),
    left_ee_link_options_("Left_Endeffector", planning_group_, node_),
    right_ee_link_options_("Right_Endeffector", planning_group_, node_),
    full_robot_group_(full_robot_options_),
    left_arm_group_(left_arm_options_),
    right_arm_group_(right_arm_options_),
    left_ee_link_group_(left_ee_link_options_),
    right_ee_link_group_(right_ee_link_options_) {

  ROS_INFO_STREAM("[GantryControl::GantryControl] constructor called... ");
}

/**
 * @brief callback for the shelf subscriber
 * @param string
 * @return None
 */
void GantryControl::shelf_callback(std::string shelf_name) {
  tf::TransformListener listener;
  ros::Rate rate(10.0);
  while (node_.ok()) {
    tf::StampedTransform transform;
    try {
      listener.lookupTransform("/world", shelf_name, ros::Time(0), transform);
      tf::Transform tf(transform.getBasis(), transform.getOrigin());
      tf::Vector3 tfVec;
      tf::Matrix3x3 tfR;
      tf::Quaternion quat;
      tfVec = tf.getOrigin();
      if (shelf_name == "/shelf3_frame") {
        shelf_vector[0][0] = double(tfVec.getX());
        shelf_vector[0][1] = double(tfVec.getY());
        shelf_vector[0][2] = double(tfVec.getZ());
      }
      if (shelf_name == "/shelf4_frame") {
        shelf_vector[1][0] = double(tfVec.getX());
        shelf_4_x = shelf_vector[1][0];
        shelf_vector[1][1] = double(tfVec.getY());
        shelf_vector[1][2] = double(tfVec.getZ());
      }
      if (shelf_name == "/shelf5_frame") {
        shelf_vector[2][0] = double(tfVec.getX());
        shelf_vector[2][1] = double(tfVec.getY());
        shelf_vector[2][2] = double(tfVec.getZ());
      }
      if (shelf_name == "/shelf6_frame") {
        shelf_vector[3][0] = double(tfVec.getX());
        shelf_vector[3][1] = double(tfVec.getY());
        shelf_vector[3][2] = double(tfVec.getZ());
      }
      if (shelf_name == "/shelf7_frame") {
        shelf_vector[4][0] = double(tfVec.getX());
        shelf_7_x = shelf_vector[4][0];
        shelf_vector[4][1] = double(tfVec.getY());
        shelf_vector[4][2] = double(tfVec.getZ());
      }
      if (shelf_name == "/shelf8_frame") {
        shelf_vector[5][0] = double(tfVec.getX());
        shelf_vector[5][1] = double(tfVec.getY());
        shelf_vector[5][2] = double(tfVec.getZ());
      }
      if (shelf_name == "/shelf9_frame") {
        shelf_vector[6][0] = double(tfVec.getX());
        shelf_vector[6][1] = double(tfVec.getY());
        shelf_vector[6][2] = double(tfVec.getZ());
      }
      if (shelf_name == "/shelf10_frame") {
        shelf_vector[7][0] = double(tfVec.getX());
        shelf_10_x = shelf_vector[7][0];
        shelf_vector[7][1] = double(tfVec.getY());
        shelf_vector[7][2] = double(tfVec.getZ());
      }
      if (shelf_name == "/shelf11_frame") {
        shelf_vector[8][0] = double(tfVec.getX());
        shelf_vector[8][1] = double(tfVec.getY());
        shelf_vector[8][2] = double(tfVec.getZ());
      }

      for (int i = 0; i <= 7; i++) {
        if (5 <= (abs(shelf_vector[i][0] - shelf_vector[i + 1][0]))
            and (abs(shelf_vector[i][0] - shelf_vector[i + 1][0])) <= 7) {
//                    ROS_INFO_STREAM("Gaps between shelves "<<i+3<<" and "<<i+4<<" "<<abs(shelf_vector[i][0] - shelf_vector[i+1][0]));
          if (i + 3 == 3) {
            ROS_WARN_STREAM("GAP AISLE : 1, GAP : 1");
            shelf_1_gap = 1;
          }
          if (i + 3 == 4) {
            ROS_WARN_STREAM("GAP AISLE : 1, GAP : 2");
            shelf_1_gap = 2;
          }
          if (i + 3 == 6) {
            ROS_WARN_STREAM("GAP AISLE : 2, GAP : 1");
            shelf_2_gap = 1;
          }
          if (i + 3 == 7) {
            ROS_WARN_STREAM("GAP AISLE : 2, GAP : 2");
            shelf_2_gap = 2;
          }
          if (i + 3 == 9) {
            ROS_WARN_STREAM("GAP AISLE : 3, GAP : 1");
            shelf_3_gap = 1;
          }
          if (i + 3 == 10) {
            ROS_WARN_STREAM("GAP AISLE : 3, GAP : 2");
            shelf_3_gap = 2;
          }
        }
      }
      break;
    } catch (tf::TransformException ex) {
      ROS_ERROR("%s", ex.what());
      ros::Duration(1.0).sleep();
    }
  }
}

/**
 * @brief method which returns the shelf vector containing the shelf gap information
 * @return vector containing the shelf gap information
 */
std::vector<std::vector<double>> GantryControl::get_shelf_vector() {

  return shelf_vector;
}

/**
 * @brief Sets the Robot movement speed
 * @param speed_factor
 * @param acc_factor
 * @return None
 */
void GantryControl::setRobotSpeed(double speed_factor, double acc_factor) {
  full_robot_group_.setMaxVelocityScalingFactor(speed_factor);
  full_robot_group_.setMaxAccelerationScalingFactor(acc_factor);
  ROS_INFO_STREAM("<<<<<<<<<<<MOVE VELOCITY CHANGED>>>>>>>>>>>>>>");
}

/**
 * @brief method to set the aisle 1 as the choise
 * @param integer to choose the corresponding aisle
 * @return None
 * */
void GantryControl::set_aisle_1_choice(int new_choice) {
  aisle_1_choice = new_choice;
}

/**
 * @brief method to set the aisle 2 as the choise
 * @param integer to choose the corresponding aisle
 * @return None
 * */
void GantryControl::set_aisle_2_choice(int new_choice) {
  aisle_2_choice = new_choice;
}

/**
 * @brief method to set the aisle 3 as the choise
 * @param integer to choose the corresponding aisle
 * @return None
 * */
void GantryControl::set_aisle_3_choice(int new_choice) {
  aisle_3_choice = new_choice;
}

/**
 * @brief method to set the aisle 4 as the choise
 * @param integer to choose the corresponding aisle
 * @return None
 * */
void GantryControl::set_aisle_4_choice(int new_choice) {
  aisle_4_choice = new_choice;
}

/**
 * @brief method which returns the gap information of shelf 1
 * @return shelf 1 gap
 * */
int GantryControl::get_shelf_1_gap() {
  return shelf_1_gap;
}

/**
 * @brief method which returns the gap information of shelf 2
 * @return shelf 2 gap
 * */
int GantryControl::get_shelf_2_gap() {
  return shelf_2_gap;
}

/**
 * @brief method which returns the gap information of shelf 3
 * @return shelf 3 gap
 * */
int GantryControl::get_shelf_3_gap() {
  return shelf_3_gap;
}

/**
 * @brief Initializes all the gantry related subscribers and callbacks
 * @return None
 */
void GantryControl::init() {
  double time_called = ros::Time::now().toSec();
  full_robot_group_.setMaxVelocityScalingFactor(0.6);
  full_robot_group_.setMaxAccelerationScalingFactor(0.1);

  ROS_INFO_STREAM("EACH AISLE HAS MADE ITS CHOICE : ");
  ROS_INFO_STREAM("aisle_1_choice : " << aisle_1_choice);
  ROS_INFO_STREAM("aisle_2_choice : " << aisle_2_choice);
  ROS_INFO_STREAM("aisle_3_choice : " << aisle_3_choice);
  ROS_INFO_STREAM("aisle_4_choice : " << aisle_4_choice);

  ROS_INFO_STREAM("<<<<<<<<<<<<<< ALL CHOICES DONE!! >>>>>>>>>>>>>>>>>>");
  ROS_INFO_STREAM("MOVE VELOCITY CHANGED");
  ROS_INFO_NAMED("init", "Planning frame: %s",
                 left_arm_group_.getPlanningFrame().c_str());
  ROS_INFO_NAMED("init", "Planning frame: %s",
                 right_arm_group_.getPlanningFrame().c_str());
  ROS_INFO_NAMED("init", "Planning frame: %s",
                 full_robot_group_.getPlanningFrame().c_str());
  ROS_INFO_NAMED("init", "Planning frame: %s",
                 right_ee_link_group_.getPlanningFrame().c_str());
  ROS_INFO_NAMED("init", "Planning frame: %s",
                 left_ee_link_group_.getPlanningFrame().c_str());

  ROS_INFO_NAMED("init", "End effector link: %s",
                 left_arm_group_.getEndEffectorLink().c_str());
  ROS_INFO_NAMED("init", "End effector link: %s",
                 right_arm_group_.getEndEffectorLink().c_str());

  left_arm_group_.setPoseReferenceFrame("world");

  // location where gantry hover to pick up part at belt

  bin1_w1.gantry = { 2.75, -0.77, PI / 2 };
  bin1_w1.left_arm = { 0.0, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };
  bin1_w1.right_arm = { PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };

  bin1_w2.gantry = { 0, 0, 0 };
  bin1_w2.left_arm = { 0.0, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };
  bin1_w2.right_arm = { PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };

  //GAP 1 AISLE 1
  gap_1_1.gantry = { -11.4, -5.1, 0.0 };
  gap_1_1.left_arm = { -PI / 2, -PI / 4, PI / 2, -PI / 4, -0.2, 0 };
  gap_1_1.right_arm = { PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };

  //GAP 1 AISLE 2
  gap_1_2.gantry = { -11.4, -1.1, 3.14 };
  gap_1_2.left_arm = { -PI / 2, -PI / 4, PI / 2, -PI / 4, -0.2, 0 };
  gap_1_2.right_arm = { PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };

  //GAP 2 AISLE 2

  gap_2_2.gantry = { -11.4, -2.03, 0.0 };
  gap_2_2.left_arm = { -PI / 2, -PI / 4, PI / 2, -PI / 4, -0.2, 0 };
  gap_2_2.right_arm = { PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };

  //GAP 2 AISLE 3

  gap_2_3.gantry = { -11.4, 2.03, 3.14 };
  gap_2_3.left_arm = { -PI / 2, -PI / 4, PI / 2, -PI / 4, -0.2, 0 };
  gap_2_3.right_arm = { PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };

  //GAP 3 AISLE 3

  gap_3_3.gantry = { -11.4, 1.0, 0.0 };
  gap_3_3.left_arm = { -PI / 2, -PI / 4, 2.2, -PI / 4, -0.2, 0 };
  gap_3_3.right_arm = { PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };

  //GAP 3 AISLE 4

  gap_3_4.gantry = { -11.4, 4.9, 3.14 };
  gap_3_4.left_arm = { -PI / 2, -PI / 4, PI / 2, -PI / 4, -0.2, 0 };
  gap_3_4.right_arm = { PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };

  //GAP 1

  GAP_1.gantry = { -11.4, -2.99, 3.45 };
  GAP_1.left_arm = { 0.0, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };
  GAP_1.right_arm = { PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };

  //GAP 2

  GAP_2.gantry = { -11.4, 0, 0.73 };
  GAP_2.left_arm = { 0.0, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };
  GAP_2.right_arm = { PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };

  //GAP 3

  GAP_3.gantry = { -11.4, 2.99, 0.73 };
  GAP_3.left_arm = { 0.0, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };
  GAP_3.right_arm = { PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };

  //----NEW WAYPOINTS

  //Shelf 1, left front, Camera 7
  shelf1_lf_w1.gantry = { 0, -5.1, 0 };
  shelf1_lf_w1.left_arm = { 0.0, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };
  shelf1_lf_w1.right_arm = { PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };

  shelf1_lf_w2.gantry = { 4.5, -5.1, 0 };
  shelf1_lf_w2.left_arm = { 0.0, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };
  shelf1_lf_w2.right_arm = { PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };

  shelf1_lf_w3.gantry = { 4.5, -5.1, 0 };
  shelf1_lf_w3.left_arm = { -PI / 2, -PI / 4, PI / 2, -PI / 4, -0.2, 0 };
  shelf1_lf_w3.right_arm = { PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };

  shelf1_lf_w4.gantry = { 4.5, -5.0, 0 };
  shelf1_lf_w4.left_arm = { -1.78, -PI / 4, PI / 2, -PI / 4, -0.2, 0 };
  shelf1_lf_w4.right_arm = { PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };

  //Shelf 1, left back, Camera 7
  shelf1_lb_w1.gantry = { 0, -2.15, 3.14 };
  shelf1_lb_w1.left_arm = { 0.0, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };
  shelf1_lb_w1.right_arm = { PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };

  shelf1_lb_w2.gantry = { 5.7, -2.15, 3.14 };
  shelf1_lb_w2.left_arm = { 0.0, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };
  shelf1_lb_w2.right_arm = { PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };

  shelf1_lb_w3.gantry = { 5.7, -2.15, 3.14 };
  shelf1_lb_w3.left_arm = { -PI / 2, -PI / 4, PI / 2, -PI / 4, -0.2, 0 };
  shelf1_lb_w3.right_arm = { PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };

  shelf1_lb_w4.gantry = { 5.7, -2.35, 3.14 };
  shelf1_lb_w4.left_arm = { -1.78, -PI / 4, PI / 2, -PI / 4, -0.2, 0 };
  shelf1_lb_w4.right_arm = { PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };

  //Shelf 1, left front, Camera 10
  shelf1_rf_w1.gantry = { 0, -5.1, 0 };
  shelf1_rf_w1.left_arm = { 0.0, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };
  shelf1_rf_w1.right_arm = { PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };

  shelf1_rf_w2.gantry = { 2.6, -5.1, 0 };
  shelf1_rf_w2.left_arm = { 0.0, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };
  shelf1_rf_w2.right_arm = { PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };

  shelf1_rf_w3.gantry = { 2.6, -5.1, 0 };
  shelf1_rf_w3.left_arm = { -PI / 2, -PI / 4, PI / 2, -PI / 4, -0.2, 0 };
  shelf1_rf_w3.right_arm = { PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };

  shelf1_rf_w4.gantry = { 2.6, -5.0, 0 };
  shelf1_rf_w4.left_arm = { -1.78, -PI / 4, PI / 2, -PI / 4, -0.2, 0 };
  shelf1_rf_w4.right_arm = { PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };

  //Shelf 1, left back, Camera 10
  shelf1_rb_w1.gantry = { 0, -2.15, 3.14 };
  shelf1_rb_w1.left_arm = { 0.0, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };
  shelf1_rb_w1.right_arm = { PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };

  shelf1_rb_w2.gantry = { 3.6, -2.15, 3.14 };
  shelf1_rb_w2.left_arm = { 0.0, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };
  shelf1_rb_w2.right_arm = { PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };

  shelf1_rb_w3.gantry = { 3.6, -2.15, 3.14 };
  shelf1_rb_w3.left_arm = { -PI / 2, -PI / 4, PI / 2, -PI / 4, -0.2, 0 };
  shelf1_rb_w3.right_arm = { PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };

  shelf1_rb_w4.gantry = { 3.6, -2.35, 3.14 };
  shelf1_rb_w4.left_arm = { -1.78, -PI / 4, PI / 2, -PI / 4, -0.2, 0 };
  shelf1_rb_w4.right_arm = { PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };

  //Shelf 2, left front, Camera 9
  shelf2_lf_w1.gantry = { 0, 2.15, 0 };
  shelf2_lf_w1.left_arm = { 0.0, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };
  shelf2_lf_w1.right_arm = { PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };

  shelf2_lf_w2.gantry = { 4.5, 2.15, 0 };
  shelf2_lf_w2.left_arm = { 0.0, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };
  shelf2_lf_w2.right_arm = { PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };

  shelf2_lf_w3.gantry = { 4.5, 2.15, 0 };
  shelf2_lf_w3.left_arm = { -PI / 2, -PI / 4, PI / 2, -PI / 4, -0.2, 0 };
  shelf2_lf_w3.right_arm = { PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };

  shelf2_lf_w4.gantry = { 4.5, 2.35, 0 };
  shelf2_lf_w4.left_arm = { -1.78, -PI / 4, PI / 2, -PI / 4, -0.2, 0 };
  shelf2_lf_w4.right_arm = { PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };

  //Shelf 2, left back, Camera 9
  shelf2_lb_w1.gantry = { 0, 5.1, 3.14 };
  shelf2_lb_w1.left_arm = { 0.0, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };
  shelf2_lb_w1.right_arm = { PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };

  shelf2_lb_w2.gantry = { 5.7, 5.1, 3.14 };
  shelf2_lb_w2.left_arm = { 0.0, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };
  shelf2_lb_w2.right_arm = { PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };

  shelf2_lb_w3.gantry = { 5.7, 5.1, 3.14 };
  shelf2_lb_w3.left_arm = { -PI / 2, -PI / 4, PI / 2, -PI / 4, -0.2, 0 };
  shelf2_lb_w3.right_arm = { PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };

  shelf2_lb_w4.gantry = { 5.7, 5, 3.14 };
  shelf2_lb_w4.left_arm = { -1.78, -PI / 4, PI / 2, -PI / 4, -0.2, 0 };
  shelf2_lb_w4.right_arm = { PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };

  //Shelf 2, left front, Camera 8
  shelf2_rf_w1.gantry = { 0, 2.15, 0 };
  shelf2_rf_w1.left_arm = { 0.0, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };
  shelf2_rf_w1.right_arm = { PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };

  shelf2_rf_w2.gantry = { 2.6, 2.15, 0 };
  shelf2_rf_w2.left_arm = { 0.0, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };
  shelf2_rf_w2.right_arm = { PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };

  shelf2_rf_w3.gantry = { 2.6, 2.15, 0 };
  shelf2_rf_w3.left_arm = { -PI / 2, -PI / 4, PI / 2, -PI / 4, -0.2, 0 };
  shelf2_rf_w3.right_arm = { PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };

  shelf2_rf_w4.gantry = { 2.6, 2.35, 0 };
  shelf2_rf_w4.left_arm = { -1.78, -PI / 4, PI / 2, -PI / 4, -0.2, 0 };
  shelf2_rf_w4.right_arm = { PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };

  //Shelf 2, left back, Camera 8
  shelf2_rb_w1.gantry = { 0, 5.1, 3.14 };
  shelf2_rb_w1.left_arm = { 0.0, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };
  shelf2_rb_w1.right_arm = { PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };

  shelf2_rb_w2.gantry = { 3.6, 5.1, 3.14 };
  shelf2_rb_w2.left_arm = { 0.0, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };
  shelf2_rb_w2.right_arm = { PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };

  shelf2_rb_w3.gantry = { 3.6, 5.1, 3.14 };
  shelf2_rb_w3.left_arm = { -PI / 2, -PI / 4, PI / 2, -PI / 4, -0.2, 0 };
  shelf2_rb_w3.right_arm = { PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };

  shelf2_rb_w4.gantry = { 3.6, 5.0, 3.14 };
  shelf2_rb_w4.left_arm = { -1.78, -PI / 4, PI / 2, -PI / 4, -0.2, 0 };
  shelf2_rb_w4.right_arm = { PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };

  //Shelf 8, Left Front, Camera 4

  shelf8_lf_w1.gantry = { 0.4, -1.68, 0.0 };
  shelf8_lf_w1.left_arm = { -PI / 2, -PI / 4, PI / 2, -PI / 4, -0.2, 0 };
  shelf8_lf_w1.right_arm = { PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };

  shelf8_lf_w2.gantry = { -13.5, -1.6, 0.0 };
  shelf8_lf_w2.left_arm = { -PI / 2, -PI / 4, PI / 2, -PI / 4, -0.2, 0 };
  shelf8_lf_w2.right_arm = { PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };

  shelf8_lf_w3.gantry = { -14, -1.3, 0.0 };
  shelf8_lf_w3.left_arm = { -1.78, -PI / 4, PI / 2, -PI / 4, -0.2, 0 };
  shelf8_lf_w3.right_arm = { PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };

//    //Shelf 8, Right Front, Camera 3
  shelf8_rf_w1.gantry = { 0.4, -1.68, 0.0 };
  shelf8_rf_w1.left_arm = { -PI / 2, -PI / 4, PI / 2, -PI / 4, -0.2, 0 };
  shelf8_rf_w1.right_arm = { PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };

  shelf8_rf_w2.gantry = { -15.4, -1.2, 0.0 };
  shelf8_rf_w2.left_arm = { -PI / 2, -PI / 4, PI / 2, -PI / 4, -0.2, 0 };
  shelf8_rf_w2.right_arm = { PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };

  shelf8_rf_w3.gantry = { -15.4, -1.2, 0.0 };
  shelf8_rf_w3.left_arm = { -1.78, -PI / 4, PI / 2, -PI / 4, -0.2, 0 };
  shelf8_rf_w3.right_arm = { PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };

//    //Shelf 8, Right Back, Camera 3

  shelf8_rb_w1.gantry = { 0.4, 1.68, 3.14 };
  shelf8_rb_w1.left_arm = { -PI / 2, -PI / 4, PI / 2, -PI / 4, -0.2, 0 };
  shelf8_rb_w1.right_arm = { PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };

  shelf8_rb_w2.gantry = { -13.5, 1.6, 3.14 };
  shelf8_rb_w2.left_arm = { -PI / 2, -PI / 4, PI / 2, -PI / 4, -0.2, 0 };
  shelf8_rb_w2.right_arm = { PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };

  shelf8_rb_w3.gantry = { -14.7, 1.2, 3.14 };
  shelf8_rb_w3.left_arm = { -1.78, -PI / 4, PI / 2, -PI / 4, -0.2, 0 };
  shelf8_rb_w3.right_arm = { PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };

//    //Shelf 8, Left Back, Camera 4

  shelf8_lb_w1.gantry = { 0.4, 1.68, 3.14 };
  shelf8_lb_w1.left_arm = { -PI / 2, -PI / 4, PI / 2, -PI / 4, -0.2, 0 };
  shelf8_lb_w1.right_arm = { PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };

  shelf8_lb_w2.gantry = { -13.0, 1.6, 3.14 };
  shelf8_lb_w2.left_arm = { -PI / 2, -PI / 4, PI / 2, -PI / 4, -0.2, 0 };
  shelf8_lb_w2.right_arm = { PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };

  shelf8_lb_w3.gantry = { -13.5, 1.2, 3.14 };
  shelf8_lb_w3.left_arm = { -1.78, -PI / 4, PI / 2, -PI / 4, -0.2, 0 };
  shelf8_lb_w3.right_arm = { PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };

  //Shelf 5, Left Front, Camera 2
  shelf5_lf_w1.gantry = { 0.4, -4.48, 0.0 };
  shelf5_lf_w1.left_arm = { 0.0, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };
  shelf5_lf_w1.right_arm = { PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };

  shelf5_lf_w2.gantry = { -13.5, -4.6973, 0.0 };
  shelf5_lf_w2.left_arm = { -PI / 2, -PI / 4, PI / 2, -PI / 4, -0.2, 0 };
  shelf5_lf_w2.right_arm = { PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };

  shelf5_lf_w3.gantry = { -14.7, -4.2973, 0.0 };
  shelf5_lf_w3.left_arm = { -1.78, -PI / 4, PI / 2, -PI / 4, -0.2, 0 };
  shelf5_lf_w3.right_arm = { PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };

  //Shelf 5, Right Front, Camera 1
  shelf5_rf_w1.gantry = { 0.4, -4.48, 0.0 };
  shelf5_rf_w1.left_arm = { 0.0, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };
  shelf5_rf_w1.right_arm = { PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };

  shelf5_rf_w2.gantry = { -15.4, -4.6973, 0.0 };
  shelf5_rf_w2.left_arm = { -PI / 2, -PI / 4, PI / 2, -PI / 4, -0.2, 0 };
  shelf5_rf_w2.right_arm = { PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };

  shelf5_rf_w3.gantry = { -15.4, -4.3973, 0.0 };
  shelf5_rf_w3.left_arm = { -1.78, -PI / 4, PI / 2, -PI / 4, -0.2, 0 };
  shelf5_rf_w3.right_arm = { PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };

  //Shelf 5, Right Back, Camera 1
  shelf5_rb_w1.gantry = { 0.4, -1.3827, 3.14 };
  shelf5_rb_w1.left_arm = { 0.0, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };
  shelf5_rb_w1.right_arm = { PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };

  shelf5_rb_w2.gantry = { -14.7, -1.4973, 3.14 };
  shelf5_rb_w2.left_arm = { -PI / 2, -PI / 4, PI / 2, -PI / 4, -0.2, 0 };
  shelf5_rb_w2.right_arm = { PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };

  shelf5_rb_w3.gantry = { -14.7, -1.8, 3.14 };
  shelf5_rb_w3.left_arm = { -1.78, -PI / 4, PI / 2, -PI / 4, -0.2, 0 };
  shelf5_rb_w3.right_arm = { PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };

  //Shelf 5, Left Back, Camera 2
  shelf5_lb_w1.gantry = { 0.4, -1.3827, 3.14 };
  shelf5_lb_w1.left_arm = { 0.0, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };
  shelf5_lb_w1.right_arm = { PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };

  shelf5_lb_w2.gantry = { -12.8, -1.4973, 3.14 };
  shelf5_lb_w2.left_arm = { -PI / 2, -PI / 4, PI / 2, -PI / 4, -0.2, 0 };
  shelf5_lb_w2.right_arm = { PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };

  shelf5_lb_w3.gantry = { -12.8, -1.8, 3.14 };
  shelf5_lb_w3.left_arm = { -1.78, -PI / 4, PI / 2, -PI / 4, -0.2, 0 };
  shelf5_lb_w3.right_arm = { PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };

  //Shelf 11, Left Front, Camera 6
  shelf11_lf_w1.gantry = { 0.4, 1.628, 0.0 };
  shelf11_lf_w1.left_arm = { 0.0, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };
  shelf11_lf_w1.right_arm = { PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };

  shelf11_lf_w2.gantry = { -13.5, 1.41, 0.0 };
  shelf11_lf_w2.left_arm = { -PI / 2, -PI / 4, PI / 2, -PI / 4, -0.2, 0 };
  shelf11_lf_w2.right_arm = { PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };

  shelf11_lf_w3.gantry = { -14, 1.81, 0.0 };
  shelf11_lf_w3.left_arm = { -1.78, -PI / 4, PI / 2, -PI / 4, -0.2, 0 };
  shelf11_lf_w3.right_arm = { PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };

  //Shelf 11, Right Front, Camera 5
  shelf11_rf_w1.gantry = { 0.4, 1.628, 0.0 };
  shelf11_rf_w1.left_arm = { 0.0, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };
  shelf11_rf_w1.right_arm = { PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };

  shelf11_rf_w2.gantry = { -15.4, 1.41, 0.0 };
  shelf11_rf_w2.left_arm = { -PI / 2, -PI / 4, PI / 2, -PI / 4, -0.2, 0 };
  shelf11_rf_w2.right_arm = { PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };

  shelf11_rf_w3.gantry = { -15.4, 1.81, 0.0 };
  shelf11_rf_w3.left_arm = { -1.78, -PI / 4, PI / 2, -PI / 4, -0.2, 0 };
  shelf11_rf_w3.right_arm = { PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };

  //Shelf 11, Right Back, Camera 5
  shelf11_rb_w1.gantry = { 0.4, 4.7253, 3.14 };
  shelf11_rb_w1.left_arm = { 0.0, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };
  shelf11_rb_w1.right_arm = { PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };

  shelf11_rb_w2.gantry = { -14.7, 4.61, 3.14 };
  shelf11_rb_w2.left_arm = { -PI / 2, -PI / 4, PI / 2, -PI / 4, -0.2, 0 };
  shelf11_rb_w2.right_arm = { PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };

  shelf11_rb_w3.gantry = { -14.7, 4.31, 3.14 };
  shelf11_rb_w3.left_arm = { -1.78, -PI / 4, PI / 2, -PI / 4, -0.2, 0 };
  shelf11_rb_w3.right_arm = { PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };

  //Shelf 11, Left Back, Camera 6
  shelf11_lb_w1.gantry = { 0.4, 4.7253, 3.14 };
  shelf11_lb_w1.left_arm = { 0.0, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };
  shelf11_lb_w1.right_arm = { PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };

  shelf11_lb_w2.gantry = { -12.8, 4.61, 3.14 };
  shelf11_lb_w2.left_arm = { -PI / 2, -PI / 4, PI / 2, -PI / 4, -0.2, 0 };
  shelf11_lb_w2.right_arm = { PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };

  shelf11_lb_w3.gantry = { -12.8, 4.31, 3.14 };
  shelf11_lb_w3.left_arm = { -1.78, -PI / 4, PI / 2, -PI / 4, -0.2, 0 };
  shelf11_lb_w3.right_arm = { PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };

  //Bin 1, camera 11
  bin1_.gantry = { 2.75, -0.77, PI / 2 };
  bin1_.left_arm = { 0.0, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };
  bin1_.right_arm = { PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };

  //Bin 2, camera 11
  bin2_.gantry = { 3.65, -0.77, PI / 2 };
  bin2_.left_arm = { 0.0, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };
  bin2_.right_arm = { PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };

  //Bin 5, camera 11
  bin5_.gantry = { 2.15, -2.15, 0 };
  bin5_.left_arm = { 0.0, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };
  bin5_.right_arm = { PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };

  //Bin 6, camera 11
  bin6_.gantry = { 3.95, -1.67, PI / 2 };
  bin6_.left_arm = { 0.0, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };
  bin6_.right_arm = { PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };

  //Bin 3, camera 12
  bin3_.gantry = { 4.55, -0.77, PI / 2 };
  bin3_.left_arm = { 0.0, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };
  bin3_.right_arm = { PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };

  //Bin 4, camera 12
  bin4_.gantry = { 5.45, -0.77, PI / 2 };
  bin4_.left_arm = { 0.0, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };
  bin4_.right_arm = { PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };

  //Bin 7, camera 12
  bin7_.gantry = { 4.026, -2.15, 0 };
  bin7_.left_arm = { 0.0, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };
  bin7_.right_arm = { PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };

  //Bin 8, camera 12
  bin8_.gantry = { 5.426, -1.47, PI / 2 };
  bin8_.left_arm = { 0.0, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };
  bin8_.right_arm = { PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };

  //Bin 15, camera 14
  bin15_.gantry = { 3.9, 2.29, 0 };
  bin15_.left_arm = { 0.0, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };
  bin15_.right_arm = { PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };

  //Bin 16, camera 14
  bin16_.gantry = { 4.967, 2.29, 0 };
  bin16_.left_arm = { 0.0, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };
  bin16_.right_arm = { PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };

  //Bin 11, camera 14
  bin11_.gantry = { 4.026, 1.45, 0 };
  bin11_.left_arm = { 0.0, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };
  bin11_.right_arm = { PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };

  //Bin 12, camera 14
  bin12_.gantry = { 4.967, 1.45, 0 };
  bin12_.left_arm = { 0.0, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };
  bin12_.right_arm = { PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };

  //Bin 13, camera 13
  bin13_.gantry = { 2.145, 2.29, 0 };
  bin13_.left_arm = { 0.0, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };
  bin13_.right_arm = { PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };

  //Bin 14, camera 13
  bin14_.gantry = { 3.086, 2.29, 0 };
  bin14_.left_arm = { 0.0, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };
  bin14_.right_arm = { PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };

  //Bin 9, camera 13
  bin9_.gantry = { 2.145, 1.45, 0 };
  bin9_.left_arm = { 0.0, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };
  bin9_.right_arm = { PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };

  //Bin 10, camera 13
  bin10_.gantry = { 3.086, 1.45, 0 };
  bin10_.left_arm = { 0.0, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };
  bin10_.right_arm = { PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };

  std::string cam;
  std::vector < PresetLocation > waypoints;

  // AISLE 1
  cam = "1f";
  waypoints.clear();
  if (aisle_1_choice == 2) {
    waypoints.push_back(shelf5_rf_w1);
    waypoints.push_back(gap_1_1);
    waypoints.push_back(GAP_1);
    waypoints.push_back(gap_1_1);
    waypoints.push_back(shelf5_rf_w3);
    pickup_locations[cam] = waypoints;
  } else {
    waypoints.push_back(shelf5_rf_w1);
    waypoints.push_back(gap_1_1);
    waypoints.push_back(shelf5_rf_w3);
    pickup_locations[cam] = waypoints;
  }

  cam = "2f";
  waypoints.clear();
  if (aisle_1_choice == 2) {
    waypoints.push_back(shelf5_lf_w1);
    waypoints.push_back(gap_1_1);
    waypoints.push_back(GAP_1);
    waypoints.push_back(gap_1_1);
    waypoints.push_back(shelf5_lf_w3);
    pickup_locations[cam] = waypoints;
  } else {
    waypoints.push_back(shelf5_lf_w1);
    waypoints.push_back(gap_1_1);
    waypoints.push_back(shelf5_lf_w3);
    pickup_locations[cam] = waypoints;
  }

  // AISLE 1 COMPLETE ----

  //AISLE 2

  cam = "1b";
  waypoints.clear();
  if (aisle_2_choice == 2) {
    waypoints.push_back(shelf5_rb_w1);
    waypoints.push_back(gap_1_2);
    if (shelf_1_gap == 2) {
      waypoints.push_back(GAP_1);
    } else {
      waypoints.push_back(GAP_2);
    }
    waypoints.push_back(gap_1_2);
    waypoints.push_back(shelf5_rb_w3);
    pickup_locations[cam] = waypoints;
  } else {
    waypoints.push_back(shelf5_rb_w1);
    waypoints.push_back(gap_1_2);
    waypoints.push_back(shelf5_rb_w3);
    pickup_locations[cam] = waypoints;
  }
  ;

  cam = "2b";
  waypoints.clear();
  if (aisle_2_choice == 2) {
    waypoints.push_back(shelf5_lb_w1);
    waypoints.push_back(gap_1_2);
    if (shelf_1_gap == 2) {
      waypoints.push_back(GAP_1);
    } else {
      waypoints.push_back(GAP_2);
    }
    waypoints.push_back(gap_1_2);
    waypoints.push_back(shelf5_lb_w3);
    pickup_locations[cam] = waypoints;
  } else {
    waypoints.push_back(shelf5_lb_w1);
    waypoints.push_back(gap_1_2);
    waypoints.push_back(shelf5_lb_w3);
    pickup_locations[cam] = waypoints;
  }

  cam = "3f";
  waypoints.clear();
  if (aisle_2_choice == 2) {
    waypoints.push_back(shelf8_rf_w1);
    waypoints.push_back(gap_2_2);
    if (shelf_1_gap == 2) {
      waypoints.push_back(GAP_1);
    } else {
      waypoints.push_back(GAP_2);
    }
    waypoints.push_back(gap_2_2);
    waypoints.push_back(shelf8_rf_w3);
    pickup_locations[cam] = waypoints;
  } else {
    waypoints.push_back(shelf8_rf_w1);
    waypoints.push_back(gap_2_2);
    waypoints.push_back(shelf8_rf_w3);
    pickup_locations[cam] = waypoints;
  }

  cam = "4f";
  waypoints.clear();
  if (aisle_2_choice == 2) {
    waypoints.push_back(shelf8_lf_w1);
    waypoints.push_back(gap_2_2);
    if (shelf_1_gap == 2) {
      waypoints.push_back(GAP_1);
    } else {
      waypoints.push_back(GAP_2);
    }
    waypoints.push_back(gap_2_2);
    waypoints.push_back(shelf8_lf_w3);
    pickup_locations[cam] = waypoints;
  } else {
    waypoints.push_back(shelf8_lf_w1);
    waypoints.push_back(gap_2_2);
    waypoints.push_back(shelf8_lf_w3);
    pickup_locations[cam] = waypoints;
  }

  // AISLE 2 COMPLETE ----

  // AISLE 3

  cam = "3b";
  waypoints.clear();
  if (aisle_3_choice == 2) {
    waypoints.push_back(shelf8_rb_w1);
    waypoints.push_back(gap_2_3);
    if (shelf_2_gap == 2) {
      waypoints.push_back(GAP_2);
    } else {
      waypoints.push_back(GAP_3);
    }
    waypoints.push_back(gap_2_3);
    waypoints.push_back(shelf8_rb_w3);
    pickup_locations[cam] = waypoints;
  } else {
    waypoints.push_back(shelf8_rb_w1);
    waypoints.push_back(gap_2_3);
    waypoints.push_back(shelf8_rb_w3);
    pickup_locations[cam] = waypoints;
  }

  cam = "4b";
  waypoints.clear();
  if (aisle_3_choice == 2) {
    waypoints.push_back(shelf8_lb_w1);
    waypoints.push_back(gap_2_3);
    if (shelf_2_gap == 2) {
      waypoints.push_back(GAP_2);
    } else {
      waypoints.push_back(GAP_3);
    }
    waypoints.push_back(gap_2_3);
    waypoints.push_back(shelf8_lb_w3);
    pickup_locations[cam] = waypoints;
  } else {
    waypoints.push_back(shelf8_lb_w1);
    waypoints.push_back(gap_2_3);
    waypoints.push_back(shelf8_lb_w3);
    pickup_locations[cam] = waypoints;
  }

  cam = "6f";
  waypoints.clear();
  if (aisle_3_choice == 2) {
      ROS_INFO_STREAM("shelf11_lf_w1");
    waypoints.push_back(shelf11_lf_w1);
    waypoints.push_back(gap_3_3);
    if (shelf_2_gap == 2) {
        ROS_INFO_STREAM("GAP_2");
      waypoints.push_back(GAP_2);
    } else {
        ROS_INFO_STREAM("GAP_3");
      waypoints.push_back(GAP_3);
    }
    waypoints.push_back(gap_3_3);
    waypoints.push_back(shelf11_lf_w3);
    pickup_locations[cam] = waypoints;
  } else {
      ROS_INFO_STREAM("else loop");
    waypoints.push_back(shelf11_lf_w1);
    waypoints.push_back(gap_3_3);
    waypoints.push_back(shelf11_lf_w3);
    pickup_locations[cam] = waypoints;
  }

  cam = "5f";
  waypoints.clear();
  if (aisle_3_choice == 2) {
    waypoints.push_back(shelf11_rf_w1);
    waypoints.push_back(gap_3_3);
    if (shelf_2_gap == 2) {
      waypoints.push_back(GAP_2);
    } else {
      waypoints.push_back(GAP_3);
    }
    waypoints.push_back(gap_3_3);
    waypoints.push_back(shelf11_rf_w3);
    pickup_locations[cam] = waypoints;
  } else {
    waypoints.push_back(shelf11_rf_w1);
    waypoints.push_back(gap_3_3);
    waypoints.push_back(shelf11_rf_w3);
    pickup_locations[cam] = waypoints;
  }

  // AISLE 3 COMPLETE ----

  // AISLE 4

  cam = "5b";
  waypoints.clear();
  if (aisle_4_choice == 2) {
    waypoints.push_back(shelf11_rb_w1);
    waypoints.push_back(gap_3_4);
    waypoints.push_back(GAP_3);
    waypoints.push_back(gap_3_4);
    waypoints.push_back(shelf11_rb_w3);
    pickup_locations[cam] = waypoints;
  } else {
    waypoints.push_back(shelf11_rb_w1);
    waypoints.push_back(gap_3_4);
    waypoints.push_back(shelf11_rb_w3);
    pickup_locations[cam] = waypoints;
  }

  cam = "6b";
  waypoints.clear();
  if (aisle_4_choice == 2) {
    waypoints.push_back(shelf11_lb_w1);
    waypoints.push_back(gap_3_4);
    waypoints.push_back(GAP_3);
    waypoints.push_back(gap_3_4);
    waypoints.push_back(shelf11_lb_w3);
    pickup_locations[cam] = waypoints;
  } else {
    waypoints.push_back(shelf11_lb_w1);
    waypoints.push_back(gap_3_4);
    waypoints.push_back(shelf11_lb_w3);
    pickup_locations[cam] = waypoints;
  }

  // AISLE 4 COMPLETE ----

  cam = "11";
  waypoints.clear();
  waypoints.push_back(bin1_w1);
  pickup_locations[cam] = waypoints;

  //Shelf 1, left front, camera 7
  cam = "7f";
  waypoints.clear();
  waypoints.push_back(shelf1_lf_w1);
  waypoints.push_back(shelf1_lf_w2);
  waypoints.push_back(shelf1_lf_w3);
  waypoints.push_back(shelf1_lf_w4);
  pickup_locations[cam] = waypoints;

  //Shelf 1, left back, camera 7
  cam = "7b";
  waypoints.clear();
  waypoints.push_back(shelf1_lb_w1);
  waypoints.push_back(shelf1_lb_w2);
  waypoints.push_back(shelf1_lb_w3);
  waypoints.push_back(shelf1_lb_w4);
  pickup_locations[cam] = waypoints;

  //Shelf 1, right front, camera 10
  cam = "10f";
  waypoints.clear();
  waypoints.push_back(shelf1_rf_w1);
  waypoints.push_back(shelf1_rf_w2);
  waypoints.push_back(shelf1_rf_w3);
  waypoints.push_back(shelf1_rf_w4);
  pickup_locations[cam] = waypoints;

  //Shelf 1, right back, camera 10
  cam = "10b";
  waypoints.clear();
  waypoints.push_back(shelf1_rb_w1);
  waypoints.push_back(shelf1_rb_w2);
  waypoints.push_back(shelf1_rb_w3);
  waypoints.push_back(shelf1_rb_w4);
  pickup_locations[cam] = waypoints;

  //Shelf 2, left front, camera 9
  cam = "9f";
  waypoints.clear();
  waypoints.push_back(shelf2_lf_w1);
  waypoints.push_back(shelf2_lf_w2);
  waypoints.push_back(shelf2_lf_w3);
  waypoints.push_back(shelf2_lf_w4);
  pickup_locations[cam] = waypoints;

  //Shelf 2, left back, camera 9
  cam = "9b";
  waypoints.clear();
  waypoints.push_back(shelf2_lb_w1);
  waypoints.push_back(shelf2_lb_w2);
  waypoints.push_back(shelf2_lb_w3);
  waypoints.push_back(shelf2_lb_w4);
  pickup_locations[cam] = waypoints;

  //Shelf 2, right front, camera 8
  cam = "8f";
  waypoints.clear();
  waypoints.push_back(shelf2_rf_w1);
  waypoints.push_back(shelf2_rf_w2);
  waypoints.push_back(shelf2_rf_w3);
  waypoints.push_back(shelf2_rf_w4);
  pickup_locations[cam] = waypoints;

  //Shelf 2, right back, camera 8
  cam = "8b";
  waypoints.clear();
  waypoints.push_back(shelf2_rb_w1);
  waypoints.push_back(shelf2_rb_w2);
  waypoints.push_back(shelf2_rb_w3);
  waypoints.push_back(shelf2_rb_w4);
  pickup_locations[cam] = waypoints;

  //Bin 1, camera 11
  cam = "11_1";
  waypoints.clear();
  waypoints.push_back(bin1_);
  pickup_locations[cam] = waypoints;

  //Bin 2, camera 11
  cam = "11_2";
  waypoints.clear();
  waypoints.push_back(bin2_);
  pickup_locations[cam] = waypoints;

  //Bin 5, camera 11
  cam = "11_5";
  waypoints.clear();
  waypoints.push_back(bin5_);
  pickup_locations[cam] = waypoints;

  //Bin 6, camera 11
  cam = "11_6";
  waypoints.clear();
  waypoints.push_back(bin6_);
  pickup_locations[cam] = waypoints;

  //Bin 3, camera 12
  cam = "12_3";
  waypoints.clear();
  waypoints.push_back(bin3_);
  pickup_locations[cam] = waypoints;

  //Bin 4, camera 12
  cam = "12_4";
  waypoints.clear();
  waypoints.push_back(bin4_);
  pickup_locations[cam] = waypoints;

  //Bin 7, camera 12
  cam = "12_7";
  waypoints.clear();
  waypoints.push_back(bin7_);
  pickup_locations[cam] = waypoints;

  //Bin 8, camera 12
  cam = "12_8";
  waypoints.clear();
  waypoints.push_back(bin8_);
  pickup_locations[cam] = waypoints;

  //Bin 15, camera 14
  cam = "14_15";
  waypoints.clear();
  waypoints.push_back(bin15_);
  pickup_locations[cam] = waypoints;

  //Bin 16, camera 14
  cam = "14_16";
  waypoints.clear();
  waypoints.push_back(bin16_);
  pickup_locations[cam] = waypoints;

  //Bin 11, camera 14
  cam = "14_11";
  waypoints.clear();
  waypoints.push_back(bin11_);
  pickup_locations[cam] = waypoints;

  //Bin 12, camera 14
  cam = "14_12";
  waypoints.clear();
  waypoints.push_back(bin12_);
  pickup_locations[cam] = waypoints;

  //Bin 13, camera 13
  cam = "13_13";
  waypoints.clear();
  waypoints.push_back(bin13_);
  pickup_locations[cam] = waypoints;

  //Bin 14, camera 13
  cam = "13_14";
  waypoints.clear();
  waypoints.push_back(bin14_);
  pickup_locations[cam] = waypoints;

  //Bin 9, camera 13
  cam = "13_9";
  waypoints.clear();
  waypoints.push_back(bin9_);
  pickup_locations[cam] = waypoints;

  //Bin 10, camera 13
  cam = "13_10";
  waypoints.clear();
  waypoints.push_back(bin10_);
  pickup_locations[cam] = waypoints;

//    pulley_part_red located on waypoint_1
  waypoint_1_.gantry = { 0.0, -4.7, 0.0 };
  waypoint_1_.left_arm = { 0.0, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };
  waypoint_1_.right_arm = { PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };

//    pulley_part_red located on waypoint_2
  waypoint_2_.gantry = { -14.5, -4.7, 0.0 };
  waypoint_2_.left_arm = { 0.0, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };
  waypoint_2_.right_arm = { PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };

  //    pulley_part_red located on waypoint_3
  waypoint_3_.gantry = { -14.5, -4.7, 0.0 };
  waypoint_3_.left_arm = { -PI / 2, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };
  waypoint_3_.right_arm = { PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };

  //    pulley_part_red located on waypoint_4
  waypoint_4_.gantry = { -14.5, -4.3, 0.0 };
  waypoint_4_.left_arm = { -2.79, -PI / 4, PI / 2, -PI / 4, -1.39626, 0 };
  waypoint_4_.right_arm = { PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };

//    pulley_part_red located on shelf5
  shelf5_.gantry = { -14.00, -4.76, 0.0 };
  shelf5_.left_arm = { 0.0, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };
  shelf5_.right_arm = { PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };

//    Start location
  start_.gantry = { 0, 0, 0 };
  start_.left_arm = { 0.0, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };
  start_.right_arm = { PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };

//    Bin3 location
//    bin3_.gantry = {4.0, -1.1, 0.};
//    bin3_.left_arm = {0.0, -PI/4, PI/2, -PI/4, PI/2, 0};
//    bin3_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

//    Agv2 location
  agv2_.gantry = { 0.6, 6.9, PI };
  agv2_.left_arm = { 0.0, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };
  agv2_.right_arm = { PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };

  //    Agv1 location
  agv1_.gantry = { 0.6, -6.95, PI };
  // agv1_.left_arm = {-0.38, -PI/4, PI/2, -PI/4, 1.15, 0};
  agv1_.left_arm = { 0.0, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };
  agv1_.right_arm = { PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };

  //    Agv2 faulty part Drop location
  agv2_drop_.gantry = { 1, 6.9, PI };
  agv2_drop_.left_arm = { 0.0, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };
  agv2_drop_.right_arm = { PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };

  //    Agv1 faulty part Drop location
  agv1_drop_.gantry = { 1, -6.9, PI };
  agv1_drop_.left_arm = { 0.0, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };
  agv1_drop_.right_arm = { PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };

  // pose change wavepoints

  pose_change_1_agv1.gantry = { 0.0, -5, PI };
  pose_change_1_agv1.left_arm = { PI / 4, -0.2, 1.3, 0.5, PI / 2, 0.00 };
  pose_change_1_agv1.right_arm = { -PI / 4, -3, -PI / 2, -0.1, PI / 2, -0.79 };

  // switching wavepoint
  pose_change_2_agv1.gantry = { 0.0, -5, PI };
  pose_change_2_agv1.left_arm = { 0.77, -0.2, 1.3, 0.49, 1.59, 0.00 };
  pose_change_2_agv1.right_arm =
      { -PI / 4, -3.2, -1.5, -0.02, PI / 2, -PI / 4 };

  agv1_flip_.gantry = { 0.0, -2, PI };
  agv1_flip_.left_arm = { 0.0, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };
  agv1_flip_.right_arm = { PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };

  agv1_flip_target_.gantry = { -0.66, -6.9, PI };
  agv1_flip_target_.left_arm = { 0.0, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };
  agv1_flip_target_.right_arm = { PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };

  // pose change wavepoints

  pose_change_1_agv2.gantry = { 0.0, 5, PI };
  pose_change_1_agv2.left_arm = { PI / 4, -0.2, 1.3, 0.5, PI / 2, 0.00 };
  pose_change_1_agv2.right_arm = { -PI / 4, -3, -PI / 2, -0.1, PI / 2, -0.79 };

  // switching wavepoint
  pose_change_2_agv2.gantry = { 0.0, 5, PI };
  pose_change_2_agv2.left_arm = { 0.77, -0.2, 1.3, 0.49, 1.59, 0.00 };
  pose_change_2_agv2.right_arm =
      { -PI / 4, -3.2, -1.5, -0.02, PI / 2, -PI / 4 };

  agv2_flip_.gantry = { 0.0, 2, PI };
  agv2_flip_.left_arm = { 0.0, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };
  agv2_flip_.right_arm = { PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };

  agv2_flip_target_.gantry = { -0.66, 6.9, PI };
  agv2_flip_target_.left_arm = { 0.0, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };
  agv2_flip_target_.right_arm = { PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };

  //Moving to shelf 11
  // gasket part blue
  shelf11_w1_.gantry = { 0.0, 1.4, 0 };
  shelf11_w1_.left_arm = { 0.0, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };
  shelf11_w1_.right_arm = { PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };

  shelf11_w2_.gantry = { -14.8, 1.4, 0 };
  shelf11_w2_.left_arm = { 0.0, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };
  shelf11_w2_.right_arm = { PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };

  shelf11_w3_.gantry = { -14.8, 1.4, 0.0 };
  shelf11_w3_.left_arm = { -PI / 2, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };
  shelf11_w3_.right_arm = { PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };

  shelf11_w4_.gantry = { -14.8, 1.8, 0.0 };
  shelf11_w4_.left_arm = { -2.79, -PI / 4, PI / 2, -PI / 4, -1.39626, 0 };
  shelf11_w4_.right_arm = { PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };

  // location where gantry hover to pick up part at belt
  belt_pickup_.gantry = { 0.52, -3.27, PI / 2 };
  belt_pickup_.left_arm = { 0.38, -0.41, 1.05, -0.63, 1.94, 0.0 };
  belt_pickup_.right_arm = { PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };

  // initial arm position
  belt_pickup_1.gantry = { 0.52, -3.27, PI / 2 };
  belt_pickup_1.left_arm = { 0.0, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };
  belt_pickup_1.right_arm = { PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };

  // bin1 drop location after picking part from conveyor belt
  bin1_drop_.gantry = { 2.7, -0.87, PI / 2 };
  bin1_drop_.left_arm = { 0.0, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };
  bin1_drop_.right_arm = { PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };

  // bin3 drop location after picking part from conveyor belt
  bin3_drop_.gantry = { 4.58, -0.87, PI / 2 };
  bin3_drop_.left_arm = { 0.0, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };
  bin3_drop_.right_arm = { PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };

  // bin9 drop location after picking part from conveyor belt
  bin9_drop_.gantry = { 2.7, 1.87, PI / 2 };
  bin9_drop_.left_arm = { 0.0, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };
  bin9_drop_.right_arm = { PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };

  // bin11 drop location after picking part from conveyor belt
  bin11_drop_.gantry = { 4.58, 1.87, PI / 2 };
  bin11_drop_.left_arm = { 0.0, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };
  bin11_drop_.right_arm = { PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0 };

  //--Raw pointers are frequently used to refer to the planning group for improved performance.
  //--To start, we will create a pointer that references the current robot’s state.
  const moveit::core::JointModelGroup *joint_model_group = full_robot_group_
      .getCurrentState()->getJointModelGroup("Full_Robot");

  //--Let’s set a joint space goal and move towards it.
  moveit::core::RobotStatePtr current_state =
      full_robot_group_.getCurrentState();

  //--Next get the current set of joint values for the group.
//    std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group,
                                         joint_group_positions_);

  gantry_joint_trajectory_publisher_ = node_.advertise
      < trajectory_msgs::JointTrajectory
      > ("/ariac/gantry/gantry_controller/command", 10);

  left_arm_joint_trajectory_publisher_ = node_.advertise
      < trajectory_msgs::JointTrajectory
      > ("/ariac/gantry/left_arm_controller/command", 10);

  right_arm_joint_trajectory_publisher_ = node_.advertise
      < trajectory_msgs::JointTrajectory
      > ("/ariac/gantry/right_arm_controller/command", 10);

  joint_states_subscriber_ = node_.subscribe(
      "/ariac/gantry/joint_states", 10, &GantryControl::joint_states_callback,
      this);

  left_gripper_state_subscriber_ = node_.subscribe(
      "/ariac/gantry/left_arm/gripper/state", 10,
      &GantryControl::left_gripper_state_callback, this);

  right_gripper_state_subscriber_ = node_.subscribe(
      "/ariac/gantry/right_arm/gripper/state", 10,
      &GantryControl::right_gripper_state_callback, this);

  gantry_controller_state_subscriber_ = node_.subscribe(
      "/ariac/gantry/gantry_controller/state", 10,
      &GantryControl::gantry_controller_state_callback, this);

  left_arm_controller_state_subscriber_ = node_.subscribe(
      "/ariac/gantry/left_arm_controller/state", 10,
      &GantryControl::left_arm_controller_state_callback, this);

  right_arm_controller_state_subscriber_ = node_.subscribe(
      "/ariac/gantry/right_arm_controller/state", 10,
      &GantryControl::right_arm_controller_state_callback, this);

  while ((current_gantry_controller_state_.joint_names.size() == 0)
      || (current_left_arm_controller_state_.joint_names.size() == 0)
      || (current_right_arm_controller_state_.joint_names.size() == 0)) {
    ROS_WARN(
        "[GantryControl::init] Waiting for first controller_state callbacks...");
    ros::Duration(0.1).sleep();
  }

  left_gripper_control_client = node_.serviceClient
      < nist_gear::VacuumGripperControl
      > ("/ariac/gantry/left_arm/gripper/control");
  left_gripper_control_client.waitForExistence();

  right_gripper_control_client = node_.serviceClient
      < nist_gear::VacuumGripperControl
      > ("/ariac/gantry/right_arm/gripper/control");
  right_gripper_control_client.waitForExistence();

  // Move robot to init position
  ROS_INFO("[GantryControl::init] Init position ready)...");
}

/**
 * @brief Returns the Stats
 * @param function
 * @return stats
 */
stats GantryControl::getStats(std::string function) {
  if (function == "init")
    return init_;
  if (function == "moveJ")
    return moveJ_;
  if (function == "IK")
    return IK_;
  if (function == "moveGantry")
    return moveGantry_;
  if (function == "pickPart")
    return pickPart_;
  if (function == "placePart")
    return placePart_;
  if (function == "dropPart")
    return dropPart_;
  if (function == "gripFirmly")
    return gripFirmly_;
  if (function == "gripFromBelt")
    return gripFromBelt_;
  if (function == "grip")
    return grip_;
}

/**
 * @brief Function which attempts to pickup a moving part from the conveyor belt
 * @param part
 * @return bool value based on successful picking up of part
 */
bool GantryControl::pickMovingPart(part part) {
  //--Activate gripper
  activateGripper("left_arm");
//    ros::AsyncSpinner spinner(1);
//    spinner.start();

//    left_arm_group_.setPoseReferenceFrame("world");
  geometry_msgs::Pose currentPose = left_arm_group_.getCurrentPose().pose;

//    ROS_INFO_STREAM("[left_arm_group_]= " << currentPose.position.x << ", " << currentPose.position.y << "," << currentPose.position.z);

  part.pose.position.z = part.pose.position.z + model_height.at(part.type)
      + GRIPPER_HEIGHT - EPSILON;
  part.pose.orientation.x = currentPose.orientation.x;
  part.pose.orientation.y = currentPose.orientation.y;
  part.pose.orientation.z = currentPose.orientation.z;
  part.pose.orientation.w = currentPose.orientation.w;

  auto state = getGripperState("left_arm");
  if (state.enabled) {
    ROS_INFO_STREAM("[Gripper] = enabled");
    //--Move arm to part
    left_arm_group_.setPoseTarget(part.pose);
    left_arm_group_.move();
    auto state = getGripperState("left_arm");
    if (state.attached) {
      ROS_INFO_STREAM("[Gripper] = object attached");
      return true;
    }
  } else {
    ROS_INFO_STREAM("[Gripper] = not enabled");
  }
  return false;

  /**
   * We want the Cartesian path to be interpolated at a resolution of 1 cm which is why
   * we will specify 0.01 as the max step in Cartesian translation.
   * We will specify the jump threshold as 0.0, effectively disabling it.
   */
}

geometry_msgs::Pose GantryControl::getTargetWorldPose(
    geometry_msgs::Pose target, std::string agv) {
  static tf2_ros::StaticTransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped;
  std::string kit_tray;
  if (agv.compare("agv1") == 0) {
//        ROS_INFO_STREAM("AGV1?" << agv);
    kit_tray = "kit_tray_1";
  } else {
    kit_tray = "kit_tray_2";
//        ROS_INFO_STREAM(" which AGV?" << agv);
  }

  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = kit_tray;
  transformStamped.child_frame_id = "target_frame";
  transformStamped.transform.translation.x = target.position.x;
  transformStamped.transform.translation.y = target.position.y;
  transformStamped.transform.translation.z = target.position.z;
  transformStamped.transform.rotation.x = target.orientation.x;
  transformStamped.transform.rotation.y = target.orientation.y;
  transformStamped.transform.rotation.z = target.orientation.z;
  transformStamped.transform.rotation.w = target.orientation.w;

  for (int i { 0 }; i < 15; ++i)
    br.sendTransform(transformStamped);

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  ros::Rate rate(10);
  ros::Duration timeout(5.0);

  geometry_msgs::TransformStamped world_target_tf;
  geometry_msgs::TransformStamped ee_target_tf;
  for (int i = 0; i < 10; i++) {
    try {
      world_target_tf = tfBuffer.lookupTransform("world", "target_frame",
                                                 ros::Time(0), timeout);
    } catch (tf2::TransformException &ex) {
      ROS_WARN("%s", ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    try {
      ee_target_tf = tfBuffer.lookupTransform("target_frame", "left_ee_link",
                                              ros::Time(0), timeout);
    } catch (tf2::TransformException &ex) {
      ROS_WARN("%s", ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }
  }
  geometry_msgs::Pose world_target { target };
  world_target.position.x = world_target_tf.transform.translation.x;
  world_target.position.y = world_target_tf.transform.translation.y;
  world_target.position.z = world_target_tf.transform.translation.z;
  world_target.orientation.x = ee_target_tf.transform.rotation.x;
  world_target.orientation.y = ee_target_tf.transform.rotation.y;
  world_target.orientation.z = ee_target_tf.transform.rotation.z;
  world_target.orientation.w = ee_target_tf.transform.rotation.w;
  return world_target;
}

/**
 * @brief Converts the pose from tray's coordinates to the world's coordinates and broadcasts a frame for tray
 * @param target
 * @param agv
 * @return Pose in world coordinates for right arm
 */
geometry_msgs::Pose GantryControl::getTargetWorldPose_right_arm(
    geometry_msgs::Pose target, std::string agv) {
  static tf2_ros::StaticTransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped;

  std::string kit_tray;
  if (agv.compare("agv1") == 0)
    kit_tray = "kit_tray_1";
  else
    kit_tray = "kit_tray_2";
  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = kit_tray;
  transformStamped.child_frame_id = "target_frame";
  transformStamped.transform.translation.x = target.position.x;
  transformStamped.transform.translation.y = target.position.y;
  transformStamped.transform.translation.z = target.position.z;
  transformStamped.transform.rotation.x = target.orientation.x;
  transformStamped.transform.rotation.y = target.orientation.y;
  transformStamped.transform.rotation.z = target.orientation.z;
  transformStamped.transform.rotation.w = target.orientation.w;

  for (int i { 0 }; i < 15; ++i)
    br.sendTransform(transformStamped);

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  ros::Rate rate(10);
  ros::Duration timeout(5.0);

  geometry_msgs::TransformStamped world_target_tf;
  geometry_msgs::TransformStamped ee_target_tf;
  for (int i = 0; i < 10; i++) {
    try {
      world_target_tf = tfBuffer.lookupTransform("world", "target_frame",
                                                 ros::Time(0), timeout);
    } catch (tf2::TransformException &ex) {
      ROS_WARN("%s", ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    try {
      ee_target_tf = tfBuffer.lookupTransform("target_frame", "right_ee_link",
                                              ros::Time(0), timeout);
    } catch (tf2::TransformException &ex) {
      ROS_WARN("%s", ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }
  }

  geometry_msgs::Pose world_target { target };
  world_target.position.x = world_target_tf.transform.translation.x;
  world_target.position.y = world_target_tf.transform.translation.y;
  world_target.position.z = world_target_tf.transform.translation.z;
  world_target.orientation.x = ee_target_tf.transform.rotation.x;
  world_target.orientation.y = ee_target_tf.transform.rotation.y;
  world_target.orientation.z = ee_target_tf.transform.rotation.z;
  world_target.orientation.w = ee_target_tf.transform.rotation.w;
  ROS_INFO_STREAM("printing right arm placement coordinates");
  ROS_INFO_STREAM(world_target);
  return world_target;
}

/**
 * @brief Places the part using the right arm on the tray of the agv
 * @param part
 * @param agv
 * @return None
 */
void GantryControl::placePart_right_arm(part part, std::string agv) {
  auto target_pose_in_tray = getTargetWorldPose_right_arm(part.pose, agv);
  target_pose_in_tray.position.z += (ABOVE_TARGET
      + 1.5 * model_height[part.type]);

  right_arm_group_.setPoseTarget(target_pose_in_tray);
  right_arm_group_.move();
  deactivateGripper("right_arm");
}

/**
 * @brief Converts the pose from tray's coordinates to the world's coordinates and broadcasts a frame for tray
 * @param target
 * @param agv
 * @return Pose in world coordinates
 */
geometry_msgs::Pose GantryControl::getTargetWorldPose_dummy(
    geometry_msgs::Pose target, std::string agv) {
  geometry_msgs::TransformStamped transformStamped;
  std::string kit_tray;
  if (agv.compare("agv1") == 0)
    kit_tray = "kit_tray_1";
  else
    kit_tray = "kit_tray_2";
  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = kit_tray;
  transformStamped.child_frame_id = "target_frame";
  transformStamped.transform.translation.x = target.position.x;
  transformStamped.transform.translation.y = target.position.y;
  transformStamped.transform.translation.z = target.position.z;
  transformStamped.transform.rotation.x = target.orientation.x;
  transformStamped.transform.rotation.y = target.orientation.y;
  transformStamped.transform.rotation.z = target.orientation.z;
  transformStamped.transform.rotation.w = target.orientation.w;

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  ros::Rate rate(10);
  ros::Duration timeout(5.0);

  geometry_msgs::TransformStamped world_target_tf;
  geometry_msgs::TransformStamped ee_target_tf;
  for (int i = 0; i < 10; i++) {
    try {
      world_target_tf = tfBuffer.lookupTransform("world", "target_frame",
                                                 ros::Time(0), timeout);
    } catch (tf2::TransformException &ex) {
      ROS_WARN("%s", ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    try {
      ee_target_tf = tfBuffer.lookupTransform("target_frame", "left_ee_link",
                                              ros::Time(0), timeout);
    } catch (tf2::TransformException &ex) {
      ROS_WARN("%s", ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }
  }

  geometry_msgs::Pose world_target { target };
  world_target.position.x = world_target_tf.transform.translation.x;
  world_target.position.y = world_target_tf.transform.translation.y;
  world_target.position.z = world_target_tf.transform.translation.z;
  world_target.orientation.x = ee_target_tf.transform.rotation.x;
  world_target.orientation.y = ee_target_tf.transform.rotation.y;
  world_target.orientation.z = ee_target_tf.transform.rotation.z;
  world_target.orientation.w = ee_target_tf.transform.rotation.w;

  return world_target;
}

/**
 * @brief Function which makes the robot to pickup a part
 * @param part
 * @return bool value based on successful picking up of part
 */
bool GantryControl::pickPart(part part) {
  //--Activate gripper
  activateGripper("left_arm");
  geometry_msgs::Pose currentPose = left_arm_group_.getCurrentPose().pose;

  part.pose.position.z = part.pose.position.z + model_height.at(part.type)
      + GRIPPER_HEIGHT - EPSILON;
  part.pose.orientation.x = currentPose.orientation.x;
  part.pose.orientation.y = currentPose.orientation.y;
  part.pose.orientation.z = currentPose.orientation.z;
  part.pose.orientation.w = currentPose.orientation.w;

  auto state = getGripperState("left_arm");
  if (state.enabled) {
    ROS_INFO_STREAM("[Gripper] = enabled");
    //--Move arm to part
    left_arm_group_.setPoseTarget(part.pose);
    left_arm_group_.move();
    auto state = getGripperState("left_arm");
    if (state.attached) {
      ROS_INFO_STREAM("[Gripper] = object attached");
      return true;
    } else {
      ROS_INFO_STREAM("[Gripper] = object not attached");
      int attempt { 0 }, max_attempts { 5 };
      int current_attempt { 0 };
      while (!state.attached || (attempt != max_attempts)) {
        ROS_INFO_STREAM("Attached status = " << state.attached);
        left_arm_group_.setPoseTarget(currentPose);
        left_arm_group_.move();
        ros::Duration(0.5).sleep();
        left_arm_group_.setPoseTarget(part.pose);
        left_arm_group_.move();
        activateGripper("left_arm");
        auto state = getGripperState("left_arm");
        if (state.attached) {
          return true;
        }
        attempt += 1;
      }
    }
  } else {
    ROS_INFO_STREAM("[Gripper] = not enabled");
  }
  return false;
}

/**
 * @brief Places the part using the left arm on the tray of the agv
 * @param part
 * @param agv
 * @return None
 */
void GantryControl::placePart(part part, std::string agv) {
  geometry_msgs::Pose initial_pose, final_pose;

  initial_pose = part.initial_pose;
  final_pose = getTargetWorldPose(part.pose, agv);

  // Orientation quaternion
  tf2::Quaternion q1(initial_pose.orientation.x, initial_pose.orientation.y,
                     initial_pose.orientation.z, initial_pose.orientation.w);
  tf2::Quaternion q2(final_pose.orientation.x, final_pose.orientation.y,
                     final_pose.orientation.z, final_pose.orientation.w);

  // 3x3 Rotation matrix from quaternion
  tf2::Matrix3x3 m(q1);
  tf2::Matrix3x3 m1(q2);

  // Roll Pitch and Yaw from rotation matrix
  double initial_roll, initial_pitch, initial_yaw, final_roll, final_pitch,
      final_yaw, target_roll, target_pitch, target_yaw;
  m.getRPY(initial_roll, initial_pitch, initial_yaw);
  m1.getRPY(final_roll, final_pitch, final_yaw);

  target_roll = final_roll - initial_roll;
  target_pitch = final_pitch - initial_pitch;

  if (initial_yaw < 0) {
    ROS_INFO_STREAM("Initial pose was negative -45 ");
    target_yaw = -(final_yaw - (initial_yaw + 3.14) - 3.14 - 0.733);  //RWA5 Green Gasket picking
  } else {
    ROS_INFO_STREAM("Initial pose was positive 45 ");
    target_yaw = -(final_yaw - (initial_yaw - 3.2) + 3.14) - 2.36;
  }

  auto final_pose_ = ToQuaternion(target_roll, target_pitch, target_yaw);
  final_pose.orientation.x = final_pose_.x;
  final_pose.orientation.y = final_pose_.y;
  final_pose.orientation.z = final_pose_.z;
  final_pose.orientation.w = final_pose_.w;

  geometry_msgs::Pose target_pose_in_tray = final_pose;
  if (agv == "agv2") {
    goToPresetLocation (agv2_);
  } else {
    goToPresetLocation (agv1_);
    ROS_INFO_STREAM("AGV Location Reached");
  }

  target_pose_in_tray.position.z += (ABOVE_TARGET
      + 1.5 * model_height[part.type]);
  ROS_INFO_STREAM("target_pose_in_tray = " << target_pose_in_tray);
  left_arm_group_.setPoseTarget(target_pose_in_tray);
  left_arm_group_.move();
  deactivateGripper("left_arm");
}

/**
 * @brief Moves the robot to a preset location
 * @param location
 * @return None
 */
void GantryControl::goToPresetLocation(PresetLocation location) {
  //--gantry
  joint_group_positions_.at(0) = location.gantry.at(0);
  joint_group_positions_.at(1) = location.gantry.at(1);
  joint_group_positions_.at(2) = location.gantry.at(2);
  //--left arm
  joint_group_positions_.at(3) = location.left_arm.at(0);
  joint_group_positions_.at(4) = location.left_arm.at(1);
  joint_group_positions_.at(5) = location.left_arm.at(2);
  joint_group_positions_.at(6) = location.left_arm.at(3);
  joint_group_positions_.at(7) = location.left_arm.at(4);
  joint_group_positions_.at(8) = location.left_arm.at(5);
  //--right arm
  joint_group_positions_.at(9) = location.right_arm.at(0);
  joint_group_positions_.at(10) = location.right_arm.at(1);
  joint_group_positions_.at(11) = location.right_arm.at(2);
  joint_group_positions_.at(12) = location.right_arm.at(3);
  joint_group_positions_.at(13) = location.right_arm.at(4);
  joint_group_positions_.at(14) = location.right_arm.at(5);

  full_robot_group_.setJointValueTarget(joint_group_positions_);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (full_robot_group_.plan(my_plan)
      == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if (success)
    full_robot_group_.move();
}

/**
 * @brief Activates the robot gripper
 * @param gripper_id
 * @return None
 */
void GantryControl::activateGripper(std::string arm_name) {
  nist_gear::VacuumGripperControl srv;
  srv.request.enable = true;

  if (arm_name == "left_arm") {
    left_gripper_control_client.call(srv);
  } else {
    right_gripper_control_client.call(srv);
  }
  ROS_INFO_STREAM(
      "[GantryControl][activateGripper] DEBUG: srv.response =" << srv.response);
}

/**
 * @brief Deactivates the robot gripper
 * @param gripper_id
 * @return None
 */
void GantryControl::deactivateGripper(std::string arm_name) {
  nist_gear::VacuumGripperControl srv;
  srv.request.enable = false;

  if (arm_name == "left_arm") {
    left_gripper_control_client.call(srv);
  } else {
    right_gripper_control_client.call(srv);
  }
  ROS_INFO_STREAM(
      "[GantryControl][deactivateGripper] DEBUG: srv.response ="
          << srv.response);
}

/**
 * @brief Function that returns the gripper attached status
 * @param arm_name
 * @return Returns the gripper attached status
 */
nist_gear::VacuumGripperState GantryControl::getGripperState(
    std::string arm_name) {
  if (arm_name == "left_arm") {
    return current_left_gripper_state_;
  } else {
    return current_right_gripper_state_;
  }
}

/**
 * @brief Callback for the left gripper state
 * @param msg
 */
void GantryControl::left_gripper_state_callback(
    const nist_gear::VacuumGripperState::ConstPtr &gripper_state_msg) {
  // ROS_INFO_STREAM_THROTTLE(10,
  //   "Gripper States (throttled to 0.1 Hz):\n" << *gripper_state_msg);
  current_left_gripper_state_ = *gripper_state_msg;
}

/**
 * @brief Callback for the right gripper state
 * @param msg
 */
void GantryControl::right_gripper_state_callback(
    const nist_gear::VacuumGripperState::ConstPtr &gripper_state_msg) {
  // ROS_INFO_STREAM_THROTTLE(10,
  //   "Gripper States (throttled to 0.1 Hz):\n" << *gripper_state_msg);
  current_right_gripper_state_ = *gripper_state_msg;
}

/**
 * @brief Callback for the different joint states of the robot
 * @param joint_state_msg
 */
void GantryControl::joint_states_callback(
    const sensor_msgs::JointState::ConstPtr &joint_state_msg) {
  if (joint_state_msg->position.size() == 0) {
    ROS_ERROR(
        "[gantry_control][joint_states_callback] msg->position.size() == 0!");
  }
  current_joint_states_ = *joint_state_msg;
}

/**
 * @brief Callback for the gantry's controller state
 * @param msg
 */
void GantryControl::gantry_controller_state_callback(
    const control_msgs::JointTrajectoryControllerState::ConstPtr &msg) {
  // ROS_INFO_STREAM_THROTTLE(10,
  //   "Gantry controller states (throttled to 0.1 Hz):\n" << *msg);
  current_gantry_controller_state_ = *msg;
}

/**
 * @brief Callback for the left arm's controller state
 * @param msg
 */
void GantryControl::left_arm_controller_state_callback(
    const control_msgs::JointTrajectoryControllerState::ConstPtr &msg) {
  // ROS_INFO_STREAM_THROTTLE(10,
  //   "Left arm controller states (throttled to 0.1 Hz):\n" << *msg);
  current_left_arm_controller_state_ = *msg;
}

/**
 * @brief Callback for the right arm's controller state
 * @param msg
 */
void GantryControl::right_arm_controller_state_callback(
    const control_msgs::JointTrajectoryControllerState::ConstPtr &msg) {
  // ROS_INFO_STREAM_THROTTLE(10,
  //   "Right arm controller states (throttled to 0.1 Hz):\n" << *msg);
  current_right_arm_controller_state_ = *msg;
}

/**
 * @brief Send command message to robot controller
 * @param command_msg
 * @return bool
 */
bool GantryControl::send_command(trajectory_msgs::JointTrajectory command_msg) {
  // ROS_INFO_STREAM("[gantry_control][send_command] called.");

  if (command_msg.points.size() == 0) {
    ROS_WARN(
        "[gantry_control][send_command] Trajectory is empty or NAN, returning.");
    return false;
  } else if ((command_msg.joint_names[0] == "small_long_joint")  // command is for gantry
      && (command_msg.points[0].positions[0]
          == command_msg.points[0].positions[0])) {

    gantry_joint_trajectory_publisher_.publish(command_msg);
    // ROS_INFO_STREAM("[gantry_control][send_command] gantry command published!");
    return true;
  } else if ((command_msg.joint_names[0].substr(0, 4) == "left")  // command is for left_arm
      && (command_msg.points[0].positions[0]
          == command_msg.points[0].positions[0])) {

    left_arm_joint_trajectory_publisher_.publish(command_msg);
    // ROS_INFO_STREAM("[gantry_control][send_command] left_arm command published!");
    return true;
  } else if ((command_msg.joint_names[0].substr(0, 5) == "right")  // command is for right arm
      && (command_msg.points[0].positions[0]
          == command_msg.points[0].positions[0])) {

    right_arm_joint_trajectory_publisher_.publish(command_msg);
    // ROS_INFO_STREAM("[gantry_control][send_command] right_arm command published!");
    return true;
  } else {
    return false;
  }
}

