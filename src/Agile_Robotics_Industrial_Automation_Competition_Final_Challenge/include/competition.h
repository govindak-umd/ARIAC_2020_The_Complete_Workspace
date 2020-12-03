/**
 * @file competition.cpp
 * @author Pradeep Gopal, Govind Ajith Kumar, Rajesh NS, Cheng, Dakota Abernathy
 * @copyright MIT License
 * @brief Header of the competition class
 * This class contains all the functions which are required to control all the sensors
 * in the environment and also to control the competition start and ending
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

#ifndef COMPETITION_H
#define COMPETITION_H
#include <vector>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <rosgraph_msgs/Clock.h>
#include <nist_gear/Order.h>
#include <nist_gear/LogicalCameraImage.h>
#include <nist_gear/Proximity.h>
#include <tf/transform_listener.h> 
#include <tf/LinearMath/Vector3.h> 
#include "utils.h"

/**
 * @brief      The class is the competition class
 * that has all the functionalities related to reading
 * sensor data, kitting, completion of ordee, checking the status
 * of the human being and the conveyor belt
 */
class Competition {

 public:
  /**
   * @brief      User-defined constructor
   *
   * @param      node  The node
   */
  explicit Competition(ros::NodeHandle &node);

  /**
   * @brief      Initializes the constructor with various parameters
   * and gets the code set before any execition by the robot.
   */
  void init();

  /**
   * @brief      Starts a competition.
   */
  void startCompetition();

  /**
   * @brief      Ends a competition.
   */
  void endCompetition();

  /**
   * @brief      Checks the status of the competition
   *
   * @param[in]  msg   The message
   */
  void competition_state_callback(const std_msgs::String::ConstPtr &msg);

  /**
   * @brief      Callback for the logical camera subscribers
   *
   * @param[in]  msg      The message
   * @param[in]  cam_idx  The camera index
   */
  void logical_camera_callback(
      const nist_gear::LogicalCameraImage::ConstPtr &msg, int cam_idx);

  /**
   * @brief      Callback for the first quality sensor camera
   * @param[in]  msg   The message
   */
  void quality_control_sensor_1_subscriber_callback(
      const nist_gear::LogicalCameraImage::ConstPtr &msg);

  /**
   * @brief      Callback for the second quality sensor camera
   * @param[in]  msg   The message
   */
  void quality_control_sensor_2_subscriber_callback(
      const nist_gear::LogicalCameraImage::ConstPtr &msg);

  /**
   * @brief      Master array that contains the parts from 
   * all the logical cameras
   *
   * @return     The parts from camera.
   */
  std::array<std::array<part, 20>, 20> get_parts_from_camera();

  /**
   * @brief      Gets the received order vector.
   *
   * @return     The received order vector.
   */
  std::vector<nist_gear::Order> get_received_order_vector();

  /**
   * @brief      Gets the quality sensor status agv 2.
   *
   * @return     The quality sensor status agv 2.
   */
  part get_quality_sensor_status_agv2();

  /**
   * @brief      Gets the quality sensor status agv 1.
   *
   * @return     The quality sensor status agv 1.
   */
  part get_quality_sensor_status_agv1();

  /**
   * @brief      Prints all the parts detected parts by every camera
   */
  void print_parts_detected();

  /**
   * @brief      Prints parts to be picked.
   */
  void print_parts_to_pick();

  /**
   * @brief      Executes the pre-kitting
   */
  void pre_kitting();

  /**
   * @brief      Function to be called during kitting for every tray
   */
  void during_kitting(part);

  /**
   * @brief      Callback function for competition callback 
   * subscribers
   *
   * @param[in]  msg   The Clock type ROS message
   */
  void competition_clock_callback(const rosgraph_msgs::Clock::ConstPtr &msg);

  /**
   * @brief      To subscribe to the order topic
   *
   * @param[in]  msg   The message
   */
  void order_callback(const nist_gear::Order::ConstPtr &msg);

  /**
   * @brief      Gets clock information
   *
   * @return     The time spent in the competition.
   */
  double getClock();

  /**
   * @brief      Gets the start time.
   *
   * @return     The start time.
   */
  double getStartTime();

  /**
   * @brief      Gets the competition state.
   *
   * @return     The competition state.
   */
  std::string getCompetitionState();

  /**
   * @brief      Gets information regarding comp.init().
   *
   * @return     The statistics.
   */
  stats getStats(std::string function);

  /**
   * @brief      Gets the status of deliver for every part in 
   * every shipment in every order
   * 
   * @param[in]  i     order
   * @param[in]  j    shipment
   * @param[in]  k    product
   */
  void setter_delivered(int i, int j, int k);

  /**
   * @brief      deletes an order when done
   */
  void delete_completed_order(int i);

  /**
   * returns the status of the parts on the conveyor belt
   */
  bool conveyor_belt_part_status = false;

  /**
   * @brief      Breakbeam sensor 0 callback
   *
   * @param[in]  msg   Sensor message
   */
  void breakbeam_sensor_0_callback(const nist_gear::Proximity::ConstPtr &msg);

  /**
   * @brief      Breakbeam sensor 1 callback
   *
   * @param[in]  msg   Sensor message
   */
  void breakbeam_sensor_1_callback(const nist_gear::Proximity::ConstPtr &msg);

  /**
   * @brief      Breakbeam sensor 11 callback
   *
   * @param[in]  msg   Sensor message
   */
  void breakbeam_sensor_11_callback(const nist_gear::Proximity::ConstPtr &msg);

  /**
   * @brief      Breakbeam sensor 12 callback
   *
   * @param[in]  msg   Sensor message
   */
  void breakbeam_sensor_12_callback(const nist_gear::Proximity::ConstPtr &msg);

  /**
   * @brief      Breakbeam sensor 13 callback
   *
   * @param[in]  msg   Sensor message
   */
  void breakbeam_sensor_13_callback(const nist_gear::Proximity::ConstPtr &msg);

  /**
   * @brief      Breakbeam sensor 14 callback
   *
   * @param[in]  msg   Sensor message
   */
  void breakbeam_sensor_14_callback(const nist_gear::Proximity::ConstPtr &msg);

  /**
   * @brief      Breakbeam sensor 15 callback
   *
   * @param[in]  msg   Sensor message
   */
  void breakbeam_sensor_15_callback(const nist_gear::Proximity::ConstPtr &msg);

  /**
   * @brief      Breakbeam sensor 16 callback
   *
   * @param[in]  msg   Sensor message
   */
  void breakbeam_sensor_16_callback(const nist_gear::Proximity::ConstPtr &msg);

  /**
   * @brief      Breakbeam sensor 21 callback
   *
   * @param[in]  msg   Sensor message
   */
  void breakbeam_sensor_21_callback(const nist_gear::Proximity::ConstPtr &msg);

  /**
   * @brief      Breakbeam sensor 22 callback
   *
   * @param[in]  msg   Sensor message
   */
  void breakbeam_sensor_22_callback(const nist_gear::Proximity::ConstPtr &msg);

  /**
   * @brief      Breakbeam sensor 23 callback
   *
   * @param[in]  msg   Sensor message
   */
  void breakbeam_sensor_23_callback(const nist_gear::Proximity::ConstPtr &msg);

  /**
   * @brief      Breakbeam sensor 24 callback
   *
   * @param[in]  msg   Sensor message
   */
  void breakbeam_sensor_24_callback(const nist_gear::Proximity::ConstPtr &msg);

  /**
   * @brief      Breakbeam sensor 25 callback
   *
   * @param[in]  msg   Sensor message
   */
  void breakbeam_sensor_25_callback(const nist_gear::Proximity::ConstPtr &msg);

  /**
   * @brief      Breakbeam sensor 26 callback
   *
   * @param[in]  msg   Sensor message
   */
  void breakbeam_sensor_26_callback(const nist_gear::Proximity::ConstPtr &msg);

  /**
   * @brief      Breakbeam sensor 31 callback
   *
   * @param[in]  msg   Sensor message
   */
  void breakbeam_sensor_31_callback(const nist_gear::Proximity::ConstPtr &msg);

  /**
   * @brief      Breakbeam sensor 32 callback
   *
   * @param[in]  msg   Sensor message
   */
  void breakbeam_sensor_32_callback(const nist_gear::Proximity::ConstPtr &msg);

  /**
   * @brief      Breakbeam sensor 33 callback
   *
   * @param[in]  msg   Sensor message
   */
  void breakbeam_sensor_33_callback(const nist_gear::Proximity::ConstPtr &msg);

  /**
   * @brief      Breakbeam sensor 34 callback
   *
   * @param[in]  msg   Sensor message
   */
  void breakbeam_sensor_34_callback(const nist_gear::Proximity::ConstPtr &msg);

  /**
   * @brief      Breakbeam sensor 35 callback
   *
   * @param[in]  msg   Sensor message
   */
  void breakbeam_sensor_35_callback(const nist_gear::Proximity::ConstPtr &msg);

  /**
   * @brief      Breakbeam sensor 36 callback
   *
   * @param[in]  msg   Sensor message
   */
  void breakbeam_sensor_36_callback(const nist_gear::Proximity::ConstPtr &msg);

  /**
   * @brief      Breakbeam sensor 41 callback
   *
   * @param[in]  msg   Sensor message
   */
  void breakbeam_sensor_41_callback(const nist_gear::Proximity::ConstPtr &msg);

  /**
   * @brief      Breakbeam sensor 42 callback
   *
   * @param[in]  msg   Sensor message
   */
  void breakbeam_sensor_42_callback(const nist_gear::Proximity::ConstPtr &msg);

  /**
   * @brief      Breakbeam sensor 43 callback
   *
   * @param[in]  msg   Sensor message
   */
  void breakbeam_sensor_43_callback(const nist_gear::Proximity::ConstPtr &msg);

  /**
   * @brief      Breakbeam sensor 44 callback
   *
   * @param[in]  msg   Sensor message
   */
  void breakbeam_sensor_44_callback(const nist_gear::Proximity::ConstPtr &msg);

  /**
   * @brief      Breakbeam sensor 45 callback
   *
   * @param[in]  msg   Sensor message
   */
  void breakbeam_sensor_45_callback(const nist_gear::Proximity::ConstPtr &msg);

  /**
   * @brief      Breakbeam sensor 46 callback
   *
   * @param[in]  msg   Sensor message
   */
  void breakbeam_sensor_46_callback(const nist_gear::Proximity::ConstPtr &msg);

  /**
   * Boolean to check the conveyor belt part status for Breakbeam-0
   */
  bool breakbeam_conveyor_belt_part_status_0 = false;

  /**
   * Boolean to check the conveyor belt part status for Breakbeam-1
   */
  bool breakbeam_conveyor_belt_part_status_1 = false;

  /**
   * Boolean to check the part detection by 
   * Breakbeam-11
   */
  bool breakbeam_part_status_11 = false;

  /**
   * Boolean to check the part detection by 
   * Breakbeam-12
   */
  bool breakbeam_part_status_12 = false;

  /**
   * Boolean to check the part detection by 
   * Breakbeam-13
   */
  bool breakbeam_part_status_13 = false;

  /**
   * Boolean to check the part detection by 
   * Breakbeam-14
   */
  bool breakbeam_part_status_14 = false;

  /**
   * Boolean to check the part detection by 
   * Breakbeam-15
   */
  bool breakbeam_part_status_15 = false;

  /**
   * Boolean to check the part detection by 
   * Breakbeam-16
   */
  bool breakbeam_part_status_16 = false;

  /**
   * Boolean to check the part detection by 
   * Breakbeam-21
   */
  bool breakbeam_part_status_21 = false;

  /**
   * Boolean to check the part detection by 
   * Breakbeam-22
   */
  bool breakbeam_part_status_22 = false;

  /**
   * Boolean to check the part detection by 
   * Breakbeam-23
   */
  bool breakbeam_part_status_23 = false;

  /**
   * Boolean to check the part detection by 
   * Breakbeam-24
   */
  bool breakbeam_part_status_24 = false;

  /**
   * Boolean to check the part detection by 
   * Breakbeam-25
   */
  bool breakbeam_part_status_25 = false;

  /**
   * Boolean to check the part detection by 
   * Breakbeam-26
   */
  bool breakbeam_part_status_26 = false;

  /**
   * Boolean to check the part detection by 
   * Breakbeam-31
   */
  bool breakbeam_part_status_31 = false;

  /**
   * Boolean to check the part detection by 
   * Breakbeam-32
   */
  bool breakbeam_part_status_32 = false;

  /**
   * Boolean to check the part detection by 
   * Breakbeam-33
   */
  bool breakbeam_part_status_33 = false;

  /**
   * Boolean to check the part detection by 
   * Breakbeam-34
   */
  bool breakbeam_part_status_34 = false;

  /**
   * Boolean to check the part detection by 
   * Breakbeam-35
   */
  bool breakbeam_part_status_35 = false;

  /**
   * Boolean to check the part detection by 
   * Breakbeam-36
   */
  bool breakbeam_part_status_36 = false;

  /**
   * Boolean to check the part detection by 
   * Breakbeam-41
   */
  bool breakbeam_part_status_41 = false;

  /**
   * Boolean to check the part detection by 
   * Breakbeam-42
   */
  bool breakbeam_part_status_42 = false;

  /**
   * Boolean to check the part detection by 
   * Breakbeam-43
   */
  bool breakbeam_part_status_43 = false;

  /**
   * Boolean to check the part detection by 
   * Breakbeam-44
   */
  bool breakbeam_part_status_44 = false;

  /**
   * Boolean to check the part detection by 
   * Breakbeam-45
   */
  bool breakbeam_part_status_45 = false;

  /**
   * Boolean to check the part detection by 
   * Breakbeam-46
   */
  bool breakbeam_part_status_46 = false;

  /**
   * @brief      Gets the existence of a human in Aisle-1
   *
   * @return     The human  existence.
   */
  int get_human_1_existence();

  /**
   * @brief      Gets the existence of a human in Aisle-2
   *
   * @return     The human 2 existence.
   */
  int get_human_2_existence();

  /**
   * @brief      Gets the existence of a human in Aisle-3
   *
   * @return     The human 3 existence.
   */
  int get_human_3_existence();

  /**
   * @brief      Gets the existence of a human in Aisle-4
   *
   * @return     The human 4 existence.
   */
  int get_human_4_existence();

  /**
   * to check if a human was detected in Aisle-1
   */
  int human_1_detected = 0;

  /**
   * to check if a human was detected in Aisle-2
   */
  int human_2_detected = 0;

  /**
   * to check if a human was detected in Aisle-3
   */
  int human_3_detected = 0;

  /**
   * to check if a human was detected in Aisle-4
   */
  int human_4_detected = 0;

  /**
   * Vector to store the parts from camera 15
   */
  std::vector<part> parts_from_15_camera;

  /**
   * @brief      Gets the parts from camera-15.
   *
   * @return     The parts from camera-15.
   */
  std::vector<part> get_parts_from_15_camera();

  /**
   * @brief      Gets the parts from camera-16.
   *
   * @return     The parts from camera-16.
   */
  std::array<part, 20> get_parts_from_16_camera();

  /**
   * @brief      Gets the parts from camera-17.
   *
   * @return     The parts from camera-17.
   */
  std::array<part, 20> get_parts_from_17_camera();

  /**
   * Array to store parts from camera-11
   */
  std::array<part, 20> parts_from_11_camera;

  /**
   * Array to store parts from camera-14
   */
  std::array<part, 20> parts_from_14_camera;

  /**
   * Array to store parts from camera-11
   */
  std::array<part, 20> parts_from_12_camera;

  /**
   * Array to store parts from camera-11
   */
  std::array<part, 20> parts_from_13_camera;

  /**
   * Array to store parts from camera-16
   */
  std::array<part, 20> parts_from_16_camera;

  /**
   * Array to store parts from camera-17
   */
  std::array<part, 20> parts_from_17_camera;

  /**
   * @brief      Master vector that keeps track of every order, shipment
   * and product.
   *
   * @return     The master vector.
   */
  std::vector<std::vector<std::vector<master_struct> > > get_master_vector();

  /**
   * Stores the number of parts detected by logical camera 11
   */
  int parts_in_logical_camera_11;

  /**
   * Stores the number of parts detected by logical camera 12
   */
  int parts_in_logical_camera_12;

  /**
   * Stores the number of parts detected by logical camera 13
   */
  int parts_in_logical_camera_13;

  /**
   * Stores the number of parts detected by logical camera 13
   */
  int parts_in_logical_camera_14;

 private:
  /**
   * ROS Node handle
   */
  ros::NodeHandle node_;

  /**
   * Gets the competition state
   */
  std::string competition_state_;

  /**
   * Keeps track of the current score
   */
  double current_score_;

  /**
   * ROS Clock
   */
  ros::Time competition_clock_;

  /**
   * Starting time of the competition in seconds
   */
  double competition_start_time_;

  /**
   * Subscribes to the current score
   */
  ros::Subscriber current_score_subscriber_;

  /**
   * Subscribes to the competition state
   */
  ros::Subscriber competition_state_subscriber_;

  /**
   * Subscribes to the competition clock
   */
  ros::Subscriber competition_clock_subscriber_;

  /**
   * Subscribes to the orders
   */
  ros::Subscriber orders_subscriber_;

  /**
   * Subscribes to the Quality sensor-1
   */
  ros::Subscriber quality_control_sensor_1_subscriber_;

  /**
   * Subscribes to the Quality sensor-2
   */
  ros::Subscriber quality_control_sensor_2_subscriber_;

  /**
   * Vector of received orders
   */
  std::vector<nist_gear::Order> received_orders_;

  /**
   * Subscribes to the Breakbeam Sensor-0 feedback
   */
  ros::Subscriber breakbeam_sensor_0_subscriber_;

  /**
   * Subscribes to the Breakbeam Sensor-1 feedback
   */
  ros::Subscriber breakbeam_sensor_1_subscriber_;

  /**
   * Subscribes to the Breakbeam Sensor-11 feedback
   */
  ros::Subscriber breakbeam_sensor_11_subscriber_;

  /**
   * Subscribes to the Breakbeam Sensor-12 feedback
   */
  ros::Subscriber breakbeam_sensor_12_subscriber_;

  /**
   * Subscribes to the Breakbeam Sensor-13 feedback
   */
  ros::Subscriber breakbeam_sensor_13_subscriber_;

  /**
   * Subscribes to the Breakbeam Sensor-14 feedback
   */
  ros::Subscriber breakbeam_sensor_14_subscriber_;

  /**
   * Subscribes to the Breakbeam Sensor-15 feedback
   */
  ros::Subscriber breakbeam_sensor_15_subscriber_;

  /**
   * Subscribes to the Breakbeam Sensor-16 feedback
   */
  ros::Subscriber breakbeam_sensor_16_subscriber_;

  /**
   * Subscribes to the Breakbeam Sensor-21 feedback
   */
  ros::Subscriber breakbeam_sensor_21_subscriber_;

  /**
   * Subscribes to the Breakbeam Sensor-22 feedback
   */
  ros::Subscriber breakbeam_sensor_22_subscriber_;

  /**
   * Subscribes to the Breakbeam Sensor-23 feedback
   */
  ros::Subscriber breakbeam_sensor_23_subscriber_;

  /**
   * Subscribes to the Breakbeam Sensor-24 feedback
   */
  ros::Subscriber breakbeam_sensor_24_subscriber_;

  /**
   * Subscribes to the Breakbeam Sensor-25 feedback
   */
  ros::Subscriber breakbeam_sensor_25_subscriber_;

  /**
   * Subscribes to the Breakbeam Sensor-26 feedback
   */
  ros::Subscriber breakbeam_sensor_26_subscriber_;

  /**
   * Subscribes to the Breakbeam Sensor-31 feedback
   */
  ros::Subscriber breakbeam_sensor_31_subscriber_;

  /**
   * Subscribes to the Breakbeam Sensor-32 feedback
   */
  ros::Subscriber breakbeam_sensor_32_subscriber_;

  /**
   * Subscribes to the Breakbeam Sensor-33 feedback
   */
  ros::Subscriber breakbeam_sensor_33_subscriber_;

  /**
   * Subscribes to the Breakbeam Sensor-34 feedback
   */
  ros::Subscriber breakbeam_sensor_34_subscriber_;

  /**
   * Subscribes to the Breakbeam Sensor-35 feedback
   */
  ros::Subscriber breakbeam_sensor_35_subscriber_;

  /**
   * Subscribes to the Breakbeam Sensor-36 feedback
   */
  ros::Subscriber breakbeam_sensor_36_subscriber_;

  /**
   * Subscribes to the Breakbeam Sensor-41 feedback
   */
  ros::Subscriber breakbeam_sensor_41_subscriber_;

  /**
   * Subscribes to the Breakbeam Sensor-42 feedback
   */
  ros::Subscriber breakbeam_sensor_42_subscriber_;

  /**
   * Subscribes to the Breakbeam Sensor-43 feedback
   */
  ros::Subscriber breakbeam_sensor_43_subscriber_;

  /**
   * Subscribes to the Breakbeam Sensor-44 feedback
   */
  ros::Subscriber breakbeam_sensor_44_subscriber_;

  /**
   * Subscribes to the Breakbeam Sensor-45 feedback
   */
  ros::Subscriber breakbeam_sensor_45_subscriber_;

  /**
   * Subscribes to the Breakbeam Sensor-46 feedback
   */
  ros::Subscriber breakbeam_sensor_46_subscriber_;

  /**
   * to Collects statistics
   */
  stats init_;
};

#endif
