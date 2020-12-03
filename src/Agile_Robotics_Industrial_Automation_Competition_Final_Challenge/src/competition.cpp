/**
 * @file competition.cpp
 * @author Pradeep Gopal, Govind Ajith Kumar, Rajesh NS, Cheng, Dakota Abernathy
 * @copyright MIT License
 * @brief Implementation of the competition class
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

#include "competition.h"
#include "utils.h"
#include <nist_gear/LogicalCameraImage.h>
#include <nist_gear/Proximity.h>
#include <nist_gear/Order.h>
#include <std_srvs/Trigger.h>
#include "gantry_control.h"
#include <string>
#include <vector>

/**
 * General iterator variables
 */
int p = 0, camera_no = 0;

/**
 * Faulty_parts on agv2 declaration as an object of part struct
 */
part faulty_part_agv2;

/**
 * Faulty_parts on agv1 declaration as an object of part struct
 */
part faulty_part_agv1;

/**
 * Vector of orders and order type
 */
std::vector<order> orders_vector;

/**
 * Vector of shipments and shipment type
 */
std::vector<shipment> shipment_vector;

/**
 * Vector of products and product type
 */
std::vector<product> product_vector;

/**
 * Vector to record the pick and place of objects by the gantry
 */
std::vector<pick_and_place> pick_and_place_poses_vector;

/**
 * 2 dimensional vector for the parts from the camera
 */
std::array<std::array<part, 20>, 20> parts_from_camera;

/**
 * @brief      Main 3 dimensional vector for all products in
 * all the shipments in every order
 */
std::vector<std::vector<std::vector<master_struct> > > master_vector(
    10,
    std::vector < std::vector<master_struct>
        > (10, std::vector < master_struct > (20)));
Competition::Competition(ros::NodeHandle &node)
    :
    current_score_(0) {
  node_ = node;
}

/**
 * @brief      Initializes the competition function, subscribers and callbacks
 */
void Competition::init() {
  // Subscribe to the '/ariac/current_score' topic.
  double time_called = ros::Time::now().toSec();
  competition_start_time_ = ros::Time::now().toSec();

  // Subscribe to the '/ariac/competition_state' topic.
  ROS_INFO("Subscribe to the /ariac/competition_state topic...");
  competition_state_subscriber_ = node_.subscribe(
      "/ariac/competition_state", 10, &Competition::competition_state_callback,
      this);

  // Subscribe to the '/clock' topic.
  ROS_INFO("Subscribe to the /clock...");
  competition_clock_subscriber_ = node_.subscribe(
      "/clock", 10, &Competition::competition_clock_callback, this);

  ROS_INFO("Subscribe to the /orders...");
  orders_subscriber_ = node_.subscribe("/ariac/orders", 10,
                                       &Competition::order_callback, this);

  ROS_INFO("Subscribe to the /ariac/quality_control_sensor_1");  //AGV2
  quality_control_sensor_1_subscriber_ = node_.subscribe(
      "/ariac/quality_control_sensor_1", 10,
      &Competition::quality_control_sensor_1_subscriber_callback, this);

  ROS_INFO("Subscribe to the /ariac/quality_control_sensor_2");  //AGV1
  quality_control_sensor_2_subscriber_ = node_.subscribe(
      "/ariac/quality_control_sensor_2", 10,
      &Competition::quality_control_sensor_2_subscriber_callback, this);

  ROS_INFO("Subscribe to the /ariac/breakbeam_0");
  breakbeam_sensor_0_subscriber_ = node_.subscribe(
      "/ariac/breakbeam_0", 10, &Competition::breakbeam_sensor_0_callback,
      this);
  ROS_INFO("Subscribe to the /ariac/breakbeam_1");
  breakbeam_sensor_1_subscriber_ = node_.subscribe(
      "/ariac/breakbeam_1", 10, &Competition::breakbeam_sensor_1_callback,
      this);

  ROS_INFO("Subscribe to the /ariac/breakbeam_11");
  breakbeam_sensor_11_subscriber_ = node_.subscribe(
      "/ariac/breakbeam_11", 10, &Competition::breakbeam_sensor_11_callback,
      this);
  ROS_INFO("Subscribe to the /ariac/breakbeam_12");
  breakbeam_sensor_12_subscriber_ = node_.subscribe(
      "/ariac/breakbeam_12", 10, &Competition::breakbeam_sensor_12_callback,
      this);
  ROS_INFO("Subscribe to the /ariac/breakbeam_13");
  breakbeam_sensor_13_subscriber_ = node_.subscribe(
      "/ariac/breakbeam_13", 10, &Competition::breakbeam_sensor_13_callback,
      this);
  ROS_INFO("Subscribe to the /ariac/breakbeam_14");
  breakbeam_sensor_14_subscriber_ = node_.subscribe(
      "/ariac/breakbeam_14", 10, &Competition::breakbeam_sensor_14_callback,
      this);
  ROS_INFO("Subscribe to the /ariac/breakbeam_15");
  breakbeam_sensor_15_subscriber_ = node_.subscribe(
      "/ariac/breakbeam_15", 10, &Competition::breakbeam_sensor_15_callback,
      this);
  ROS_INFO("Subscribe to the /ariac/breakbeam_16");
  breakbeam_sensor_16_subscriber_ = node_.subscribe(
      "/ariac/breakbeam_16", 10, &Competition::breakbeam_sensor_16_callback,
      this);

  ROS_INFO("Subscribe to the /ariac/breakbeam_21");
  breakbeam_sensor_21_subscriber_ = node_.subscribe(
      "/ariac/breakbeam_21", 10, &Competition::breakbeam_sensor_21_callback,
      this);
  ROS_INFO("Subscribe to the /ariac/breakbeam_22");
  breakbeam_sensor_22_subscriber_ = node_.subscribe(
      "/ariac/breakbeam_22", 10, &Competition::breakbeam_sensor_22_callback,
      this);
  ROS_INFO("Subscribe to the /ariac/breakbeam_23");
  breakbeam_sensor_23_subscriber_ = node_.subscribe(
      "/ariac/breakbeam_23", 10, &Competition::breakbeam_sensor_23_callback,
      this);
  ROS_INFO("Subscribe to the /ariac/breakbeam_24");
  breakbeam_sensor_24_subscriber_ = node_.subscribe(
      "/ariac/breakbeam_24", 10, &Competition::breakbeam_sensor_24_callback,
      this);
  ROS_INFO("Subscribe to the /ariac/breakbeam_25");
  breakbeam_sensor_25_subscriber_ = node_.subscribe(
      "/ariac/breakbeam_25", 10, &Competition::breakbeam_sensor_25_callback,
      this);
  ROS_INFO("Subscribe to the /ariac/breakbeam_26");
  breakbeam_sensor_26_subscriber_ = node_.subscribe(
      "/ariac/breakbeam_26", 10, &Competition::breakbeam_sensor_26_callback,
      this);

  ROS_INFO("Subscribe to the /ariac/breakbeam_31");
  breakbeam_sensor_31_subscriber_ = node_.subscribe(
      "/ariac/breakbeam_31", 10, &Competition::breakbeam_sensor_31_callback,
      this);
  ROS_INFO("Subscribe to the /ariac/breakbeam_32");
  breakbeam_sensor_32_subscriber_ = node_.subscribe(
      "/ariac/breakbeam_32", 10, &Competition::breakbeam_sensor_32_callback,
      this);
  ROS_INFO("Subscribe to the /ariac/breakbeam_33");
  breakbeam_sensor_33_subscriber_ = node_.subscribe(
      "/ariac/breakbeam_33", 10, &Competition::breakbeam_sensor_33_callback,
      this);
  ROS_INFO("Subscribe to the /ariac/breakbeam_34");
  breakbeam_sensor_34_subscriber_ = node_.subscribe(
      "/ariac/breakbeam_34", 10, &Competition::breakbeam_sensor_34_callback,
      this);
  ROS_INFO("Subscribe to the /ariac/breakbeam_35");
  breakbeam_sensor_35_subscriber_ = node_.subscribe(
      "/ariac/breakbeam_35", 10, &Competition::breakbeam_sensor_35_callback,
      this);
  ROS_INFO("Subscribe to the /ariac/breakbeam_36");
  breakbeam_sensor_36_subscriber_ = node_.subscribe(
      "/ariac/breakbeam_36", 10, &Competition::breakbeam_sensor_36_callback,
      this);

  ROS_INFO("Subscribe to the /ariac/breakbeam_41");
  breakbeam_sensor_41_subscriber_ = node_.subscribe(
      "/ariac/breakbeam_41", 10, &Competition::breakbeam_sensor_41_callback,
      this);
  ROS_INFO("Subscribe to the /ariac/breakbeam_42");
  breakbeam_sensor_42_subscriber_ = node_.subscribe(
      "/ariac/breakbeam_42", 10, &Competition::breakbeam_sensor_42_callback,
      this);
  ROS_INFO("Subscribe to the /ariac/breakbeam_43");
  breakbeam_sensor_43_subscriber_ = node_.subscribe(
      "/ariac/breakbeam_43", 10, &Competition::breakbeam_sensor_43_callback,
      this);
  ROS_INFO("Subscribe to the /ariac/breakbeam_44");
  breakbeam_sensor_44_subscriber_ = node_.subscribe(
      "/ariac/breakbeam_44", 10, &Competition::breakbeam_sensor_44_callback,
      this);
  ROS_INFO("Subscribe to the /ariac/breakbeam_45");
  breakbeam_sensor_45_subscriber_ = node_.subscribe(
      "/ariac/breakbeam_45", 10, &Competition::breakbeam_sensor_45_callback,
      this);
  ROS_INFO("Subscribe to the /ariac/breakbeam_46");
  breakbeam_sensor_46_subscriber_ = node_.subscribe(
      "/ariac/breakbeam_46", 10, &Competition::breakbeam_sensor_46_callback,
      this);

  startCompetition();

  init_.total_time += ros::Time::now().toSec() - time_called;

}

/**
 * @brief      Callback from breakbeam-0
 *
 * @param[in]  msg   Proximity sensor message
 */
void Competition::breakbeam_sensor_0_callback(
    const nist_gear::Proximity::ConstPtr &msg) {
  breakbeam_conveyor_belt_part_status_0 = msg->object_detected;
}

/**
 * @brief      Callback from breakbeam-1
 *
 * @param[in]  msg   Proximity sensor message
 */
void Competition::breakbeam_sensor_1_callback(
    const nist_gear::Proximity::ConstPtr &msg) {
  breakbeam_conveyor_belt_part_status_1 = msg->object_detected;
}

/**
 * @brief      Callback from breakbeam-11
 *
 * @param[in]  msg   Proximity sensor message
 */
void Competition::breakbeam_sensor_11_callback(
    const nist_gear::Proximity::ConstPtr &msg) {
  breakbeam_part_status_11 = msg->object_detected;
  if (breakbeam_part_status_11 == true) {
    human_1_detected = 1;
  }
}

/**
 * @brief      Callback from breakbeam-12
 *
 * @param[in]  msg   Proximity sensor message
 */
void Competition::breakbeam_sensor_12_callback(
    const nist_gear::Proximity::ConstPtr &msg) {
  breakbeam_part_status_12 = msg->object_detected;
  if (breakbeam_part_status_12 == true) {
    human_1_detected = 1;
  }
}

/**
 * @brief      Callback from breakbeam-13
 *
 * @param[in]  msg   Proximity sensor message
 */
void Competition::breakbeam_sensor_13_callback(
    const nist_gear::Proximity::ConstPtr &msg) {
  breakbeam_part_status_13 = msg->object_detected;
  if (breakbeam_part_status_13 == true) {
    human_1_detected = 1;
  }
}

/**
 * @brief      Callback from breakbeam-14
 *
 * @param[in]  msg   Proximity sensor message
 */
void Competition::breakbeam_sensor_14_callback(
    const nist_gear::Proximity::ConstPtr &msg) {
  breakbeam_part_status_14 = msg->object_detected;
  if (breakbeam_part_status_14 == true) {
    human_1_detected = 1;
  }
}

/**
 * @brief      Callback from breakbeam-15
 *
 * @param[in]  msg   Proximity sensor message
 */
void Competition::breakbeam_sensor_15_callback(
    const nist_gear::Proximity::ConstPtr &msg) {
  breakbeam_part_status_15 = msg->object_detected;
  if (breakbeam_part_status_15 == true) {
    human_1_detected = 1;
  }
}

/**
 * @brief      Callback from breakbeam-16
 *
 * @param[in]  msg   Proximity sensor message
 */
void Competition::breakbeam_sensor_16_callback(
    const nist_gear::Proximity::ConstPtr &msg) {
  breakbeam_part_status_16 = msg->object_detected;
  if (breakbeam_part_status_16 == true) {
    human_1_detected = 1;
  }
}

/**
 * @brief      Callback from breakbeam-21
 *
 * @param[in]  msg   Proximity sensor message
 */
void Competition::breakbeam_sensor_21_callback(
    const nist_gear::Proximity::ConstPtr &msg) {
  breakbeam_part_status_21 = msg->object_detected;
  if (breakbeam_part_status_21 == true) {
    human_2_detected = 1;
  }
}

/**
 * @brief      Callback from breakbeam-22
 *
 * @param[in]  msg   Proximity sensor message
 */
void Competition::breakbeam_sensor_22_callback(
    const nist_gear::Proximity::ConstPtr &msg) {
  breakbeam_part_status_22 = msg->object_detected;
  if (breakbeam_part_status_22 == true) {
    human_2_detected = 1;
  }
}

/**
 * @brief      Callback from breakbeam-23
 *
 * @param[in]  msg   Proximity sensor message
 */
void Competition::breakbeam_sensor_23_callback(
    const nist_gear::Proximity::ConstPtr &msg) {
  breakbeam_part_status_23 = msg->object_detected;
  if (breakbeam_part_status_23 == true) {
    human_2_detected = 1;
  }
}

/**
 * @brief      Callback from breakbeam-24
 *
 * @param[in]  msg   Proximity sensor message
 */
void Competition::breakbeam_sensor_24_callback(
    const nist_gear::Proximity::ConstPtr &msg) {
  breakbeam_part_status_24 = msg->object_detected;
  if (breakbeam_part_status_24 == true) {
    human_2_detected = 1;
  }
}

/**
 * @brief      Callback from breakbeam-25
 *
 * @param[in]  msg   Proximity sensor message
 */
void Competition::breakbeam_sensor_25_callback(
    const nist_gear::Proximity::ConstPtr &msg) {
  breakbeam_part_status_25 = msg->object_detected;
  if (breakbeam_part_status_25 == true) {
    human_2_detected = 1;
  }
}

/**
 * @brief      Callback from breakbeam-26
 *
 * @param[in]  msg   Proximity sensor message
 */
void Competition::breakbeam_sensor_26_callback(
    const nist_gear::Proximity::ConstPtr &msg) {
  breakbeam_part_status_26 = msg->object_detected;
  if (breakbeam_part_status_26 == true) {
    human_2_detected = 1;
  }
}

/**
 * @brief      Callback from breakbeam-31
 *
 * @param[in]  msg   Proximity sensor message
 */
void Competition::breakbeam_sensor_31_callback(
    const nist_gear::Proximity::ConstPtr &msg) {
  breakbeam_part_status_31 = msg->object_detected;
  if (breakbeam_part_status_31 == true) {
    human_3_detected = 1;
  }
}

/**
 * @brief      Callback from breakbeam-32
 *
 * @param[in]  msg   Proximity sensor message
 */
void Competition::breakbeam_sensor_32_callback(
    const nist_gear::Proximity::ConstPtr &msg) {
  breakbeam_part_status_32 = msg->object_detected;
  if (breakbeam_part_status_32 == true) {
    human_3_detected = 1;
  }
}

/**
 * @brief      Callback from breakbeam-33
 *
 * @param[in]  msg   Proximity sensor message
 */
void Competition::breakbeam_sensor_33_callback(
    const nist_gear::Proximity::ConstPtr &msg) {
  breakbeam_part_status_33 = msg->object_detected;
  if (breakbeam_part_status_33 == true) {
    human_3_detected = 1;
  }
}

/**
 * @brief      Callback from breakbeam-34
 *
 * @param[in]  msg   Proximity sensor message
 */
void Competition::breakbeam_sensor_34_callback(
    const nist_gear::Proximity::ConstPtr &msg) {
  breakbeam_part_status_34 = msg->object_detected;
  if (breakbeam_part_status_34 == true) {
    human_3_detected = 1;
  }
}

/**
 * @brief      Callback from breakbeam-35
 *
 * @param[in]  msg   Proximity sensor message
 */
void Competition::breakbeam_sensor_35_callback(
    const nist_gear::Proximity::ConstPtr &msg) {
  breakbeam_part_status_35 = msg->object_detected;
  if (breakbeam_part_status_35 == true) {
    human_3_detected = 1;
  }
}

/**
 * @brief      Callback from breakbeam-36
 *
 * @param[in]  msg   Proximity sensor message
 */
void Competition::breakbeam_sensor_36_callback(
    const nist_gear::Proximity::ConstPtr &msg) {
  breakbeam_part_status_36 = msg->object_detected;
  if (breakbeam_part_status_36 == true) {
    human_3_detected = 1;
  }
}

/**
 * @brief      Callback from breakbeam-41
 *
 * @param[in]  msg   Proximity sensor message
 */
void Competition::breakbeam_sensor_41_callback(
    const nist_gear::Proximity::ConstPtr &msg) {
  breakbeam_part_status_41 = msg->object_detected;
  if (breakbeam_part_status_41 == true) {
    human_4_detected = 1;
  }
}

/**
 * @brief      Callback from breakbeam-42
 *
 * @param[in]  msg   Proximity sensor message
 */
void Competition::breakbeam_sensor_42_callback(
    const nist_gear::Proximity::ConstPtr &msg) {
  breakbeam_part_status_42 = msg->object_detected;
  if (breakbeam_part_status_42 == true) {
    human_4_detected = 1;
  }
}

/**
 * @brief      Callback from breakbeam-43
 *
 * @param[in]  msg   Proximity sensor message
 */
void Competition::breakbeam_sensor_43_callback(
    const nist_gear::Proximity::ConstPtr &msg) {
  breakbeam_part_status_43 = msg->object_detected;
  if (breakbeam_part_status_43 == true) {
    human_4_detected = 1;
  }
}

/**
 * @brief      Callback from breakbeam-44
 *
 * @param[in]  msg   Proximity sensor message
 */
void Competition::breakbeam_sensor_44_callback(
    const nist_gear::Proximity::ConstPtr &msg) {
  breakbeam_part_status_44 = msg->object_detected;
  if (breakbeam_part_status_44 == true) {
    human_4_detected = 1;
  }
}

/**
 * @brief      Callback from breakbeam-45
 *
 * @param[in]  msg   Proximity sensor message
 */
void Competition::breakbeam_sensor_45_callback(
    const nist_gear::Proximity::ConstPtr &msg) {
  breakbeam_part_status_45 = msg->object_detected;
  if (breakbeam_part_status_45 == true) {
    human_4_detected = 1;
  }
}

/**
 * @brief      Callback from breakbeam-46
 *
 * @param[in]  msg   Proximity sensor message
 */
void Competition::breakbeam_sensor_46_callback(
    const nist_gear::Proximity::ConstPtr &msg) {
  breakbeam_part_status_46 = msg->object_detected;
  if (breakbeam_part_status_46 == true) {
    human_4_detected = 1;
  }
}

/**
 * @brief      Gets the existence of a human in Aisle-1
 *
 * @return     The human  existence.
 */
int Competition::get_human_1_existence() {
  return human_1_detected;
}

/**
 * @brief      Gets the existence of a human in Aisle-2
 *
 * @return     The human 2 existence.
 */
int Competition::get_human_2_existence() {
  return human_2_detected;
}

/**
 * @brief      Gets the existence of a human in Aisle-3
 *
 * @return     The human 3 existence.
 */
int Competition::get_human_3_existence() {
  return human_3_detected;
}

/**
 * @brief      Gets the existence of a human in Aisle-4
 *
 * @return     The human 4 existence.
 */
int Competition::get_human_4_existence() {
  return human_4_detected;
}

/**
 * @brief      Gets the status of deliver for every part in 
 * every shipment in every order
 * 
 * @param[in]  i     order
 * @param[in]  j    shipment
 * @param[in]  k    product
 */
void Competition::setter_delivered(int i, int j, int k) {
  master_vector[i][j][k].delivered = true;
}

/**
 * @brief      Prints all the parts detected parts by every camera
 */
void Competition::print_parts_detected() {
  for (int i = 0; i < parts_from_camera.size(); i++) {
    std::cout << "parts from camera = " << i << std::endl;
    std::cout << std::endl;
    for (int j = 0; j < parts_from_camera[i].size(); j++) {
      std::cout << " " << parts_from_camera[i][j].type;
    }
    std::cout << std::endl;
    std::cout << std::endl;
  }
}

/**
 * @brief      Gets the received order vector.
 *
 * @return     The received order vector.
 */
std::vector<nist_gear::Order> Competition::get_received_order_vector() {
  return received_orders_;
}

/**
 * @brief      Executes the pre-kitting
 */
void Competition::pre_kitting() {
  // Populating Orders vector
  for (p; p < received_orders_.size(); p++) {
    order order_instance;
    order_instance.order_id = received_orders_[p].order_id;
    order_instance.shipments = received_orders_[p].shipments;
    orders_vector.push_back(order_instance);

    //Populating Shipment vector for each order
    for (int j = 0; j < orders_vector[p].shipments.size(); j++) {
      shipment shipment_instance;
      shipment_instance.shipment_type = orders_vector[p].shipments[j]
          .shipment_type;
      shipment_instance.agv_id = orders_vector[p].shipments[j].agv_id;
      shipment_instance.products = orders_vector[p].shipments[j].products;
      shipment_vector.push_back(shipment_instance);

      for (int k = 0; k < orders_vector[p].shipments[j].products.size(); k++) {
        if (shipment_vector[j].products[k].type == ("pulley_part_red")
            || ("pulley_part_blue") || ("pulley_part_green")
            || ("piston_part_red") || ("piston_part_green")
            || ("piston_part_blue") || ("disk_part_red") || ("disk_part_green")
            || ("disk_part_blue") || ("gasket_part_red")
            || ("gasket_part_green") || ("gasket_part_blue")) {
          part part_to_be_placed;
          part_to_be_placed.type = orders_vector[p].shipments[j].products[k]
              .type;
          part_to_be_placed.pose.position.x = orders_vector[p].shipments[j]
              .products[k].pose.position.x;
          part_to_be_placed.pose.position.y = orders_vector[p].shipments[j]
              .products[k].pose.position.y;
          part_to_be_placed.pose.position.z = orders_vector[p].shipments[j]
              .products[k].pose.position.z;
          part_to_be_placed.pose.orientation.x = orders_vector[p].shipments[j]
              .products[k].pose.orientation.x;
          part_to_be_placed.pose.orientation.y = orders_vector[p].shipments[j]
              .products[k].pose.orientation.y;
          part_to_be_placed.pose.orientation.z = orders_vector[p].shipments[j]
              .products[k].pose.orientation.z;
          part_to_be_placed.pose.orientation.w = orders_vector[p].shipments[j]
              .products[k].pose.orientation.w;

          master_struct master_struct_instance;
          master_struct_instance.type = part_to_be_placed.type;
          master_struct_instance.place_part_pose.position.x = part_to_be_placed
              .pose.position.x;
          master_struct_instance.place_part_pose.position.y = part_to_be_placed
              .pose.position.y;
          master_struct_instance.place_part_pose.position.z = part_to_be_placed
              .pose.position.z;
          master_struct_instance.place_part_pose.orientation.x =
              part_to_be_placed.pose.orientation.x;
          master_struct_instance.place_part_pose.orientation.y =
              part_to_be_placed.pose.orientation.y;
          master_struct_instance.place_part_pose.orientation.z =
              part_to_be_placed.pose.orientation.z;
          master_struct_instance.place_part_pose.orientation.w =
              part_to_be_placed.pose.orientation.w;
          master_struct_instance.order_id = order_instance.order_id;
          master_struct_instance.shipment_type =
              shipment_instance.shipment_type;
          master_struct_instance.agv_id = shipment_instance.agv_id;
          master_vector[p][j][k] = master_struct_instance;
        }
      }
    }
  }
}

/**
 * @brief      Prints parts to be picked.
 */
void Competition::print_parts_to_pick() {
  ROS_INFO_STREAM("Parts in master vector");
  for (int i = 0; i < 10; i++) {
    ROS_INFO_STREAM("ORder No = " << i);
    for (int j = 0; j < 10; j++) {
      for (int k = 0; k < 20; k++) {
        if ((master_vector[i][j][k].type == "pulley_part_red")
            || (master_vector[i][j][k].type == "pulley_part_blue")
            || (master_vector[i][j][k].type == "pulley_part_green")
            || (master_vector[i][j][k].type == "disk_part_blue")
            || (master_vector[i][j][k].type == "disk_part_red")
            || (master_vector[i][j][k].type == "disk_part_green")
            || (master_vector[i][j][k].type == "piston_part_blue")
            || (master_vector[i][j][k].type == "piston_part_green")
            || (master_vector[i][j][k].type == "piston_part_red")
            || (master_vector[i][j][k].type == "gasket_part_blue")
            || (master_vector[i][j][k].type == "gasket_part_red")
            || (master_vector[i][j][k].type == "gasket_part_green")) {
          ROS_INFO_STREAM(master_vector[i][j][k].type);
        }
      }
    }
  }
}

/**
 * @brief      Master array that contains the parts from
 * all the logical cameras
 *
 * @return     The parts from camera.
 */
std::array<std::array<part, 20>, 20> Competition::get_parts_from_camera() {
  return parts_from_camera;
}

/**
 * @brief      Master vector that keeps track of every order, shipment
 * and product.
 *
 * @return     The master vector.
 */
std::vector<std::vector<std::vector<master_struct> > > Competition::get_master_vector() {
  return master_vector;
}

/**
 * @brief      deletes an order when done
 */
void Competition::delete_completed_order(int i) {
  received_orders_.erase(received_orders_.begin() + i);
  ROS_INFO_STREAM("Deleting Order = " << i);
}

/**
 * @brief      Gets the quality sensor status agv 2.
 *
 * @return     The quality sensor status agv 2.
 */
part Competition::get_quality_sensor_status_agv2() {
  return faulty_part_agv2;
}

/**
 * @brief      Gets the quality sensor status agv 1.
 *
 * @return     The quality sensor status agv 1.
 */
part Competition::get_quality_sensor_status_agv1() {
  return faulty_part_agv1;
}

/**
 * @brief      Callback for the logical camera subscribers
 *
 * @param[in]  msg      The message
 * @param[in]  cam_idx  The camera index
 */
void Competition::logical_camera_callback(
    const nist_gear::LogicalCameraImage::ConstPtr &msg, int cam_idx) {
  std::vector < part > parts_from_15_camera_new;

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  std::ostringstream otopic;
  std::string topic;
  std::ostringstream otopic_part;
  std::string topic_part;

  geometry_msgs::PoseStamped pose_target, pose_rel;
  if (msg->models.size() != 0) {
    int part_no = 0;
    for (int i = 0; i < msg->models.size(); i++) {
      part_no++;
      otopic.str("");
      otopic.clear();
      otopic << "logical_camera_" << cam_idx << "_" << msg->models[i].type
          << "_frame";
      topic = otopic.str();
      ros::Duration timeout(5.0);

      geometry_msgs::TransformStamped transformStamped;
      pose_rel.header.frame_id = "logical_camera_" + std::to_string(cam_idx)
          + "_frame";
      pose_rel.pose = msg->models[i].pose;

      try {
        transformStamped = tfBuffer.lookupTransform("world",
                                                    pose_rel.header.frame_id,
                                                    ros::Time(0), timeout);
      } catch (tf2::TransformException &ex) {
        ROS_WARN("%s", ex.what());
        ros::Duration(1.0).sleep();
        continue;
      }
      tf2::doTransform(pose_rel, pose_target, transformStamped);

      double tx = pose_target.pose.position.x;
      double ty = pose_target.pose.position.y;
      double tz = pose_target.pose.position.z;

      // Orientation quaternion
      tf2::Quaternion q(pose_target.pose.orientation.x,
                        pose_target.pose.orientation.y,
                        pose_target.pose.orientation.z,
                        pose_target.pose.orientation.w);

      // 3x3 Rotation matrix from quaternion
      tf2::Matrix3x3 m(q);

      // Roll Pitch and Yaw from rotation matrix
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);

      otopic_part.str("");
      otopic_part.clear();
      otopic_part << msg->models[i].type << "_" << cam_idx << "_" << part_no;
      topic_part = otopic_part.str();

      if (cam_idx == 15) {
        if ((msg->models[i].type == "pulley_part_red")
            || (msg->models[i].type == "pulley_part_blue")
            || (msg->models[i].type == "pulley_part_green")
            || (msg->models[i].type == "disk_part_blue")
            || (msg->models[i].type == "disk_part_red")
            || (msg->models[i].type == "disk_part_green")
            || (msg->models[i].type == "piston_rod_part_blue")
            || (msg->models[i].type == "piston_rod_part_green")
            || (msg->models[i].type == "piston_rod_part_red")
            || (msg->models[i].type == "gasket_part_blue")
            || (msg->models[i].type == "gasket_part_red")
            || (msg->models[i].type == "gasket_part_green")) {
          part part_under_camera15;
          part_under_camera15.type = msg->models[i].type;
          part_under_camera15.pose.position.x = tx;
          part_under_camera15.pose.position.y = ty;
          part_under_camera15.pose.position.z = tz;
          part_under_camera15.pose.orientation.x = pose_target.pose.orientation
              .x;
          part_under_camera15.pose.orientation.y = pose_target.pose.orientation
              .y;
          part_under_camera15.pose.orientation.z = pose_target.pose.orientation
              .z;
          part_under_camera15.pose.orientation.w = pose_target.pose.orientation
              .w;
          part_under_camera15.faulty = false;
          part_under_camera15.picked = false;
          parts_from_15_camera_new.push_back(part_under_camera15);
          parts_from_15_camera = parts_from_15_camera_new;

          if (msg->models.size() > 0) {
            conveyor_belt_part_status = true;
          }

        }
      }
      if (cam_idx == 11) {
        parts_in_logical_camera_11 = msg->models.size();
        if (!((msg->models[i].type).empty())) {
          parts_from_11_camera[i].type = msg->models[i].type;
          parts_from_11_camera[i].pose.position.x = tx;
          parts_from_11_camera[i].pose.position.y = ty;
          parts_from_11_camera[i].pose.position.z = tz;
          parts_from_11_camera[i].pose.orientation.x = pose_target.pose
              .orientation.x;
          parts_from_11_camera[i].pose.orientation.y = pose_target.pose
              .orientation.y;
          parts_from_11_camera[i].pose.orientation.z = pose_target.pose
              .orientation.z;
          parts_from_11_camera[i].pose.orientation.w = pose_target.pose
              .orientation.w;
          parts_from_11_camera[i].faulty = false;
          parts_from_11_camera[i].picked = false;
        }
      }

      if (cam_idx == 12) {
        parts_in_logical_camera_12 = msg->models.size();
        if (!((msg->models[i].type).empty())) {
          parts_from_12_camera[i].type = msg->models[i].type;
          parts_from_12_camera[i].pose.position.x = tx;
          parts_from_12_camera[i].pose.position.y = ty;
          parts_from_12_camera[i].pose.position.z = tz;
          parts_from_12_camera[i].pose.orientation.x = pose_target.pose
              .orientation.x;
          parts_from_12_camera[i].pose.orientation.y = pose_target.pose
              .orientation.y;
          parts_from_12_camera[i].pose.orientation.z = pose_target.pose
              .orientation.z;
          parts_from_12_camera[i].pose.orientation.w = pose_target.pose
              .orientation.w;
          parts_from_12_camera[i].faulty = false;
          parts_from_12_camera[i].picked = false;
        }
      }

      if (cam_idx == 13) {
        parts_in_logical_camera_13 = msg->models.size();
        if (!((msg->models[i].type).empty())) {
          parts_from_13_camera[i].type = msg->models[i].type;
          parts_from_13_camera[i].pose.position.x = tx;
          parts_from_13_camera[i].pose.position.y = ty;
          parts_from_13_camera[i].pose.position.z = tz;
          parts_from_13_camera[i].pose.orientation.x = pose_target.pose
              .orientation.x;
          parts_from_13_camera[i].pose.orientation.y = pose_target.pose
              .orientation.y;
          parts_from_13_camera[i].pose.orientation.z = pose_target.pose
              .orientation.z;
          parts_from_13_camera[i].pose.orientation.w = pose_target.pose
              .orientation.w;
          parts_from_13_camera[i].faulty = false;
          parts_from_13_camera[i].picked = false;
        }
      }

      if (cam_idx == 14) {
        parts_in_logical_camera_14 = msg->models.size();
        if (!((msg->models[i].type).empty())) {
          parts_from_14_camera[i].type = msg->models[i].type;
          parts_from_14_camera[i].pose.position.x = tx;
          parts_from_14_camera[i].pose.position.y = ty;
          parts_from_14_camera[i].pose.position.z = tz;
          parts_from_14_camera[i].pose.orientation.x = pose_target.pose
              .orientation.x;
          parts_from_14_camera[i].pose.orientation.y = pose_target.pose
              .orientation.y;
          parts_from_14_camera[i].pose.orientation.z = pose_target.pose
              .orientation.z;
          parts_from_14_camera[i].pose.orientation.w = pose_target.pose
              .orientation.w;
          parts_from_14_camera[i].faulty = false;
          parts_from_14_camera[i].picked = false;
        }
      }

      if (cam_idx == 16) {
        if (!((msg->models[i].type).empty())) {
          parts_from_16_camera[i].type = msg->models[i].type;
          parts_from_16_camera[i].pose.position.x = tx;
          parts_from_16_camera[i].pose.position.y = ty;
          parts_from_16_camera[i].pose.position.z = tz;
          parts_from_16_camera[i].pose.orientation.x = pose_target.pose
              .orientation.x;
          parts_from_16_camera[i].pose.orientation.y = pose_target.pose
              .orientation.y;
          parts_from_16_camera[i].pose.orientation.z = pose_target.pose
              .orientation.z;
          parts_from_16_camera[i].pose.orientation.w = pose_target.pose
              .orientation.w;
          parts_from_16_camera[i].faulty = false;
          parts_from_16_camera[i].picked = false;
        }
      }

      if (cam_idx == 17) {
        if (!((msg->models[i].type).empty())) {
          parts_from_17_camera[i].type = msg->models[i].type;
          parts_from_17_camera[i].pose.position.x = tx;
          parts_from_17_camera[i].pose.position.y = ty;
          parts_from_17_camera[i].pose.position.z = tz;
          parts_from_17_camera[i].pose.orientation.x = pose_target.pose
              .orientation.x;
          parts_from_17_camera[i].pose.orientation.y = pose_target.pose
              .orientation.y;
          parts_from_17_camera[i].pose.orientation.z = pose_target.pose
              .orientation.z;
          parts_from_17_camera[i].pose.orientation.w = pose_target.pose
              .orientation.w;
          parts_from_17_camera[i].faulty = false;
          parts_from_17_camera[i].picked = false;
        }
      }

      parts_from_camera[cam_idx][i].type = msg->models[i].type;
      parts_from_camera[cam_idx][i].pose.position.x = tx;
      parts_from_camera[cam_idx][i].pose.position.y = ty;
      parts_from_camera[cam_idx][i].pose.position.z = tz;
      parts_from_camera[cam_idx][i].pose.orientation.x = pose_target.pose
          .orientation.x;
      parts_from_camera[cam_idx][i].pose.orientation.y = pose_target.pose
          .orientation.y;
      parts_from_camera[cam_idx][i].pose.orientation.z = pose_target.pose
          .orientation.z;
      parts_from_camera[cam_idx][i].pose.orientation.w = pose_target.pose
          .orientation.w;
      parts_from_camera[cam_idx][i].faulty = false;
      parts_from_camera[cam_idx][i].picked = false;
    }

  }
}

/**
 * @brief      Checks the status of the competition
 *
 * @param[in]  msg   The message
 */
void Competition::competition_state_callback(
    const std_msgs::String::ConstPtr &msg) {
  if (msg->data == "done" && competition_state_ != "done") {
    ROS_INFO("Competition ended.");
  }
  competition_state_ = msg->data;
}

/**
 * @brief      To subscribe to the order topic
 *
 * @param[in]  msg   The message
 */
void Competition::order_callback(const nist_gear::Order::ConstPtr &msg) {
  ROS_INFO_STREAM("New High Priority ordered received");
  ROS_INFO_STREAM("Received order:\n" << *msg);
  received_orders_.push_back(*msg);
  Competition::pre_kitting();
}

/**
 * @brief      Gets the parts from camera-16.
 *
 * @return     The parts from camera-16.
 */
std::array<part, 20> Competition::get_parts_from_16_camera() {
  return parts_from_16_camera;
}

/**
 * @brief      Gets the parts from camera-17.
 *
 * @return     The parts from camera-17.
 */
std::array<part, 20> Competition::get_parts_from_17_camera() {
  return parts_from_17_camera;
}

/**
 * @brief      Gets the parts from camera-15.
 *
 * @return     The parts from camera-15.
 */
std::vector<part> Competition::get_parts_from_15_camera() {
  return parts_from_15_camera;
}

/**
 * @brief      Callback for the first quality sensor camera
 * @param[in]  msg   The message
 */
void Competition::quality_control_sensor_1_subscriber_callback(
    const nist_gear::LogicalCameraImage::ConstPtr &msg) {
  if (msg->models.size() != 0) {
    for (int i = 0; i < msg->models.size(); i++) {
      faulty_part_agv2.pose = msg->models[i].pose;
      faulty_part_agv2.faulty = true;
    }
  } else
    faulty_part_agv2.faulty = false;
}

/**
 * @brief      Callback for the second quality sensor camera
 * @param[in]  msg   The message
 */
void Competition::quality_control_sensor_2_subscriber_callback(
    const nist_gear::LogicalCameraImage::ConstPtr &msg) {
  if (msg->models.size() != 0) {
    for (int i = 0; i < msg->models.size(); i++) {
      faulty_part_agv1.pose = msg->models[i].pose;
      faulty_part_agv1.faulty = true;
    }
  } else
    faulty_part_agv1.faulty = false;
}

/**
 * @brief      Callback function for competition callback
 * subscribers
 *
 * @param[in]  msg   The Clock type ROS message
 */
void Competition::competition_clock_callback(
    const rosgraph_msgs::Clock::ConstPtr &msg) {
  competition_clock_ = msg->clock;
}

/**
 * @brief      Starts a competition.
 */
void Competition::startCompetition() {
  ros::ServiceClient start_client = node_.serviceClient < std_srvs::Trigger
      > ("/ariac/start_competition");
  if (!start_client.exists()) {
    ROS_INFO(
        "[competition][startCompetition] Waiting for the competition to be ready...");
    start_client.waitForExistence();
    ROS_INFO("[competition][startCompetition] Competition is now ready.");
  }
  ROS_INFO("[competition][startCompetition] Requesting competition start...");
  std_srvs::Trigger srv;
  start_client.call(srv);
  if (!srv.response.success) {
    ROS_ERROR_STREAM(
        "[competition][startCompetition] Failed to start the competition: "
            << srv.response.message);
  } else {
    ROS_INFO("[competition][startCompetition] Competition started!");
  }
}

/**
 * @brief      Ends a competition.
 */
void Competition::endCompetition() {
  ros::ServiceClient end_client = node_.serviceClient < std_srvs::Trigger
      > ("/ariac/end_competition");
  if (!end_client.exists()) {
    ROS_INFO(
        "[competition][endCompetition] Waiting for the end_competition to be ready...");
    end_client.waitForExistence();
    ROS_INFO("[competition][endCompetition] end_competition is now ready.");
  }
  ROS_INFO("[competition][endCompetition] Requesting competition end...");
  std_srvs::Trigger srv;
  end_client.call(srv);
  if (!srv.response.success) {
    ROS_ERROR_STREAM(
        "[competition][endCompetition] Failed to end the competition: "
            << srv.response.message);
  } else {
    ROS_INFO("[competition][endCompetition] Competition ended!");
  }
}

/**
 * @brief      Gets information regarding comp.init().
 *
 * @return     The statistics.
 */
stats Competition::getStats(std::string function) {
  if (function == "init")
    return init_;

}

/**
 * @brief      Gets the start time.
 *
 * @return     The start time.
 */
double Competition::getStartTime() {
  return competition_start_time_;
}

/**
 * @brief      Gets clock information
 *
 * @return     The time spent in the competition.
 */
double Competition::getClock() {
  double time_spent = competition_clock_.toSec();
  ROS_INFO_STREAM(
      "[competition][getClock] competition time spent (getClock()) ="
          << time_spent);
  return time_spent;
}

/**
 * @brief      Gets the competition state.
 *
 * @return     The competition state.
 */
std::string Competition::getCompetitionState() {
  return competition_state_;
}
