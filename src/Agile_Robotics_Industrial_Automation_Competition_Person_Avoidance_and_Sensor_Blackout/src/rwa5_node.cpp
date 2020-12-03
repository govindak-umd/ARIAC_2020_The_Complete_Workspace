// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


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
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> //--needed for tf2::Matrix3x3

#include "competition.h"
#include "utils.h"
#include "gantry_control.h"

#include <nist_gear/AGVControl.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/transform_listener.h> //for shelves gap
#include <tf/LinearMath/Vector3.h>

#define MAX_NUMBER_OF_CAMERAS 18
std::array<std::array<part, 20>, 20>  parts_from_camera_main ;
std::vector<std::vector<std::vector<master_struct> > > master_vector_main (10,std::vector<std::vector<master_struct> >(10,std::vector <master_struct>(20)));
bool part_placed = false;
int k = 0, i = 0, temp = 45;
const double flip = -3.14159;
part faulty_part;
int size_of_order = 0;
std::string shipment_type, agv_id;
int parts_delivered[5]{};
std::array<part, 20> parts_from_camera_16 ;
std::array<part, 20> parts_from_camera_17 ;
bool conveyor_part_picked = false;

// AVG id(= 1,2) to identify what AVG to submit to
// shipment_type is the order type

bool submitOrder(std::string AVG_id, std::string shipment_type){

    ROS_INFO_STREAM("Delivering " << AVG_id << " with shipment = " << shipment_type);
    ROS_INFO("[submitOrder] Submitting order via AVG");

    // Create a node to call service from. Would be better to use one existing node
    // rather than creating a new one every time
    ros::NodeHandle node;

    // Create a Service client for the correct service, i.e. '/ariac/agv{AVG_id}'
    ros::ServiceClient avg_client1;
    ros::ServiceClient avg_client2;

    // Assign the service client to the correct service
    if(AVG_id == "agv1"){
        avg_client1 = node.serviceClient<nist_gear::AGVControl>("/ariac/agv1");
        // Wait for client to start
        if (!avg_client1.exists()) {
            avg_client1.waitForExistence();
        }

        // Debug what you're doing
        ROS_INFO_STREAM("[submitOrder] Sending AVG " << AVG_id << " to submit order");

        // Create the message and assign the shipment type to it
        nist_gear::AGVControl srv;
        srv.request.shipment_type = shipment_type;

        // Send message and retrieve response
        avg_client1.call(srv);
        if (!srv.response.success) {  // If not successful, print out why.
            ROS_ERROR_STREAM("[submitOrder]  Failed to submit: " << srv.response.message);
        } else {
            ROS_INFO("[submitOrder] Submitted");
        }

        return srv.response.success;
    }else if(AVG_id == "agv2"){
        avg_client2 = node.serviceClient<nist_gear::AGVControl>("/ariac/agv2");
        // Wait for client to start
        if (!avg_client2.exists()) {
            avg_client2.waitForExistence();
        }

        // Debug what you're doing
        ROS_INFO_STREAM("[submitOrder] Sending AVG " << AVG_id << " to submit order");

        // Create the message and assign the shipment type to it
        nist_gear::AGVControl srv;
        srv.request.shipment_type = shipment_type;

        // Send message and retrieve response
        avg_client2.call(srv);
        if (!srv.response.success) {  // If not successful, print out why.
            ROS_ERROR_STREAM("[submitOrder]  Failed to submit: " << srv.response.message);
        } else {
            ROS_INFO("[submitOrder] Submitted");
        }

        return srv.response.success;
    }else{
        ROS_ERROR_STREAM("[submitOrder] No AVG with id " << AVG_id <<". Valid ids are 1 and 2 only");
    }


}

void fix_part_pose(Competition &comp, master_struct master_vector_main, GantryControl &gantry, part &part_in_tray) {
    double offset = 0.2;
    parts_from_camera_16 = comp.get_parts_from_16_camera();
    parts_from_camera_17 = comp.get_parts_from_17_camera();

    if(master_vector_main.agv_id == "agv1") {
//        ROS_INFO_STREAM("Parts present in AGV1, checking to see if part needs to repicked up");
        for (int part_idx = 0; part_idx < parts_from_camera_16.size(); part_idx++) {

            if (master_vector_main.type == parts_from_camera_16[part_idx].type) {
//                ROS_INFO_STREAM("PART THAT IS IN THE 16TH CAMERA IS : ");
//                ROS_INFO_STREAM("Current status of the " << comp.parts_from_16_camera[part_idx].type << " part as read from the camera!");
//                ROS_INFO_STREAM(comp.parts_from_16_camera[part_idx].pose);
//                ROS_INFO_STREAM(master_vector_main.type << "part was supposed to be at: ");
//                ROS_INFO_STREAM(gantry.getTargetWorldPose(master_vector_main.place_part_pose,
//                                                          master_vector_main.agv_id));
//                ROS_INFO_STREAM("The part is placed at an offset of : " << abs(comp.parts_from_16_camera[part_idx].pose.position.x -gantry.getTargetWorldPose(master_vector_main.place_part_pose,master_vector_main.agv_id).position.x)
//                <<" ," << abs(comp.parts_from_16_camera[part_idx].pose.position.y -gantry.getTargetWorldPose(master_vector_main.place_part_pose,master_vector_main.agv_id).position.y)
//                <<" ," << abs(comp.parts_from_16_camera[part_idx].pose.position.z -gantry.getTargetWorldPose(master_vector_main.place_part_pose,master_vector_main.agv_id).position.z)
//                <<" ," <<abs(comp.parts_from_16_camera[part_idx].pose.orientation.x - gantry.getTargetWorldPose(master_vector_main.place_part_pose,master_vector_main.agv_id).orientation.x)
//                <<" ," <<abs(comp.parts_from_16_camera[part_idx].pose.orientation.y - gantry.getTargetWorldPose(master_vector_main.place_part_pose,master_vector_main.agv_id).orientation.y)
//                <<" ," <<abs(comp.parts_from_16_camera[part_idx].pose.orientation.z - gantry.getTargetWorldPose(master_vector_main.place_part_pose,master_vector_main.agv_id).orientation.z)
//                <<" ," <<abs(comp.parts_from_16_camera[part_idx].pose.orientation.w - gantry.getTargetWorldPose(master_vector_main.place_part_pose,master_vector_main.agv_id).orientation.w));

                if ((abs(comp.parts_from_16_camera[part_idx].pose.position.x -
                         gantry.getTargetWorldPose(
                                 master_vector_main.place_part_pose,
                                 master_vector_main.agv_id).position.x) >
                     offset) || (abs(comp.parts_from_16_camera[part_idx].pose.position.y -
                                     gantry.getTargetWorldPose(
                                             master_vector_main.place_part_pose,
                                             master_vector_main.agv_id).position.y) >
                                 offset) ||
                    (abs(comp.parts_from_16_camera[part_idx].pose.position.z -
                         gantry.getTargetWorldPose(
                                 master_vector_main.place_part_pose,
                                 master_vector_main.agv_id).position.z) >
                     offset)) {

                    ROS_INFO_STREAM("Attempting replacement of the part");
//                    ROS_INFO_STREAM("going to agv");
                    if (master_vector_main.agv_id == "agv1")
                        gantry.goToPresetLocation(gantry.agv1_);
                    else
                        gantry.goToPresetLocation(gantry.agv2_);

                    part part_re_pick;
                    part_re_pick = comp.parts_from_16_camera[part_idx];
                    part_re_pick.pose.position.z = part_re_pick.pose.position.z + 0.03;

                    gantry.pickPart(part_re_pick);
//                    ROS_INFO_STREAM("Part Picked again");
//                    ROS_INFO_STREAM("going to agv location");
                    if (master_vector_main.agv_id == "agv1")
                        gantry.goToPresetLocation(gantry.agv1_);
                    else
                        gantry.goToPresetLocation(gantry.agv2_);
                    ROS_INFO_STREAM("Placing part again");
                    gantry.placePart(part_in_tray, master_vector_main.agv_id);
//                    ROS_INFO_STREAM(
//                            "Current status of the disk_part_green part As read from the camera after re-placing!");
//                    ROS_INFO_STREAM(comp.parts_from_16_camera[part_idx].type);
//                    ROS_INFO_STREAM(comp.parts_from_16_camera[part_idx].pose);
//                    ROS_INFO_STREAM(master_vector_main.type << " was supposed to be at: ");
//                    ROS_INFO_STREAM(
//                            gantry.getTargetWorldPose(
//                                    master_vector_main.place_part_pose,
//                                    master_vector_main.agv_id));
//                    ROS_INFO_STREAM("The part is placed at an offset after replacing : " <<
//                            abs(comp.parts_from_16_camera[part_idx].pose.position.x - gantry.getTargetWorldPose(master_vector_main.place_part_pose, master_vector_main.agv_id).position.x)
//                            << " ," << abs(comp.parts_from_16_camera[part_idx].pose.position.y - gantry.getTargetWorldPose(master_vector_main.place_part_pose, master_vector_main.agv_id).position.y)
//                            << " ," << abs(comp.parts_from_16_camera[part_idx].pose.position.z - gantry.getTargetWorldPose(master_vector_main.place_part_pose,master_vector_main.agv_id).position.z)
//                            << " ," << abs(comp.parts_from_16_camera[part_idx].pose.orientation.x - gantry.getTargetWorldPose(master_vector_main.place_part_pose,master_vector_main.agv_id).orientation.x)
//                            << " ," << abs(comp.parts_from_16_camera[part_idx].pose.orientation.y - gantry.getTargetWorldPose(master_vector_main.place_part_pose,master_vector_main.agv_id).orientation.y)
//                            << " ," << abs(comp.parts_from_16_camera[part_idx].pose.orientation.z - gantry.getTargetWorldPose(master_vector_main.place_part_pose,master_vector_main.agv_id).orientation.z)
//                            << " ," << abs(comp.parts_from_16_camera[part_idx].pose.orientation.w - gantry.getTargetWorldPose(master_vector_main.place_part_pose,master_vector_main.agv_id).orientation.w));
                }
            }
        }
    }
    else{
        ROS_INFO_STREAM("Parts present in AGV1, checking to see if part needs to re-picked up");
        for (int part_idx = 0; part_idx < parts_from_camera_17.size(); part_idx++) {
            if (master_vector_main.type == parts_from_camera_17[part_idx].type) {
//                ROS_INFO_STREAM("PART THAT IS IN THE 17TH CAMERA IS : ");
//                ROS_INFO_STREAM("Current status of the " <<  master_vector_main.type << " part As read from the camera!");
//                ROS_INFO_STREAM(comp.parts_from_17_camera[part_idx].pose);
//                ROS_INFO_STREAM(master_vector_main.type << " part was supposed to be at: ");
//                ROS_INFO_STREAM(gantry.getTargetWorldPose(master_vector_main.place_part_pose,
//                                                          master_vector_main.agv_id));
//                ROS_INFO_STREAM("The part is placed at an offset of : ");
//                ROS_INFO_STREAM(abs(comp.parts_from_17_camera[part_idx].pose.position.x -gantry.getTargetWorldPose(master_vector_main.place_part_pose,master_vector_main.agv_id).position.x)
//                                        <<" ," << abs(comp.parts_from_17_camera[part_idx].pose.position.y -gantry.getTargetWorldPose(master_vector_main.place_part_pose,master_vector_main.agv_id).position.y)
//                                        <<" ," << abs(comp.parts_from_17_camera[part_idx].pose.position.z -gantry.getTargetWorldPose(master_vector_main.place_part_pose,master_vector_main.agv_id).position.z)
//                                        <<" ," <<abs(comp.parts_from_17_camera[part_idx].pose.orientation.x - gantry.getTargetWorldPose(master_vector_main.place_part_pose,master_vector_main.agv_id).orientation.x)
//                                        <<" ," <<abs(comp.parts_from_17_camera[part_idx].pose.orientation.y - gantry.getTargetWorldPose(master_vector_main.place_part_pose,master_vector_main.agv_id).orientation.y)
//                                        <<" ," <<abs(comp.parts_from_17_camera[part_idx].pose.orientation.z - gantry.getTargetWorldPose(master_vector_main.place_part_pose,master_vector_main.agv_id).orientation.z)
//                                        <<" ," <<abs(comp.parts_from_17_camera[part_idx].pose.orientation.w - gantry.getTargetWorldPose(master_vector_main.place_part_pose,master_vector_main.agv_id).orientation.w));

                if ((abs(comp.parts_from_17_camera[part_idx].pose.position.x -
                         gantry.getTargetWorldPose(
                                 master_vector_main.place_part_pose,
                                 master_vector_main.agv_id).position.x) >
                     offset) || (abs(comp.parts_from_17_camera[part_idx].pose.position.y -
                                     gantry.getTargetWorldPose(
                                             master_vector_main.place_part_pose,
                                             master_vector_main.agv_id).position.y) >
                                 offset) ||
                    (abs(comp.parts_from_17_camera[part_idx].pose.position.z -
                         gantry.getTargetWorldPose(
                                 master_vector_main.place_part_pose,
                                 master_vector_main.agv_id).position.z) >
                     offset)) {

                    ROS_INFO_STREAM("Attempting replacement of the part");
//                    ROS_INFO_STREAM("going to agv");
                    if (master_vector_main.agv_id == "agv1")
                        gantry.goToPresetLocation(gantry.agv1_);
                    else
                        gantry.goToPresetLocation(gantry.agv2_);

                    part part_re_pick;
                    part_re_pick = comp.parts_from_17_camera[part_idx];
                    part_re_pick.pose.position.z = part_re_pick.pose.position.z + 0.02;

                    gantry.pickPart(part_re_pick);
                    ROS_INFO_STREAM("Part Picked again");
//                    ROS_INFO_STREAM("going to agv");
                    if (master_vector_main.agv_id == "agv1")
                        gantry.goToPresetLocation(gantry.agv1_);
                    else
                        gantry.goToPresetLocation(gantry.agv2_);
//                    ROS_INFO_STREAM("Placing part again");
                    gantry.placePart(part_in_tray, master_vector_main.agv_id);
                    ROS_INFO_STREAM(
                            "Current status of the disk_part_green part As read from the camera!");
//                    ROS_INFO_STREAM(comp.parts_from_17_camera[part_idx].type);
//                    ROS_INFO_STREAM(comp.parts_from_17_camera[part_idx].pose);
//                    ROS_INFO_STREAM(master_vector_main.type << " was supposed to be at: ");
//                    ROS_INFO_STREAM(master_vector_main.type << gantry.getTargetWorldPose(
//                                    master_vector_main.place_part_pose,
//                                    master_vector_main.agv_id));
//
//                    ROS_INFO_STREAM("The part is placed at an offset after replacing of : ");
//                    ROS_INFO_STREAM(
//                            abs(comp.parts_from_17_camera[part_idx].pose.position.x - gantry.getTargetWorldPose(master_vector_main.place_part_pose, master_vector_main.agv_id).position.x)
//                                    << " ," << abs(comp.parts_from_17_camera[part_idx].pose.position.y - gantry.getTargetWorldPose(master_vector_main.place_part_pose, master_vector_main.agv_id).position.y)
//                                    << " ," << abs(comp.parts_from_17_camera[part_idx].pose.position.z - gantry.getTargetWorldPose(master_vector_main.place_part_pose,master_vector_main.agv_id).position.z)
//                                    << " ," << abs(comp.parts_from_17_camera[part_idx].pose.orientation.x - gantry.getTargetWorldPose(master_vector_main.place_part_pose,master_vector_main.agv_id).orientation.x)
//                                    << " ," << abs(comp.parts_from_17_camera[part_idx].pose.orientation.y - gantry.getTargetWorldPose(master_vector_main.place_part_pose,master_vector_main.agv_id).orientation.y)
//                                    << " ," << abs(comp.parts_from_17_camera[part_idx].pose.orientation.z - gantry.getTargetWorldPose(master_vector_main.place_part_pose,master_vector_main.agv_id).orientation.z)
//                                    << " ," << abs(comp.parts_from_17_camera[part_idx].pose.orientation.w - gantry.getTargetWorldPose(master_vector_main.place_part_pose,master_vector_main.agv_id).orientation.w));
                }
            }
        }
    }
}

void pick_part_from_conveyor(Competition& comp, GantryControl& gantry){
    ROS_INFO_STREAM("Picking up part from conveyor belt");
    gantry.goToPresetLocation(gantry.start_);
    ROS_INFO_STREAM("Start location reached");
    // move above pick location above belt
//    gantry.goToPresetLocation(gantry.belt_pickup_);
//    ROS_INFO_STREAM("belt pick up location reached");

    double offset_est = 0.292;
    int no_of_parts{2}, count{0};
    while(count < no_of_parts) {
        // move above pick location above belt
        gantry.goToPresetLocation(gantry.belt_pickup_);
        ROS_INFO_STREAM("belt pick up location reached");

        ROS_INFO_STREAM("Picking up part number " << count + 1);
        while ((comp.breakbeam_conveyor_belt_part_status_0 == true) || (comp.breakbeam_conveyor_belt_part_status_1 == true)){
//            ROS_INFO_STREAM("Breakbeam sensor triggered, waiting to turn off");
        }
        ROS_INFO_STREAM("Attempting to pickup part on belt");
        if (!comp.get_parts_from_15_camera().empty()) { // if no part detected in camera 15
            part part_picking = comp.get_parts_from_15_camera().back();
//            ROS_INFO_STREAM("Attempting to pick " << part_picking.type << " from " << part_picking.pose);
            part_picking.pose.position.z += 0.009;
            part_picking.pose.position.y -= offset_est;

            if (gantry.pickMovingPart(part_picking)) {    // if part picked up
                ROS_INFO_STREAM("Part picked");
                gantry.goToPresetLocation(gantry.belt_pickup_);
                ROS_INFO_STREAM("belt pick up location reached");

                //// drop part at desired location on bin1
                PresetLocation bin1_drop = gantry.bin1_;
                bin1_drop.gantry[0] += (count)*0.25;    // offset the next drop off location by 0.25
                gantry.goToPresetLocation(bin1_drop);
                ROS_INFO_STREAM("bin 1 location reached");
                gantry.deactivateGripper("left_arm");
                ROS_INFO_STREAM("Gripper Deactivated");

                gantry.goToPresetLocation(gantry.start_);
                ROS_INFO_STREAM("Start Location Reached");

                //// update parts in camera info (use camera 11 call back function)
                parts_from_camera_main[11][count] = comp.parts_from_11_camera[count];   // update parts in camera above bin1

                count += 1;
            } else {
                ROS_INFO_STREAM("Part not pick, try again");
            }
        } else {
            ROS_INFO_STREAM("no part on belt");
        }

        ros::Duration(2).sleep();
    }
    conveyor_part_picked = true;
    ROS_INFO_STREAM("first part " << parts_from_camera_main[11][0].pose);

    ROS_INFO_STREAM("second part " << parts_from_camera_main[11][1].pose);

}


int main(int argc, char ** argv) {

    ros::init(argc, argv, "rwa3_node");
    ros::NodeHandle node;
    ros::AsyncSpinner spinner(8);
    spinner.start();

    Competition comp(node);

    //Array of Logical Camera Subscribers
    ros::Subscriber logical_camera_subscriber_ [MAX_NUMBER_OF_CAMERAS];
    std::ostringstream otopic;
    std::string topic;

    for (int idx = 0; idx < MAX_NUMBER_OF_CAMERAS; idx++){
        otopic.str("");
        otopic.clear();
        otopic << "/ariac/logical_camera_" << idx;
        topic = otopic.str();
        logical_camera_subscriber_[idx] = node.subscribe<nist_gear::LogicalCameraImage>
                (topic, 10, boost::bind(&Competition::logical_camera_callback, &comp, _1, idx));
    }

    comp.init();


    std::string c_state = comp.getCompetitionState();
    comp.getClock();

    GantryControl gantry(node);
    gantry.init();

    parts_from_camera_main = comp.get_parts_from_camera();
    master_vector_main = comp.get_master_vector();

    //checks if a human was ever detected in an aisle
    ROS_INFO_STREAM("CHECKING FOR HUMAN IN ALL AISLES....");
    int human_exists = 0;
    human_exists = comp.get_human_existence();
    if (human_exists == 1){
        ROS_INFO_STREAM("< --- HUMAN FOUND --- >");
    }
    else if (human_exists ==0){
        ROS_INFO_STREAM(" -x-x-x-THERE IS A NO HUMAN AT ALL!-x-x-x- ");
    }
    ROS_INFO_STREAM("CHECKING FOR HUMANS COMPLETE....");
    // end of the human being detection


    // Picking parts from the conveyor belt
    pick_part_from_conveyor(comp, gantry);

    LOOP3:for(i; i < comp.get_received_order_vector().size();  i++) {
    for (int j = 0; j < comp.get_received_order_vector()[i].shipments.size(); j++) {
        shipment_type = comp.get_received_order_vector()[i].shipments[j].shipment_type;
        agv_id = comp.get_received_order_vector()[i].shipments[j].agv_id;

        k = 0;
        LOOP:while(k< comp.get_received_order_vector()[i].shipments[j].products.size()) {
        size_of_order = comp.get_received_order_vector()[i].shipments[j].products.size();
        ROS_INFO_STREAM("SIZE OF THE ORDER" << size_of_order);
        ROS_INFO_STREAM("loop reached, part not faulty");
        //                for (int k = 0; k < 20; k++){
        ROS_INFO_STREAM("NEWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW part");
        ROS_INFO_STREAM(i << j << k);

        if ((master_vector_main[i][j][k].type == "pulley_part_red") ||
            (master_vector_main[i][j][k].type == "pulley_part_blue") ||
            (master_vector_main[i][j][k].type == "pulley_part_green") ||
            (master_vector_main[i][j][k].type == "disk_part_blue") ||
            (master_vector_main[i][j][k].type == "disk_part_red") ||
            (master_vector_main[i][j][k].type == "disk_part_green") ||
            (master_vector_main[i][j][k].type == "piston_rod_part_blue") ||
            (master_vector_main[i][j][k].type == "piston_rod_part_green") ||
            (master_vector_main[i][j][k].type == "piston_rod_part_red") ||
            (master_vector_main[i][j][k].type == "gasket_part_blue") ||
            (master_vector_main[i][j][k].type == "gasket_part_red") ||
            (master_vector_main[i][j][k].type == "gasket_part_green")) {


            ROS_INFO_STREAM("Parts found from orders");
            ROS_INFO_STREAM(master_vector_main[i][j][k].type);
            ROS_INFO_STREAM(master_vector_main[i][j][k].delivered);
            ROS_INFO_STREAM("checking i j k" << i << j << k);


            if (master_vector_main[i][j][k].delivered == false) {
                part_placed = false;
                LOOP2:
                for (int l = 0; l < parts_from_camera_main.size(); l++) {
                    ROS_INFO_STREAM("Loop 2 reached to replace faulty part");
                    ROS_INFO_STREAM(" Camera number - " << l);
                    for (int m = 0; m < parts_from_camera_main[i].size(); m++) {
                        //                            parts_from_camera_main = comp.get_parts_from_camera();
                        if ((master_vector_main[i][j][k].type == parts_from_camera_main[l][m].type) &&
                            (parts_from_camera_main[l][m].faulty == false) &&
                            (parts_from_camera_main[l][m].picked == false)) {
                            ROS_INFO_STREAM("part_from_camera_index" << l << m);
                            parts_from_camera_main[l][m].picked = true;
                            ROS_INFO_STREAM("picked status " << parts_from_camera_main[l][m].picked);
                            ROS_INFO_STREAM("Part found in environment");
                            ROS_INFO_STREAM(parts_from_camera_main[l][m].type);


                            if (master_vector_main[i][j][k].type == "disk_part_blue") {
                                ROS_INFO_STREAM(master_vector_main[i][j][k].place_part_pose);
                                ROS_INFO_STREAM(parts_from_camera_main[l][m].pose);
                                std::string location = "bin_13";
                                gantry.goToPresetLocation(gantry.start_);
                                ROS_INFO_STREAM("Start location reached");
                                gantry.goToPresetLocation(gantry.bin13_);
                                ROS_INFO_STREAM("bin13 location reached");
                                gantry.pickPart(parts_from_camera_main[l][m]);
                                ROS_INFO_STREAM("Part picked");
                                gantry.goToPresetLocation(gantry.bin13_);
                                ROS_INFO_STREAM("bin13 location reached");

                                gantry.goToPresetLocation(gantry.start_);
                                ROS_INFO_STREAM("Start location reached");

                                part part_in_tray;
                                part_in_tray.type = master_vector_main[i][j][k].type;
                                part_in_tray.pose.position.x = master_vector_main[i][j][k].place_part_pose.position.x;
                                part_in_tray.pose.position.y = master_vector_main[i][j][k].place_part_pose.position.y;
                                part_in_tray.pose.position.z = master_vector_main[i][j][k].place_part_pose.position.z;
                                part_in_tray.pose.orientation.x = master_vector_main[i][j][k].place_part_pose.orientation.x;
                                part_in_tray.pose.orientation.y = master_vector_main[i][j][k].place_part_pose.orientation.y;
                                part_in_tray.pose.orientation.z = master_vector_main[i][j][k].place_part_pose.orientation.z;
                                part_in_tray.pose.orientation.w = master_vector_main[i][j][k].place_part_pose.orientation.w;

                                gantry.placePart(part_in_tray, master_vector_main[i][j][k].agv_id);
                                ROS_INFO_STREAM("Part placed");

                                if (master_vector_main[i][j][k].agv_id == "agv2") {
                                    gantry.goToPresetLocation(gantry.agv2_);
                                    ROS_INFO_STREAM("AGV2 location reached");
                                } else {
                                    gantry.goToPresetLocation(gantry.agv1_);
                                    ROS_INFO_STREAM("AGV1 location reached");
                                }

                                faulty_part = comp.get_quality_sensor_status_agv2();
                                ROS_INFO_STREAM("Status of faulty part = ");
                                ROS_INFO_STREAM(faulty_part.faulty);
                                if (faulty_part.faulty == true) // If placed part is faulty
                                {
                                    part faulty_part;
                                    faulty_part.pose = gantry.getTargetWorldPose_dummy(faulty_part.pose,
                                                                                       master_vector_main[i][j][k].agv_id);
                                    ROS_INFO_STREAM("Black sheep location");
                                    ROS_INFO_STREAM(faulty_part.pose);
                                    faulty_part.type = parts_from_camera_main[l][m].type;
                                    faulty_part.pose.position.x = faulty_part.pose.position.x;
                                    faulty_part.pose.position.y = faulty_part.pose.position.y;
                                    faulty_part.pose.position.z = faulty_part.pose.position.z + 0.03;
                                    faulty_part.pose.orientation.x = faulty_part.pose.orientation.x;
                                    faulty_part.pose.orientation.y = faulty_part.pose.orientation.y;
                                    faulty_part.pose.orientation.z = faulty_part.pose.orientation.z;
                                    faulty_part.pose.orientation.w = faulty_part.pose.orientation.w;
                                    gantry.pickPart(faulty_part);
                                    gantry.goToPresetLocation(gantry.agv2_drop_);
                                    gantry.deactivateGripper("left_arm");
                                    ROS_INFO_STREAM(
                                            "BLack Sheeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeep MEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEHHHHHHHHHHHHHHHHHHHHH");
                                    ROS_INFO_STREAM("Go to Loop2 triggered");
                                    //                                        parts_from_camera_main = comp.get_parts_from_camera();
                                    goto LOOP2;
                                } else { //Part successfully placed, increment K to go to next part in vector
                                    //                                    comp.print_parts_to_pick();
                                    ROS_INFO_STREAM("Go to Loop Triggered");
                                    master_vector_main[i][j][k].delivered = true;
                                    parts_delivered[i]++;
                                    comp.setter_delivered(i, j, k);
                                    ROS_INFO_STREAM(" i j k" << i << j << k);
                                    ROS_INFO_STREAM(parts_from_camera_main[l][m].type << "  successfully delivered");
                                    k++;
                                    if (comp.get_received_order_vector().size() > i + 1) {
                                        ROS_INFO_STREAM("NEW ORDER RECEIVED");
                                        i++;
                                        master_vector_main = comp.get_master_vector();
                                        goto LOOP3;
                                    }
                                    goto LOOP;
                                }
                            } else if (master_vector_main[i][j][k].type == "disk_part_green") {
                                ROS_INFO_STREAM(master_vector_main[i][j][k].place_part_pose);
                                ROS_INFO_STREAM(parts_from_camera_main[l][m].pose);
                                std::string location = "bin_13";
                                gantry.goToPresetLocation(gantry.start_);
                                ROS_INFO_STREAM("Start location reached");
                                gantry.goToPresetLocation(gantry.bin13_);
                                ROS_INFO_STREAM("bin13 location reached");
                                gantry.pickPart(parts_from_camera_main[l][m]);
                                ROS_INFO_STREAM("Part picked");
                                gantry.goToPresetLocation(gantry.bin13_);
                                ROS_INFO_STREAM("bin13 location reached");

                                gantry.goToPresetLocation(gantry.start_);
                                ROS_INFO_STREAM("Start location reached");

                                part part_in_tray;
                                part_in_tray.type = master_vector_main[i][j][k].type;
                                part_in_tray.pose.position.x = master_vector_main[i][j][k].place_part_pose.position.x;
                                part_in_tray.pose.position.y = master_vector_main[i][j][k].place_part_pose.position.y;
                                part_in_tray.pose.position.z = master_vector_main[i][j][k].place_part_pose.position.z;
                                part_in_tray.pose.orientation.x = master_vector_main[i][j][k].place_part_pose.orientation.x;
                                part_in_tray.pose.orientation.y = master_vector_main[i][j][k].place_part_pose.orientation.y;
                                part_in_tray.pose.orientation.z = master_vector_main[i][j][k].place_part_pose.orientation.z;
                                part_in_tray.pose.orientation.w = master_vector_main[i][j][k].place_part_pose.orientation.w;

                                gantry.placePart(part_in_tray, master_vector_main[i][j][k].agv_id);
                                ROS_INFO_STREAM("Part placed");

                                if (master_vector_main[i][j][k].agv_id == "agv2") {
                                    gantry.goToPresetLocation(gantry.agv2_);
                                    ROS_INFO_STREAM("AGV2 location reached");
                                } else {
                                    gantry.goToPresetLocation(gantry.agv1_);
                                    ROS_INFO_STREAM("AGV1 location reached");
                                }

                                fix_part_pose(comp, master_vector_main[i][j][k], gantry, part_in_tray);
                                gantry.goToPresetLocation(gantry.start_);

                                faulty_part = comp.get_quality_sensor_status_agv2();
                                ROS_INFO_STREAM("Status of faulty part = ");
                                ROS_INFO_STREAM(faulty_part.faulty);
                                if (faulty_part.faulty == true) {
                                    part faulty_part;
                                    faulty_part.pose = gantry.getTargetWorldPose_dummy(faulty_part.pose,
                                                                                       master_vector_main[i][j][k].agv_id);
                                    ROS_INFO_STREAM("Black sheep location");
                                    ROS_INFO_STREAM(faulty_part.pose);
                                    faulty_part.type = parts_from_camera_main[l][m].type;
                                    faulty_part.pose.position.x = faulty_part.pose.position.x;
                                    faulty_part.pose.position.y = faulty_part.pose.position.y;
                                    faulty_part.pose.position.z = faulty_part.pose.position.z + 0.03;
                                    faulty_part.pose.orientation.x = faulty_part.pose.orientation.x;
                                    faulty_part.pose.orientation.y = faulty_part.pose.orientation.y;
                                    faulty_part.pose.orientation.z = faulty_part.pose.orientation.z;
                                    faulty_part.pose.orientation.w = faulty_part.pose.orientation.w;
                                    gantry.pickPart(faulty_part);
                                    gantry.goToPresetLocation(gantry.agv2_drop_);
                                    gantry.deactivateGripper("left_arm");
                                    ROS_INFO_STREAM(
                                            "BLack Sheeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeep MEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEHHHHHHHHHHHHHHHHHHHHH");
                                    ROS_INFO_STREAM("Go to Loop2 triggered");
                                    //                                        parts_from_camera_main = comp.get_parts_from_camera();
                                    goto LOOP2;
                                } else {
                                    //                                    comp.print_parts_to_pick();
                                    ROS_INFO_STREAM("Go to Loop Triggered");
                                    master_vector_main[i][j][k].delivered = true;
                                    parts_delivered[i]++;
                                    comp.setter_delivered(i, j, k);
                                    ROS_INFO_STREAM(" i j k" << i << j << k);
                                    ROS_INFO_STREAM(parts_from_camera_main[l][m].type << "  successfully delivered");
                                    k++;
                                    if (comp.get_received_order_vector().size() > i + 1) {
                                        ROS_INFO_STREAM("NEW ORDER RECEIVED");
                                        i++;
                                        master_vector_main = comp.get_master_vector();
                                        goto LOOP3;
                                    }
                                    goto LOOP;
                                }
                            } else if (master_vector_main[i][j][k].type == "gasket_part_green") {
                                ROS_INFO_STREAM("Part to be picked = " << master_vector_main[i][j][k].type);
                                part part_in_tray;
                                part_in_tray.type = master_vector_main[i][j][k].type;
                                part_in_tray.pose.position.x = master_vector_main[i][j][k].place_part_pose.position.x;
                                part_in_tray.pose.position.y = master_vector_main[i][j][k].place_part_pose.position.y;
                                part_in_tray.pose.position.z = master_vector_main[i][j][k].place_part_pose.position.z;
                                part_in_tray.pose.orientation.x = master_vector_main[i][j][k].place_part_pose.orientation.x;
                                part_in_tray.pose.orientation.y = master_vector_main[i][j][k].place_part_pose.orientation.y;
                                part_in_tray.pose.orientation.z = master_vector_main[i][j][k].place_part_pose.orientation.z;
                                part_in_tray.pose.orientation.w = master_vector_main[i][j][k].place_part_pose.orientation.w;
                                part_in_tray.initial_pose = parts_from_camera_main[l][m].pose;

                                ROS_INFO_STREAM("Printing World coordinates where the parts has to be placed for AGV 2");
                                ROS_INFO_STREAM(gantry.getTargetWorldPose(part_in_tray.pose, "agv2"));

                                ROS_INFO_STREAM("Printing World coordinates where the parts has to be placed for AGV1");
                                ROS_INFO_STREAM(gantry.getTargetWorldPose(part_in_tray.pose, "agv1"));

                                ROS_INFO_STREAM("Part to be placed at = ");
                                ROS_INFO_STREAM(master_vector_main[i][j][k].place_part_pose);
                                ROS_INFO_STREAM("Part to be picked from = ");
                                ROS_INFO_STREAM(parts_from_camera_main[l][m].pose);
                                std::string location = "shelf 8";
                                gantry.goToPresetLocation(gantry.start_);
                                ROS_INFO_STREAM("Start location reached");


                                auto q = gantry.pickup_locations.find(l);
                                int green_gasket_counter = 0;
                                for (auto y: q->second){
                                    if(green_gasket_counter==4 && human_exists == 1){
                                        ROS_INFO_STREAM("Waiting for the person to move");
                                        bool breakbeam_4_triggered = false;
                                        ros::Time time_4;
                                        bool breakbeam_5_triggered = false;
                                        ros::Time time_5;
                                        while (true){
                                            if (comp.breakbeam_part_status_4 == true and  breakbeam_4_triggered == false){
                                                ROS_INFO_STREAM("4 TRIGGERED ");
                                                time_4 = ros::Time::now();
                                                breakbeam_4_triggered = true;
                                            }
                                            if (comp.breakbeam_part_status_5 == true and  breakbeam_5_triggered == false){
                                                ROS_INFO_STREAM("5 TRIGGERED ");
                                                time_5 = ros::Time::now();
                                                breakbeam_5_triggered = true;
                                            }

                                            if(breakbeam_4_triggered == true and breakbeam_5_triggered == true){
                                                ROS_INFO_STREAM("BOTH TRIGGERED");
                                                ROS_INFO_STREAM(time_4);
                                                ROS_INFO_STREAM(time_5);
                                                ros::Duration diff = time_4 - time_5;
                                                ROS_INFO_STREAM("diff in time between both is : "<<diff);
                                                if(time_4 > time_5){
                                                    gantry.goToPresetLocation(y);
                                                    green_gasket_counter +=1;
                                                    break;
                                                }
                                                else{
                                                    breakbeam_4_triggered = false;
                                                    breakbeam_5_triggered = false;
                                                }
                                            }

                                        }
                                    }
                                    else{
                                        gantry.goToPresetLocation(y);
                                        green_gasket_counter +=1;
                                    }
                                    ros::Duration timeout(0.5);
                                }

                                gantry.pickPart(parts_from_camera_main[l][m]);
                                ROS_INFO_STREAM("Part picked");

                                for (auto it = q->second.rbegin(); it != q->second.rend(); it++){
                                    gantry.goToPresetLocation(*it);
                                    ros::Duration timeout(0.5);
                                }

                                ROS_INFO_STREAM("AGVVVVVVVVVVVVVVVVVVVVVVV");
                                ROS_INFO_STREAM(master_vector_main[i][j][k].agv_id);
                                gantry.placePart(part_in_tray, master_vector_main[i][j][k].agv_id);
                                ROS_INFO_STREAM("Part placed");

                                if (master_vector_main[i][j][k].agv_id == "agv2") {
                                    gantry.goToPresetLocation(gantry.agv2_);
                                    ROS_INFO_STREAM("AGV2 location reached");
                                } else {
                                    gantry.goToPresetLocation(gantry.agv1_);
                                    ROS_INFO_STREAM("AGV1 location reached");
                                }

//                                //Fixing part pose if gripper is Faulty
//                                fix_part_pose(comp, master_vector_main[i][j][k], gantry, part_in_tray);
//
                                // Checking if parts have arrived on conveyor belt

                                ROS_INFO_STREAM("First condition " << comp.conveyor_belt_part_status << "Second condition " << conveyor_part_picked);
                                if((comp.conveyor_belt_part_status == true) && (conveyor_part_picked == false))
                                {
                                    ROS_INFO_STREAM("Picking part from conveyor belt");
                                    pick_part_from_conveyor(comp, gantry);
                                }

                                if(master_vector_main[i][j][k].agv_id == "agv2") {
                                    ROS_INFO_STREAM("Loading faulty part status from agv2");
                                    faulty_part = comp.get_quality_sensor_status_agv2();
                                }
                                else{
                                    ROS_INFO_STREAM("Loading faulty part status from agv1");
                                    faulty_part = comp.get_quality_sensor_status_agv1();
                                }

                                ROS_INFO_STREAM("Status of faulty part = ");
                                ROS_INFO_STREAM(faulty_part.faulty);

                                if (faulty_part.faulty == true) {
                                    part faulty_part;
                                    faulty_part.pose = gantry.getTargetWorldPose_dummy(faulty_part.pose,
                                                                                       master_vector_main[i][j][k].agv_id);
                                    ROS_INFO_STREAM("Black sheep location");
                                    ROS_INFO_STREAM(faulty_part.pose);
                                    faulty_part.type = parts_from_camera_main[l][m].type;
                                    faulty_part.pose.position.x = faulty_part.pose.position.x;
                                    faulty_part.pose.position.y = faulty_part.pose.position.y;
                                    if(master_vector_main[i][j][k].agv_id == "agv1")
                                        faulty_part.pose.position.z = faulty_part.pose.position.z + 0.03; //0.0365235
                                    else
                                        faulty_part.pose.position.z = faulty_part.pose.position.z + 0.03;
                                    faulty_part.pose.orientation.x = faulty_part.pose.orientation.x;
                                    faulty_part.pose.orientation.y = faulty_part.pose.orientation.y;
                                    faulty_part.pose.orientation.z = faulty_part.pose.orientation.z;
                                    faulty_part.pose.orientation.w = faulty_part.pose.orientation.w;
                                    gantry.pickPart(faulty_part);
                                    if(master_vector_main[i][j][k].agv_id == "agv2"){
                                        gantry.goToPresetLocation(gantry.agv2_);
                                        gantry.goToPresetLocation(gantry.agv2_drop_);
                                    }

                                    else {
                                        gantry.goToPresetLocation(gantry.agv1_);
                                        gantry.goToPresetLocation(gantry.agv1_drop_);
                                    }
                                    gantry.deactivateGripper("left_arm");
                                    ROS_INFO_STREAM(
                                            "BLack Sheeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeep MEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEHHHHHHHHHHHHHHHHHHHHH");
                                    ROS_INFO_STREAM("Go to Loop2 triggered");
                                    //                                        parts_from_camera_main = comp.get_parts_from_camera();
                                    goto LOOP2;
                                } else {
                                    ROS_INFO_STREAM(
                                            "Checking if vector size increased");
                                    //                                    comp.print_parts_to_pick();
                                    ROS_INFO_STREAM("Go to Loop Triggered");
                                    master_vector_main[i][j][k].delivered = true;
                                    parts_delivered[i]++;
                                    comp.setter_delivered(i, j, k);
                                    ROS_INFO_STREAM(
                                            "Checking delivered status" << master_vector_main[i][j][k].delivered);
                                    ROS_INFO_STREAM(" i j k" << i << j << k);
                                    ROS_INFO_STREAM(parts_from_camera_main[l][m].type << "  successfully delivered");
                                    if (comp.get_received_order_vector().size() > i + 1) {
                                        ROS_INFO_STREAM("NEW ORDER RECEIVED");
                                        master_vector_main = comp.get_master_vector();
                                        ROS_INFO_STREAM(" after getting new master vector, i j k" << i << j << k);
                                        ROS_INFO_STREAM("Checking if delivered status changed for part = "
                                                                << master_vector_main[i][j][k].type
                                                                << master_vector_main[i][j][k].delivered);
                                        i++;
                                        goto LOOP3;
                                    }
                                    k++;
                                    goto LOOP;
                                }
                            }

                            else if (master_vector_main[i][j][k].type == "piston_rod_part_red") {
                                ROS_INFO_STREAM(
                                        "Part to be pickedddddddddddddddd = " << master_vector_main[i][j][k].type);
                                part part_in_tray;
                                part_in_tray.type = master_vector_main[i][j][k].type;
                                part_in_tray.pose.position.x = master_vector_main[i][j][k].place_part_pose.position.x;
                                part_in_tray.pose.position.y = master_vector_main[i][j][k].place_part_pose.position.y;
                                part_in_tray.pose.position.z = master_vector_main[i][j][k].place_part_pose.position.z;
                                part_in_tray.pose.orientation.x = master_vector_main[i][j][k].place_part_pose.orientation.x;
                                part_in_tray.pose.orientation.y = master_vector_main[i][j][k].place_part_pose.orientation.y;
                                part_in_tray.pose.orientation.z = master_vector_main[i][j][k].place_part_pose.orientation.z;
                                part_in_tray.pose.orientation.w = master_vector_main[i][j][k].place_part_pose.orientation.w;
                                part_in_tray.initial_pose = parts_from_camera_main[l][m].pose;

                                ROS_INFO_STREAM("Part to be placed at = ");
                                ROS_INFO_STREAM(master_vector_main[i][j][k].place_part_pose);
                                ROS_INFO_STREAM("Part to be picked from = ");
                                ROS_INFO_STREAM(parts_from_camera_main[l][m].pose);
                                std::string location = "Bin 1";
                                gantry.goToPresetLocation(gantry.start_);
                                ROS_INFO_STREAM("Start location reached");

                                auto q = gantry.pickup_locations.find(l);
                                for (auto y: q->second) {
                                    gantry.goToPresetLocation(y);
                                }

//                                while(true)
//                                {
//                                    int v = 1;
//                                }

                                gantry.pickPart(parts_from_camera_main[l][m]);

                                for (auto it = q->second.rbegin(); it != q->second.rend(); it++) {
                                    gantry.goToPresetLocation(*it);
                                }
                                gantry.goToPresetLocation(gantry.start_);
                                ROS_INFO_STREAM("Part picked");
                                
                                ROS_INFO_STREAM("AGVVVVVVVVVVVVVVVVVVVVVVV");
                                ROS_INFO_STREAM(master_vector_main[i][j][k].agv_id);
                                gantry.placePart(part_in_tray, master_vector_main[i][j][k].agv_id);
                                ROS_INFO_STREAM("Part placed");

                                if (master_vector_main[i][j][k].agv_id == "agv2") {
                                    gantry.goToPresetLocation(gantry.agv2_);
                                    ROS_INFO_STREAM("AGV2 location reached");
                                } else {
                                    gantry.goToPresetLocation(gantry.agv1_);
                                    ROS_INFO_STREAM("AGV1 location reached");
                                }

//                                //Fixing part pose if gripper is Faulty
//                                fix_part_pose(comp, master_vector_main[i][j][k], gantry, part_in_tray);
//
//                                // Checking if parts have arrived on conveyor belt
//                                if((comp.conveyor_belt_part_status == true) && (conveyor_part_picked == false))
//                                {
//                                    pick_part_from_conveyor(comp, gantry);
//                                }

                                faulty_part = comp.get_quality_sensor_status_agv2();
                                ROS_INFO_STREAM("Status of faulty part = ");
                                ROS_INFO_STREAM(faulty_part.faulty);
                                if (faulty_part.faulty == true) {
                                    part faulty_part;
                                    faulty_part.pose = gantry.getTargetWorldPose_dummy(faulty_part.pose,
                                                                                       master_vector_main[i][j][k].agv_id);
                                    ROS_INFO_STREAM("Black sheep location");
                                    ROS_INFO_STREAM(faulty_part.pose);
                                    faulty_part.type = parts_from_camera_main[l][m].type;
                                    faulty_part.pose.position.x = faulty_part.pose.position.x;
                                    faulty_part.pose.position.y = faulty_part.pose.position.y;
                                    faulty_part.pose.position.z = faulty_part.pose.position.z + 0.03;
                                    faulty_part.pose.orientation.x = faulty_part.pose.orientation.x;
                                    faulty_part.pose.orientation.y = faulty_part.pose.orientation.y;
                                    faulty_part.pose.orientation.z = faulty_part.pose.orientation.z;
                                    faulty_part.pose.orientation.w = faulty_part.pose.orientation.w;
                                    gantry.pickPart(faulty_part);
                                    gantry.goToPresetLocation(gantry.agv2_drop_);
                                    gantry.deactivateGripper("left_arm");
                                    ROS_INFO_STREAM(
                                            "BLack Sheeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeep MEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEHHHHHHHHHHHHHHHHHHHHH");
                                    ROS_INFO_STREAM("Go to Loop2 triggered");
                                    //                                        parts_from_camera_main = comp.get_parts_from_camera();
                                    goto LOOP2;
                                } else {
                                    ROS_INFO_STREAM(
                                            "Checking if vector size increased");
                                    //                                    comp.print_parts_to_pick();
                                    ROS_INFO_STREAM("Go to Loop Triggered");
                                    master_vector_main[i][j][k].delivered = true;
                                    parts_delivered[i]++;
                                    comp.setter_delivered(i, j, k);
                                    ROS_INFO_STREAM(
                                            "Checking delivered status" << master_vector_main[i][j][k].delivered);
                                    ROS_INFO_STREAM(" i j k" << i << j << k);
                                    ROS_INFO_STREAM(parts_from_camera_main[l][m].type << "  successfully delivered");
                                    if (comp.get_received_order_vector().size() > i + 1) {
                                        ROS_INFO_STREAM("NEW ORDER RECEIVED");
                                        master_vector_main = comp.get_master_vector();
                                        ROS_INFO_STREAM(" after getting new master vector, i j k" << i << j << k);
                                        ROS_INFO_STREAM("Checking if delivered status changed for part = "
                                                                << master_vector_main[i][j][k].type
                                                                << master_vector_main[i][j][k].delivered);
                                        i++;
                                        goto LOOP3;
                                    }
                                    k++;
                                    goto LOOP;
                                }
                            }

                            else if (master_vector_main[i][j][k].type == "pulley_part_blue") {
                                ROS_INFO_STREAM(
                                        "Part to be picked = " << master_vector_main[i][j][k].type);
                                part part_in_tray;
                                part_in_tray.type = master_vector_main[i][j][k].type;
                                part_in_tray.pose.position.x = master_vector_main[i][j][k].place_part_pose.position.x;
                                part_in_tray.pose.position.y = master_vector_main[i][j][k].place_part_pose.position.y;
                                part_in_tray.pose.position.z = master_vector_main[i][j][k].place_part_pose.position.z;
                                part_in_tray.pose.orientation.x = master_vector_main[i][j][k].place_part_pose.orientation.x;
                                part_in_tray.pose.orientation.y = master_vector_main[i][j][k].place_part_pose.orientation.y;
                                part_in_tray.pose.orientation.z = master_vector_main[i][j][k].place_part_pose.orientation.z;
                                part_in_tray.pose.orientation.w = master_vector_main[i][j][k].place_part_pose.orientation.w;
                                part_in_tray.initial_pose = parts_from_camera_main[l][m].pose;

                                ROS_INFO_STREAM("Part to be placed at = ");
                                ROS_INFO_STREAM(master_vector_main[i][j][k].place_part_pose);
                                ROS_INFO_STREAM("Part to be picked from = ");
                                ROS_INFO_STREAM(parts_from_camera_main[l][m].pose);
                                std::string location = "shelf 8a";
                                gantry.goToPresetLocation(gantry.start_);
                                ROS_INFO_STREAM("Start location reached");

                                auto q = gantry.pickup_locations.find(l);
                                int blue_pulley_counter = 0;
                                for (auto y: q->second){
                                    if(blue_pulley_counter==3 && human_exists == 1 ){
                                        ROS_INFO_STREAM("Waiting for the person to move");
                                        bool breakbeam_8_triggered = false;
                                        ros::Time time_8;
                                        bool breakbeam_9_triggered = false;
                                        ros::Time time_9;
                                        while (true){
                                            if (comp.breakbeam_part_status_8 == true and  breakbeam_8_triggered == false){
                                                ROS_INFO_STREAM("8 TRIGGERED ");
                                                time_8 = ros::Time::now();
                                                breakbeam_8_triggered = true;
                                            }
                                            if (comp.breakbeam_part_status_9 == true and  breakbeam_9_triggered == false){
                                                ROS_INFO_STREAM("9 TRIGGERED ");
                                                time_9 = ros::Time::now();
                                                breakbeam_9_triggered = true;
                                            }

                                            if(breakbeam_9_triggered == true and breakbeam_8_triggered == true){
                                                ROS_INFO_STREAM("BOTH TRIGGERED");
                                                ROS_INFO_STREAM(time_8);
                                                ROS_INFO_STREAM(time_9);
                                                ros::Duration diff = time_8 - time_9;
                                                ROS_INFO_STREAM("diff in time between both is : "<<diff);
                                                if(time_8 > time_9){
                                                    gantry.goToPresetLocation(y);
                                                    blue_pulley_counter +=1;
                                                    break;
                                                }
                                                else{
                                                    breakbeam_8_triggered = false;
                                                    breakbeam_9_triggered = false;
                                                }
                                            }

                                        }
                                    }
                                    else{
                                        gantry.goToPresetLocation(y);
                                        blue_pulley_counter +=1;
                                    }
                                    ros::Duration timeout(0.5);
                                }

                                gantry.pickPart(parts_from_camera_main[l][m]);
                                ROS_INFO_STREAM("Part picked");

                                for (auto it = q->second.rbegin(); it != q->second.rend(); it++) {
//                                    cout << *it << " ";
                                    gantry.goToPresetLocation(*it);
                                }

                                ROS_INFO_STREAM("AGVVVVVVVVVVVVVVVVVVVVVVV");
                                ROS_INFO_STREAM(master_vector_main[i][j][k].agv_id);

                                if (part_in_tray.pose.orientation.x == 1) {
                                    if(master_vector_main[i][j][k].agv_id=="agv1") {
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
                                    if(master_vector_main[i][j][k].agv_id=="agv2") {
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
                                    gantry.placePart_right_arm(part_in_tray, master_vector_main[i][j][k].agv_id);
                                    ROS_INFO_STREAM("Part placed");
                                } else {


                                    gantry.placePart(part_in_tray, master_vector_main[i][j][k].agv_id);
                                    ROS_INFO_STREAM("Part placed");

                                    if (master_vector_main[i][j][k].agv_id == "agv2") {
                                        gantry.goToPresetLocation(gantry.agv2_);
                                        ROS_INFO_STREAM("AGV2 location reached");
                                    } else {
                                        gantry.goToPresetLocation(gantry.agv1_);
                                        ROS_INFO_STREAM("AGV1 location reached");
                                    }
                                }

//                                //Fixing part pose if gripper is Faulty
//                                fix_part_pose(comp, master_vector_main[i][j][k], gantry, part_in_tray);
//
//                                // Checking if parts have arrived on conveyor belt
//                                if((comp.conveyor_belt_part_status == true) && (conveyor_part_picked == false))
//                                {
//                                    pick_part_from_conveyor(comp, gantry);
//                                }

                                faulty_part = comp.get_quality_sensor_status_agv2();
                                ROS_INFO_STREAM("Status of faulty part = ");
                                ROS_INFO_STREAM(faulty_part.faulty);
                                if (faulty_part.faulty == true) {
                                    part faulty_part;
                                    faulty_part.pose = gantry.getTargetWorldPose_dummy(faulty_part.pose,
                                                                                       master_vector_main[i][j][k].agv_id);
                                    ROS_INFO_STREAM("Black sheep location");
                                    ROS_INFO_STREAM(faulty_part.pose);
                                    faulty_part.type = parts_from_camera_main[l][m].type;
                                    faulty_part.pose.position.x = faulty_part.pose.position.x;
                                    faulty_part.pose.position.y = faulty_part.pose.position.y;
                                    faulty_part.pose.position.z = faulty_part.pose.position.z + 0.03;
                                    faulty_part.pose.orientation.x = faulty_part.pose.orientation.x;
                                    faulty_part.pose.orientation.y = faulty_part.pose.orientation.y;
                                    faulty_part.pose.orientation.z = faulty_part.pose.orientation.z;
                                    faulty_part.pose.orientation.w = faulty_part.pose.orientation.w;
                                    gantry.pickPart(faulty_part);
                                    gantry.goToPresetLocation(gantry.agv2_drop_);
                                    gantry.deactivateGripper("left_arm");
                                    ROS_INFO_STREAM(
                                            "BLack Sheeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeep MEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEHHHHHHHHHHHHHHHHHHHHH");
                                    ROS_INFO_STREAM("Go to Loop2 triggered");
                                    //                                        parts_from_camera_main = comp.get_parts_from_camera();
                                    goto LOOP2;
                                } else {
                                    ROS_INFO_STREAM(
                                            "Checking if vector size increased");
                                    //                                    comp.print_parts_to_pick();
                                    ROS_INFO_STREAM("Go to Loop Triggered");
                                    master_vector_main[i][j][k].delivered = true;
                                    parts_delivered[i]++;
                                    comp.setter_delivered(i, j, k);
                                    ROS_INFO_STREAM(
                                            "Checking delivered status" << master_vector_main[i][j][k].delivered);
                                    ROS_INFO_STREAM(" i j k" << i << j << k);
                                    ROS_INFO_STREAM(parts_from_camera_main[l][m].type << "  successfully delivered");
                                    if (comp.get_received_order_vector().size() > i + 1) {
                                        ROS_INFO_STREAM("NEW ORDER RECEIVED");
                                        master_vector_main = comp.get_master_vector();
                                        ROS_INFO_STREAM(" after getting new master vector, i j k" << i << j << k);
                                        ROS_INFO_STREAM("Checking if delivered status changed for part = "
                                                                << master_vector_main[i][j][k].type
                                                                << master_vector_main[i][j][k].delivered);
                                        i++;
                                        goto LOOP3;
                                    }
                                    k++;
                                    goto LOOP;
                                }
                            }


                            else if (master_vector_main[i][j][k].type == "gasket_part_blue") {
                                ROS_INFO_STREAM(master_vector_main[i][j][k].place_part_pose);
                                ROS_INFO_STREAM(parts_from_camera_main[l][m].pose);
                                std::string location = "shelf 11";
                                gantry.goToPresetLocation(gantry.start_);
                                ROS_INFO_STREAM("Start location reached");
                                gantry.goToPresetLocation(gantry.shelf11_w1_);
                                ROS_INFO_STREAM("Wavepoint1 location reached");
                                gantry.goToPresetLocation(gantry.shelf11_w2_);
                                ROS_INFO_STREAM("Wavepoint2 location reached");
                                gantry.goToPresetLocation(gantry.shelf11_w3_);
                                ROS_INFO_STREAM("Wavepoint2 location reached");
                                gantry.goToPresetLocation(gantry.shelf11_w4_);
                                ROS_INFO_STREAM("Wavepoint2 location reached");

                                gantry.pickPart(parts_from_camera_main[l][m]);
                                ROS_INFO_STREAM("Part picked");
                                gantry.goToPresetLocation(gantry.shelf11_w4_);
                                ROS_INFO_STREAM("Wavepoint2 location reached");
                                gantry.goToPresetLocation(gantry.shelf11_w3_);
                                ROS_INFO_STREAM("Wavepoint2 location reached");
                                gantry.goToPresetLocation(gantry.shelf11_w2_);
                                ROS_INFO_STREAM("Wavepoint2 location reached");
                                gantry.goToPresetLocation(gantry.shelf11_w1_);
                                ROS_INFO_STREAM("Wavepoint1 location reached");

                                part part_in_tray;
                                part_in_tray.type = master_vector_main[i][j][k].type;
                                part_in_tray.pose.position.x = master_vector_main[i][j][k].place_part_pose.position.x;
                                part_in_tray.pose.position.y = master_vector_main[i][j][k].place_part_pose.position.y;
                                part_in_tray.pose.position.z = master_vector_main[i][j][k].place_part_pose.position.z;
                                part_in_tray.pose.orientation.x = master_vector_main[i][j][k].place_part_pose.orientation.x;
                                part_in_tray.pose.orientation.y = master_vector_main[i][j][k].place_part_pose.orientation.y;
                                part_in_tray.pose.orientation.z = master_vector_main[i][j][k].place_part_pose.orientation.z;
                                part_in_tray.pose.orientation.w = master_vector_main[i][j][k].place_part_pose.orientation.w;

                                ROS_INFO_STREAM("AGVVVVVVVVVVVVVVVVVVVVVVV");
                                ROS_INFO_STREAM(master_vector_main[i][j][k].agv_id);
                                gantry.placePart(part_in_tray, master_vector_main[i][j][k].agv_id);
                                ROS_INFO_STREAM("Part placed");

                                if (master_vector_main[i][j][k].agv_id == "agv2") {
                                    gantry.goToPresetLocation(gantry.agv2_);
                                    ROS_INFO_STREAM("AGV2 location reached");
                                } else {
                                    gantry.goToPresetLocation(gantry.agv1_);
                                    ROS_INFO_STREAM("AGV1 location reached");
                                }

                                fix_part_pose(comp, master_vector_main[i][j][k], gantry, part_in_tray);

                                faulty_part = comp.get_quality_sensor_status_agv2();
                                ROS_INFO_STREAM("Status of faulty part = ");
                                ROS_INFO_STREAM(faulty_part.faulty);
                                if (faulty_part.faulty == true) {
                                    faulty_part.pose = gantry.getTargetWorldPose_dummy(faulty_part.pose,
                                                                                       master_vector_main[i][j][k].agv_id);
                                    ROS_INFO_STREAM("Black sheep location");
                                    ROS_INFO_STREAM(faulty_part.pose);
                                    faulty_part.type = parts_from_camera_main[l][m].type;
                                    faulty_part.pose.position.x = faulty_part.pose.position.x;
                                    faulty_part.pose.position.y = faulty_part.pose.position.y;
                                    faulty_part.pose.position.z = faulty_part.pose.position.z + 0.03;
                                    faulty_part.pose.orientation.x = faulty_part.pose.orientation.x;
                                    faulty_part.pose.orientation.y = faulty_part.pose.orientation.y;
                                    faulty_part.pose.orientation.z = faulty_part.pose.orientation.z;
                                    faulty_part.pose.orientation.w = faulty_part.pose.orientation.w;
                                    gantry.pickPart(faulty_part);
                                    gantry.goToPresetLocation(gantry.agv2_drop_);
                                    gantry.deactivateGripper("left_arm");
                                    ROS_INFO_STREAM(
                                            "BLack Sheeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeep MEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEHHHHHHHHHHHHHHHHHHHHH");
                                    ROS_INFO_STREAM("Go to Loop2 triggered");
                                    //                                        parts_from_camera_main = comp.get_parts_from_camera();
                                    goto LOOP2;
                                } else {
                                    //                                    comp.print_parts_to_pick();
                                    master_vector_main[i][j][k].delivered = true;
                                    parts_delivered[i]++;
                                    comp.setter_delivered(i, j, k);
                                    ROS_INFO_STREAM(" i j k" << i << j << k);
                                    ROS_INFO_STREAM(parts_from_camera_main[l][m].type << "  successfully delivered");
                                    k++;
                                    if (comp.get_received_order_vector().size() > i + 1) {
                                        ROS_INFO_STREAM("NEW ORDER RECEIVED");
                                        i++;
                                        master_vector_main = comp.get_master_vector();
                                        goto LOOP3;
                                    }
                                    goto LOOP;
                                }
                            }
                            else if (master_vector_main[i][j][k].type == "pulley_part_red") {
                                ROS_INFO_STREAM(master_vector_main[i][j][k].place_part_pose);
                                ROS_INFO_STREAM(parts_from_camera_main[l][m].pose);
                                part part_in_tray;
                                part_in_tray.type = master_vector_main[i][j][k].type;
                                part_in_tray.pose.position.x = master_vector_main[i][j][k].place_part_pose.position.x;
                                part_in_tray.pose.position.y = master_vector_main[i][j][k].place_part_pose.position.y;
                                part_in_tray.pose.position.z = master_vector_main[i][j][k].place_part_pose.position.z;
                                part_in_tray.pose.orientation.x = master_vector_main[i][j][k].place_part_pose.orientation.x;
                                part_in_tray.pose.orientation.y = master_vector_main[i][j][k].place_part_pose.orientation.y;
                                part_in_tray.pose.orientation.z = master_vector_main[i][j][k].place_part_pose.orientation.z;
                                part_in_tray.pose.orientation.w = master_vector_main[i][j][k].place_part_pose.orientation.w;

                                ROS_INFO_STREAM("Placing coordinates");
                                ROS_INFO_STREAM(part_in_tray.pose);
                                ROS_INFO_STREAM("Picking coordinates");
                                ROS_INFO_STREAM(parts_from_camera_main[l][m].pose);

                                ROS_INFO_STREAM("Printing World coordinates where the parts has to be placed for AGV 2");
                                ROS_INFO_STREAM(gantry.getTargetWorldPose(part_in_tray.pose, "agv2"));

                                ROS_INFO_STREAM("Printing World coordinates where the parts has to be placed for AGV1");
                                ROS_INFO_STREAM(gantry.getTargetWorldPose(part_in_tray.pose, "agv1"));
                                std::string location = "shelf5";
                                gantry.goToPresetLocation(gantry.start_);
                                ROS_INFO_STREAM("Start location reached");
                                gantry.goToPresetLocation(gantry.bin1_);
                                ROS_INFO_STREAM("bin1 location reached");
//                                gantry.goToPresetLocation(gantry.waypoint_1_);
//                                ROS_INFO_STREAM("waypont1 location reached");
//                                gantry.goToPresetLocation(gantry.waypoint_2_);
//                                ROS_INFO_STREAM("waypoint2 location reached");
//                                gantry.goToPresetLocation(gantry.waypoint_3_);
//                                ROS_INFO_STREAM("waypoint3 location reached");
//                                gantry.goToPresetLocation(gantry.waypoint_4_);
//                                ROS_INFO_STREAM("waypoint4 location reached");
                                gantry.pickPart(parts_from_camera_main[l][m]);
                                gantry.goToPresetLocation(gantry.bin1_);
                                ROS_INFO_STREAM("Part picked");
//                                gantry.goToPresetLocation(gantry.waypoint_4_);
//                                ROS_INFO_STREAM("waypoint4 location reached");
//                                gantry.goToPresetLocation(gantry.waypoint_3_);
//                                ROS_INFO_STREAM("waypoint3 location reached");
//                                gantry.goToPresetLocation(gantry.waypoint_2_);
//                                ROS_INFO_STREAM("waypoint2 location reached");
//                                gantry.goToPresetLocation(gantry.waypoint_1_);
//                                ROS_INFO_STREAM("waypoint1 location reached");

//                                part part_in_tray;
//                                part_in_tray.type = master_vector_main[i][j][k].type;
//                                part_in_tray.pose.position.x = master_vector_main[i][j][k].place_part_pose.position.x;
//                                part_in_tray.pose.position.y = master_vector_main[i][j][k].place_part_pose.position.y;
//                                part_in_tray.pose.position.z = master_vector_main[i][j][k].place_part_pose.position.z;
//                                part_in_tray.pose.orientation.x = master_vector_main[i][j][k].place_part_pose.orientation.x;
//                                part_in_tray.pose.orientation.y = master_vector_main[i][j][k].place_part_pose.orientation.y;
//                                part_in_tray.pose.orientation.z = master_vector_main[i][j][k].place_part_pose.orientation.z;
//                                part_in_tray.pose.orientation.w = master_vector_main[i][j][k].place_part_pose.orientation.w;

                                if (part_in_tray.pose.orientation.x == 1) {
                                    if(master_vector_main[i][j][k].agv_id=="agv1") {
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
                                    if(master_vector_main[i][j][k].agv_id=="agv2") {
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
                                    gantry.placePart_right_arm(part_in_tray, master_vector_main[i][j][k].agv_id);
                                    ROS_INFO_STREAM("Part placed");
                                } else {
                                    gantry.goToPresetLocation(gantry.start_);
                                    ROS_INFO_STREAM("Start location reached");
                                    gantry.placePart(part_in_tray, master_vector_main[i][j][k].agv_id);
                                    ROS_INFO_STREAM("Part placed");
                                    if (master_vector_main[i][j][k].agv_id == "agv2") {
                                        gantry.goToPresetLocation(gantry.agv2_);
                                        ROS_INFO_STREAM("AGV2 location reached");
                                    } else {
                                        gantry.goToPresetLocation(gantry.agv1_);
                                        ROS_INFO_STREAM("AGV2 location reached");
                                    }
                                }
                                fix_part_pose(comp, master_vector_main[i][j][k], gantry, part_in_tray);

                                faulty_part = comp.get_quality_sensor_status_agv2();
                                ROS_INFO_STREAM("Status of faulty part = ");
                                ROS_INFO_STREAM(faulty_part.faulty);
                                if (faulty_part.faulty == true) {
                                    part faulty_part;
                                    faulty_part.pose = gantry.getTargetWorldPose_dummy(faulty_part.pose,
                                                                                       master_vector_main[i][j][k].agv_id);
                                    ROS_INFO_STREAM("Black sheep location");
                                    ROS_INFO_STREAM(faulty_part.pose);
                                    faulty_part.type = parts_from_camera_main[l][m].type;
                                    faulty_part.pose.position.x = faulty_part.pose.position.x;
                                    faulty_part.pose.position.y = faulty_part.pose.position.y;
                                    faulty_part.pose.position.z = faulty_part.pose.position.z + 0.03;
                                    faulty_part.pose.orientation.x = faulty_part.pose.orientation.x;
                                    faulty_part.pose.orientation.y = faulty_part.pose.orientation.y;
                                    faulty_part.pose.orientation.z = faulty_part.pose.orientation.z;
                                    faulty_part.pose.orientation.w = faulty_part.pose.orientation.w;
                                    gantry.pickPart(faulty_part);
                                    gantry.goToPresetLocation(gantry.agv2_drop_);
                                    gantry.deactivateGripper("left_arm");
                                    ROS_INFO_STREAM(
                                            "BLack Sheeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeep MEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEHHHHHHHHHHHHHHHHHHHHH");
                                    ROS_INFO_STREAM("Go to Loop2 triggered");
                                    //                                        parts_from_camera_main = comp.get_parts_from_camera();
                                    goto LOOP2;
                                } else {
                                    //                                    comp.print_parts_to_pick();
                                    master_vector_main[i][j][k].delivered = true;
                                    parts_delivered[i]++;
                                    comp.setter_delivered(i, j, k);
                                    ROS_INFO_STREAM(" i j k" << i << j << k);
                                    ROS_INFO_STREAM(parts_from_camera_main[l][m].type << "  successfully delivered");
                                    ROS_INFO_STREAM("Go to Loop triggered");
                                    k++;
                                    if (comp.get_received_order_vector().size() > i + 1) {
                                        ROS_INFO_STREAM("NEW ORDER RECEIVED");
                                        i++;
                                        master_vector_main = comp.get_master_vector();
                                        goto LOOP3;
                                    }
                                    goto LOOP;
                                }
                            }

                        }
                    }
                }
                ROS_INFO_STREAM("Second for loop katham");
            }
        }
        k++;
    }
    }
    ROS_INFO_STREAM("No of parts delivered = " << parts_delivered[i]);
    ROS_INFO_STREAM("Order Size = " << size_of_order);
    if(parts_delivered[i] == size_of_order){
        ROS_INFO_STREAM(" Order " << i << "completed successfully");
        comp.delete_completed_order(i);
        ROS_INFO_STREAM(" Order " << i << "deleted successfully");
        ROS_INFO_STREAM("Size of vector now = " << comp.get_received_order_vector().size());

        ROS_INFO_STREAM("Delivering Shipment type = " << shipment_type << "  in agv = " <<agv_id);
        submitOrder(agv_id, shipment_type);
    }

}

    ROS_INFO_STREAM("FOR LOOP TERMINATED");

    gantry.goToPresetLocation(gantry.start_);
    ros::Duration timeout(5.0);
    if((i > 1) && (temp!= i-2))
    {
        ROS_INFO_STREAM("wwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwww");
        i = i-2;
        k = 0;
        ROS_INFO_STREAM("Executing Order = " << i-1);
        ROS_INFO_STREAM("Value of I = " << i);
        temp = i;
        goto LOOP3;
    }
    submitOrder("agv1", "order_0_shipment_0");
    submitOrder("agv2", "order_0_shipment_1");

//    submitOrder(2, "order_0_shipment_0");
    ROS_INFO_STREAM("Mangathaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa DaWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW");

    comp.endCompetition();
    spinner.stop();
    ros::shutdown();
    return 0;
}