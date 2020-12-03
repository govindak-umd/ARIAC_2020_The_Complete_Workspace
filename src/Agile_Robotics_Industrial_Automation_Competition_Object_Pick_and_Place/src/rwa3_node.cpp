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

#define MAX_NUMBER_OF_CAMERAS 17
std::array<std::array<part, 20>, 20>  parts_from_camera_main ;
std::vector<std::vector<std::vector<master_struct> > > master_vector_main (10,std::vector<std::vector<master_struct> >(10,std::vector <master_struct>(20)));
bool part_placed = false;
int k = 0;
const double flip = -3.14159;
part faulty_part;

// AVG id(= 1,2) to identify what AVG to submit to
// shipment_type is the order type
bool submitOrder(int AVG_id, std::string shipment_type){
    ROS_INFO("[submitOrder] Submitting order via AVG");

    // Create a node to call service from. Would be better to use one existing node
    // rather than creating a new one every time
    ros::NodeHandle node;

    // Create a Service client for the correct service, i.e. '/ariac/agv{AVG_id}'
    ros::ServiceClient avg_client;

    // Assign the service client to the correct service
    if(AVG_id == 1){
        avg_client = node.serviceClient<nist_gear::AGVControl>("/ariac/agv1");
    }else if(AVG_id == 2){
        avg_client = node.serviceClient<nist_gear::AGVControl>("/ariac/agv2");
    }else{
        ROS_ERROR_STREAM("[submitOrder] No AVG with id " << AVG_id <<". Valid ids are 1 and 2 only");
    }

    // Wait for client to start
    if (!avg_client.exists()) {
        avg_client.waitForExistence();
    }

    // Debug what you're doing
    ROS_INFO_STREAM("[submitOrder] Sending AVG " << AVG_id << " to submit order");

    // Create the message and assign the shipment type to it
    nist_gear::AGVControl srv;
    srv.request.shipment_type = shipment_type;

    // Send message and retrieve response
    avg_client.call(srv);
    if (!srv.response.success) {  // If not successful, print out why.
        ROS_ERROR_STREAM("[submitOrder]  Failed to submit: " << srv.response.message);
    } else {
        ROS_INFO("[submitOrder] Submitted");
    }

    return srv.response.success;
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

    for(int i=0; i < 10;  i++) {

        for (int j = 0; j < 10; j++) {
            LOOP:while(k< 20)
            {
                ROS_INFO_STREAM("loop reached, part not faulty");
//                for (int k = 0; k < 20; k++){
                ROS_INFO_STREAM("NEWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW part");
                ROS_INFO_STREAM(i << j << k);
                if((master_vector_main[i][j][k].type == "pulley_part_red") || (master_vector_main[i][j][k].type == "pulley_part_blue") || (master_vector_main[i][j][k].type == "pulley_part_green")|| (master_vector_main[i][j][k].type == "disk_part_blue")|| (master_vector_main[i][j][k].type == "disk_part_red")|| (master_vector_main[i][j][k].type == "disk_part_green")|| (master_vector_main[i][j][k].type == "piston_part_blue")|| (master_vector_main[i][j][k].type == "piston_part_green")|| (master_vector_main[i][j][k].type == "piston_part_red")|| (master_vector_main[i][j][k].type == "gasket_part_blue")|| (master_vector_main[i][j][k].type == "gasket_part_red")|| (master_vector_main[i][j][k].type == "gasket_part_green"))
                {
                    part_placed = false;
                    LOOP2:for (int l = 0 ; l < parts_from_camera_main.size(); l++)
                    {
                        ROS_INFO_STREAM("Loop 2 reached to replace faulty part");
                        for (int m = 0; m < parts_from_camera_main[i].size(); m++)
                        {
//                            parts_from_camera_main = comp.get_parts_from_camera();
                            if ((master_vector_main[i][j][k].type == parts_from_camera_main[l][m].type) && (parts_from_camera_main[l][m].faulty == false) &&(parts_from_camera_main[l][m].picked == false))
                            {
                                ROS_INFO_STREAM("part_from_camera_index"<< l << m);
                                parts_from_camera_main[l][m].picked = true;
                                ROS_INFO_STREAM("picked status "<< parts_from_camera_main[l][m].picked);
                                ROS_INFO_STREAM("Part found in environment");
                                ROS_INFO_STREAM(parts_from_camera_main[l][m].type);
                                if (master_vector_main[i][j][k].type == "disk_part_blue")
                                {
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

                                    if(master_vector_main[i][j][k].agv_id == "agv2")
                                    {
                                        gantry.goToPresetLocation(gantry.agv2_);
                                        ROS_INFO_STREAM("AGV2 location reached");
                                    }

                                    faulty_part = comp.get_quality_sensor_status();
                                    ROS_INFO_STREAM("Status of faulty part = ");
                                    ROS_INFO_STREAM(faulty_part.faulty);
                                    if(faulty_part.faulty == true)
                                    {
                                        part faulty_part;
                                        faulty_part.pose = gantry.getTargetWorldPose_dummy(faulty_part.pose, master_vector_main[i][j][k].agv_id);
                                        ROS_INFO_STREAM("Black sheep location");
                                        ROS_INFO_STREAM(faulty_part.pose);
                                        faulty_part.type = parts_from_camera_main[l][m].type;
                                        faulty_part.pose.position.x = faulty_part.pose.position.x;
                                        faulty_part.pose.position.y = faulty_part.pose.position.y;
                                        faulty_part.pose.position.z = faulty_part.pose.position.z+0.03;
                                        faulty_part.pose.orientation.x = faulty_part.pose.orientation.x;
                                        faulty_part.pose.orientation.y = faulty_part.pose.orientation.y;
                                        faulty_part.pose.orientation.z = faulty_part.pose.orientation.z;
                                        faulty_part.pose.orientation.w = faulty_part.pose.orientation.w;
                                        gantry.pickPart(faulty_part);
                                        gantry.goToPresetLocation(gantry.agv2_drop_);
                                        gantry.deactivateGripper("left_arm");
                                        ROS_INFO_STREAM("BLack Sheeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeep MEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEHHHHHHHHHHHHHHHHHHHHH");
                                        ROS_INFO_STREAM("Go to Loop2 triggered");
//                                        parts_from_camera_main = comp.get_parts_from_camera();
                                        goto LOOP2;
                                    }
                                    else {
                                        k++;
                                        ROS_INFO_STREAM("Go to Loop Triggered");
                                        goto LOOP;
                                    }

                                }
                                else if (master_vector_main[i][j][k].type == "disk_part_green")
                                {
                                    ROS_INFO_STREAM(master_vector_main[i][j][k].place_part_pose);
                                    ROS_INFO_STREAM(parts_from_camera_main[l][m].pose);
                                    std::string location = "bin_16";
                                    gantry.goToPresetLocation(gantry.start_);
                                    ROS_INFO_STREAM("Start location reached");
                                    gantry.goToPresetLocation(gantry.bin16_);
                                    ROS_INFO_STREAM("bin16 location reached");
                                    gantry.pickPart(parts_from_camera_main[l][m]);
                                    ROS_INFO_STREAM("Part picked");
                                    gantry.goToPresetLocation(gantry.bin16_);
                                    ROS_INFO_STREAM("bin16 location reached");

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

                                    if(master_vector_main[i][j][k].agv_id == "agv2")
                                    {
                                        gantry.goToPresetLocation(gantry.agv2_);
                                        ROS_INFO_STREAM("AGV2 location reached");
                                    }

                                    faulty_part = comp.get_quality_sensor_status();
                                    ROS_INFO_STREAM("Status of faulty part = ");
                                    ROS_INFO_STREAM(faulty_part.faulty);
                                    if(faulty_part.faulty == true)
                                    {
                                        part faulty_part;
                                        faulty_part.pose = gantry.getTargetWorldPose_dummy(faulty_part.pose, master_vector_main[i][j][k].agv_id);
                                        ROS_INFO_STREAM("Black sheep location");
                                        ROS_INFO_STREAM(faulty_part.pose);
                                        faulty_part.type = parts_from_camera_main[l][m].type;
                                        faulty_part.pose.position.x = faulty_part.pose.position.x;
                                        faulty_part.pose.position.y = faulty_part.pose.position.y;
                                        faulty_part.pose.position.z = faulty_part.pose.position.z+0.03;
                                        faulty_part.pose.orientation.x = faulty_part.pose.orientation.x;
                                        faulty_part.pose.orientation.y = faulty_part.pose.orientation.y;
                                        faulty_part.pose.orientation.z = faulty_part.pose.orientation.z;
                                        faulty_part.pose.orientation.w = faulty_part.pose.orientation.w;
                                        gantry.pickPart(faulty_part);
                                        gantry.goToPresetLocation(gantry.agv2_drop_);
                                        gantry.deactivateGripper("left_arm");
                                        ROS_INFO_STREAM("BLack Sheeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeep MEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEHHHHHHHHHHHHHHHHHHHHH");
                                        ROS_INFO_STREAM("Go to Loop2 triggered");
//                                        parts_from_camera_main = comp.get_parts_from_camera();
                                        goto LOOP2;
                                    }
                                    else {
                                        k++;
                                        ROS_INFO_STREAM("Go to Loop triggered");
                                        goto LOOP;
                                    }
                                }
                                else if (master_vector_main[i][j][k].type == "pulley_part_red")
                                {
                                    ROS_INFO_STREAM(master_vector_main[i][j][k].place_part_pose);
                                    ROS_INFO_STREAM(parts_from_camera_main[l][m].pose);
                                    std::string location = "shelf5";
                                    gantry.goToPresetLocation(gantry.start_);
                                    ROS_INFO_STREAM("Start location reached");
                                    gantry.goToPresetLocation(gantry.waypoint_1_);
                                    ROS_INFO_STREAM("waypont1 location reached");
                                    gantry.goToPresetLocation(gantry.waypoint_2_);
                                    ROS_INFO_STREAM("waypoint2 location reached");
                                    gantry.goToPresetLocation(gantry.waypoint_3_);
                                    ROS_INFO_STREAM("waypoint3 location reached");
                                    gantry.goToPresetLocation(gantry.waypoint_4_);
                                    ROS_INFO_STREAM("waypoint4 location reached");
                                    gantry.pickPart(parts_from_camera_main[l][m]);
                                    ROS_INFO_STREAM("Part picked");
                                    gantry.goToPresetLocation(gantry.waypoint_4_);
                                    ROS_INFO_STREAM("waypoint4 location reached");
                                    gantry.goToPresetLocation(gantry.waypoint_3_);
                                    ROS_INFO_STREAM("waypoint3 location reached");
                                    gantry.goToPresetLocation(gantry.waypoint_2_);
                                    ROS_INFO_STREAM("waypoint2 location reached");
                                    gantry.goToPresetLocation(gantry.waypoint_1_);
                                    ROS_INFO_STREAM("waypoint1 location reached");

                                    part part_in_tray;
                                    part_in_tray.type = master_vector_main[i][j][k].type;
                                    part_in_tray.pose.position.x = master_vector_main[i][j][k].place_part_pose.position.x;
                                    part_in_tray.pose.position.y = master_vector_main[i][j][k].place_part_pose.position.y;
                                    part_in_tray.pose.position.z = master_vector_main[i][j][k].place_part_pose.position.z;
                                    part_in_tray.pose.orientation.x = master_vector_main[i][j][k].place_part_pose.orientation.x;
                                    part_in_tray.pose.orientation.y = master_vector_main[i][j][k].place_part_pose.orientation.y;
                                    part_in_tray.pose.orientation.z = master_vector_main[i][j][k].place_part_pose.orientation.z;
                                    part_in_tray.pose.orientation.w = master_vector_main[i][j][k].place_part_pose.orientation.w;

                                    tf2::Quaternion q(
                                            part_in_tray.pose.orientation.x,
                                            part_in_tray.pose.orientation.y,
                                            part_in_tray.pose.orientation.z,
                                            part_in_tray.pose.orientation.w);

                                    // 3x3 Rotation matrix from quaternion
                                    tf2::Matrix3x3 m1(q);

                                    // Roll Pitch and Yaw from rotation matrix
                                    double roll, pitch, yaw;
                                    m1.getRPY(roll, pitch, yaw);
                                    ROS_INFO_STREAM("Rollllllllllllllllllllllllllllllllllllllllllll" << roll);
                                    ROS_INFO_STREAM("Flippppppppppppppppppppppppppppppppppppppppppp" << flip);
                                    roll = (double)roll;
                                    if(part_in_tray.pose.orientation.x == 1)
                                    {
                                        gantry.activateGripper("right_arm");
                                        ros::Duration(2).sleep();

                                        ROS_INFO_STREAM("Right Gripper activated");
                                        ROS_INFO_STREAM("Thirupi Podu Dosaiiiiiii");
                                        gantry.goToPresetLocation(gantry.agv2_flip_);
                                        ROS_INFO_STREAM("AGV 2 location reached");
//
//                                        ros::Duration(0.5).sleep();
//                                        ROS_INFO_STREAM("Right Gripper activated");
                                        gantry.goToPresetLocation(gantry.pose_change_1);
                                        ROS_INFO_STREAM("Flipping pose1 reached");
                                        gantry.goToPresetLocation(gantry.pose_change_2);
                                        ROS_INFO_STREAM("Flipping pose2 reached");
                                        gantry.deactivateGripper("left_arm");
                                        ros::Duration(2).sleep();
                                        ROS_INFO_STREAM("Left Gripper Disabled");
                                        gantry.goToPresetLocation(gantry.flip_target_);
                                        ROS_INFO_STREAM("Reached AGV");

                                        roll = 0; //Setting roll back to zero after flipping
                                        tf2::Quaternion myQuaternion;
                                        myQuaternion.setRPY(roll, pitch, yaw);  // Create this quaternion from roll/pitch/yaw (in radians)
                                        ROS_INFO_STREAM("Printing Roll pitch yaw values");
                                        ROS_INFO_STREAM(roll << pitch << yaw);
                                        ROS_INFO_STREAM("Printing Quaternion values");
                                        ROS_INFO_STREAM(q[0]);
                                        ROS_INFO_STREAM(q[1]);
                                        ROS_INFO_STREAM(q[2]);
                                        ROS_INFO_STREAM(q[3]);

//                                        part_in_tray.pose.orientation.x = q[0];
//                                        part_in_tray.pose.orientation.y = q[1];
//                                        part_in_tray.pose.orientation.z = q[2];
//                                        part_in_tray.pose.orientation.w = q[3];

                                        part_in_tray.pose.orientation.x = 0;
                                        part_in_tray.pose.orientation.y = 0;
                                        part_in_tray.pose.orientation.z = 0;
                                        part_in_tray.pose.orientation.w = 1;

                                        gantry.placePart_right_arm(part_in_tray, master_vector_main[i][j][k].agv_id);
                                        ROS_INFO_STREAM("Part placed");
                                    }

                                    else {
                                        gantry.goToPresetLocation(gantry.start_);
                                        ROS_INFO_STREAM("Start location reached");
                                        gantry.placePart(part_in_tray, master_vector_main[i][j][k].agv_id);
                                        ROS_INFO_STREAM("Part placed");
                                        if(master_vector_main[i][j][k].agv_id == "agv2")
                                        {
                                            gantry.goToPresetLocation(gantry.agv2_);
                                            ROS_INFO_STREAM("AGV2 location reached");
                                        }
                                    }

                                    faulty_part = comp.get_quality_sensor_status();
                                    ROS_INFO_STREAM("Status of faulty part = ");
                                    ROS_INFO_STREAM(faulty_part.faulty);
                                    if(faulty_part.faulty == true)
                                    {
                                        part faulty_part;
                                        faulty_part.pose = gantry.getTargetWorldPose_dummy(faulty_part.pose, master_vector_main[i][j][k].agv_id);
                                        ROS_INFO_STREAM("Black sheep location");
                                        ROS_INFO_STREAM(faulty_part.pose);
                                        faulty_part.type = parts_from_camera_main[l][m].type;
                                        faulty_part.pose.position.x = faulty_part.pose.position.x;
                                        faulty_part.pose.position.y = faulty_part.pose.position.y;
                                        faulty_part.pose.position.z = faulty_part.pose.position.z+0.03;
                                        faulty_part.pose.orientation.x = faulty_part.pose.orientation.x;
                                        faulty_part.pose.orientation.y = faulty_part.pose.orientation.y;
                                        faulty_part.pose.orientation.z = faulty_part.pose.orientation.z;
                                        faulty_part.pose.orientation.w = faulty_part.pose.orientation.w;
                                        gantry.pickPart(faulty_part);
                                        gantry.goToPresetLocation(gantry.agv2_drop_);
                                        gantry.deactivateGripper("left_arm");
                                        ROS_INFO_STREAM("BLack Sheeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeep MEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEHHHHHHHHHHHHHHHHHHHHH");
                                        ROS_INFO_STREAM("Go to Loop2 triggered");
//                                        parts_from_camera_main = comp.get_parts_from_camera();
                                        goto LOOP2;
                                    }
                                    else {
                                        k++;
                                        ROS_INFO_STREAM("Go to Loop triggered");
                                        goto LOOP;
                                    }
                                }

                            }
                        }
                    }
                    ROS_INFO_STREAM("Second for loop katham");
                }
                k++;
            }
        }
    }

    gantry.goToPresetLocation(gantry.start_);
    submitOrder(2, "order_0_shipment_0");
    ROS_INFO_STREAM("Mangathaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa DaWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW");

    comp.endCompetition();
    spinner.stop();
    ros::shutdown();
    return 0;
}