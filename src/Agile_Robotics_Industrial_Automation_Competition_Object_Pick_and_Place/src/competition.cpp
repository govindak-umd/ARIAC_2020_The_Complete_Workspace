#include "competition.h"
#include "utils.h"
#include <nist_gear/LogicalCameraImage.h>
#include <nist_gear/Order.h>
#include <std_srvs/Trigger.h>
#include "gantry_control.h"
#include <string>
#include <vector>

int camera_no = 0;
part faulty_part_agv2;

std::vector<order> orders_vector;
std::vector<shipment> shipment_vector;
std::vector<product> product_vector;
std::vector<pick_and_place> pick_and_place_poses_vector;
std::array<std::array<part, 20>, 20>  parts_from_camera ;
std::vector<std::vector<std::vector<master_struct> > > master_vector (10,std::vector<std::vector<master_struct> >(10,std::vector <master_struct>(20)));

////////////////////////////////////////////////////

Competition::Competition(ros::NodeHandle & node): current_score_(0)
{
  node_ = node;
}

void Competition::init() {
  // Subscribe to the '/ariac/current_score' topic.
  double time_called = ros::Time::now().toSec();
  competition_start_time_ = ros::Time::now().toSec();

  // Subscribe to the '/ariac/competition_state' topic.
  ROS_INFO("Subscribe to the /ariac/competition_state topic...");
  competition_state_subscriber_ = node_.subscribe(
    "/ariac/competition_state", 10, &Competition::competition_state_callback, this);

  // Subscribe to the '/clock' topic.
  ROS_INFO("Subscribe to the /clock...");
  competition_clock_subscriber_ = node_.subscribe(
    "/clock", 10, &Competition::competition_clock_callback, this);

    ROS_INFO("Subscribe to the /orders...");
    orders_subscriber_ = node_.subscribe(
            "/ariac/orders", 10, &Competition::order_callback, this);

    ROS_INFO("Subscribe to the /ariac/quality_control_sensor_1");
    quality_control_sensor_1_subscriber_ = node_.subscribe(
            "/ariac/quality_control_sensor_1", 10, &Competition::quality_control_sensor_1_subscriber_callback, this);

  startCompetition();

  init_.total_time += ros::Time::now().toSec() - time_called;

}

void Competition::print_parts_detected(){
    for (int i = 0; i < parts_from_camera.size(); i++)
    {
        std::cout << "parts from camera = " << i << std::endl;
        std::cout << std::endl;
        for (int j = 0; j < parts_from_camera[i].size(); j++){
            std::cout << " " << parts_from_camera[i][j].type;
        }
        std::cout << std::endl;
        std::cout << std::endl;
    }
}


void Competition::pre_kitting()
{

    // Populating Orders vector
    for (int i =0; i < received_orders_.size(); i++)
    {
        order order_instance;
        order_instance.order_id = received_orders_[i].order_id;
        order_instance.shipments = received_orders_[i].shipments;
        orders_vector.push_back(order_instance);

        //Populating Shipment vector for each order
        for (int j = 0; j < orders_vector[i].shipments.size(); j++)
        {
            shipment shipment_instance;
            shipment_instance.shipment_type = orders_vector[i].shipments[j].shipment_type;
            shipment_instance.agv_id  = orders_vector[i].shipments[j].agv_id;
            shipment_instance.products  = orders_vector[i].shipments[j].products;

            shipment_vector.push_back(shipment_instance);

//            ROS_INFO_STREAM("==========================PARTS TO BE PICKED==============================");
            for (int k = 0; k < shipment_vector[j].products.size(); k++)
            {
//                ROS_INFO_STREAM(shipment_vector[j].products[k].type);
                if(shipment_vector[j].products[k].type == ("pulley_part_red") || ("pulley_part_blue") || ("pulley_part_green") || ("piston_part_red") || ("piston_part_green") || ("piston_part_blue") || ("disk_part_red") || ("disk_part_green") || ("disk_part_blue") || ("gasket_part_red") || ("gasket_part_green") || ("gasket_part_blue") ) {
                    ROS_INFO_STREAM("Part kidachiduchu doiiii");
                    ROS_INFO_STREAM(shipment_vector[j].products[k].pose);

                    part part_to_be_placed;
                    part_to_be_placed.type = shipment_vector[j].products[k].type;
                    part_to_be_placed.pose.position.x = shipment_vector[j].products[k].pose.position.x;
                    part_to_be_placed.pose.position.y = shipment_vector[j].products[k].pose.position.y;
                    part_to_be_placed.pose.position.z = shipment_vector[j].products[k].pose.position.z;
                    part_to_be_placed.pose.orientation.x = shipment_vector[j].products[k].pose.orientation.x;
                    part_to_be_placed.pose.orientation.y = shipment_vector[j].products[k].pose.orientation.y;
                    part_to_be_placed.pose.orientation.z = shipment_vector[j].products[k].pose.orientation.z;
                    part_to_be_placed.pose.orientation.w = shipment_vector[j].products[k].pose.orientation.w;

                    master_struct master_struct_instance;
                    master_struct_instance.type = shipment_vector[j].products[k].type;
                    master_struct_instance.place_part_pose.position.x = part_to_be_placed.pose.position.x;
                    master_struct_instance.place_part_pose.position.y = part_to_be_placed.pose.position.y;
                    master_struct_instance.place_part_pose.position.z = part_to_be_placed.pose.position.z;
                    master_struct_instance.place_part_pose.orientation.x = part_to_be_placed.pose.orientation.x;
                    master_struct_instance.place_part_pose.orientation.y = part_to_be_placed.pose.orientation.y;
                    master_struct_instance.place_part_pose.orientation.z = part_to_be_placed.pose.orientation.z;
                    master_struct_instance.place_part_pose.orientation.w = part_to_be_placed.pose.orientation.w;
                    master_struct_instance.order_id = order_instance.order_id;
                    master_struct_instance.shipment_type = shipment_instance.shipment_type;
                    master_struct_instance.agv_id = shipment_instance.agv_id;
                    master_vector[i][j][k] = master_struct_instance;
                }

//                product product_vector_instance;
//                product_vector_instance.type = part_to_be_placed.type;
//                product_vector_instance.pose.position.x = part_to_be_placed.pose.position.x;
//                product_vector_instance.pose.position.y = part_to_be_placed.pose.position.y;
//                product_vector_instance.pose.position.z = part_to_be_placed.pose.position.z;
//                product_vector_instance.pose.orientation.x = part_to_be_placed.pose.orientation.x;
//                product_vector_instance.pose.orientation.y = part_to_be_placed.pose.orientation.y;
//                product_vector_instance.pose.orientation.z = part_to_be_placed.pose.orientation.z;
//                product_vector_instance.pose.orientation.w = part_to_be_placed.pose.orientation.w;
//                product_vector_instance.agv_id = shipment_instance.agv_id;
//                product_vector.push_back(product_vector_instance);

//                Competition::during_kitting(part_to_be_placed);
            }
        }
    }
//    Competition::print_parts_to_pick();
//    ROS_INFO_STREAM("===================Thats all folks!!!!======================");
}

void Competition::print_parts_to_pick()
{
    for(int i=0; i < 10;  i++) {
        for (int j = 0; j < 10; j++) {
            for (int k = 0; k < 20; k++) {
                if((master_vector[i][j][k].type == "pulley_part_red") || (master_vector[i][j][k].type == "pulley_part_blue") || (master_vector[i][j][k].type == "pulley_part_green")|| (master_vector[i][j][k].type == "disk_part_blue")|| (master_vector[i][j][k].type == "disk_part_red")|| (master_vector[i][j][k].type == "disk_part_green")|| (master_vector[i][j][k].type == "piston_part_blue")|| (master_vector[i][j][k].type == "piston_part_green")|| (master_vector[i][j][k].type == "piston_part_red")|| (master_vector[i][j][k].type == "gasket_part_blue")|| (master_vector[i][j][k].type == "gasket_part_red")|| (master_vector[i][j][k].type == "gasket_part_green"))
                {
                    ROS_INFO_STREAM("Parts in master vector");
                    ROS_INFO_STREAM(master_vector[i][j][k].type);
                }
            }
        }
    }
}

std::array<std::array<part, 20>, 20> Competition::get_parts_from_camera()
{
    return parts_from_camera;
}

std::vector<std::vector<std::vector<master_struct> > > Competition::get_master_vector()
{
    return master_vector;
}


//void Competition::print_parts_detected(){
//    ROS_INFO_STREAM("EEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE");
//
//    for( auto &row : parts_from_camera)
//        for(auto &col : row)
//            ROS_INFO_STREAM(col.type);,
//}

part Competition::get_quality_sensor_status(){
    return faulty_part_agv2;
}


void Competition::logical_camera_callback(const nist_gear::LogicalCameraImage::ConstPtr & msg, int cam_idx)
{
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    std::ostringstream otopic;
    std::string topic;
    std::ostringstream otopic_part;
    std::string topic_part;

    geometry_msgs::PoseStamped pose_target, pose_rel;
    if(msg->models.size() != 0){

        // ROS_INFO_STREAM("Camera_id : " << cam_idx);
        // ROS_INFO_STREAM("Logical camera: '" << msg->models.size() << "' objects.");
        int part_no = 0;
//        ROS_INFO_STREAM("Parts detected by Logical camera " << cam_idx);
//        ROS_INFO_STREAM(" ");
        for(int i = 0; i<msg->models.size(); i++)
        {
            part_no++;
            otopic.str("");
            otopic.clear();
            otopic << "logical_camera_" << cam_idx << "_" << msg->models[i].type<< "_frame";
            topic = otopic.str();
            // ROS_INFO_STREAM(topic);
            ros::Duration timeout(5.0);
            geometry_msgs::TransformStamped transformStamped;
            pose_rel.header.frame_id = "logical_camera_" + std::to_string(cam_idx) + "_frame";
            pose_rel.pose = msg->models[i].pose;

            try{
                transformStamped = tfBuffer.lookupTransform("world", pose_rel.header.frame_id,
                                                            ros::Time(0), timeout);
            }
            catch (tf2::TransformException &ex) {
                ROS_WARN("%s",ex.what());
                ros::Duration(1.0).sleep();
                continue;
            }
            tf2::doTransform(pose_rel, pose_target, transformStamped);
            // ROS_INFO_STREAM("Camera coordinates of " << topic << " no of parts - " << msg->models.size());
            // ROS_INFO_STREAM(pose_target);

            double tx = pose_target.pose.position.x;
            double ty = pose_target.pose.position.y;
            double tz = pose_target.pose.position.z;

            // Orientation quaternion
            tf2::Quaternion q(
                    pose_target.pose.orientation.x,
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

            parts_from_camera[cam_idx][i].type = msg->models[i].type;
            parts_from_camera[cam_idx][i].pose.position.x = tx;
            parts_from_camera[cam_idx][i].pose.position.y = ty;
            parts_from_camera[cam_idx][i].pose.position.z = tz;
            parts_from_camera[cam_idx][i].pose.orientation.x = pose_target.pose.orientation.x;
            parts_from_camera[cam_idx][i].pose.orientation.y = pose_target.pose.orientation.y;
            parts_from_camera[cam_idx][i].pose.orientation.z = pose_target.pose.orientation.z;
            parts_from_camera[cam_idx][i].pose.orientation.w = pose_target.pose.orientation.w;
            parts_from_camera[cam_idx][i].faulty = false;
            parts_from_camera[cam_idx][i].picked = false;


//            parts_from_camera[i].type = msg->models[i].type;
//            parts_from_camera[i].pose.position.x = tx;
//            parts_from_camera[i].pose.position.y = ty;
//            parts_from_camera[i].pose.position.z = tz;
//            parts_from_camera[i].pose.orientation.x = pose_target.pose.orientation.x;
//            parts_from_camera[i].pose.orientation.y = pose_target.pose.orientation.y;
//            parts_from_camera[i].pose.orientation.z = pose_target.pose.orientation.z;
//            parts_from_camera[i].pose.orientation.w = pose_target.pose.orientation.w;


            // Output the measure
//            ROS_INFO("'%s' in '%s' frame : X: %.2f Y: %.2f Z: %.2f - R: %.2f P: %.2f Y: %.2f",
//                     topic.c_str(),
//                     pose_target.header.frame_id.c_str(),
//                     tx, ty, tz,
//                     roll, pitch, yaw);

        }

//        ROS_INFO_STREAM(" ");
//        ROS_INFO_STREAM(" ");
    }
}


/// Called when a new message is received.
void Competition::competition_state_callback(const std_msgs::String::ConstPtr & msg) {
  if (msg->data == "done" && competition_state_ != "done")
  {
    ROS_INFO("Competition ended.");
  }
  competition_state_ = msg->data;
}

void Competition::order_callback(const nist_gear::Order::ConstPtr & msg) {
//    ROS_INFO_STREAM("Received order:\n" << *msg);
    received_orders_.push_back(*msg);
    Competition::pre_kitting();
}

void Competition::quality_control_sensor_1_subscriber_callback(const nist_gear::LogicalCameraImage::ConstPtr & msg)
{
    if(msg->models.size() != 0) {
        for (int i = 0; i < msg->models.size(); i++) {
//            ROS_INFO_STREAM("Faulty Part Detected from Callback");
            faulty_part_agv2.pose = msg->models[i].pose;
            faulty_part_agv2.faulty = true;
        }
    }
    else
        faulty_part_agv2.faulty = false;
}

/// Called when a new message is received.
void Competition::competition_clock_callback(const rosgraph_msgs::Clock::ConstPtr & msg) {
  competition_clock_ = msg->clock;
}


void Competition::startCompetition() {
  // Create a Service client for the correct service, i.e. '/ariac/start_competition'.
  ros::ServiceClient start_client =
    node_.serviceClient<std_srvs::Trigger>("/ariac/start_competition");
  // If it's not already ready, wait for it to be ready.
  // Calling the Service using the client before the server is ready would fail.
  if (!start_client.exists()) {
    ROS_INFO("[competition][startCompetition] Waiting for the competition to be ready...");
    start_client.waitForExistence();
    ROS_INFO("[competition][startCompetition] Competition is now ready.");
  }
  ROS_INFO("[competition][startCompetition] Requesting competition start...");
  std_srvs::Trigger srv;
  start_client.call(srv);
  if (!srv.response.success) {  // If not successful, print out why.
    ROS_ERROR_STREAM("[competition][startCompetition] Failed to start the competition: " << srv.response.message);
  } else {
    ROS_INFO("[competition][startCompetition] Competition started!");
  }
}


void Competition::endCompetition() {
  // Create a Service client for the correct service, i.e. '/ariac/start_competition'.
  ros::ServiceClient end_client =
    node_.serviceClient<std_srvs::Trigger>("/ariac/end_competition");
  // If it's not already ready, wait for it to be ready.
  // Calling the Service using the client before the server is ready would fail.
  if (!end_client.exists()) {
    ROS_INFO("[competition][endCompetition] Waiting for the end_competition to be ready...");
    end_client.waitForExistence();
    ROS_INFO("[competition][endCompetition] end_competition is now ready.");
  }
  ROS_INFO("[competition][endCompetition] Requesting competition end...");
  std_srvs::Trigger srv;
  end_client.call(srv);
  if (!srv.response.success) {  // If not successful, print out why.
    ROS_ERROR_STREAM("[competition][endCompetition] Failed to end the competition: " << srv.response.message);
  } else {
    ROS_INFO("[competition][endCompetition] Competition ended!");
  }
}


stats Competition::getStats(std::string function) {
  if (function == "init") return init_;

}

double Competition::getStartTime() {
  return competition_start_time_;
}

double Competition::getClock() {
  double time_spent = competition_clock_.toSec();
  ROS_INFO_STREAM("[competition][getClock] competition time spent (getClock()) =" << time_spent);
  return time_spent;
}


std::string Competition::getCompetitionState() {
  return competition_state_;
}
