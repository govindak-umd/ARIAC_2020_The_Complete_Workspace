#include "competition.h"
#include "utils.h"
#include <nist_gear/LogicalCameraImage.h>
#include <nist_gear/Proximity.h>
#include <nist_gear/Order.h>
#include <std_srvs/Trigger.h>
#include "gantry_control.h"
#include <string>
#include <vector>

int p =0;
int camera_no = 0;
part faulty_part_agv2;
part faulty_part_agv1;

std::vector<order> orders_vector;
std::vector<shipment> shipment_vector;
std::vector<product> product_vector;
std::vector<pick_and_place> pick_and_place_poses_vector;
std::array<std::array<part, 20>, 20>  parts_from_camera ;
std::vector<std::vector<std::vector<master_struct> > > master_vector (10,std::vector<std::vector<master_struct> >(10,std::vector <master_struct>(20)));
//shelf vector
std::vector<std::vector<double>> shelf_vector(9,std::vector<double>(3));

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

    ROS_INFO("Subscribe to the /ariac/quality_control_sensor_1"); //AGV2
    quality_control_sensor_1_subscriber_ = node_.subscribe(
            "/ariac/quality_control_sensor_1", 10, &Competition::quality_control_sensor_1_subscriber_callback, this);

    ROS_INFO("Subscribe to the /ariac/quality_control_sensor_2"); //AGV1
    quality_control_sensor_2_subscriber_ = node_.subscribe(
            "/ariac/quality_control_sensor_2", 10, &Competition::quality_control_sensor_2_subscriber_callback, this);

    ROS_INFO("Subscribe to the /ariac/breakbeam_0");
    breakbeam_sensor_0_subscriber_ = node_.subscribe(
            "/ariac/breakbeam_0", 10, &Competition::breakbeam_sensor_0_callback, this);

    ROS_INFO("Subscribe to the /ariac/breakbeam_1");
    breakbeam_sensor_1_subscriber_ = node_.subscribe(
            "/ariac/breakbeam_1", 10, &Competition::breakbeam_sensor_1_callback, this);

    ROS_INFO("Subscribe to the /ariac/breakbeam_2");
    breakbeam_sensor_2_subscriber_ = node_.subscribe(
            "/ariac/breakbeam_2", 10, &Competition::breakbeam_sensor_2_callback, this);

    ROS_INFO("Subscribe to the /ariac/breakbeam_3");
    breakbeam_sensor_3_subscriber_ = node_.subscribe(
            "/ariac/breakbeam_3", 10, &Competition::breakbeam_sensor_3_callback, this);

    ROS_INFO("Subscribe to the /ariac/breakbeam_4");
    breakbeam_sensor_4_subscriber_ = node_.subscribe(
            "/ariac/breakbeam_4", 10, &Competition::breakbeam_sensor_4_callback, this);

    ROS_INFO("Subscribe to the /ariac/breakbeam_5");
    breakbeam_sensor_5_subscriber_ = node_.subscribe(
            "/ariac/breakbeam_5", 10, &Competition::breakbeam_sensor_5_callback, this);

    ROS_INFO("Subscribe to the /ariac/breakbeam_6");
    breakbeam_sensor_6_subscriber_ = node_.subscribe(
            "/ariac/breakbeam_6", 10, &Competition::breakbeam_sensor_6_callback, this);

    ROS_INFO("Subscribe to the /ariac/breakbeam_7");
    breakbeam_sensor_7_subscriber_ = node_.subscribe(
            "/ariac/breakbeam_7", 10, &Competition::breakbeam_sensor_7_callback, this);

    ROS_INFO("Subscribe to the /ariac/breakbeam_8");
    breakbeam_sensor_8_subscriber_ = node_.subscribe(
            "/ariac/breakbeam_8", 10, &Competition::breakbeam_sensor_8_callback, this);

    ROS_INFO("Subscribe to the /ariac/breakbeam_9");
    breakbeam_sensor_9_subscriber_ = node_.subscribe(
            "/ariac/breakbeam_9", 10, &Competition::breakbeam_sensor_9_callback, this);

    ROS_INFO("Subscribe to the /ariac/breakbeam_10");
    breakbeam_sensor_10_subscriber_ = node_.subscribe(
            "/ariac/breakbeam_10", 10, &Competition::breakbeam_sensor_10_callback, this);



  startCompetition();

  init_.total_time += ros::Time::now().toSec() - time_called;

}

void Competition::breakbeam_sensor_0_callback(const nist_gear::Proximity::ConstPtr & msg){
    breakbeam_conveyor_belt_part_status_0 = msg->object_detected;
}

void Competition::breakbeam_sensor_1_callback(const nist_gear::Proximity::ConstPtr & msg){
    breakbeam_conveyor_belt_part_status_1 = msg->object_detected;
}

void Competition::breakbeam_sensor_2_callback(const nist_gear::Proximity::ConstPtr & msg){
    breakbeam_part_status_2 = msg->object_detected;
    if(breakbeam_part_status_2 == true){
        human_detected=1;
    }
}

void Competition::breakbeam_sensor_3_callback(const nist_gear::Proximity::ConstPtr & msg){
    breakbeam_part_status_3 = msg->object_detected;
    if(breakbeam_part_status_3 == true){
        human_detected=1;
    }
}

void Competition::breakbeam_sensor_4_callback(const nist_gear::Proximity::ConstPtr & msg){
    breakbeam_part_status_4 = msg->object_detected;
    if(breakbeam_part_status_4 == true){
        human_detected=1;
    }
}

void Competition::breakbeam_sensor_5_callback(const nist_gear::Proximity::ConstPtr & msg){
    breakbeam_part_status_5 = msg->object_detected;
    if(breakbeam_part_status_5 == true){
        human_detected=1;
    }
}

void Competition::breakbeam_sensor_6_callback(const nist_gear::Proximity::ConstPtr & msg){
    breakbeam_part_status_6 = msg->object_detected;
    if(breakbeam_part_status_6 == true){
        human_detected=1;
    }
}

void Competition::breakbeam_sensor_7_callback(const nist_gear::Proximity::ConstPtr & msg){
    breakbeam_part_status_7 = msg->object_detected;
    if(breakbeam_part_status_7 == true){
        human_detected=1;
    }
}

void Competition::breakbeam_sensor_8_callback(const nist_gear::Proximity::ConstPtr & msg){
    breakbeam_part_status_8 = msg->object_detected;
    if(breakbeam_part_status_8 == true){
        human_detected=1;
    }
}

void Competition::breakbeam_sensor_9_callback(const nist_gear::Proximity::ConstPtr & msg){
    breakbeam_part_status_9 = msg->object_detected;
    if(breakbeam_part_status_9 == true){
        human_detected=1;
    }
}

void Competition::breakbeam_sensor_10_callback(const nist_gear::Proximity::ConstPtr & msg){
    breakbeam_part_status_10 = msg->object_detected;
    if(breakbeam_part_status_10 == true){
        human_detected=1;
    }
}


//checks if a human was EVER detected
// can include functionality for which aisle as well
int Competition::get_human_existence(){
    return human_detected;
}

void Competition::shelf_callback(std::string shelf_name)
{
//    ros::init(argc, argv, "getShelfDistances");
//    ros::NodeHandle node;
    tf::TransformListener listener;
    ros::Rate rate(10.0);
    while (node_.ok()) {
        tf::StampedTransform transform;
        try {
            ROS_INFO_STREAM(shelf_name);
            listener.lookupTransform("/world", shelf_name,
                                     ros::Time(0), transform);
            tf::Transform tf(transform.getBasis(), transform.getOrigin());
            tf::Vector3 tfVec;
            tf::Matrix3x3 tfR;
            tf::Quaternion quat;
            tfVec = tf.getOrigin();
            ROS_INFO_STREAM(double(tfVec.getX()));
            if (shelf_name == "/shelf3_frame"){
                shelf_vector[0][0] = double(tfVec.getX());
                shelf_vector[0][1] = double(tfVec.getY());
                shelf_vector[0][2] = double(tfVec.getZ());
            }
            if (shelf_name == "/shelf4_frame"){
                shelf_vector[1][0] = double(tfVec.getX());
                shelf_vector[1][1] = double(tfVec.getY());
                shelf_vector[1][2] = double(tfVec.getZ());
            }
            if (shelf_name == "/shelf5_frame"){
                shelf_vector[2][0] = double(tfVec.getX());
                shelf_vector[2][1] = double(tfVec.getY());
                shelf_vector[2][2] = double(tfVec.getZ());
            }
            if (shelf_name == "/shelf6_frame"){
                shelf_vector[3][0] = double(tfVec.getX());
                shelf_vector[3][1] = double(tfVec.getY());
                shelf_vector[3][2] = double(tfVec.getZ());
            }
            if (shelf_name == "/shelf7_frame"){
                shelf_vector[4][0] = double(tfVec.getX());
                shelf_vector[4][1] = double(tfVec.getY());
                shelf_vector[4][2] = double(tfVec.getZ());
            }
            if (shelf_name == "/shelf8_frame"){
                shelf_vector[5][0] = double(tfVec.getX());
                shelf_vector[5][1] = double(tfVec.getY());
                shelf_vector[5][2] = double(tfVec.getZ());
            }
            if (shelf_name == "/shelf9_frame"){
                shelf_vector[6][0] = double(tfVec.getX());
                shelf_vector[6][1] = double(tfVec.getY());
                shelf_vector[6][2] = double(tfVec.getZ());
            }
            if (shelf_name == "/shelf10_frame"){
                shelf_vector[7][0] = double(tfVec.getX());
                shelf_vector[7][1] = double(tfVec.getY());
                shelf_vector[7][2] = double(tfVec.getZ());
            }
            if (shelf_name == "/shelf11_frame"){
                shelf_vector[8][0] = double(tfVec.getX());
                shelf_vector[8][1] = double(tfVec.getY());
                shelf_vector[8][2] = double(tfVec.getZ());
            }
            ROS_INFO_STREAM(tfVec.getX() << "," << tfVec.getY() << "," << tfVec.getZ());
            break;

        }
        catch (tf::TransformException ex) {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
        }
    }
}

std::vector<std::vector<double>> Competition::get_shelf_vector(){
    return shelf_vector;
}

void Competition::setter_delivered(int i, int j, int k)
{
    master_vector[i][j][k].delivered = true;
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

std::vector<nist_gear::Order> Competition::get_received_order_vector()
{
 return received_orders_;
}

void Competition::pre_kitting()
{
    // Populating Orders vector
    for (p; p < received_orders_.size(); p++)
    {
//        ROS_INFO_STREAM("ORDER NUMBER    =    " << p);
        order order_instance;
        order_instance.order_id = received_orders_[p].order_id;
//        ROS_INFO_STREAM("Order ID = " << order_instance.order_id);
        order_instance.shipments = received_orders_[p].shipments;
        orders_vector.push_back(order_instance);

        //Populating Shipment vector for each order
        for (int j = 0; j < orders_vector[p].shipments.size(); j++)
        {
            shipment shipment_instance;
            shipment_instance.shipment_type = orders_vector[p].shipments[j].shipment_type;
//            ROS_INFO_STREAM("Shipment ID = " << shipment_instance.shipment_type);
            shipment_instance.agv_id  = orders_vector[p].shipments[j].agv_id;
//            ROS_INFO_STREAM("AGV ID = " << orders_vector[p].shipments[j].agv_id);
//            ROS_INFO_STREAM("PRODUCT  = " << orders_vector[p].shipments[j].products.size());
            shipment_instance.products  = orders_vector[p].shipments[j].products;
            shipment_vector.push_back(shipment_instance);
//            ROS_INFO_STREAM("Size of the order = " << orders_vector[p].shipments[j].products.size());
//            ROS_INFO_STREAM("Product type = " << orders_vector[p].shipments[j].products[0].type);



//            ROS_INFO_STREAM("==========================PARTS TO BE PICKED==============================");
            for (int k = 0; k < orders_vector[p].shipments[j].products.size(); k++)
            {
                if(shipment_vector[j].products[k].type == ("pulley_part_red") || ("pulley_part_blue") || ("pulley_part_green") || ("piston_part_red") || ("piston_part_green") || ("piston_part_blue") || ("disk_part_red") || ("disk_part_green") || ("disk_part_blue") || ("gasket_part_red") || ("gasket_part_green") || ("gasket_part_blue") ) {
//                    ROS_INFO_STREAM("Part from prekitting function");
//                    ROS_INFO_STREAM(orders_vector[p].shipments[j].products[k].type);
//                    ROS_INFO_STREAM(orders_vector[p].shipments[j].products[k].pose);

                    part part_to_be_placed;
                    part_to_be_placed.type = orders_vector[p].shipments[j].products[k].type;
                    part_to_be_placed.pose.position.x = orders_vector[p].shipments[j].products[k].pose.position.x;
                    part_to_be_placed.pose.position.y = orders_vector[p].shipments[j].products[k].pose.position.y;
                    part_to_be_placed.pose.position.z = orders_vector[p].shipments[j].products[k].pose.position.z;
                    part_to_be_placed.pose.orientation.x = orders_vector[p].shipments[j].products[k].pose.orientation.x;
                    part_to_be_placed.pose.orientation.y = orders_vector[p].shipments[j].products[k].pose.orientation.y;
                    part_to_be_placed.pose.orientation.z = orders_vector[p].shipments[j].products[k].pose.orientation.z;
                    part_to_be_placed.pose.orientation.w = orders_vector[p].shipments[j].products[k].pose.orientation.w;

                    master_struct master_struct_instance;
                    master_struct_instance.type = part_to_be_placed.type;
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
                    master_vector[p][j][k] = master_struct_instance;
                }
            }
        }
    }

//    ROS_INFO_STREAM(" P = " << p);
//    Competition::print_parts_to_pick();
//    ROS_INFO_STREAM("===================Thats all folks!!!!======================");
}

void Competition::print_parts_to_pick()
{
    ROS_INFO_STREAM("Parts in master vector");
    for(int i=0; i < 10;  i++) {
        ROS_INFO_STREAM("ORder No = " << i);
        for (int j = 0; j < 10; j++) {
//            ROS_INFO_STREAM("SHipment NO = " << j);
            for (int k = 0; k < 20; k++) {
                if((master_vector[i][j][k].type == "pulley_part_red") || (master_vector[i][j][k].type == "pulley_part_blue") || (master_vector[i][j][k].type == "pulley_part_green")|| (master_vector[i][j][k].type == "disk_part_blue")|| (master_vector[i][j][k].type == "disk_part_red")|| (master_vector[i][j][k].type == "disk_part_green")|| (master_vector[i][j][k].type == "piston_part_blue")|| (master_vector[i][j][k].type == "piston_part_green")|| (master_vector[i][j][k].type == "piston_part_red")|| (master_vector[i][j][k].type == "gasket_part_blue")|| (master_vector[i][j][k].type == "gasket_part_red")|| (master_vector[i][j][k].type == "gasket_part_green"))
                {
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

void Competition::delete_completed_order(int i) {
    received_orders_.erase(received_orders_.begin() + i);
    ROS_INFO_STREAM("Deleting Order = " << i);
}


//void Competition::print_parts_detected(){
//    ROS_INFO_STREAM("EEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE");
//
//    for( auto &row : parts_from_camera)
//        for(auto &col : row)
//            ROS_INFO_STREAM(col.type);,
//}

part Competition::get_quality_sensor_status_agv2(){
    return faulty_part_agv2;
}

part Competition::get_quality_sensor_status_agv1(){
    return faulty_part_agv1;
}


void Competition::logical_camera_callback(const nist_gear::LogicalCameraImage::ConstPtr & msg, int cam_idx)
{
    std::vector<part> parts_from_15_camera_new;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    std::ostringstream otopic;
    std::string topic;
    std::ostringstream otopic_part;
    std::string topic_part;

    geometry_msgs::PoseStamped pose_target, pose_rel;
    if(msg->models.size() != 0){
        int part_no = 0;
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

            if (cam_idx == 15){
//                ROS_INFO_STREAM("Parts detected by 15 camera = " << msg->models.size());
                if ((msg->models[i].type == "pulley_part_red") ||
                    (msg->models[i].type == "pulley_part_blue") ||
                    (msg->models[i].type == "pulley_part_green") ||
                    (msg->models[i].type == "disk_part_blue") ||
                    (msg->models[i].type == "disk_part_red") ||
                    (msg->models[i].type == "disk_part_green") ||
                    (msg->models[i].type == "piston_rod_part_blue") ||
                    (msg->models[i].type == "piston_rod_part_green") ||
                    (msg->models[i].type == "piston_rod_part_red") ||
                    (msg->models[i].type == "gasket_part_blue") ||
                    (msg->models[i].type == "gasket_part_red") ||
                    (msg->models[i].type == "gasket_part_green")) {
//                    ROS_INFO_STREAM(msg->models[i].type << " found on belt");
                    part part_under_camera15;
                    part_under_camera15.type = msg->models[i].type;
                    part_under_camera15.pose.position.x = tx;
                    part_under_camera15.pose.position.y = ty;
                    part_under_camera15.pose.position.z = tz;
                    part_under_camera15.pose.orientation.x = pose_target.pose.orientation.x;
                    part_under_camera15.pose.orientation.y = pose_target.pose.orientation.y;
                    part_under_camera15.pose.orientation.z = pose_target.pose.orientation.z;
                    part_under_camera15.pose.orientation.w = pose_target.pose.orientation.w;
                    part_under_camera15.faulty = false;
                    part_under_camera15.picked = false;
                    parts_from_15_camera_new.push_back(part_under_camera15);
                    parts_from_15_camera = parts_from_15_camera_new;

//                    parts_from_15_camera[i].type = msg->models[i].type;
//                    parts_from_15_camera[i].pose.position.x = tx;
//                    parts_from_15_camera[i].pose.position.y = ty;
//                    parts_from_15_camera[i].pose.position.z = tz;
//                    parts_from_15_camera[i].pose.orientation.x = pose_target.pose.orientation.x;
//                    parts_from_15_camera[i].pose.orientation.y = pose_target.pose.orientation.y;
//                    parts_from_15_camera[i].pose.orientation.z = pose_target.pose.orientation.z;
//                    parts_from_15_camera[i].pose.orientation.w = pose_target.pose.orientation.w;
//                    parts_from_15_camera[i].faulty = false;
//                    parts_from_15_camera[i].picked = false;

                    if(msg->models.size() > 0)
                    {
//                        ROS_INFO_STREAM("Camera Matrix loaded with conveyor belt part");
                        conveyor_belt_part_status = true;
                    }

                }
            }
            if (cam_idx == 11){
                if (!((msg->models[i].type).empty())) {
                    parts_from_11_camera[i].type = msg->models[i].type;
                    parts_from_11_camera[i].pose.position.x = tx;
                    parts_from_11_camera[i].pose.position.y = ty;
                    parts_from_11_camera[i].pose.position.z = tz;
                    parts_from_11_camera[i].pose.orientation.x = pose_target.pose.orientation.x;
                    parts_from_11_camera[i].pose.orientation.y = pose_target.pose.orientation.y;
                    parts_from_11_camera[i].pose.orientation.z = pose_target.pose.orientation.z;
                    parts_from_11_camera[i].pose.orientation.w = pose_target.pose.orientation.w;
                    parts_from_11_camera[i].faulty = false;
                    parts_from_11_camera[i].picked = false;
                }
            }
            if (cam_idx == 16){
                if (!((msg->models[i].type).empty())) {
                    parts_from_16_camera[i].type = msg->models[i].type;
                    parts_from_16_camera[i].pose.position.x = tx;
                    parts_from_16_camera[i].pose.position.y = ty;
                    parts_from_16_camera[i].pose.position.z = tz;
                    parts_from_16_camera[i].pose.orientation.x = pose_target.pose.orientation.x;
                    parts_from_16_camera[i].pose.orientation.y = pose_target.pose.orientation.y;
                    parts_from_16_camera[i].pose.orientation.z = pose_target.pose.orientation.z;
                    parts_from_16_camera[i].pose.orientation.w = pose_target.pose.orientation.w;
                    parts_from_16_camera[i].faulty = false;
                    parts_from_16_camera[i].picked = false;
                }
            }

            if (cam_idx == 17){
                if (!((msg->models[i].type).empty())) {
                    parts_from_17_camera[i].type = msg->models[i].type;
                    parts_from_17_camera[i].pose.position.x = tx;
                    parts_from_17_camera[i].pose.position.y = ty;
                    parts_from_17_camera[i].pose.position.z = tz;
                    parts_from_17_camera[i].pose.orientation.x = pose_target.pose.orientation.x;
                    parts_from_17_camera[i].pose.orientation.y = pose_target.pose.orientation.y;
                    parts_from_17_camera[i].pose.orientation.z = pose_target.pose.orientation.z;
                    parts_from_17_camera[i].pose.orientation.w = pose_target.pose.orientation.w;
                    parts_from_17_camera[i].faulty = false;
                    parts_from_17_camera[i].picked = false;
                }
            }

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
        }

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
    ROS_INFO_STREAM("New High Priority ordered received");
    ROS_INFO_STREAM("Received order:\n" << *msg);
    received_orders_.push_back(*msg);
    Competition::pre_kitting();
}

std::array<part, 20> Competition::get_parts_from_16_camera()
{
    return parts_from_16_camera;
}

std::array<part, 20> Competition::get_parts_from_17_camera()
{
    return parts_from_17_camera;
}

std::vector<part> Competition::get_parts_from_15_camera()
{
    return parts_from_15_camera;
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

void Competition::quality_control_sensor_2_subscriber_callback(const nist_gear::LogicalCameraImage::ConstPtr & msg)
{
    if(msg->models.size() != 0) {
        for (int i = 0; i < msg->models.size(); i++) {
//            ROS_INFO_STREAM("Faulty Part Detected from Callback");
            faulty_part_agv1.pose = msg->models[i].pose;
            faulty_part_agv1.faulty = true;
        }
    }
    else
        faulty_part_agv1.faulty = false;
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
