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

#include <tf/transform_listener.h> //for shelves gap
#include <tf/LinearMath/Vector3.h> //for shelves gap

#include "utils.h"

/**
 * @brief Competition class
 * 
 */
class Competition
{
public:
    //--methods
    explicit Competition(ros::NodeHandle & node);
    void init();
    void startCompetition();
    void endCompetition();
    void competition_state_callback(const std_msgs::String::ConstPtr & msg);
    void logical_camera_callback(const nist_gear::LogicalCameraImage::ConstPtr & msg, int cam_idx);
    void quality_control_sensor_1_subscriber_callback(const nist_gear::LogicalCameraImage::ConstPtr & msg);
    void quality_control_sensor_2_subscriber_callback(const nist_gear::LogicalCameraImage::ConstPtr & msg);
    std::array<std::array<part, 20>, 20> get_parts_from_camera();
    part get_quality_sensor_status_agv2();
    part get_quality_sensor_status_agv1();
    std::vector<std::vector<std::vector<master_struct> > > get_master_vector();
    void print_parts_detected();
    void print_parts_to_pick();
    void pre_kitting();
    void during_kitting(part);
    void competition_clock_callback(const rosgraph_msgs::Clock::ConstPtr & msg);
    void order_callback(const nist_gear::Order::ConstPtr & msg);
    double getClock();
    double getStartTime();
    std::string getCompetitionState();
    stats getStats(std::string function);
    std::vector<nist_gear::Order> get_received_order_vector();
    void setter_delivered(int i, int j, int k);
    void delete_completed_order(int i);
    std::vector<part> parts_from_15_camera;
    std::vector <part> get_parts_from_15_camera();
    std::array <part, 20> get_parts_from_16_camera();
    std::array <part, 20> get_parts_from_17_camera();

    //--attributes
    std::array<part, 20> parts_from_11_camera;
//    std::array<part, 20> parts_from_15_camera;
    std::array<part, 20> parts_from_16_camera;
    std::array<part, 20> parts_from_17_camera;
    bool conveyor_belt_part_status = false;

    //vector to store shelves
    std::vector<std::vector<double>> get_shelf_vector();

    // all breakbeam related initialization
    void breakbeam_sensor_0_callback(const nist_gear::Proximity::ConstPtr & msg);
    void breakbeam_sensor_1_callback(const nist_gear::Proximity::ConstPtr & msg);
    void breakbeam_sensor_2_callback(const nist_gear::Proximity::ConstPtr & msg);
    void breakbeam_sensor_3_callback(const nist_gear::Proximity::ConstPtr & msg);
    void breakbeam_sensor_4_callback(const nist_gear::Proximity::ConstPtr & msg);
    void breakbeam_sensor_5_callback(const nist_gear::Proximity::ConstPtr & msg);
    void breakbeam_sensor_6_callback(const nist_gear::Proximity::ConstPtr & msg);
    void breakbeam_sensor_7_callback(const nist_gear::Proximity::ConstPtr & msg);
    void breakbeam_sensor_8_callback(const nist_gear::Proximity::ConstPtr & msg);
    void breakbeam_sensor_9_callback(const nist_gear::Proximity::ConstPtr & msg);
    void breakbeam_sensor_10_callback(const nist_gear::Proximity::ConstPtr & msg);
    bool breakbeam_conveyor_belt_part_status_0= false;
    bool breakbeam_conveyor_belt_part_status_1= false;
    bool breakbeam_part_status_2= false;
    bool breakbeam_part_status_3= false;
    bool breakbeam_part_status_4= false;
    bool breakbeam_part_status_5= false;
    bool breakbeam_part_status_6= false;
    bool breakbeam_part_status_7= false;
    bool breakbeam_part_status_8= false;
    bool breakbeam_part_status_9= false;
    bool breakbeam_part_status_10= false;

    void shelf_callback(std::string);

    int get_human_existence();
    int human_detected = 0;

private:
    ros::NodeHandle node_;

    std::string competition_state_;
    double current_score_;
    ros::Time competition_clock_;
    double competition_start_time_; // wall time in sec

    ros::Subscriber current_score_subscriber_;
    ros::Subscriber competition_state_subscriber_;
    ros::Subscriber competition_clock_subscriber_;
    ros::Subscriber orders_subscriber_;
    ros::Subscriber quality_control_sensor_1_subscriber_;
    ros::Subscriber quality_control_sensor_2_subscriber_;
    std::vector<nist_gear::Order> received_orders_;


    //BREAK_BEAM subscribers

    ros::Subscriber breakbeam_sensor_0_subscriber_;
    ros::Subscriber breakbeam_sensor_1_subscriber_;
    ros::Subscriber breakbeam_sensor_2_subscriber_;
    ros::Subscriber breakbeam_sensor_3_subscriber_;
    ros::Subscriber breakbeam_sensor_4_subscriber_;
    ros::Subscriber breakbeam_sensor_5_subscriber_;
    ros::Subscriber breakbeam_sensor_6_subscriber_;
    ros::Subscriber breakbeam_sensor_7_subscriber_;
    ros::Subscriber breakbeam_sensor_8_subscriber_;
    ros::Subscriber breakbeam_sensor_9_subscriber_;
    ros::Subscriber breakbeam_sensor_10_subscriber_;

    // to collect statistics
    stats init_;
};

#endif
