#ifndef GANTRYCONTROL_H
#define GANTRYCONTROL_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <string>
#include <vector>
#include <ros/console.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <array>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>


#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

#include <Eigen/Dense>
#include <tf2/LinearMath/Quaternion.h>

#include "geometric_shapes/shapes.h"
#include "geometric_shapes/mesh_operations.h"
#include "geometric_shapes/shape_operations.h"

#include <nist_gear/VacuumGripperState.h>
#include <nist_gear/VacuumGripperControl.h>

#include <sensor_msgs/JointState.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <trajectory_msgs/JointTrajectory.h>

#include "utils.h"
#include <unordered_map>


class GantryControl {

  public:
    GantryControl(ros::NodeHandle & node);

    void init();
    stats getStats(std::string function);
    Quat ToQuaternion(double roll, double pitch, double yaw);

//    bool moveGantry(std::string waypoints);

//    bool pickPart(part part, std::string arm_name);
    bool pickPart(part part);
    bool pickMovingPart(part part);
    void placePart(part part, std::string agv);
    void placePart_right_arm(part part,std::string agv);
    
    /// Send command message to robot controller
    bool send_command(trajectory_msgs::JointTrajectory command_msg);
    void goToPresetLocation(PresetLocation location);

    void activateGripper(std::string gripper_id);
    void deactivateGripper(std::string gripper_id);
    nist_gear::VacuumGripperState getGripperState(std::string arm_name);
    geometry_msgs::Pose getTargetWorldPose(geometry_msgs::Pose target, std::string agv);
    geometry_msgs::Pose getTargetWorldPose_dummy(geometry_msgs::Pose target,
                                                                std::string agv);
    geometry_msgs::Pose getTargetWorldPose_right_arm(geometry_msgs::Pose target, std::string agv);

    //--preset locations;
    start start_, belt_pickup_;
    bin3 bin3_, bin1_, bin1_w1, bin1_w2;
    agv2 agv2_, agv1_;
    agv2_drop agv2_drop_, agv1_drop_;
    bin13 bin13_;
    bin16 bin16_;
    shelf5 shelf5_, shelf8_w1_, shelf8_w2_, shelf8_w3_, shelf8_w4_, shelf8_w5_, shelf8_w6_,shelf8_w7_, shelf8a_w1_, shelf8a_w2_, shelf8a_w3_, shelf8a_w4_, shelf8a_w5_, shelf8a_w6_,shelf11_w1_, shelf11_w2_, shelf11_w3_, shelf11_w4_;

    waypoint_1 waypoint_1_;
    waypoint_2 waypoint_2_;
    waypoint_3 waypoint_3_;
    waypoint_4 waypoint_4_;

    pose_change pose_change_1_agv1, pose_change_2_agv1,pose_change_1_agv2,pose_change_2_agv2;
    agv2_flip agv2_flip_,agv1_flip_;
    flip_target agv2_flip_target_,agv1_flip_target_;
    std::unordered_map<int, std::vector<PresetLocation>> pickup_locations;
    std::unordered_map<int, char>:: iterator itr;

  private:
    std::vector<double> joint_group_positions_;
    ros::NodeHandle node_;
    std::string planning_group_;
    moveit::planning_interface::MoveGroupInterface::Options full_robot_options_;
    moveit::planning_interface::MoveGroupInterface::Options left_arm_options_;
    moveit::planning_interface::MoveGroupInterface::Options right_arm_options_;
    moveit::planning_interface::MoveGroupInterface::Options left_ee_link_options_;
    moveit::planning_interface::MoveGroupInterface::Options right_ee_link_options_;
    moveit::planning_interface::MoveGroupInterface full_robot_group_;
    moveit::planning_interface::MoveGroupInterface left_arm_group_;
    moveit::planning_interface::MoveGroupInterface right_arm_group_;
    moveit::planning_interface::MoveGroupInterface left_ee_link_group_;
    moveit::planning_interface::MoveGroupInterface right_ee_link_group_;

    double left_ee_roll_;
    double left_ee_pitch_;
    double left_ee_yaw_;
    std::array<float,4> left_ee_quaternion_;

    sensor_msgs::JointState current_joint_states_;


    nist_gear::VacuumGripperState current_left_gripper_state_;
    nist_gear::VacuumGripperState current_right_gripper_state_;

    control_msgs::JointTrajectoryControllerState current_gantry_controller_state_;
    control_msgs::JointTrajectoryControllerState current_left_arm_controller_state_;
    control_msgs::JointTrajectoryControllerState current_right_arm_controller_state_;

    ros::Publisher gantry_joint_trajectory_publisher_;
    ros::Publisher left_arm_joint_trajectory_publisher_;
    ros::Publisher right_arm_joint_trajectory_publisher_;

    ros::Subscriber joint_states_subscriber_;
    ros::Subscriber left_gripper_state_subscriber_;
    ros::Subscriber right_gripper_state_subscriber_;
    ros::Subscriber gantry_controller_state_subscriber_;
    ros::Subscriber left_arm_controller_state_subscriber_;
    ros::Subscriber right_arm_controller_state_subscriber_;

    ros::ServiceClient left_gripper_control_client;
    ros::ServiceClient right_gripper_control_client;

    // ---------- Callbacks ----------
    void joint_states_callback(const sensor_msgs::JointState::ConstPtr & joint_state_msg);
    void left_gripper_state_callback(const nist_gear::VacuumGripperState::ConstPtr & msg);
    void right_gripper_state_callback(const nist_gear::VacuumGripperState::ConstPtr & msg);
    void gantry_controller_state_callback(const control_msgs::JointTrajectoryControllerState::ConstPtr & msg);
    void left_arm_controller_state_callback(const control_msgs::JointTrajectoryControllerState::ConstPtr & msg);
    void right_arm_controller_state_callback(const control_msgs::JointTrajectoryControllerState::ConstPtr & msg);




     // collect stats
    stats init_;
    stats moveJ_;
    stats IK_;
    stats moveGantry_;
    stats pickPart_;
    stats placePart_;
    stats dropPart_;
    stats gripFirmly_;
    stats gripFromBelt_;
    stats grip_;
};

#endif
