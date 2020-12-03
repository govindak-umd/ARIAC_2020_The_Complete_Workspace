#include "gantry_control.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_listener.h> //for shelves gap
#include <tf/LinearMath/Vector3.h>
#include "competition.h"

Quat GantryControl::ToQuaternion(double roll, double pitch, double yaw) // yaw (Z), pitch (Y), roll (X)
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

GantryControl::GantryControl(ros::NodeHandle & node):
        node_("/ariac/gantry"),
        planning_group_ ("/ariac/gantry/robot_description"),
        full_robot_options_("Full_Robot",planning_group_,node_),
        left_arm_options_("Left_Arm",planning_group_,node_),
        right_arm_options_("Right_Arm",planning_group_,node_),
        left_ee_link_options_("Left_Endeffector",planning_group_,node_),
        right_ee_link_options_("Right_Endeffector",planning_group_,node_),
        full_robot_group_(full_robot_options_),
        left_arm_group_(left_arm_options_),
        right_arm_group_(right_arm_options_),
        left_ee_link_group_(left_ee_link_options_),
        right_ee_link_group_(right_ee_link_options_)
{
    ROS_INFO_STREAM("[GantryControl::GantryControl] constructor called... ");
}


void GantryControl::init() {
    ROS_INFO_STREAM("[GantryControl::init] init... ");
    double time_called = ros::Time::now().toSec();


    ROS_INFO_NAMED("init", "Planning frame: %s", left_arm_group_.getPlanningFrame().c_str());
    ROS_INFO_NAMED("init", "Planning frame: %s", right_arm_group_.getPlanningFrame().c_str());
    ROS_INFO_NAMED("init", "Planning frame: %s", full_robot_group_.getPlanningFrame().c_str());
    ROS_INFO_NAMED("init", "Planning frame: %s", right_ee_link_group_.getPlanningFrame().c_str());
    ROS_INFO_NAMED("init", "Planning frame: %s", left_ee_link_group_.getPlanningFrame().c_str());

    ROS_INFO_NAMED("init", "End effector link: %s", left_arm_group_.getEndEffectorLink().c_str());
    ROS_INFO_NAMED("init", "End effector link: %s", right_arm_group_.getEndEffectorLink().c_str());

    left_arm_group_.setPoseReferenceFrame("world");

    //Moving to shelf 8
    // gasket part green
//    shelf8_w1_.gantry = {0.0,-1.6,0};
//    shelf8_w1_.left_arm = {0.0, -PI/4, PI/2, -PI/4, PI/2, 0};
//    shelf8_w1_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};
//
//    shelf8_w2_.gantry = {-13.5,-1.6,0};
//    shelf8_w2_.left_arm = {0.0, -PI/4, PI/2, -PI/4, PI/2, 0};
//    shelf8_w2_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

// waypoints by Pradeep to reach green
//    shelf8_w1_.gantry = {0.0, -4.48, 0};
//    shelf8_w1_.left_arm = {0.0, -PI/4, PI/2, -PI/4, PI/2, 0};
//    shelf8_w1_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};
//
//    shelf8_w2_.gantry = {-11.4, -4.48,0};
//    shelf8_w2_.left_arm = {0.0, -PI/4, PI/2, -PI/4, PI/2, 0};
//    shelf8_w2_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};
//
//    shelf8_w3_.gantry = {-11.4, -1.6, 0};
//    shelf8_w3_.left_arm = {0.0, -PI/4, PI/2, -PI/4, PI/2, 0};
//    shelf8_w3_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};
//
//    shelf8_w4_.gantry = {-13.5, -1.6, 0};
//    shelf8_w4_.left_arm = {0.0, -PI/4, PI/2, -PI/4, PI/2, 0};
//    shelf8_w4_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};
//
//    shelf8_w5_.gantry = {-13.5, -1.6, 0.0};
//    shelf8_w5_.left_arm = {-PI/2, -PI/4, PI/2, -PI/4, -0.2, 0};
//    shelf8_w5_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};
//
//    shelf8_w6_.gantry = {-14, -1.2, 0.0};
//    shelf8_w6_.left_arm = {-1.78, -PI/4, PI/2, -PI/4, -0.2, 0};
//    shelf8_w6_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

//new waypoints by Rajesh and Govind to reach green

    shelf8_w1_.gantry = {0.0,-4.48,0.0};
    shelf8_w1_.left_arm = {0.0, -PI/4, PI/2, -PI/4, PI/2, 0};
    shelf8_w1_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    shelf8_w2_.gantry = {-11.58,-4.48,0};
    shelf8_w2_.left_arm = {0.0, -PI/4, PI/2, -PI/4, PI/2, 0};
    shelf8_w2_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    shelf8_w3_.gantry = {-11.58,-4.48,1.57};
    shelf8_w3_.left_arm = {0.0, -PI/4, PI/2, -PI/4, PI/2, 0};
    shelf8_w3_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    shelf8_w4_.gantry = {-11.58, -2.99, 0.73};
    shelf8_w4_.left_arm = {-PI/2, -PI/4, PI/2, -PI/4, -0.2, 0};
    shelf8_w4_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    shelf8_w5_.gantry = {-11.47, -1.68, 0.0};
    shelf8_w5_.left_arm = {-1.78, -PI/4, PI/2, -PI/4, -0.2, 0};
    shelf8_w5_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    shelf8_w6_.gantry = {-13.5, -1.6, 0.0};
    shelf8_w6_.left_arm = {-PI/2, -PI/4, PI/2, -PI/4, -0.2, 0};
    shelf8_w6_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    shelf8_w7_.gantry = {-14, -1.2, 0.0};
    shelf8_w7_.left_arm = {-1.78, -PI/4, PI/2, -PI/4, -0.2, 0};
    shelf8_w7_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

// Pradeep Waypoints to Blue

    shelf8a_w1_.gantry = {0.0, 4.48, 3.14};
    shelf8a_w1_.left_arm = {0.0, -PI/4, PI/2, -PI/4, PI/2, 0};
    shelf8a_w1_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

//    shelf8a_w2_.gantry = {-11.4, 4.48,3.14};
//    shelf8a_w2_.left_arm = {0.0, -PI/4, PI/2, -PI/4, PI/2, 0};
//    shelf8a_w2_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    shelf8a_w2_.gantry = {-11.40, 4.48,3.45};
    shelf8a_w2_.left_arm = {0.0, -PI/4, PI/2, -PI/4, PI/2, 0};
    shelf8a_w2_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

//    shelf8a_w3_.gantry = {-11.4, 1.6, 3.14};
//    shelf8a_w3_.left_arm = {0.0, -PI/4, PI/2, -PI/4, PI/2, 0};
//    shelf8a_w3_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    //this is where the gantry waits
    shelf8a_w3_.gantry = {-11.4, 2.99, 3.45};
    shelf8a_w3_.left_arm = {0.0, -PI/4, PI/2, -PI/4, PI/2, 0};
    shelf8a_w3_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    shelf8a_w4_.gantry = {-11.40, 1.6, 3.14};
    shelf8a_w4_.left_arm = {0.0, -PI/4, PI/2, -PI/4, PI/2, 0};
    shelf8a_w4_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

//    shelf8a_w4_.gantry = {-14.7, 1.6, 3.14};
//    shelf8a_w4_.left_arm = {0.0, -PI/4, PI/2, -PI/4, PI/2, 0};
//    shelf8a_w4_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    shelf8a_w5_.gantry = {-14.7, 1.6, 3.14};
    shelf8a_w5_.left_arm = {-PI/2, -PI/4, PI/2, -PI/4, -0.2, 0};
    shelf8a_w5_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    shelf8a_w6_.gantry = {-14.7, 1.3, 3.14};
    shelf8a_w6_.left_arm = {-1.78, -PI/4, PI/2, -PI/4, -0.2, 0};
    shelf8a_w6_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};



    // location where gantry hover to pick up part at belt
    bin1_w1.gantry = {2.75, - 0.77, PI/2};
    bin1_w1.left_arm = {0.0, -PI/4, PI/2, -PI/4, PI/2, 0};
    bin1_w1.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    bin1_w2.gantry = {0,0,0};
    bin1_w2.left_arm = {0.0, -PI/4, PI/2, -PI/4, PI/2, 0};
    bin1_w2.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};


    //shelf 4
    int cam = 4;
    std::vector<PresetLocation> waypoints;
    waypoints.push_back(shelf8_w1_);
    waypoints.push_back(shelf8_w2_);
    waypoints.push_back(shelf8_w3_);
    waypoints.push_back(shelf8_w4_);
    waypoints.push_back(shelf8_w5_);
    waypoints.push_back(shelf8_w6_);
    waypoints.push_back(shelf8_w7_);
    pickup_locations[cam] = waypoints;

    cam = 3;
    waypoints.clear();
    waypoints.push_back(shelf8a_w1_);
    waypoints.push_back(shelf8a_w2_);
    waypoints.push_back(shelf8a_w3_);
    waypoints.push_back(shelf8a_w4_);
    waypoints.push_back(shelf8a_w5_);
    waypoints.push_back(shelf8a_w6_);
    pickup_locations[cam] = waypoints;

    cam = 11;
    waypoints.clear();
    waypoints.push_back(bin1_w1);
//    waypoints.push_back(bin1_w2);
    pickup_locations[cam] = waypoints;

// BIN 16 preset location
    bin16_.gantry = {5.00, 1.95,0.0};
    bin16_.left_arm = {0.0, -PI/4, PI/2, -PI/4, PI/2, 0};
    bin16_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

//    disk_part_green located in bin13
//    bin13_.gantry = {2.0, 2.35,0.0};
//    bin13_.left_arm = {0.0, -PI/4, PI/2, -PI/4, PI/2, 0};
//    bin13_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

//    bin13_.gantry = {2.55, -1.58, 1.54};
//    bin13_.gantry = {2.55, 1.56, -1.58};
//    bin13_.left_arm = {0.0, -PI/4, PI/2, -PI/4, PI/2, 0};
//    bin13_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    //    bin13_.gantry = {2.55, -1.58, 1.54};
    bin13_.gantry = {3.1, 1.68, 3.77};
    bin13_.left_arm = {0.0, -0.63, 1.26, -0.65, PI/2, 0};
    bin13_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

//    pulley_part_red located on waypoint_1
    waypoint_1_.gantry = {0.0, -4.7, 0.0};
    waypoint_1_.left_arm = {0.0, -PI/4, PI/2, -PI/4, PI/2, 0};
    waypoint_1_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};


//    pulley_part_red located on waypoint_2
    waypoint_2_.gantry = {-14.5, -4.7, 0.0};
    waypoint_2_.left_arm = {0.0, -PI/4, PI/2, -PI/4, PI/2, 0};
    waypoint_2_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    //    pulley_part_red located on waypoint_3
    waypoint_3_.gantry = {-14.5, -4.7, 0.0};
    waypoint_3_.left_arm = {-PI/2, -PI/4, PI/2, -PI/4, PI/2, 0};
    waypoint_3_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    //    pulley_part_red located on waypoint_4
    waypoint_4_.gantry = {-14.5, -4.3, 0.0};
    waypoint_4_.left_arm = {-2.79, -PI/4, PI/2, -PI/4, -1.39626, 0};
    waypoint_4_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};


//    pulley_part_red located on shelf5
    shelf5_.gantry = {-14.00, -4.76,0.0};
    shelf5_.left_arm = {0.0, -PI/4, PI/2, -PI/4, PI/2, 0};
    shelf5_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};


//    Start location
    start_.gantry = {0,0,0};
    start_.left_arm = {0.0, -PI/4, PI/2, -PI/4, PI/2, 0};
    start_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};


//    Bin3 location
    bin3_.gantry = {4.0, -1.1, 0.};
    bin3_.left_arm = {0.0, -PI/4, PI/2, -PI/4, PI/2, 0};
    bin3_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

//    Agv2 location
    agv2_.gantry = {0.6, 6.9, PI};
    agv2_.left_arm = {0.0, -PI/4, PI/2, -PI/4, PI/2, 0};
    agv2_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    //    Agv1 location
    agv1_.gantry = {0.6, -6.95, PI};
    // agv1_.left_arm = {-0.38, -PI/4, PI/2, -PI/4, 1.15, 0};
    agv1_.left_arm = {0.0, -PI/4, PI/2, -PI/4, PI/2, 0};
    agv1_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    //    Agv2 faulty part Drop location
    agv2_drop_.gantry = {1, 6.9, PI};
    agv2_drop_.left_arm = {0.0, -PI/4, PI/2, -PI/4, PI/2, 0};
    agv2_drop_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    //    Agv1 faulty part Drop location
    agv1_drop_.gantry = {1, -6.9, PI};
    agv1_drop_.left_arm = {0.0, -PI/4, PI/2, -PI/4, PI/2, 0};
    agv1_drop_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    // pose change wavepoints

    pose_change_1_agv1.gantry = {0.0,-5,PI};
    pose_change_1_agv1.left_arm = {PI/4,-0.2,1.3,0.5,PI/2,0.00};
    pose_change_1_agv1.right_arm = {-PI/4,-3,-PI/2,-0.1,PI/2,-0.79};

    // switching wavepoint
    pose_change_2_agv1.gantry = {0.0,-5,PI};
    pose_change_2_agv1.left_arm = {0.77,-0.2,1.3,0.49,1.59,0.00};
    pose_change_2_agv1.right_arm = {-PI/4,-3.2,-1.5,-0.02,PI/2,-PI/4};

    agv1_flip_.gantry = {0.0, -2, PI};
    agv1_flip_.left_arm = {0.0, -PI/4, PI/2, -PI/4, PI/2, 0};
    agv1_flip_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    agv1_flip_target_.gantry = {-0.66, -6.9, PI};
    agv1_flip_target_.left_arm = {0.0, -PI/4, PI/2, -PI/4, PI/2, 0};
    agv1_flip_target_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};


    // pose change wavepoints

    pose_change_1_agv2.gantry = {0.0,5,PI};
    pose_change_1_agv2.left_arm = {PI/4,-0.2,1.3,0.5,PI/2,0.00};
    pose_change_1_agv2.right_arm = {-PI/4,-3,-PI/2,-0.1,PI/2,-0.79};

    // switching wavepoint
    pose_change_2_agv2.gantry = {0.0,5,PI};
    pose_change_2_agv2.left_arm = {0.77,-0.2,1.3,0.49,1.59,0.00};
    pose_change_2_agv2.right_arm = {-PI/4,-3.2,-1.5,-0.02,PI/2,-PI/4};

    agv2_flip_.gantry = {0.0, 2, PI};
    agv2_flip_.left_arm = {0.0, -PI/4, PI/2, -PI/4, PI/2, 0};
    agv2_flip_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    agv2_flip_target_.gantry = {-0.66, 6.9, PI};
    agv2_flip_target_.left_arm = {0.0, -PI/4, PI/2, -PI/4, PI/2, 0};
    agv2_flip_target_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};



    //Moving to shelf 11
    // gasket part blue
    shelf11_w1_.gantry = {0.0,1.4,0};
    shelf11_w1_.left_arm = {0.0, -PI/4, PI/2, -PI/4, PI/2, 0};
    shelf11_w1_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    shelf11_w2_.gantry = {-14.8,1.4,0};
    shelf11_w2_.left_arm = {0.0, -PI/4, PI/2, -PI/4, PI/2, 0};
    shelf11_w2_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    shelf11_w3_.gantry = {-14.8, 1.4, 0.0};
    shelf11_w3_.left_arm = {-PI/2, -PI/4, PI/2, -PI/4, PI/2, 0};
    shelf11_w3_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    shelf11_w4_.gantry = {-14.8, 1.8, 0.0};
    shelf11_w4_.left_arm = {-2.79, -PI/4, PI/2, -PI/4, -1.39626, 0};
    shelf11_w4_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    // location where gantry hover to pick up part at belt
    belt_pickup_.gantry = {0.52, -3.27, PI/2};
    belt_pickup_.left_arm = {0.38, -0.41, 1.05, -0.63, 1.94, 0.0};
    belt_pickup_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    // location where gantry hover to pick up part at belt
    bin1_.gantry = {2.75, - 0.77, PI/2};
    bin1_.left_arm = {0.0, -PI/4, PI/2, -PI/4, PI/2, 0};
    bin1_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

//    tf2_ros::Buffer tfBuffer;
//    tf2_ros::TransformListener tfListener(tfBuffer);
//    ros::Rate rate(10);
//    ros::Duration timeout(5.0);
//
//
//    geometry_msgs::TransformStamped transformStamped;
//    for (int i=0; i< 10; i++) {
//        try {
//            transformStamped = tfBuffer.lookupTransform("world", "left_ee_link",
//                                                        ros::Time(0), timeout);
//        }
//        catch (tf2::TransformException &ex) {
//            ROS_WARN("%s", ex.what());
//            ros::Duration(1.0).sleep();
//            continue;
//        }
//    }


    //--converting quaternions to rpy
//        tf2::Quaternion q(
//                transformStamped.transform.rotation.x,
//                transformStamped.transform.rotation.y,
//                transformStamped.transform.rotation.z,
//                transformStamped.transform.rotation.w);

//    left_ee_quaternion_.at(0) = transformStamped.transform.rotation.x;
//    left_ee_quaternion_.at(1) = transformStamped.transform.rotation.y;
//    left_ee_quaternion_.at(2) = transformStamped.transform.rotation.z;
//    left_ee_quaternion_.at(3) = transformStamped.transform.rotation.w;



    //--Raw pointers are frequently used to refer to the planning group for improved performance.
    //--To start, we will create a pointer that references the current robot’s state.
    const moveit::core::JointModelGroup* joint_model_group =
            full_robot_group_.getCurrentState()->getJointModelGroup("Full_Robot");

    //--Let’s set a joint space goal and move towards it.
    moveit::core::RobotStatePtr current_state = full_robot_group_.getCurrentState();

    //--Next get the current set of joint values for the group.
//    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions_);



    gantry_joint_trajectory_publisher_ =
            node_.advertise<trajectory_msgs::JointTrajectory>("/ariac/gantry/gantry_controller/command", 10);

    left_arm_joint_trajectory_publisher_ =
            node_.advertise<trajectory_msgs::JointTrajectory>("/ariac/gantry/left_arm_controller/command", 10);

    right_arm_joint_trajectory_publisher_ =
            node_.advertise<trajectory_msgs::JointTrajectory>("/ariac/gantry/right_arm_controller/command", 10);

    joint_states_subscriber_ = node_.subscribe(
            "/ariac/gantry/joint_states", 10, &GantryControl::joint_states_callback, this);

    left_gripper_state_subscriber_ = node_.subscribe(
            "/ariac/gantry/left_arm/gripper/state", 10, &GantryControl::left_gripper_state_callback, this);

    right_gripper_state_subscriber_ = node_.subscribe(
            "/ariac/gantry/right_arm/gripper/state", 10, &GantryControl::right_gripper_state_callback, this);

    gantry_controller_state_subscriber_ = node_.subscribe(
            "/ariac/gantry/gantry_controller/state", 10, &GantryControl::gantry_controller_state_callback, this);

    left_arm_controller_state_subscriber_ = node_.subscribe(
            "/ariac/gantry/left_arm_controller/state", 10, &GantryControl::left_arm_controller_state_callback, this);

    right_arm_controller_state_subscriber_ = node_.subscribe(
            "/ariac/gantry/right_arm_controller/state", 10, &GantryControl::right_arm_controller_state_callback, this);


    while( (current_gantry_controller_state_.joint_names.size() == 0)
           || (current_left_arm_controller_state_.joint_names.size() == 0)
           || (current_right_arm_controller_state_.joint_names.size() == 0) ) {
        ROS_WARN("[GantryControl::init] Waiting for first controller_state callbacks...");
        ros::Duration(0.1).sleep();
    }

    left_gripper_control_client =
            node_.serviceClient<nist_gear::VacuumGripperControl>("/ariac/gantry/left_arm/gripper/control");
    left_gripper_control_client.waitForExistence();

    right_gripper_control_client =
            node_.serviceClient<nist_gear::VacuumGripperControl>("/ariac/gantry/right_arm/gripper/control");
    right_gripper_control_client.waitForExistence();

    // Move robot to init position
    ROS_INFO("[GantryControl::init] Init position ready)...");
}


stats GantryControl::getStats(std::string function) {
    if (function == "init") return init_;
    if (function == "moveJ") return moveJ_;
    if (function == "IK") return IK_;
    if (function == "moveGantry") return moveGantry_;
    if (function == "pickPart") return pickPart_;
    if (function == "placePart") return placePart_;
    if (function == "dropPart") return dropPart_;
    if (function == "gripFirmly") return gripFirmly_;
    if (function == "gripFromBelt") return gripFromBelt_;
    if (function == "grip") return grip_;
}

bool GantryControl::pickMovingPart(part part) {
    //--Activate gripper
    activateGripper("left_arm");
//    ros::AsyncSpinner spinner(1);
//    spinner.start();

//    left_arm_group_.setPoseReferenceFrame("world");
    geometry_msgs::Pose currentPose = left_arm_group_.getCurrentPose().pose;

//    ROS_INFO_STREAM("[left_arm_group_]= " << currentPose.position.x << ", " << currentPose.position.y << "," << currentPose.position.z);

    part.pose.position.z = part.pose.position.z + model_height.at(part.type) + GRIPPER_HEIGHT - EPSILON;
    part.pose.orientation.x = currentPose.orientation.x;
    part.pose.orientation.y = currentPose.orientation.y;
    part.pose.orientation.z = currentPose.orientation.z;
    part.pose.orientation.w = currentPose.orientation.w;
//    ROS_INFO_STREAM("["<< part.type<<"]= " << part.pose.position.x << ", " << part.pose.position.y << "," << part.pose.position.z << "," << part.pose.orientation.x << "," << part.pose.orientation.y << "," << part.pose.orientation.z << "," << part.pose.orientation.w);

    auto state = getGripperState("left_arm");
    if (state.enabled) {
        ROS_INFO_STREAM("[Gripper] = enabled");
        //--Move arm to part
        left_arm_group_.setPoseTarget(part.pose);
        left_arm_group_.move();
        auto state = getGripperState("left_arm");
        if (state.attached) {
            ROS_INFO_STREAM("[Gripper] = object attached");
            //--Move arm to previous position
//            left_arm_group_.setPoseTarget(currentPose);
//            left_arm_group_.move();
//            goToPresetLocation(start_);
            return true;
        }
//        else {
//            ROS_INFO_STREAM("[Gripper] = object not attached");
//            int attempt{0}, max_attempts{2};
//            int current_attempt{0};
//            while(!state.attached || (attempt != max_attempts)) {
//                ROS_INFO_STREAM("Attached status = " << state.attached);
//                left_arm_group_.setPoseTarget(currentPose);
//                left_arm_group_.move();
//                ros::Duration(0.5).sleep();
//                left_arm_group_.setPoseTarget(part.pose);
//                left_arm_group_.move();
//                activateGripper("left_arm");
//                auto state = getGripperState("left_arm");
//                if(state.attached)
//                {
//                    return true;
//                }
//                attempt += 1;
//            }
//        }
    }
    else {
        ROS_INFO_STREAM("[Gripper] = not enabled");
    }
    return false;

    /**
     * We want the Cartesian path to be interpolated at a resolution of 1 cm which is why
     * we will specify 0.01 as the max step in Cartesian translation.
     * We will specify the jump threshold as 0.0, effectively disabling it.
     */
    //--define a set of waypoints
//    geometry_msgs::Pose near_pick_pose;
//    geometry_msgs::Pose pick_pose;
//    near_pick_pose = part.pose;
//    pick_pose = part.pose;
//
//    near_pick_pose.position.z += 0.1;
//    pick_pose.position.z += 0.015;
//
//    //--waypoints
//    ROS_INFO_STREAM("[near_pick_pose]= " << near_pick_pose.position.x << "," << near_pick_pose.position.y << "," << near_pick_pose.position.z << "," << near_pick_pose.orientation.x << "," << near_pick_pose.orientation.y << "," << near_pick_pose.orientation.z << "," << near_pick_pose.orientation.w);
//    ROS_INFO_STREAM("[pick_pose]= " << pick_pose.position.x << "," << pick_pose.position.y << "," << pick_pose.position.z << "," << pick_pose.orientation.x << "," << pick_pose.orientation.y << "," << pick_pose.orientation.z << "," << pick_pose.orientation.w);
//    std::vector<geometry_msgs::Pose> waypoints;
//    waypoints.push_back(near_pick_pose);
//    waypoints.push_back(pick_pose);

//    moveit_msgs::RobotTrajectory trajectory;
//    const double jump_threshold = 0.0;
//    const double eef_step = 0.001;
//    double fraction = left_arm_group_.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
//
//    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
//    bool success = (left_arm_group_.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//    if (success)
//        left_arm_group_.move();
//    ros::waitForShutdown();
}

















geometry_msgs::Pose GantryControl::getTargetWorldPose(geometry_msgs::Pose target,
                                                      std::string agv){
    static tf2_ros::StaticTransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;
    std::string kit_tray;
    if (agv.compare("agv1")==0) {
//        ROS_INFO_STREAM("AGV1?" << agv);
        kit_tray = "kit_tray_1";
    }
    else {
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


    for (int i{0}; i<15; ++i)
        br.sendTransform(transformStamped);

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    ros::Rate rate(10);
    ros::Duration timeout(5.0);


    geometry_msgs::TransformStamped world_target_tf;
    geometry_msgs::TransformStamped ee_target_tf;
    for (int i=0; i< 10; i++) {
        try {
            world_target_tf = tfBuffer.lookupTransform("world", "target_frame",
                                                       ros::Time(0), timeout);
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }

        try {
            ee_target_tf = tfBuffer.lookupTransform("target_frame", "left_ee_link",
                                                    ros::Time(0), timeout);
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
    }
    geometry_msgs::Pose world_target{target};
    world_target.position.x = world_target_tf.transform.translation.x;
    world_target.position.y = world_target_tf.transform.translation.y;
    world_target.position.z = world_target_tf.transform.translation.z;
    world_target.orientation.x = ee_target_tf.transform.rotation.x;
    world_target.orientation.y = ee_target_tf.transform.rotation.y;
    world_target.orientation.z = ee_target_tf.transform.rotation.z;
    world_target.orientation.w = ee_target_tf.transform.rotation.w;
    return world_target;
}

geometry_msgs::Pose GantryControl::getTargetWorldPose_right_arm(geometry_msgs::Pose target,
                                                                std::string agv) {
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


    for (int i{0}; i < 15; ++i)
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
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }

        try {
            ee_target_tf = tfBuffer.lookupTransform("target_frame", "right_ee_link",
                                                    ros::Time(0), timeout);
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
    }

    geometry_msgs::Pose world_target{target};
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

void GantryControl::placePart_right_arm(part part, std::string agv){
    auto target_pose_in_tray = getTargetWorldPose_right_arm(part.pose, agv);
//    ros::Duration(3.0).sleep();
//    goToPresetLocation(agv2_);
    target_pose_in_tray.position.z += (ABOVE_TARGET + 1.5*model_height[part.type]);

    right_arm_group_.setPoseTarget(target_pose_in_tray);
    right_arm_group_.move();
    deactivateGripper("right_arm");
//    auto state = getGripperState("left_arm");
//    if (state.attached)
//        goToPresetLocation(start_);
}


geometry_msgs::Pose GantryControl::getTargetWorldPose_dummy(geometry_msgs::Pose target,
                                                      std::string agv){
    geometry_msgs::TransformStamped transformStamped;
    std::string kit_tray;
    if (agv.compare("agv1")==0)
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
    for (int i=0; i< 10; i++) {
        try {
            world_target_tf = tfBuffer.lookupTransform("world", "target_frame",
                                                       ros::Time(0), timeout);
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }

        try {
            ee_target_tf = tfBuffer.lookupTransform("target_frame", "left_ee_link",
                                                    ros::Time(0), timeout);
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
    }

    geometry_msgs::Pose world_target{target};
    world_target.position.x = world_target_tf.transform.translation.x;
    world_target.position.y = world_target_tf.transform.translation.y;
    world_target.position.z = world_target_tf.transform.translation.z;
    world_target.orientation.x = ee_target_tf.transform.rotation.x;
    world_target.orientation.y = ee_target_tf.transform.rotation.y;
    world_target.orientation.z = ee_target_tf.transform.rotation.z;
    world_target.orientation.w = ee_target_tf.transform.rotation.w;

    return world_target;
}

bool GantryControl::pickPart(part part){
    //--Activate gripper
    activateGripper("left_arm");
//    ros::AsyncSpinner spinner(1);
//    spinner.start();

//    left_arm_group_.setPoseReferenceFrame("world");
    geometry_msgs::Pose currentPose = left_arm_group_.getCurrentPose().pose;

//    ROS_INFO_STREAM("[left_arm_group_]= " << currentPose.position.x << ", " << currentPose.position.y << "," << currentPose.position.z);

    part.pose.position.z = part.pose.position.z + model_height.at(part.type) + GRIPPER_HEIGHT - EPSILON;
    part.pose.orientation.x = currentPose.orientation.x;
    part.pose.orientation.y = currentPose.orientation.y;
    part.pose.orientation.z = currentPose.orientation.z;
    part.pose.orientation.w = currentPose.orientation.w;
//    ROS_INFO_STREAM("["<< part.type<<"]= " << part.pose.position.x << ", " << part.pose.position.y << "," << part.pose.position.z << "," << part.pose.orientation.x << "," << part.pose.orientation.y << "," << part.pose.orientation.z << "," << part.pose.orientation.w);


    auto state = getGripperState("left_arm");
    if (state.enabled) {
        ROS_INFO_STREAM("[Gripper] = enabled");
        //--Move arm to part
        left_arm_group_.setPoseTarget(part.pose);
        left_arm_group_.move();
        auto state = getGripperState("left_arm");
        if (state.attached) {
            ROS_INFO_STREAM("[Gripper] = object attached");
            //--Move arm to previous position
//            left_arm_group_.setPoseTarget(currentPose);
//            left_arm_group_.move();
//            goToPresetLocation(start_);
            return true;
        }
        else {
            ROS_INFO_STREAM("[Gripper] = object not attached");
            int attempt{0}, max_attempts{5};
            int current_attempt{0};
            while(!state.attached || (attempt != max_attempts)) {
                ROS_INFO_STREAM("Attached status = " << state.attached);
                left_arm_group_.setPoseTarget(currentPose);
                left_arm_group_.move();
                ros::Duration(0.5).sleep();
                left_arm_group_.setPoseTarget(part.pose);
                left_arm_group_.move();
                activateGripper("left_arm");
                auto state = getGripperState("left_arm");
                if(state.attached)
                {
                    return true;
                }
                attempt += 1;
            }
        }
    }
    else {
        ROS_INFO_STREAM("[Gripper] = not enabled");
    }
    return false;

    /**
     * We want the Cartesian path to be interpolated at a resolution of 1 cm which is why
     * we will specify 0.01 as the max step in Cartesian translation.
     * We will specify the jump threshold as 0.0, effectively disabling it.
     */
    //--define a set of waypoints
//    geometry_msgs::Pose near_pick_pose;
//    geometry_msgs::Pose pick_pose;
//    near_pick_pose = part.pose;
//    pick_pose = part.pose;
//
//    near_pick_pose.position.z += 0.1;
//    pick_pose.position.z += 0.015;
//
//    //--waypoints
//    ROS_INFO_STREAM("[near_pick_pose]= " << near_pick_pose.position.x << "," << near_pick_pose.position.y << "," << near_pick_pose.position.z << "," << near_pick_pose.orientation.x << "," << near_pick_pose.orientation.y << "," << near_pick_pose.orientation.z << "," << near_pick_pose.orientation.w);
//    ROS_INFO_STREAM("[pick_pose]= " << pick_pose.position.x << "," << pick_pose.position.y << "," << pick_pose.position.z << "," << pick_pose.orientation.x << "," << pick_pose.orientation.y << "," << pick_pose.orientation.z << "," << pick_pose.orientation.w);
//    std::vector<geometry_msgs::Pose> waypoints;
//    waypoints.push_back(near_pick_pose);
//    waypoints.push_back(pick_pose);

//    moveit_msgs::RobotTrajectory trajectory;
//    const double jump_threshold = 0.0;
//    const double eef_step = 0.001;
//    double fraction = left_arm_group_.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
//
//    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
//    bool success = (left_arm_group_.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//    if (success)
//        left_arm_group_.move();
//    ros::waitForShutdown();
}

//void GantryControl::placePart(part part, std::string agv){
//    geometry_msgs::Pose target_pose_in_tray = getTargetWorldPose(part.pose, agv);
//
//    if(agv == "agv2") {
//        goToPresetLocation(agv2_);
//    }
//    else
//        goToPresetLocation(agv1_);
//
//    target_pose_in_tray.position.z += (ABOVE_TARGET + 1.5 * model_height[part.type]);
//    left_arm_group_.setPoseTarget(target_pose_in_tray);
//    left_arm_group_.move();
//    deactivateGripper("left_arm");
////    auto state = getGripperState("left_arm");
////    if (state.attached)
////        goToPresetLocation(start_);
//}

void GantryControl::placePart(part part, std::string agv){
    geometry_msgs::Pose initial_pose, final_pose;

    initial_pose = part.initial_pose;
    final_pose = getTargetWorldPose(part.pose, agv);

    // Orientation quaternion
    tf2::Quaternion q1(
            initial_pose.orientation.x,
            initial_pose.orientation.y,
            initial_pose.orientation.z,
            initial_pose.orientation.w);
    tf2::Quaternion q2(
            final_pose.orientation.x,
            final_pose.orientation.y,
            final_pose.orientation.z,
            final_pose.orientation.w);

    // 3x3 Rotation matrix from quaternion
    tf2::Matrix3x3 m(q1);
    tf2::Matrix3x3 m1(q2);

    // Roll Pitch and Yaw from rotation matrix
    double initial_roll, initial_pitch, initial_yaw, final_roll, final_pitch, final_yaw, target_roll, target_pitch, target_yaw;
    m.getRPY(initial_roll, initial_pitch, initial_yaw);
    m1.getRPY(final_roll, final_pitch, final_yaw);


    target_roll = final_roll - initial_roll;
    target_pitch =  final_pitch - initial_pitch;

//    target_yaw =  final_yaw - (initial_yaw - 3.08) + 3.14; // Accouting for 180 degree spin by gantry // 0 degree (3.14)

//    target_yaw =  -(final_yaw - (initial_yaw - 3.08) + 3.14) - 2.36; // Accouting for 180 degree spin by gantry // 0 degree (-3.14)

//    target_yaw =  -(final_yaw - (initial_yaw - 3.2) + 3.14) - 2.36; //Rwa4 working code

//    target_yaw = -(final_yaw - (initial_yaw + 3.14) - 3.14 - 0.633); //RWA5 Green Gasket picking

    if(initial_yaw < 0) {
        ROS_INFO_STREAM("Initial pose was negative -45 ");
        target_yaw = -(final_yaw - (initial_yaw + 3.14) - 3.14 - 0.733); //RWA5 Green Gasket picking
    }
    else {
        ROS_INFO_STREAM("Initial pose was positive 45 ");
        target_yaw = -(final_yaw - (initial_yaw - 3.2) + 3.14) - 2.36;
    }

    auto final_pose_ = ToQuaternion(target_roll, target_pitch, target_yaw);
    final_pose.orientation.x = final_pose_.x;
    final_pose.orientation.y = final_pose_.y;
    final_pose.orientation.z = final_pose_.z;
    final_pose.orientation.w = final_pose_.w;

    geometry_msgs::Pose target_pose_in_tray = final_pose;
    if(agv == "agv2") {
        goToPresetLocation(agv2_);
    }
    else{
        goToPresetLocation(agv1_);
        ROS_INFO_STREAM("AGV Location Reached");
    }

    target_pose_in_tray.position.z += (ABOVE_TARGET + 1.5 * model_height[part.type]);
    ROS_INFO_STREAM("target_pose_in_tray = " << target_pose_in_tray);
    left_arm_group_.setPoseTarget(target_pose_in_tray);
    left_arm_group_.move();
    deactivateGripper("left_arm");

}

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
    bool success = (full_robot_group_.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (success)
        full_robot_group_.move();
}

/// Turn on vacuum gripper
void GantryControl::activateGripper(std::string arm_name) {
    nist_gear::VacuumGripperControl srv;
    srv.request.enable = true;

    if (arm_name == "left_arm") {
        left_gripper_control_client.call(srv);
    } else {
        right_gripper_control_client.call(srv);
    }
    ROS_INFO_STREAM("[GantryControl][activateGripper] DEBUG: srv.response =" << srv.response);
}

/// Turn off vacuum gripper
void GantryControl::deactivateGripper(std::string arm_name) {
    nist_gear::VacuumGripperControl srv;
    srv.request.enable = false;

    if (arm_name == "left_arm") {
        left_gripper_control_client.call(srv);
    } else {
        right_gripper_control_client.call(srv);
    }
    ROS_INFO_STREAM("[GantryControl][deactivateGripper] DEBUG: srv.response =" << srv.response);
}

/// Retrieve gripper state
nist_gear::VacuumGripperState GantryControl::getGripperState(std::string arm_name) {
    if (arm_name == "left_arm") {
        return current_left_gripper_state_;
    } else {
        return current_right_gripper_state_;
    }
}

/// Called when a new VacuumGripperState message is received
void GantryControl::left_gripper_state_callback(const nist_gear::VacuumGripperState::ConstPtr & gripper_state_msg) {
    // ROS_INFO_STREAM_THROTTLE(10,
    //   "Gripper States (throttled to 0.1 Hz):\n" << *gripper_state_msg);
    current_left_gripper_state_ = *gripper_state_msg;
}

void GantryControl::right_gripper_state_callback(const nist_gear::VacuumGripperState::ConstPtr & gripper_state_msg) {
    // ROS_INFO_STREAM_THROTTLE(10,
    //   "Gripper States (throttled to 0.1 Hz):\n" << *gripper_state_msg);
    current_right_gripper_state_ = *gripper_state_msg;
}

/// Called when a new JointState message is received
void GantryControl::joint_states_callback(const sensor_msgs::JointState::ConstPtr & joint_state_msg) {
    if (joint_state_msg->position.size() == 0) {
        ROS_ERROR("[gantry_control][joint_states_callback] msg->position.size() == 0!");
    }
    current_joint_states_ = *joint_state_msg;
}


void GantryControl::gantry_controller_state_callback(const control_msgs::JointTrajectoryControllerState::ConstPtr & msg) {
    // ROS_INFO_STREAM_THROTTLE(10,
    //   "Gantry controller states (throttled to 0.1 Hz):\n" << *msg);
    current_gantry_controller_state_ = *msg;
}

void GantryControl::left_arm_controller_state_callback(const control_msgs::JointTrajectoryControllerState::ConstPtr & msg) {
    // ROS_INFO_STREAM_THROTTLE(10,
    //   "Left arm controller states (throttled to 0.1 Hz):\n" << *msg);
    current_left_arm_controller_state_ = *msg;
}

void GantryControl::right_arm_controller_state_callback(const control_msgs::JointTrajectoryControllerState::ConstPtr & msg) {
    // ROS_INFO_STREAM_THROTTLE(10,
    //   "Right arm controller states (throttled to 0.1 Hz):\n" << *msg);
    current_right_arm_controller_state_ = *msg;
}


bool GantryControl::send_command(trajectory_msgs::JointTrajectory command_msg) {
    // ROS_INFO_STREAM("[gantry_control][send_command] called.");

    if(command_msg.points.size() == 0) {
        ROS_WARN("[gantry_control][send_command] Trajectory is empty or NAN, returning.");
        return false;
    }
    else if ((command_msg.joint_names[0] == "small_long_joint") // command is for gantry
             && (command_msg.points[0].positions[0] == command_msg.points[0].positions[0])) {

        gantry_joint_trajectory_publisher_.publish(command_msg);
        // ROS_INFO_STREAM("[gantry_control][send_command] gantry command published!");
        return true;
    }
    else if ((command_msg.joint_names[0].substr(0,4) == "left") // command is for left_arm
             && (command_msg.points[0].positions[0] == command_msg.points[0].positions[0])) {

        left_arm_joint_trajectory_publisher_.publish(command_msg);
        // ROS_INFO_STREAM("[gantry_control][send_command] left_arm command published!");
        return true;
    }
    else if ((command_msg.joint_names[0].substr(0,5) == "right") // command is for right arm
             && (command_msg.points[0].positions[0] == command_msg.points[0].positions[0])) {

        right_arm_joint_trajectory_publisher_.publish(command_msg);
        // ROS_INFO_STREAM("[gantry_control][send_command] right_arm command published!");
        return true;
    }
    else {
        return false;
    }
}

