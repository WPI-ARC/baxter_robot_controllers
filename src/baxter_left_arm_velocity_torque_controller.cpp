#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <vector>
#include <map>
#include <string>
#include <iostream>
#include <random>
#include <Eigen/Geometry>
#include <time.h>
#include <chrono>
#include <ros/ros.h>
#include <baxter_robot_controllers/baxter_robot_velocity_torque_controller.hpp>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "baxter_left_arm_velocity_torque_controller");
    ROS_INFO("Starting baxter_left_arm_velocity_torque_controller...");
    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");
    const std::string DEFAULT_VELOCITY_COMMAND_TOPIC = "/robot/limb/left/joint_command_velocity";
    const std::string DEFAULT_CONFIG_FEEDBACK_TOPIC = "/robot/joint_states";
    const std::string DEFAULT_TORQUE_COMMAND_TOPIC = "/robot/limb/left/joint_command";
    const std::string DEFAULT_ABORT_SERVICE = "baxter_left_arm_velocity_torque_controller/abort";
    const double DEFAULT_CONTROL_RATE = 1000.0; //25.0;
    const std::string velocity_command_topic = nhp.param(std::string("velocity_command_topic"), DEFAULT_VELOCITY_COMMAND_TOPIC);
    const std::string torque_command_topic = nhp.param(std::string("torque_command_topic"), DEFAULT_TORQUE_COMMAND_TOPIC);
    const std::string config_feedback_topic = nhp.param(std::string("config_feedback_topic"), DEFAULT_CONFIG_FEEDBACK_TOPIC);
    const std::string abort_service = nhp.param(std::string("abort_service"), DEFAULT_ABORT_SERVICE);
    const double control_rate = nhp.param(std::string("control_rate"), DEFAULT_CONTROL_RATE);
    // Get the XML string of the URDF
    const std::string xml_model_string = nh.param(std::string("robot_description"), std::string(""));
    // Joint limits
    const std::map<std::string, baxter_robot_controllers::JointLimits> joint_limits = baxter_robot_controllers::GetLeftArmLimits();
    // Joint PID params
    const std::map<std::string, baxter_robot_controllers::PIDParams> joint_controller_params = baxter_robot_controllers::GetLeftArmDefaultPositionControllerParams();
    baxter_robot_controllers::BaxterRobotVelocityTorqueController controller(nh, velocity_command_topic, config_feedback_topic, torque_command_topic, abort_service, xml_model_string, joint_limits, joint_controller_params);
    ROS_INFO("...startup complete");
    controller.Loop(control_rate);
    return 0;
}
