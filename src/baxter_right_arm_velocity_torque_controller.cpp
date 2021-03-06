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
    ros::init(argc, argv, "baxter_right_arm_velocity_torque_controller");
    ROS_INFO("Starting baxter_right_arm_velocity_torque_controller...");
    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");
    const std::string DEFAULT_VELOCITY_COMMAND_TOPIC = "/robot/limb/right/joint_command_velocity";
    const std::string DEFAULT_CONFIG_FEEDBACK_TOPIC = "/robot/joint_states";
    const std::string DEFAULT_TORQUE_COMMAND_TOPIC = "/robot/limb/right/joint_command";
    const std::string DEFAULT_ABORT_SERVICE = "baxter_right_arm_velocity_torque_controller/abort";
    const double DEFAULT_CONTROL_RATE = 1000.0; //25.0;
    const bool DEFAULT_MODEL_GRAVITY = false;
    const int32_t DEFAULT_VELOCITY_FILTER_WINDOW_SIZE = 1;
    std::string torque_command_topic;
    std::string velocity_command_topic;
    std::string config_feedback_topic;
    std::string abort_service;
    double control_rate = DEFAULT_CONTROL_RATE;
    bool model_gravity = DEFAULT_MODEL_GRAVITY;
    int32_t velocity_filter_window_size = DEFAULT_VELOCITY_FILTER_WINDOW_SIZE;
    nhp.param(std::string("torque_command_topic"), torque_command_topic, DEFAULT_TORQUE_COMMAND_TOPIC);
    nhp.param(std::string("config_feedback_topic"), config_feedback_topic, DEFAULT_CONFIG_FEEDBACK_TOPIC);
    nhp.param(std::string("velocity_command_topic"), velocity_command_topic, DEFAULT_VELOCITY_COMMAND_TOPIC);
    nhp.param(std::string("abort_service"), abort_service, DEFAULT_ABORT_SERVICE);
    nhp.param(std::string("control_rate"), control_rate, DEFAULT_CONTROL_RATE);
    nhp.param(std::string("model_gravity"), model_gravity, DEFAULT_MODEL_GRAVITY);
    nhp.param(std::string("velocity_filter_window_size"), velocity_filter_window_size, DEFAULT_VELOCITY_FILTER_WINDOW_SIZE);
    // Get the XML string of the URDF
    std::string xml_model_string;
    nh.param(std::string("robot_description"), xml_model_string, std::string(""));
    // Joint limits
    const std::map<std::string, baxter_robot_controllers::JointLimits> joint_limits = baxter_robot_controllers::GetRightArmLimits();
    // Joint PID params
    const std::map<std::string, baxter_robot_controllers::PIDParams> joint_controller_params = baxter_robot_controllers::GetRightArmDefaultVelocityControllerParams();
    baxter_robot_controllers::BaxterRobotVelocityTorqueController controller(nh, velocity_command_topic, config_feedback_topic, torque_command_topic, abort_service, xml_model_string, model_gravity, (uint32_t)velocity_filter_window_size, joint_limits, joint_controller_params);
    ROS_INFO("...startup complete");
    controller.Loop(control_rate);
    return 0;
}
