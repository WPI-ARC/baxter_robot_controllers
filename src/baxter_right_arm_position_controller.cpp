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
#include <baxter_robot_controllers/baxter_robot_position_controller.hpp>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "baxter_right_arm_position_controller");
    ROS_INFO("Starting baxter_right_arm_position_controller...");
    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");
    const std::string DEFAULT_POSITION_COMMAND_TOPIC = "/robot/limb/right/joint_command_position";
    const std::string DEFAULT_STATUS_TOPIC = "baxter_right_arm_position_controller/status";
    const std::string DEFAULT_CONFIG_FEEDBACK_TOPIC = "/robot/joint_states";
    const std::string DEFAULT_VELOCITY_COMMAND_TOPIC = "/robot/limb/right/joint_command_velocity";
    const std::string DEFAULT_ABORT_SERVICE = "baxter_right_arm_position_controller/abort";
    const double DEFAULT_CONTROL_RATE = 100.0; //25.0;
    const double DEFAULT_VELOCITY_LIMIT_SCALING = 0.75;
    std::string position_command_topic;
    std::string velocity_command_topic;
    std::string config_feedback_topic;
    std::string status_topic;
    std::string abort_service;
    double control_rate = DEFAULT_CONTROL_RATE;
    double velocity_limit_scaling = DEFAULT_VELOCITY_LIMIT_SCALING;
    nhp.param(std::string("position_command_topic"), position_command_topic, DEFAULT_POSITION_COMMAND_TOPIC);
    nhp.param(std::string("config_feedback_topic"), config_feedback_topic, DEFAULT_CONFIG_FEEDBACK_TOPIC);
    nhp.param(std::string("velocity_command_topic"), velocity_command_topic, DEFAULT_VELOCITY_COMMAND_TOPIC);
    nhp.param(std::string("status_topic"), status_topic, DEFAULT_STATUS_TOPIC);
    nhp.param(std::string("abort_service"), abort_service, DEFAULT_ABORT_SERVICE);
    nhp.param(std::string("control_rate"), control_rate, DEFAULT_CONTROL_RATE);
    nhp.param(std::string("velocity_limit_scaling"), velocity_limit_scaling, DEFAULT_VELOCITY_LIMIT_SCALING);
    const double real_velocity_limit_scaling = std::max(0.0, std::min(1.0, std::abs(velocity_limit_scaling)));
    // Joint limits
    const std::map<std::string, baxter_robot_controllers::JointLimits> joint_limits = baxter_robot_controllers::GetRightArmLimits(real_velocity_limit_scaling);
    // Joint PID params
    const std::map<std::string, baxter_robot_controllers::PIDParams> joint_controller_params = baxter_robot_controllers::GetRightArmDefaultPositionControllerParams();
    baxter_robot_controllers::BaxterRobotPositionController controller(nh, position_command_topic, config_feedback_topic, velocity_command_topic, status_topic, abort_service, joint_limits, joint_controller_params);
    ROS_INFO("...startup complete");
    controller.Loop(control_rate);
    return 0;
}
