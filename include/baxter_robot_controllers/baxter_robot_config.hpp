#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <vector>
#include <map>
#include <string>
#include <iostream>
#include <random>
#include <Eigen/Geometry>
#include <arc_utilities/arc_helpers.hpp>

#ifndef BAXTER_ROBOT_CONFIG_HPP
#define BAXTER_ROBOT_CONFIG_HPP

namespace baxter_robot_controllers
{
    class PIDParams
    {
    protected:

        double kp_;
        double ki_;
        double kd_;
        double i_clamp_;

    public:

        PIDParams(const double kp, const double ki, const double kd, const double i_clamp) : kp_(kp), ki_(ki), kd_(kd), i_clamp_(i_clamp) {}

        PIDParams() : kp_(0.0), ki_(0.0), kd_(0.0), i_clamp_(0.0) {}

        inline double Kp() const
        {
            return kp_;
        }

        inline double Ki() const
        {
            return ki_;
        }

        inline double Kd() const
        {
            return kd_;
        }

        inline double Iclamp() const
        {
            return i_clamp_;
        }
    };

    class JointLimits
    {
    protected:

        double min_position_;
        double max_position_;
        double max_velocity_;
        double max_effort_;

    public:

        JointLimits(const double min_position, const double max_position, const double max_velocity, const double max_effort)
        {
            assert(min_position <= max_position);
            min_position_ = min_position;
            max_position_ = max_position;
            max_velocity_ = std::abs(max_velocity);
            max_effort_ = std::abs(max_effort);
        }

        JointLimits() : min_position_(0.0), max_position_(0.0), max_velocity_(0.0), max_effort_(0.0) {}

        inline double MinPosition() const
        {
            return min_position_;
        }

        inline double MaxPosition() const
        {
            return max_position_;
        }

        inline std::pair<double, double> PositionLimits() const
        {
            return std::make_pair(min_position_, max_position_);
        }

        inline double MaxVelocity() const
        {
            return max_velocity_;
        }

        inline double MaxEffort() const
        {
            return max_effort_;
        }
    };

    inline std::map<std::string, JointLimits> GetLeftArmDefaultLimits()
    {
        std::map<std::string, JointLimits> joint_limits;
        joint_limits["left_s0"] = JointLimits(-1.70167993878, 1.70167993878, 0.27, 500.0);
        joint_limits["left_s1"] = JointLimits(-2.147, 1.047, 0.27, 500.0);
        joint_limits["left_e0"] = JointLimits(-3.05417993878, 3.05417993878, 0.27, 500.0);
        joint_limits["left_e1"] = JointLimits(-0.05, 2.618, 0.27, 500.0);
        joint_limits["left_w0"] = JointLimits(-3.059, 3.059, 0.3, 500.0);
        joint_limits["left_w1"] = JointLimits(-1.57079632679, 2.094, 0.3, 500.0);
        joint_limits["left_w2"] = JointLimits(-3.059, 3.059, 0.5, 500.0);
        return joint_limits;
    }

    inline std::map<std::string, JointLimits> GetRightArmDefaultLimits()
    {
        std::map<std::string, JointLimits> joint_limits;
        joint_limits["right_s0"] = JointLimits(-1.70167993878, 1.70167993878, 0.27, 500.0);
        joint_limits["right_s1"] = JointLimits(-2.147, 1.047, 0.27, 500.0);
        joint_limits["right_e0"] = JointLimits(-3.05417993878, 3.05417993878, 0.27, 500.0);
        joint_limits["right_e1"] = JointLimits(-0.05, 2.618, 0.27, 500.0);
        joint_limits["right_w0"] = JointLimits(-3.059, 3.059, 0.3, 500.0);
        joint_limits["right_w1"] = JointLimits(-1.57079632679, 2.094, 0.3, 500.0);
        joint_limits["right_w2"] = JointLimits(-3.059, 3.059, 0.5, 500.0);
        return joint_limits;
    }

    inline std::map<std::string, JointLimits> GetLeftArmLimits(const double velocity_scaling=1.0, const double effort_scaling=1.0)
    {
        assert(velocity_scaling >= 0.0);
        assert(velocity_scaling <= 1.0);
        assert(effort_scaling >= 0.0);
        assert(effort_scaling <= 1.0);
        const std::map<std::string, JointLimits> default_joint_limits = GetLeftArmDefaultLimits();
        std::map<std::string, JointLimits> joint_limits;
        const auto default_limit_pairs = arc_helpers::GetKeysAndValues(default_joint_limits);
        for (size_t idx = 0; idx < default_limit_pairs.size(); idx++)
        {
            const std::string& joint_name = default_limit_pairs[idx].first;
            const JointLimits& default_joint_limit = default_limit_pairs[idx].second;
            const JointLimits scaled_joint_limit(default_joint_limit.MinPosition(), default_joint_limit.MaxPosition(), (default_joint_limit.MaxVelocity() * velocity_scaling), (default_joint_limit.MaxEffort() * effort_scaling));
            joint_limits[joint_name] = scaled_joint_limit;
        }
        return joint_limits;
    }

    inline std::map<std::string, JointLimits> GetRightArmLimits(const double velocity_scaling=1.0, const double effort_scaling=1.0)
    {
        assert(velocity_scaling >= 0.0);
        assert(velocity_scaling <= 1.0);
        assert(effort_scaling >= 0.0);
        assert(effort_scaling <= 1.0);
        const std::map<std::string, JointLimits> default_joint_limits = GetRightArmDefaultLimits();
        std::map<std::string, JointLimits> joint_limits;
        const auto default_limit_pairs = arc_helpers::GetKeysAndValues(default_joint_limits);
        for (size_t idx = 0; idx < default_limit_pairs.size(); idx++)
        {
            const std::string& joint_name = default_limit_pairs[idx].first;
            const JointLimits& default_joint_limit = default_limit_pairs[idx].second;
            const JointLimits scaled_joint_limit(default_joint_limit.MinPosition(), default_joint_limit.MaxPosition(), (default_joint_limit.MaxVelocity() * velocity_scaling), (default_joint_limit.MaxEffort() * effort_scaling));
            joint_limits[joint_name] = scaled_joint_limit;
        }
        return joint_limits;
    }

    inline std::map<std::string, PIDParams> GetLeftArmDefaultPositionControllerParams()
    {
        std::map<std::string, baxter_robot_controllers::PIDParams> joint_controller_params;
        joint_controller_params["left_s0"] = baxter_robot_controllers::PIDParams(1.0, 0.0, 0.1, 1.0);
        joint_controller_params["left_s1"] = baxter_robot_controllers::PIDParams(1.0, 0.0, 0.1, 1.0);
        joint_controller_params["left_e0"] = baxter_robot_controllers::PIDParams(1.0, 0.0, 0.1, 1.0);
        joint_controller_params["left_e1"] = baxter_robot_controllers::PIDParams(1.0, 0.0, 0.1, 1.0);
        joint_controller_params["left_w0"] = baxter_robot_controllers::PIDParams(1.0, 0.0, 0.1, 1.0);
        joint_controller_params["left_w1"] = baxter_robot_controllers::PIDParams(1.0, 0.0, 0.1, 1.0);
        joint_controller_params["left_w2"] = baxter_robot_controllers::PIDParams(2.0, 0.0, 0.1, 1.0);
        return joint_controller_params;
    }

    inline std::map<std::string, PIDParams> GetRightArmDefaultPositionControllerParams()
    {
        std::map<std::string, baxter_robot_controllers::PIDParams> joint_controller_params;
        joint_controller_params["right_s0"] = baxter_robot_controllers::PIDParams(1.0, 0.0, 0.1, 1.0);
        joint_controller_params["right_s1"] = baxter_robot_controllers::PIDParams(1.0, 0.0, 0.1, 1.0);
        joint_controller_params["right_e0"] = baxter_robot_controllers::PIDParams(1.0, 0.0, 0.1, 1.0);
        joint_controller_params["right_e1"] = baxter_robot_controllers::PIDParams(1.0, 0.0, 0.1, 1.0);
        joint_controller_params["right_w0"] = baxter_robot_controllers::PIDParams(1.0, 0.0, 0.1, 1.0);
        joint_controller_params["right_w1"] = baxter_robot_controllers::PIDParams(1.0, 0.0, 0.1, 1.0);
        joint_controller_params["right_w2"] = baxter_robot_controllers::PIDParams(2.0, 0.0, 0.1, 1.0);
        return joint_controller_params;
    }

    inline std::map<std::string, PIDParams> GetLeftArmDefaultVelocityControllerParams()
    {
        std::map<std::string, baxter_robot_controllers::PIDParams> joint_controller_params;
        joint_controller_params["left_s0"] = baxter_robot_controllers::PIDParams(2.0, 0.0, 0.0, 0.0);
        joint_controller_params["left_s1"] = baxter_robot_controllers::PIDParams(8.0, 0.0, 0.0, 0.0);
        joint_controller_params["left_e0"] = baxter_robot_controllers::PIDParams(3.0, 0.0, 0.0, 0.0);
        joint_controller_params["left_e1"] = baxter_robot_controllers::PIDParams(5.0, 0.0, 0.0, 0.0);
        joint_controller_params["left_w0"] = baxter_robot_controllers::PIDParams(2.0, 0.0, 0.0, 0.0);
        joint_controller_params["left_w1"] = baxter_robot_controllers::PIDParams(1.0, 0.0, 0.0, 0.0);
        joint_controller_params["left_w2"] = baxter_robot_controllers::PIDParams(1.0, 0.0, 0.0, 0.0);
        return joint_controller_params;
    }

    inline std::map<std::string, PIDParams> GetRightArmDefaultVelocityControllerParams()
    {
        std::map<std::string, baxter_robot_controllers::PIDParams> joint_controller_params;
        joint_controller_params["right_s0"] = baxter_robot_controllers::PIDParams(2.0, 0.0, 0.0, 0.0);
        joint_controller_params["right_s1"] = baxter_robot_controllers::PIDParams(8.0, 0.0, 0.0, 0.0);
        joint_controller_params["right_e0"] = baxter_robot_controllers::PIDParams(3.0, 0.0, 0.0, 0.0);
        joint_controller_params["right_e1"] = baxter_robot_controllers::PIDParams(5.0, 0.0, 0.0, 0.0);
        joint_controller_params["right_w0"] = baxter_robot_controllers::PIDParams(2.0, 0.0, 0.0, 0.0);
        joint_controller_params["right_w1"] = baxter_robot_controllers::PIDParams(1.0, 0.0, 0.0, 0.0);
        joint_controller_params["right_w2"] = baxter_robot_controllers::PIDParams(1.0, 0.0, 0.0, 0.0);
        return joint_controller_params;
    }

}
#endif // BAXTER_ROBOT_CONFIG_HPP
