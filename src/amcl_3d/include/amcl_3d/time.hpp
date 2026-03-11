#pragma once

#include <cstdint>

#include <builtin_interfaces/msg/time.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/time.hpp>

namespace amcl_3d
{
struct Time
{
    Time();
    explicit Time(const builtin_interfaces::msg::Time & ros_time);
    explicit Time(const rclcpp::Time & ros_time);

    builtin_interfaces::msg::Time toROSTime() const;
    rclcpp::Time toRclcppTime(rcl_clock_type_t clock_type = RCL_ROS_TIME) const;

    static Time fromROSTime(const builtin_interfaces::msg::Time & ros_time);
    static Time fromRclcppTime(const rclcpp::Time & ros_time);
    static double getDiff(const Time &start, const Time &end);
    static Time getTimeNow();

    bool isZero() const;

    int32_t sec;
    uint32_t nsec;
};

} // namespace amcl_3d
