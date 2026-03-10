#include "amcl_3d/time.hpp"

namespace amcl_3d
{
Time::Time() : sec(0), nsec(0) {}

Time::Time(const builtin_interfaces::msg::Time & ros_time)
{
    sec = ros_time.sec;
    nsec = ros_time.nanosec;
}

Time::Time(const rclcpp::Time & ros_time)
{
    sec = static_cast<int32_t>(ros_time.seconds());
    nsec = static_cast<uint32_t>(ros_time.nanoseconds() % 1000000000LL);
}

builtin_interfaces::msg::Time Time::toROSTime() const
{
    builtin_interfaces::msg::Time time;
    time.sec = sec;
    time.nanosec = nsec;
    return time;
}

rclcpp::Time Time::toRclcppTime(rcl_clock_type_t clock_type) const
{
    return rclcpp::Time(sec, nsec, clock_type);
}

Time Time::fromROSTime(const builtin_interfaces::msg::Time & ros_time)
{
    return Time(ros_time);
}

Time Time::fromRclcppTime(const rclcpp::Time & ros_time)
{
    return Time(ros_time);
}

double Time::getDiff(const Time &start, const Time &end)
{
    return (static_cast<double>(end.sec) + static_cast<double>(end.nsec) / 1000000000.0) -
           (static_cast<double>(start.sec) + static_cast<double>(start.nsec) / 1000000000.0);
}

Time Time::getTimeNow()
{
    return fromRclcppTime(rclcpp::Clock(RCL_SYSTEM_TIME).now());
}

bool Time::isZero() const
{
    return sec == 0 && nsec == 0;
}
} // namespace amcl_3d
