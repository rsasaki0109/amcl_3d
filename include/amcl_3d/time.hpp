#pragma once

#include <ros/ros.h>

namespace amcl_3d
{
struct Time
{
    Time(const ros::Time &ros_time);
    Time();
    ros::Time toROSTime();
    static Time fromROSTime(const ros::Time &ros_time);
    static double getDiff(const Time &start, const Time &end);
    static Time getTimeNow();
    unsigned int sec;
    unsigned int nsec;
};

} // namespace amcl_3d