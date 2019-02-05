#include "amcl_3d/time.hpp"

namespace amcl_3d
{
Time::Time(const ros::Time &ros_time)
{
    sec = ros_time.sec;
    nsec = ros_time.nsec;
}
Time::Time()
{
}
ros::Time Time::toROSTime()
{
    ros::Time time;
    time.sec = sec;
    time.nsec = nsec;
    return time;
}
Time Time::fromROSTime(const ros::Time &ros_time)
{
    Time time;
    time.sec = ros_time.sec;
    time.nsec = ros_time.nsec;
    return time;
}
double Time::getDiff(const Time &start, const Time &end)
{
    return ((double)end.sec + (double)end.nsec / 1000000000.0) - ((double)start.sec + (double)start.nsec / 1000000000.0);
}
Time Time::getTimeNow()
{
    return fromROSTime(ros::Time::now());
}
} // namespace amcl_3d
