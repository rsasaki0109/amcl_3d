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

    Time time;
    if (((int)end.nsec - (int)start.nsec) < 0)
    {
        time.sec = end.sec - start.sec - 1;
        time.nsec = end.nsec - start.nsec + 1000000000;
    }
    else
    {
        time.sec = end.sec - start.sec;
        time.nsec = end.nsec - start.nsec;
    }
    return (double)time.sec + (double)time.nsec / 1000000000.0;
}
Time Time::getTimeNow()
{
    return fromROSTime(ros::Time::now());
}
} // namespace amcl_3d
