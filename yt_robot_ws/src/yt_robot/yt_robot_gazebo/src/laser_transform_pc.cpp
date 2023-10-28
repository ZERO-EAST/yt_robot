#include "ros/ros.h"
#include "laser_geometry/laser_geometry.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud.h"
#include "tf/transform_listener.h"
#include <boost/shared_ptr.hpp>

laser_geometry::LaserProjection  projector;
ros::Publisher pub;
boost::shared_ptr<tf::TransformListener> listener_;

void chatterCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    sensor_msgs::PointCloud2 cloud;
    // projector.projectLaser(*msg, cloud);    // 第一种方法

    if(!listener_->waitForTransform(
                msg->header.frame_id,
                "/base_link",
                msg->header.stamp + ros::Duration().fromSec(msg->ranges.size()*msg->time_increment),
                ros::Duration(2.0)) )
    {
        ROS_WARN("no transform");
        return;
    }
    projector.transformLaserScanToPointCloud("base_link",*msg,
                                             cloud, *listener_);
    pub.publish(cloud);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "laser_transform_pc");
    ros::NodeHandle nh;
    listener_ = boost::make_shared<tf::TransformListener>();
    pub = nh.advertise<sensor_msgs::PointCloud2>("cloud",10);
    ros::Subscriber sub = nh.subscribe("/v16_laser_scan", 1000, chatterCallback);   // 先有发布的scan话题

    ros::spin();
    return 0;
}
