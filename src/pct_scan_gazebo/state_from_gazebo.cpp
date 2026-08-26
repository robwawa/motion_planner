#include "gazebo_msgs/LinkStates.h"
#include "gazebo_msgs/ModelStates.h"
#include "geometry_msgs/TransformStamped.h"
#include "ros/ros.h"
// #include "tf2_ros/transform_listener.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <boost/bind.hpp>   // 将tf监听绑定到ROS回调函数


using namespace std;
ros::Publisher robotVelocity_BASE_frame_pub;
string robot_name = "a1";
nav_msgs::Odometry Odom;
tf::Transform transform_map2odom;
ros::Time last_dynamic_stamp;


void callback_BASE(const gazebo_msgs::LinkStates::ConstPtr &msg) {
    int index = 0;
    for (auto &linkName : msg->name) {
        if (linkName == robot_name+"_gazebo::base")
            break;
        ++index;
    }
    if (index == static_cast<int>(msg->name.size())) {
        ROS_WARN_THROTTLE(2.0, "Gazebo base link for %s was not found", robot_name.c_str());
        return;
    }

    const ros::Time stamp = ros::Time::now();
    // Gazebo can emit more than one LinkStates message per simulated clock tick.
    // TF2 rejects repeated dynamic transforms from the same authority, so only
    // publish one odom/base sample for each simulated timestamp.
    if (stamp.isZero() || stamp == last_dynamic_stamp) {
        return;
    }

    tf::Point pt_map(msg->pose[index].position.x,msg->pose[index].position.y,msg->pose[index].position.z);
    tf::Point pt_odom = transform_map2odom * pt_map;
    
    tf::Quaternion q_map(msg->pose[index].orientation.x,
                        msg->pose[index].orientation.y,
                        msg->pose[index].orientation.z,
                        msg->pose[index].orientation.w);
    tf::Quaternion q_odom = transform_map2odom.getRotation() * q_map;

    // 转换为odom的速度关系
    tf::Vector3 linear_vel(
        msg->twist[index].linear.x,
        msg->twist[index].linear.y,
        msg->twist[index].linear.z);
    tf::Vector3 transformed_linear_vel = transform_map2odom.getBasis() * linear_vel;

    tf::Vector3 angular_vel(
        msg->twist[index].angular.x,
        msg->twist[index].angular.y,
        msg->twist[index].angular.z);
    tf::Vector3 transformed_angular_vel = transform_map2odom.getBasis() * angular_vel;
    
    //发布base到odom的tf变换
    static tf::TransformBroadcaster dynamic_tf_broadcaster;
    tf::Transform transform_odom2base;
    transform_odom2base.setRotation(q_odom);
    transform_odom2base.setOrigin(pt_odom);

    dynamic_tf_broadcaster.sendTransform(
        tf::StampedTransform(transform_odom2base, stamp, "odom", "base"));

    Odom.header.stamp = stamp;
    Odom.header.frame_id = "odom";
    Odom.child_frame_id = "base";

    // set the position
    Odom.pose.pose.position.x = pt_odom.x();
    Odom.pose.pose.position.y = pt_odom.y();
    Odom.pose.pose.position.z = pt_odom.z();

    Odom.pose.pose.orientation.w = q_odom.w();
    Odom.pose.pose.orientation.x = q_odom.x();
    Odom.pose.pose.orientation.y = q_odom.y();
    Odom.pose.pose.orientation.z = q_odom.z();


    // set the velocity
    Odom.twist.twist.linear.x = transformed_linear_vel.x();
    Odom.twist.twist.linear.y = transformed_linear_vel.y();
    Odom.twist.twist.linear.z = transformed_linear_vel.z();


    Odom.twist.twist.angular.x = transformed_angular_vel.x();
    Odom.twist.twist.angular.y = transformed_angular_vel.y();
    Odom.twist.twist.angular.z = transformed_angular_vel.z();


    robotVelocity_BASE_frame_pub.publish(Odom);
    last_dynamic_stamp = stamp;
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "state_from_gazebo");
    ros::NodeHandle nh("~");
    ros::NodeHandle node;
    ros::Subscriber tfState_BASE_sub;

    // tf::TransformListener tf_listener_;

    if (argc != 7)   // x y z yaw pitch roll
    {
        ROS_ERROR("Usage: static_transform_publisher x y z yaw pitch roll");
        return -1;
    }

    const double x = atof(argv[1]);
    const double y = atof(argv[2]);
    const double z = atof(argv[3]);
    const double yaw = atof(argv[4]);
    const double pitch = atof(argv[5]);
    const double roll = atof(argv[6]);

    tf::Transform transform_odom2map;
    transform_odom2map.setOrigin(tf::Vector3(x, y, z));
    transform_odom2map.setRotation(tf::createQuaternionFromRPY(roll, pitch, yaw));
    transform_map2odom = transform_odom2map.inverse();

    geometry_msgs::TransformStamped map_to_odom;
    map_to_odom.header.stamp = ros::Time::now();
    map_to_odom.header.frame_id = "map";
    map_to_odom.child_frame_id = "odom";
    map_to_odom.transform.translation.x = x;
    map_to_odom.transform.translation.y = y;
    map_to_odom.transform.translation.z = z;
    map_to_odom.transform.rotation.x = transform_odom2map.getRotation().x();
    map_to_odom.transform.rotation.y = transform_odom2map.getRotation().y();
    map_to_odom.transform.rotation.z = transform_odom2map.getRotation().z();
    map_to_odom.transform.rotation.w = transform_odom2map.getRotation().w();
    tf2_ros::StaticTransformBroadcaster static_tf_broadcaster;
    static_tf_broadcaster.sendTransform(map_to_odom);
  
    nh.param<std::string>("robot_name", robot_name, string("a1"));
    tfState_BASE_sub = node.subscribe<gazebo_msgs::LinkStates>("/gazebo/link_states", 10, callback_BASE);
    robotVelocity_BASE_frame_pub = node.advertise<nav_msgs::Odometry>("/Odometry_gazebo", 1);

    ros::spin();
    return 0;
}
