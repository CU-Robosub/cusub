#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Imu.h"
//#include

#include <sstream>

int main(int argc, char **argv)
{

  ros::init(argc, argv, "pni_trax");

  ros::NodeHandle n;

  ros::Publisher pub = n.advertise<sensor_msgs::Imu>("imu_raw", 1000);

  ros::Rate loop_rate(10);

  while (ros::ok())
  {

    sensor_msgs::Imu msg = sensor_msgs::Imu();

    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "base_link";
    msg.orientation_covariance = [0.0001,0,0,0,0.0001,0,0,0,0.0001]
    msg.angular_velocity_covariance = [0.0001,0,0,0,0.0001,0,0,0,0.0001]
    msg.linear_acceleration_covariance = [0.0001,0,0,0,0.0001,0,0,0,0.0001]
    // msg.orientation.x = //call to function having 'x' data
    // msg.orientation.y = //call to function having 'y' data
    // msg.orientation.z = //call to function having 'z' data
    // msg.orientation.w = //call to function having 'w' data

    msg.angular_velocity.x = //call to function having 'x' data
    msg.angular_velocity.y = //call to function having 'y' data
    msg.angular_velocity.z = //call to function having 'z' data

    msg.linear_acceleration.x = //call to function having 'x' data
    msg.linear_acceleration.y = //call to function having 'y' data
    msg.linear_acceleration.z = //call to function having 'z' data

    ROS_INFO("%s", "publish imu messages");

    pub.publish(msg);

    ros::spinOnce();
    loop_rate.sleep();

  }

  return 0;
}
