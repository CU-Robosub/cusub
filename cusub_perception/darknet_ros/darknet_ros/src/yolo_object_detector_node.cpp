/*
 * yolo_obstacle_detector_node.cpp
 *
 *  Created on: Dec 19, 2016
 *      Author: Marko Bjelonic
 *   Institute: ETH Zurich, Robotic Systems Lab
 */

#include <darknet_ros/YoloObjectDetector.hpp>
#include <ros/ros.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "darknet_ros");
  darknet_ros::YoloObjectDetector yolo;
  yolo.set_not_nodelet();
  yolo.onInit();
  ros::spin();
  return 0;
}
