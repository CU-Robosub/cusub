#ifndef VISUALSERVO_CLASS_SRC_VISUALSERVO_CLASS_H_
#define VISUALSERVO_CLASS_SRC_VISUALSERVO_CLASS_H_

#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <actionlib/server/simple_action_server.h>
#include <waypoint_navigator/ToggleControl.h>
#include <perception_control/VisualServoAction.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <perception_control/bb_proportional.h>
#include <perception_control/bb_controller.h>
#include <std_msgs/Float64.h>
#include <cstdint>

namespace perception_control
{
typedef actionlib::SimpleActionServer<perception_control::VisualServoAction> vsServer;

typedef enum ImageAxis_t{
    X_AXIS=0,
    Y_AXIS=1,
    AREA_AXIS=2
} ImageAxis;

class VisualServo : public nodelet::Nodelet
{
public:
    virtual void onInit();
private:
    bool getTarget(const std::vector<darknet_ros_msgs::BoundingBox> &boxes, darknet_ros_msgs::BoundingBox &targetBox);
    bool respondError(ImageAxis axis, float error);
    bool respondDowncamError(ImageAxis axis, float error);
    bool respondOccamError(ImageAxis axis, float error);

    bool controlPids(const bool takeControl);
    void execute(const perception_control::VisualServoGoalConstPtr goal);
    void darknetCallback(const darknet_ros_msgs::BoundingBoxesConstPtr bbs);

    perception_control::VisualServoGoalConstPtr activeGoal;
    std::string target_frame;
    std::vector<std::string> target_classes;
    vsServer* server;
    ros::NodeHandle* nh;
    ros::ServiceClient wayToggleClient; 
    bool controllingPids;
    int target_pixel_x, target_pixel_y, target_box_area, target_pixel_threshold;
    std_msgs::Float64 frozen_x_set, frozen_y_set;
    bool frozen_controls;

    std::string activeCamera;
    ImageAxis x_map_axis, y_map_axis, area_map_axis;

    // Controllers
    BBController* current_controller;
    BBProportional* proportional_controller;

    // Response Methods
    void (BBController::*respondX)(float);
    void (BBController::*respondY)(float);
    void (BBController::*respondArea)(float);

    bool xResponse, yResponse, areaResponse;
    int requestNum;
};
}; // namespace perception_control

#endif // VISUALSERVO_CLASS_SRC_VISUALSERVO_CLASS_H_