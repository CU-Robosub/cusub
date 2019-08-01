#include <perception_control/bb_controller.h>

namespace perception_control
{
    BBController::BBController(ros::NodeHandle& nh)
    {
        driveSub = nh.subscribe("cusub_common/motor_controllers/pid/drive/state", 1, &BBController::driveCallback, this);
        strafeSub = nh.subscribe("cusub_common/motor_controllers/pid/strafe/state", 1, &BBController::strafeCallback, this);
        yawSub = nh.subscribe("cusub_common/motor_controllers/pid/yaw/state", 1, &BBController::yawCallback, this);
        depthSub = nh.subscribe("cusub_common/motor_controllers/pid/depth/state", 1, &BBController::depthCallback, this);
        drivePub = nh.advertise<std_msgs::Float64>("cusub_common/motor_controllers/pid/drive/setpoint",1);
        strafePub = nh.advertise<std_msgs::Float64>("cusub_common/motor_controllers/pid/strafe/setpoint",1);
        yawPub = nh.advertise<std_msgs::Float64>("cusub_common/motor_controllers/pid/yaw/setpoint",1);
        depthPub = nh.advertise<std_msgs::Float64>("cusub_common/motor_controllers/pid/depth/setpoint",1);
    }

    void BBController::driveCallback(const std_msgs::Float64ConstPtr state) { driveState = state->data; }
    void BBController::strafeCallback(const std_msgs::Float64ConstPtr state) { strafeState = state->data; }
    void BBController::yawCallback(const std_msgs::Float64ConstPtr state) { yawState = state->data; }
    void BBController::depthCallback(const std_msgs::Float64ConstPtr state) { depthState = state->data; }
}
