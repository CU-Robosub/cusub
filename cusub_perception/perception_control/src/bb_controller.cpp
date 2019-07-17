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

    void BBController::configureAxes(AxisConfig x, AxisConfig y)
    {
        switch(x)
        {
            case DRIVE_AXIS:
                x_state = &driveState;
                x_pub = &drivePub;
                break;
            case STRAFE_AXIS:
                x_state = &strafeState;
                x_pub = &strafePub;
                break;
            case YAW_AXIS:
                x_state = &yawState;
                x_pub = &yawPub;
                break;
            case DEPTH_AXIS:
                x_state = &depthState;
                x_pub = &depthPub;
                break;
            default:
                ROS_ERROR("Unrecognized AxisConfig!");
                abort();
        }

        switch(y)
        {
            case DRIVE_AXIS:
                y_state = &driveState;
                y_pub = &drivePub;
                break;
            case STRAFE_AXIS:
                y_state = &strafeState;
                y_pub = &strafePub;
                break;
            case YAW_AXIS:
                y_state = &yawState;
                y_pub = &yawPub;
                break;
            case DEPTH_AXIS:
                y_state = &depthState;
                y_pub = &depthPub;
                break;
            default:
                ROS_ERROR("Unrecognized AxisConfig!");
                abort();
        }
        ROS_INFO("BBController axes configured!");
    }

    void BBController::driveCallback(const std_msgs::Float64ConstPtr state) { driveState = state->data; }
    void BBController::strafeCallback(const std_msgs::Float64ConstPtr state) { strafeState = state->data; }
    void BBController::yawCallback(const std_msgs::Float64ConstPtr state) { yawState = state->data; }
    void BBController::depthCallback(const std_msgs::Float64ConstPtr state) { depthState = state->data; }
}
