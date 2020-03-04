#include "downcam_respawner/respawner.hpp"

using namespace downcam_watcher_ns;
using namespace std;

void Respawner::onInit()
{
    ros::NodeHandle& nh = getMTNodeHandle();
    downcam_sub = nh.subscribe("cusub_common/downcam/image_color", 1, &Respawner::downcamCallback, this);

    // ros::NodeHandle nh_private("~");
    ros::NodeHandle& nh_private = getPrivateNodeHandle();
    watchdog_period = 5; // default
    nh_private.getParam("watchdog_period", watchdog_period);

    cuprint(std::string("timeout set to \033[95m") + std::to_string(watchdog_period) + std::string("\033[0ms"));

    cuprint("waiting for first downcam message");
    // ros::topic::waitForMessage<sensor_msgs::Image>("cusub_common/downcam/image_color", ros::Duration(7));
    timer = nh.createTimer(ros::Duration(watchdog_period), &Respawner::timerCallback, this);
    image_received = true;
}

void Respawner::downcamCallback(const sensor_msgs::ImageConstPtr image)
{
    cout << "received image" << endl;
    image_received = true;
}

void Respawner::timerCallback(const ros::TimerEvent& event)
{
    if (!image_received)
    {
        cuprint_warn("no images received in last: " + std::to_string(watchdog_period) + std::string("s"));
        // kill + respawn nodes
        exit(0);
    } else
    {
        image_received = false;
    }
}
void Respawner::cuprint(std::string str)
{
    std::string print_str = std::string("[\033[92mDowncam Watchdog\033[0m] ");
    ROS_INFO( (print_str + str).c_str());
}

void Respawner::cuprint_warn(std::string str)
{
    std::string print_str = std::string("[\033[92mDowncam Watchdog\033[0m] ");
    print_str = print_str + std::string("\033[93m[WARN] ") + str + std::string("\033[0m");
    ROS_INFO( print_str.c_str());
}

PLUGINLIB_EXPORT_CLASS(downcam_watcher_ns::Respawner, nodelet::Nodelet);