#include "downcam_respawner/respawner.hpp"

using namespace downcam_watcher_ns;
using namespace std;

void Respawner::onInit()
{
    cout << "starting up!" << endl;
}

void Respawner::downcamCallback(const sensor_msgs::ImageConstPtr image)
{
    ;
}

void Respawner::timerCallback(const ros::TimerEvent&)
{
    ;
}

PLUGINLIB_EXPORT_CLASS(downcam_watcher_ns::Respawner, nodelet::Nodelet);