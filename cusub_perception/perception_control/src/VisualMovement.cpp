#include "perception_control/VisualMovement.h"

using namespace perception_control;

VisualMovement::VisualMovement()
{}

void VisualMovement::initializeMovement(ros::NodeHandle * nh)
{
    m_wayToggleClient = nh->serviceClient<waypoint_navigator::ToggleControl>("cusub_common/toggleWaypointControl");
    m_proportionalController = new BBProportional(*nh);
    m_rotationalController = new BBRotational(*nh);
}

bool VisualMovement::controlPids(const bool takeControl)
{
    if (takeControl != m_controllingPids)
    {
        bool success;
        waypoint_navigator::ToggleControl toggle_srv;
        toggle_srv.request.waypoint_controlling = !takeControl;
        toggle_srv.response.success = false;
        if( m_wayToggleClient.call(toggle_srv) )
        {
            success = true;
        }
        else
        {
            success = false;
        }
        m_controllingPids = takeControl;
        success = toggle_srv.response.success;
        return success;
    }
    return true;
}
