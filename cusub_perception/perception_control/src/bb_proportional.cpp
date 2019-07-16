#include <perception_control/bb_proportional.h>

namespace perception_control
{
    BBProportional::BBProportional(ros::NodeHandle& nh) : BBController(nh)
    {
        // load rosparams
        std::cout << "loading BB Proportional" << std::endl;
    }

    void BBProportional::respond(int xdiff, int ydiff)
    {
        ;
    }
}