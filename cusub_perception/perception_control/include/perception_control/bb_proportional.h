#ifndef BB_PROPORTIONAL_CLASS_SRC_BB_PROPORTIONAL_CLASS_H_
#define BB_PROPORTIONAL_CLASS_SRC_BB_PROPORTIONAL_CLASS_H_

#include <perception_control/bb_controller.h>

namespace perception_control
{
    class BBProportional : public BBController
    {
    public:
        void respond(int xdiff, int ydiff);
        BBProportional(ros::NodeHandle& nh);
    };
}

#endif