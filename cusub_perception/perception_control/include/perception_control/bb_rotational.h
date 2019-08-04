#ifndef BB_ROTATIONAL_CLASS_SRC_BB_PROPORTIONAL_CLASS_H_
#define BB_ROTATIONAL_CLASS_SRC_BB_PROPORTIONAL_CLASS_H_

#include <perception_control/bb_controller.h>
#include <string>
#include <std_msgs/Float64.h>

namespace perception_control
{
class BBRotational : public BBController
{
public:
    virtual float getTargetYaw(const std::string &occamFrame);
    virtual void targetYaw(float &target);

    BBRotational(ros::NodeHandle& nh);

private:
    void createOccamTfs(const std::string &frameNS, std::vector<std::string> occamFrames, const float &frameOffset);

    float m_rotationCarrot;
    float m_rotationThresh;

    float m_occamTargetYaw;
    std::map<std::string, float> m_occamTransforms;

};

}; // namespace perception_control

#endif // BB_ROTATIONAL_CLASS_SRC_BB_PROPORTIONAL_CLASS_H_