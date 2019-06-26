#ifndef MULTIPLEXER_CLASS_SRC_MULTIPLEXER_CLASS_H_
#define MULTIPLEXER_CLASS_SRC_MULTIPLEXER_CLASS_H_

#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>

namespace darknet_multiplexer_ns
{
    class Multiplexer : public nodelet::Nodelet
    {
    public:
        virtual void onInit();
    };
}
#endif