#include "detection_tree/detection_tree_node.hpp"

using namespace det_tree_ns;

void DetectionTree::onInit()
{
    NODELET_INFO("Loaded Detection Tree");
}

PLUGINLIB_EXPORT_CLASS(det_tree_ns::DetectionTree, nodelet::Nodelet);