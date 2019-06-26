/*
    Multiplexes occam and downcam camera frames into a single topic for darknet.
    Dynamically configurable via service TODO service name
    Update rate configurable via config/darknet_multiplexer.yaml
 */

 #include <darknet_multiplexer/multiplexer.h>

 namespace darknet_multiplexer_ns
 {
     void Multiplexer::onInit()
     {
         NODELET_INFO("Starting darknet multiplexer");
     }

    //  void Multiplexer::cameraCallback()
    //  {
    //      ;
    //  }

    //  void Multiplexer::publishFrame()
 }

 PLUGINLIB_EXPORT_CLASS(darknet_multiplexer_ns::Multiplexer, nodelet::Nodelet);