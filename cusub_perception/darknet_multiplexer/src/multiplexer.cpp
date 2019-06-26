/*
    Multiplexes occam and downcam camera frames into a single topic for darknet.
    Dynamically configurable via service TODO service name
    Update rate configurable via config/darknet_multiplexer.yaml
 */

 namespace darknet_multiplexer_ns
 {
     void Multiplexer::onInit()
     {
         ;
     }

     void Multiplexer::cameraCallback()
     {
         ;
     }

     void Multiplexer::publishFrame()
 }

 PLUGINLIB_EXPORT_CLASS(localizer_ns::Localizer, nodelet::Nodelet);