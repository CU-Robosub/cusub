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
         nh = getMTNodeHandle();
         int update_freq;
         if (!nh.getParam("darknet_multiplexer/update_freq", update_freq))
         {
             NODELET_ERROR("Darknet Multiplexer failed to locate params.");
             abort();
         }
         subs.push_back( nh.subscribe("cusub_common/occam/image0", 1, &Multiplexer::cameraCallback, this) );
         subs.push_back( nh.subscribe("cusub_common/occam/image1", 1, &Multiplexer::cameraCallback, this) );
         subs.push_back( nh.subscribe("cusub_common/occam/image2", 1, &Multiplexer::cameraCallback, this) );
         subs.push_back( nh.subscribe("cusub_common/occam/image3", 1, &Multiplexer::cameraCallback, this) );
         subs.push_back( nh.subscribe("cusub_common/occam/image4", 1, &Multiplexer::cameraCallback, this) );
         subs.push_back( nh.subscribe("cusub_common/downcam", 1, &Multiplexer::cameraCallback, this) );
         // setup startup configuration
         timer = nh.createTimer(ros::Duration(1 / update_freq), &Multiplexer::publishFrame, this);
     }

     void Multiplexer::cameraCallback(const sensor_msgs::ImagePtr image)
     {
         NODELET_INFO("Image received");
     }

     void Multiplexer::publishFrame(const ros::TimerEvent& event)
     {
        //  NODELET_INFO("Publishing a frame.");
        ;
     }
 }

 PLUGINLIB_EXPORT_CLASS(darknet_multiplexer_ns::Multiplexer, nodelet::Nodelet);