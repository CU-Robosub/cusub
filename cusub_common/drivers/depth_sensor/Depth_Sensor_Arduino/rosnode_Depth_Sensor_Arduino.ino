
#include <ros.h>
#include <std_msgs/Float64.h>

ros::NodeHandle nh;

std_msgs::Float64 float64_msg;
ros::Publisher reading("/drivers/depth_sensor/reading", &float64_msg); 
double nadc;
double volts;
double m;

void setup() {
  // setup the node and advertise it
  nh.initNode();
  nh.advertise(reading);
  
}

void loop() {
  for(int i=0,nadc=0; i<256; i++) nadc += analogRead(0);
  nadc /= 256;
  volts = nadc*5/1024;
  m = (volts-1.76)*1.660;
  float64_msg.data = m; //depth
  reading.publish( &float64_msg);
  nh.spinOnce();
  
}



