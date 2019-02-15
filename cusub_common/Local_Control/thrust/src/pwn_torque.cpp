#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"

#include <sstream>
#include <iostream>

class TorqueConverter
{
public:
    TorqueConverter(ros::NodeHandle *n) {
        pub = n->advertise<std_msgs::Float64MultiArray>("/local_control/thrust/torque", 1);
        sub = n->subscribe<std_msgs::Float64MultiArray>("/drivers/pololu_control/command", 1, &TorqueConverter::pwn_callback, this);
    }
private:
    ros::Publisher pub;
    ros::Subscriber sub;
    double piecewise[2][81];
    void pwn_callback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
        //quad motors to center of pitch
        double i1 = 0.377952;
        //drive motors to center of yaw
        double i2 = 0.28702;
        //drive motors to center of pitch
        double i3 = 0.035306;
        //quad motors to center of roll
        double i4 = 0.162052;
        //yaw motors to center of yaw
        double i5 = 0.45466;
        std_msgs::Float64MultiArray out;
        out.data[0]  = pwn_thrust(msg->data[2])+pwn_thrust(msg->data[3]);

        out.data[1]  = pwn_thrust(msg->data[0])+pwn_thrust(msg->data[1]);

        out.data[2]  = pwn_thrust(msg->data[4])
                    +pwn_thrust(msg->data[5])
                    +pwn_thrust(msg->data[6])
                    +pwn_thrust(msg->data[7]);


        out.data[3]  = -1*i4*pwn_thrust(msg->data[4])
                    +i4*pwn_thrust(msg->data[5])
                    -i4*pwn_thrust(msg->data[6])
                    +i4*pwn_thrust(msg->data[7]);

        out.data[4]  = i1*pwn_thrust(msg->data[4])
                    +i1*pwn_thrust(msg->data[5])
                    -i1*pwn_thrust(msg->data[6])
                    -i1*pwn_thrust(msg->data[7])
                    +i3*pwn_thrust(msg->data[2])
                    +i3*pwn_thrust(msg->data[3]);

        out.data[5]  = i1*pwn_thrust(msg->data[0])
                    -i1*pwn_thrust(msg->data[1])
                    -i2*pwn_thrust(msg->data[2])
                    +i2*pwn_thrust(msg->data[3]);

        pub.publish(out);
    }
    double pwn_thrust(double pwn) {
        double g = 9.80665;
        double x1,x2,y1,y2 = 0;
        for(int i = 0; i<80;i++){
            if(piecewise[0][i] <= pwn && piecewise[0][i+1] >= pwn){
                x1 = piecewise[0][i];
                x2 = piecewise[0][i+1];
                y1 = piecewise[1][i];
                y2 = piecewise[1][i+1];
            }
        }
        double out = (y1*(x2-pwn)+y2*(pwn-x1))/(x2-x1);
        return out;
    }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "TorqueConverter");
  ros::NodeHandle n;

  TorqueConverter m(&n);
  ros::spin();
  return 0;
}
