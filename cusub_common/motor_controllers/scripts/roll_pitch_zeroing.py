#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64

class ZeroRollPitch():
    def __init__(self):
        self.roll_pub = rospy.Publisher('cusub_common/motor_controllers/pid/roll/setpoint', Float64, queue_size=1)
        self.pitch_pub = rospy.Publisher('cusub_common/motor_controllers/pid/pitch/setpoint', Float64, queue_size=1)
        self.depth_pub = rospy.Publisher('cusub_common/motor_controllers/pid/depth/setpoint', Float64, queue_size=10)
        self.publish_depth = True
        rospy.Timer(rospy.Duration(3), self.depth_timer_callback)

    def depth_timer_callback(self, event):
        self.publish_depth = False

    def zero(self):
        zero_msg = Float64()
        zero_msg.data = 0
        r = rospy.Rate(2)
        while not rospy.is_shutdown():
            self.roll_pub.publish(zero_msg)
            self.pitch_pub.publish(zero_msg)
            if self.publish_depth:
                self.depth_pub.publish(zero_msg)
            r.sleep()

def main():
    rospy.init_node('zero_roll_pitch')
    zrp = ZeroRollPitch()    
    zrp.zero()


if __name__ == '__main__':
    main()