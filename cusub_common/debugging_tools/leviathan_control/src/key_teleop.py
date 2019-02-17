#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import Joy
from uuv_gazebo_ros_plugins_msgs.msg import FloatStamped

def key_teleop():

    pub_t0_eff = rospy.Publisher('/leviathan/thrusters/0/input', FloatStamped, queue_size=10)
    pub_t1_eff = rospy.Publisher('/leviathan/thrusters/1/input', FloatStamped, queue_size=10)
    pub_t2_eff = rospy.Publisher('/leviathan/thrusters/2/input', FloatStamped, queue_size=10)
    pub_t3_eff = rospy.Publisher('/leviathan/thrusters/3/input', FloatStamped, queue_size=10)
    pub_t4_eff = rospy.Publisher('/leviathan/thrusters/4/input', FloatStamped, queue_size=10)
    pub_t5_eff = rospy.Publisher('/leviathan/thrusters/5/input', FloatStamped, queue_size=10)
    pub_t6_eff = rospy.Publisher('/leviathan/thrusters/6/input', FloatStamped, queue_size=10)
    pub_t7_eff = rospy.Publisher('/leviathan/thrusters/7/input', FloatStamped, queue_size=10)

    rospy.init_node('key_teleop', anonymous=True)
    #rospy.Subscriber("/joy", Joy, joystick_state)
    rate = rospy.Rate(30)

    while not rospy.is_shutdown():

        thrust0 = FloatStamped()
        thrust0.header.stamp = rospy.Time.now()
        thrust0.data = 10
        pub_t0_eff.publish(thrust0)

        thrust1 = FloatStamped()
        thrust1.header.stamp = rospy.Time.now()
        thrust1.data = 10
        pub_t1_eff.publish(thrust1)

        thrust2 = FloatStamped()
        thrust2.header.stamp = rospy.Time.now()
        thrust2.data = 10
        pub_t2_eff.publish(thrust2)

        thrust3 = FloatStamped()
        thrust3.header.stamp = rospy.Time.now()
        thrust3.data = 10
        pub_t3_eff.publish(thrust3)

        thrust4 = FloatStamped()
        thrust4.header.stamp = rospy.Time.now()
        thrust4.data = 10
        pub_t4_eff.publish(thrust4)

        thrust5 = FloatStamped()
        thrust5.header.stamp = rospy.Time.now()
        thrust5.data = 10
        pub_t5_eff.publish(thrust5)

        thrust6 = FloatStamped()
        thrust6.header.stamp = rospy.Time.now()
        thrust6.data = 10
        pub_t6_eff.publish(thrust6)

        thrust7 = FloatStamped()
        thrust7.header.stamp = rospy.Time.now()
        thrust7.data = 10
        pub_t7_eff.publish(thrust7)

        rate.sleep()

if __name__ == '__main__':
    try:
        key_teleop()
    except rospy.ROSInterruptException:
        rospy.loginfo("Mother fucker!")
