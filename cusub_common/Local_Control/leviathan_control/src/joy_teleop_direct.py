#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import Joy
from uuv_gazebo_ros_plugins_msgs.msg import FloatStamped

ROLL_AXES  = 0
PITCH_AXES = 1
TWIST_AXES = 2
THROT_AXES = 3

roll_val  = 0.0
pitch_val = 0.0
twist_val = 0.0
throt_val = 0.0

TWIST_EFFORT    = 0.5
TRANLATE_EFFORT = 1.0

THRUSTER_POWER = 100.0

def joystick_state(data):

    global roll_val
    global pitch_val
    global twist_val
    global throt_val

    roll_val  = data.axes[ROLL_AXES]
    pitch_val = data.axes[PITCH_AXES]
    twist_val = data.axes[TWIST_AXES]
    throt_val = data.axes[THROT_AXES]

def joy_teleop_direct():

    global roll_val
    global pitch_val
    global twist_val
    global throt_val

    pub_t0_eff = rospy.Publisher('/leviathan/thrusters/0/input', FloatStamped, queue_size=10)
    pub_t1_eff = rospy.Publisher('/leviathan/thrusters/1/input', FloatStamped, queue_size=10)
    pub_t2_eff = rospy.Publisher('/leviathan/thrusters/2/input', FloatStamped, queue_size=10)
    pub_t3_eff = rospy.Publisher('/leviathan/thrusters/3/input', FloatStamped, queue_size=10)
    pub_t4_eff = rospy.Publisher('/leviathan/thrusters/4/input', FloatStamped, queue_size=10)
    pub_t5_eff = rospy.Publisher('/leviathan/thrusters/5/input', FloatStamped, queue_size=10)
    pub_t6_eff = rospy.Publisher('/leviathan/thrusters/6/input', FloatStamped, queue_size=10)
    pub_t7_eff = rospy.Publisher('/leviathan/thrusters/7/input', FloatStamped, queue_size=10)

    rospy.init_node('joy_teleop_direct', anonymous=True)
    rospy.Subscriber("/joy", Joy, joystick_state)
    rate = rospy.Rate(30)

    while not rospy.is_shutdown():

        thrust0 = FloatStamped()
        thrust0.header.stamp = rospy.Time.now()
        thrust0.data = pitch_val*THRUSTER_POWER
        pub_t0_eff.publish(thrust0)

        thrust1 = FloatStamped()
        thrust1.header.stamp = rospy.Time.now()
        thrust1.data = pitch_val*THRUSTER_POWER
        pub_t1_eff.publish(thrust1)

        thrust2 = FloatStamped()
        thrust2.header.stamp = rospy.Time.now()
        thrust2.data = throt_val*THRUSTER_POWER
        pub_t2_eff.publish(thrust2)

        thrust3 = FloatStamped()
        thrust3.header.stamp = rospy.Time.now()
        thrust3.data = throt_val*THRUSTER_POWER
        pub_t3_eff.publish(thrust3)

        thrust4 = FloatStamped()
        thrust4.header.stamp = rospy.Time.now()
        thrust4.data = throt_val*THRUSTER_POWER
        pub_t4_eff.publish(thrust4)

        thrust5 = FloatStamped()
        thrust5.header.stamp = rospy.Time.now()
        thrust5.data = throt_val*THRUSTER_POWER
        pub_t5_eff.publish(thrust5)

        thrust6 = FloatStamped()
        thrust6.header.stamp = rospy.Time.now()
        thrust6.data = -1*roll_val*THRUSTER_POWER-twist_val*THRUSTER_POWER*TWIST_EFFORT
        pub_t6_eff.publish(thrust6)

        thrust7 = FloatStamped()
        thrust7.header.stamp = rospy.Time.now()
        thrust7.data = -1*roll_val*THRUSTER_POWER+twist_val*THRUSTER_POWER*TWIST_EFFORT
        pub_t7_eff.publish(thrust7)

        rate.sleep()

if __name__ == '__main__':
    try:
        joy_teleop_direct_direct()
    except rospy.ROSInterruptException:
        rospy.loginfo("Mother fucker!")
