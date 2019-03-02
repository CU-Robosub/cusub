#!/usr/bin/env python
"""
Remaps DVL with correct orientation for simulating sub
"""

import rospy

from geometry_msgs.msg import TwistWithCovarianceStamped

class DVLRemap(object):
    """Remaps and transforms the DVL data from gazebo to be like the
    real sub
    """

    dvl_pub = None

    def twist_callback(self, data):
        """Gets simulated gazebo DVL data and transforms it to match
        what the dvl in the sub outputs
        """
        print "test"
        data.twist.twist.linear.z = -1*data.twist.twist.linear.z
        self.dvl_pub.publish(data)

    def run(self):
        """Starts the DVL remap node by setting up pubs/subs
        """

        rospy.init_node('dvl_gazebo', anonymous=True)
        rospy.Subscriber("description/dvl_twist", TwistWithCovarianceStamped, self.twist_callback)
        self.dvl_pub = rospy.Publisher('cusub_common/dvl', TwistWithCovarianceStamped, queue_size=1)

        rospy.spin()

if __name__ == '__main__':

    DVL_REMAP = DVLRemap()

    try:
        DVL_REMAP.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Gazebo DVL driver died!")
