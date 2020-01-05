import rospy
from std_msgs.msg import Float64, Bool
from topic_tools.srv import MuxSelect

STANDARD_ROOT_TOPIC = "cusub_common/motor_controllers/pid/"

class PIDClient:

    def __init__(self, axis, root_topic=STANDARD_ROOT_TOPIC):
        """
        Params
        ------
        axis : str
            - "drive"
            - "stfafe"
            - "depth"
            - "yaw"
        root_topic : str
        """
        if axis not in ["drive", "strafe", "depth", "yaw"]:
            rospy.logerr("PID axis unrecognized: " + axis)
        if root_topic[-1] != "/":
            root_topic = root_topic + "/"
        self.root_topic = root_topic
        self.axis = axis
        self.enabled = False
        self.sub_name = rospy.get_param("~robotname")

        if root_topic == STANDARD_ROOT_TOPIC:
            self.standard = True
        else:
            self.standard = False

        enable_topic = root_topic + axis + "/pid_enable"
        disable_topic = root_topic + axis + "/pid_enable"
        setpoint_topic = root_topic + axis + "/setpoint"
        state_topic = root_topic + axis + "/state"

        self.enable_pub = rospy.Publisher(enable_topic, Bool, queue_size=10)
        self.disable_pub = rospy.Publisher(disable_topic, Bool, queue_size=10)
        self.setpoint_pub = rospy.Publisher(setpoint_topic, Float64, queue_size=10)
        self.state_pub = rospy.Publisher(state_topic, Float64, queue_size=10)

    # Returns true for successful enabling
    def enable(self):
        if not self.standard and not self.enabled:
            # Make rosservice call to switch mux
            # prev_topic = "cusub_common/motor_controllers/pid/" + self.axis + "/control_effort"
            new_topic = "/" + self.sub_name + "/cusub_common/motor_controllers/cv/" + self.axis + "/control_effort"
            rospy.loginfo("waiting for " + self.axis + " mux service")
            srv_name = "cusub_common/motor_controllers/" + self.axis + "_mux/select"
            rospy.wait_for_service(srv_name)
            rospy.loginfo("...found service")
            mux_select = rospy.ServiceProxy(srv_name, MuxSelect)
            try:
                resp = mux_select(new_topic)
            except rospy.ServiceException as exc:
                rospy.logerr("Could not switch mux to enable PID: " + str(exc))
                return False
        
        self.enabled = True
        b = Bool()
        b.data = True
        self.repeated_publish(self.enable_pub, b)
        return True

    def disable(self):
        self.enabled = False
        b = Bool()
        b.data = False
        self.repeated_publish(self.enable_pub, b)

        # TODO if not standard root topic -> then set the setpoint equal to the current state
            
    def set_setpoint(self, data, loop=True):
        if not self.enabled:
            rospy.logwarn("Setting setpoint but PID loop is not enabled!")
        f = Float64()
        f.data = data
        if loop:
            self.repeated_publish(self.setpoint_pub, f)
        else:
            self.setpoint_pub.publish(f)

    def set_state(self, data, loop=False):
        if self.standard:
            rospy.logwarn("Setting state of a 'standard' PID loop, will be overided by robot_localization")
        if not self.enabled:
            rospy.logwarn("Setting state but PID loop is not enabled!")
        f = Float64()
        f.data = data
        if loop:
            self.repeated_publish(self.state_pub, f)
        else:
            self.state_pub.publish(f)

    # publish multiple times to ensure receival
    def repeated_publish(self, pub, msg):
        count = 10
        while not rospy.is_shutdown() and count > 0:
            pub.publish(msg)
            count -= 1
            rospy.sleep(0.1)
