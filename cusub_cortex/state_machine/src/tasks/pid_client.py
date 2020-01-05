import rospy
from std_msgs.msg import Float64, Bool

class PIDClient:

    def __init__(self, axis, root_topic="cusub_common/motor_controllers/pid/"):
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

        enable_topic = root_topic + axis + "/pid_enable"
        disable_topic = root_topic + axis + "/pid_enable"
        setpoint_topic = root_topic + axis + "/setpoint"
        state_topic = root_topic + axis + "/state"

        self.enable_pub = rospy.Publisher(enable_topic, Bool, queue_size=10)
        self.disable_pub = rospy.Publisher(disable_topic, Bool, queue_size=10)
        self.setpoint_pub = rospy.Publisher(setpoint_topic, Float64, queue_size=10)
        self.state_pub = rospy.Publisher(state_topic, Float64, queue_size=10)

    def enable(self):
        b = Bool()
        b.data = True
        self.repeated_publish(self.enable_pub, b)

    def disable(self):
        b = Bool()
        b.data = False
        self.repeated_publish(self.enable_pub, b)
            
    def set_setpoint(self, data):
        f = Float64()
        f.data = data
        self.repeated_publish(self.setpoint_pub, f)

    def set_state(self, data):
        f = Float64()
        f.data = data
        self.repeated_publish(self.state_pub, f)

    # publish multiple times to gaurantee receival
    def repeated_publish(self, pub, msg):
        count = 10
        while not rospy.is_shutdown() and count > 0:
            pub.publish(msg)
            count -= 1
            rospy.sleep(0.1)
