import rospy
from std_msgs.msg import Float64, Bool
from topic_tools.srv import MuxSelect
from cusub_print.cuprint import CUPrint

STANDARD_ROOT_TOPIC = "cusub_common/motor_controllers/pid/"

class PIDClient:

    def __init__(self, objective_name, axis, root_topic=STANDARD_ROOT_TOPIC):
        """
        Params
        ------
        objective_name : str
        axis : str
            - "drive"
            - "stfafe"
            - "depth"
            - "yaw"
        root_topic : str
        """
        self.name = objective_name + "/" + axis.upper() + " Client"
        self.cuprint = CUPrint(self.name)
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
        standard_setpoint_topic = STANDARD_ROOT_TOPIC + axis + "/setpoint"
        standard_state_topic = STANDARD_ROOT_TOPIC + axis + "/state"
        standard_enable_topic = STANDARD_ROOT_TOPIC + axis + "/pid_enable"

        self.enable_pub = rospy.Publisher(enable_topic, Bool, queue_size=10)
        self.disable_pub = rospy.Publisher(disable_topic, Bool, queue_size=10)
        self.setpoint_pub = rospy.Publisher(setpoint_topic, Float64, queue_size=10)
        self.state_pub = rospy.Publisher(state_topic, Float64, queue_size=10)
        self.standard_setpoint_pub = rospy.Publisher(standard_setpoint_topic, Float64, queue_size=10)
        self.standard_enable_pub =rospy.Publisher(standard_enable_topic, Bool, queue_size=10)

        self.standard_setpoint_msg = None
        rospy.Subscriber(standard_state_topic, Float64, self.standard_state_callback)

    def standard_state_callback(self, msg):
        self.standard_setpoint_msg = msg

    def get_standard_state(self):
        return self.standard_setpoint_msg.data

    # Returns true for successful enabling
    def enable(self):
        if not self.standard and not self.enabled:
            b = Bool()
            b.data = False
            self.repeated_publish(self.standard_enable_pub, b)

            # Make rosservice call to switch mux
            new_topic = "/" + self.sub_name + "/cusub_common/motor_controllers/cv/" + self.axis + "/control_effort"
            self.cuprint("enabling " + self.axis + " PID loop")
            srv_name = "cusub_common/motor_controllers/" + self.axis + "_mux/select"
            rospy.wait_for_service(srv_name)
            self.cuprint("...enabled")
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
        if not self.standard and self.enabled: # Standard PID loops only get disabled when enabling a nonstandard
            # Freeze in place
            setpoint = self.standard_setpoint_msg
            self.repeated_publish(self.standard_setpoint_pub, setpoint)
            b = Bool()
            b.data = True
            self.repeated_publish(self.standard_enable_pub, b)

            # Make rosservice call to switch mux
            new_topic = "/" + self.sub_name + "/cusub_common/motor_controllers/pid/" + self.axis + "/control_effort"
            self.cuprint("disabling " + self.axis + " PID loop")
            srv_name = "cusub_common/motor_controllers/" + self.axis + "_mux/select"
            rospy.wait_for_service(srv_name)
            self.cuprint("...disabled")
            mux_select = rospy.ServiceProxy(srv_name, MuxSelect)
            try:
                resp = mux_select(new_topic)
            except rospy.ServiceException as exc:
                rospy.logerr("Could not switch mux to enable PID: " + str(exc))
                return False            

            b = Bool()
            b.data = False
            self.repeated_publish(self.enable_pub, b)

            self.enabled = False
        return True
            
    def set_setpoint(self, data, loop=True):
        if not self.enabled and not self.standard:
            self.cuprint("Setting setpoint but PID loop is not enabled!", warn=True)
        f = Float64()
        f.data = data
        if loop:
            self.repeated_publish(self.setpoint_pub, f)
        else:
            self.setpoint_pub.publish(f)

    def increment_setpoint(self, data, loop=True):
        if not self.enabled and not self.standard:
            self.cuprint("Trying to increment setpoint but PID loop is not enabled!", warn=True)
        f = Float64()
        f.data = data + self.get_standard_state()
        if loop:
            self.repeated_publish(self.setpoint_pub, f)
        else:
            self.setpoint_pub.publish(f)

    def set_state(self, data, loop=False):
        if self.standard:
            self.cuprint("Setting state of a 'standard' PID loop, will be overided by robot_localization", warn=True)
        if not self.enabled and not self.standard:
            self.cuprint("Setting state but PID loop is not enabled!", warn=True)
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
            rospy.sleep(0.01)
