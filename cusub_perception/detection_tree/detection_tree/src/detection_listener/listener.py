#!/usr/bin/env python

import rospy
from detection_tree_msgs.msg import Dvector
import threading
import numpy as np

class Dobject(list):

    def __getitem__(self, key):
        return self.dvectors[key]
    
    def __init__(self, num, class_id):
        self.num = num
        self.class_id = class_id
        self.dvectors = []
        self.pose = None
        self.lock = threading.Lock()

    def add_dvector(self, dv):
        self.lock.acquire(True)
        self.dvectors.append(dv)
        self.lock.release()
    
    def update_pose(self, pose):
        self.pose = pose

    def get_num_dvectors(self):
        return len(self.dvectors)

    def get_pose(self):
        return self.pose

    def get_dvectors_since_time(self, past_time):
        self.lock.acquire(True)

        # Check if we have any available dvs
        dv_last_time = self.dvectors[-1].camera_header.stamp
        if past_time > dv_last_time:
            self.lock.release()
            return None
        else: # we have dvs available
            for i in reversed(range(len(self.dvectors))):
                dv_last_time = self.dvectors[i].camera_header.stamp
                if past_time > dv_last_time:
                    self.lock.release()
                    return self.dvectors[i+1:] # won't be out of range since we checked the first
            # All dv's are in the timespan
            self.lock.release()
            return self.dvectors

    def get_most_recent_dvectors(self, num):
        if num > len(self.dvectors):
            num = len(self.dvectors)
        first_dv = len(self.dvectors) - num
        return self.dvectors[first_dv:]

    def get_d(self, index):
        if index < 0 or index > len(self.dvectors):
            return len(self.dvectors)
        return self.dvectors[index]

"""
Subscribes to dvector topic
"""
class DetectionListener(list):

    def __getitem__(self,key):
        return self.dobjects[key]

    def __init__(self):
        self.dobjects = []
        self.new_dv_flags = []
        rospy.Subscriber("cusub_perception/detection_tree/dvectors", Dvector, self.dvector_callback)

        # Pose Subscriber
        # rospy.Subscriber("cusub_perception/detection_tree/dobject_poses", Dvector, self.dvector_callback)

    # def pose_callback(self, msg):
    #     num = msg.dobject_num

    # Query multiple classes
    def query_classes(self, class_ids):
        """
        Returns dictionary of LISTs of dobjects for each class
        """
        dobj_dict = {}
        for c in class_ids:
            dobj_nums = self.query_class(c)
            if dobj_nums:
                dobj_dict[c] = dobj_nums
        return dobj_dict

    def query_class(self, class_id):
        """
        Returns dobject numbers of all dobjects matching class_ids
        """
        nums = []
        for i in range(len(self.dobjects)):
            if self.dobjects[i].class_id == class_id:
                nums.append(i)
        return nums
        
    def dvector_callback(self, msg): # if we got dobj.num = 1 and len == 1
        num = msg.dobject_num
        if num < len(self.dobjects): # We have this dobject
            self.dobjects[num].add_dvector(msg)
            self.new_dv_flags[num] = True
        elif num == len(self.dobjects): # New dobject
            self.create_new_dobj(num, msg.class_id, msg)
        else: # Out of order creation of dobject --> we missed a few dvectors
            rospy.logwarn("We missed a dvector...trigger synchrnoization")
            return

    def check_new_dv(self, dobj_num, clear_flag=True):
        if dobj_num >= len(self.new_dv_flags):
            return None
        else:
            if self.new_dv_flags[dobj_num]:
                if clear_flag:
                    self.clear_new_dv_flag(dobj_num)
                return True
            else:
                return False

    def check_new_dvs(self, dobj_nums, clear_flag=True):
        """
        Returns the dobject number that has received a new dvector.
        (There's been a new hit on this class/dobject)

        Params
        ------
        dobj_nums : list of ints
        
        Returns
        -------
        index of dobject 
        None if no dobject in the list has had a new detection
            or if input is invalid
        """
        for dob in dobj_nums:
            if dob >= len(self.new_dv_flags):
                return None
            elif self.new_dv_flags[dob]:
                if clear_flag:
                    self.clear_new_dv_flag(dob)
                return dob
        return None
                

    def clear_new_dv_flag(self, dobj_num):
        self.new_dv_flags[dobj_num] = False

    def clear_new_dv_flags(self, dobj_nums):
        for dob in dobj_nums:
            self.new_dv_flags[dob] = False

    def create_new_dobj(self, dobj_num, class_id, dv):
        self.dobjects.append(Dobject(dobj_num, class_id))
        self.dobjects[-1].add_dvector(dv)
        self.new_dv_flags.append(True)
        
    # TODO maybe also an option for bearings within a certain distance of current position?
    def get_avg_bearing(self, dobj_num, num_dv=None, secs=None):
        """
        Gets the average bearing over a number of dvectors or time

        Assumes bearings are from the same point (else we'd pose solve)

        Params
        ------
        dobj_num : int
            Which dobject we're talking about
        num_dv : int (optional)
            Number of dvectors to average over
        secs : float (optional)
            Number of seconds to average over
        
        Returns
        -------
        [ azimuth, elevation, height, width ] : float, float, int, int
            Average bearing
        """
        if num_dv != None and secs != None:
            rospy.logerr("Incorrect Input 1")
            return None
        elif num_dv == None and secs == None:
            rospy.logerr("Incorrect Input 2")
            return None
        elif dobj_num >= len(self.dobjects):
            rospy.logerr("Incorrect Input 3. Dobject unreceived")
            return None
        elif num_dv != None:
            dvs = self.dobjects[dobj_num].get_most_recent_dvectors(num_dv)
            return self._get_averages(dvs)
        else:
            t_past = rospy.get_rostime() - rospy.Duration.from_sec(secs)
            dvs = self.dobjects[dobj_num].get_dvectors_since_time(t_past)
            if dvs == None:
                return [None, None, None, None]
            else:
                return self._get_averages(dvs)

    def _get_averages(self, dvs):
        az = np.mean([i.azimuth for i in dvs])
        el = np.mean([i.elevation for i in dvs])
        height = np.mean([i.box_height for i in dvs])
        width = np.mean([i.box_width for i in dvs])
        return [az, el, height, width]

    def synchronize_dobjects(self): # SHOULDN'T BE NECESSARY IF WE INITIALIZE AT STARTUP
        pass
