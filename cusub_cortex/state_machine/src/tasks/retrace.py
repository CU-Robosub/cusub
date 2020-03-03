"""
Retrace Objective
"""
from tasks.task import Objective
from geometry_msgs.msg import Pose
import rospy
from cusub_print.cuprint import bcolors

class Retrace(Objective):
    outcomes = ['found','not_found']

    def __init__(self, task_name, listener, target_classes, allow_duplicate_dobjects=False):
        """
        Params
        ------
        task_name : str
        listener : DetectionListener
        target_classes : list of strs
        allow_duplicate_dobjects : bool
        """
        super(Retrace, self).__init__(self.outcomes, task_name + u"/Rétrace".encode("utf-8"))
        self.task_name = task_name
        self.listener = listener
        self.target_class_ids = target_classes
        self.num_classes = len(target_classes)
        self.allow_duplicate_dobjects = allow_duplicate_dobjects
        # Params
        self.retrace_hit_cnt = rospy.get_param("tasks/" + task_name.lower() + "/retrace_hit_count")
        self.retrace_dist_min = rospy.get_param("tasks/" + task_name.lower() + "/retrace_dist_min")
        # variables to keep track of
        self.retraced_steps = []
        self.cur_id = 0
        self.cur_num = 0
        self.cur_class = ''
        self.cur_dobj = 0
        self.dobj_nums = []
        self.len_dvecs = []

    #TODO: Analyze assumption made: take most recent dvector from each target_class?
    # Or, go through each one in every dvector list?
    def find_nearest_breadcrumb(self, d_vectors):
        min_dis = float("Inf")
        index = 0
        for i in range(len(d_vectors)):
            new_dis = self.get_distance(self.cur_pose.position, d_vectors[i].sub_pt)
            if new_dis < min_dis:
                min_dis = new_dis
                index = i
        return index

    def get_distant_breadcrumb(self, distance, d_vectors):
        '''
        Given a list of d_vectors
        Returns
            -- first d_vector that is <distance> away, and its index, in tuple form
            -- None if none of the d_vectors passed are <distance> away.
        '''    
        for index in range(len(d_vectors)):
            if self.get_distance(self.cur_pose.position, d_vectors[index].sub_pt) > distance:
                return (d_vectors[index], index)
        # if none are farthest away, return most distant dvector we have. Allows for clean exit
        return None

    def find_next_breadcrumb(self, super_d_vectors, distance):
        '''
        Function takes in list of super_d_vectors (list of dvector lists from each target class)
        and finds the next breacrumb to go to. This is determined based on the min distance the next
        vector should be from the current position.
        This function must also update the retraced_steps member variable that is passed in.

        Returns
            - sub point of next breadcrumb. Other important variables are updated through <self.>
        '''
        dvec = None
        ind = 0
        next_dvs = []
        for dob_ind in range(len(self.dobj_nums)):
            #class dvs gets the dobjects dvectors, but only up until our retraced steps variable
            class_dvs = super_d_vectors[dob_ind]
            if self.retraced_steps[dob_ind] >= len(class_dvs):
                self.cuprint("This shouldn't happen...", warn=True)
                next_dvs.append(class_dvs[0])
                continue
            class_dvs = class_dvs[self.retraced_steps[dob_ind]-1:]
            # pass in class_dvs reversed for ease of searching.
            next_bc = self.get_distant_breadcrumb(distance, class_dvs[::-1])
            if next_bc == None:
                self.cuprint("reached end of a dobject's dvectors: Dobj " + str(self.dobj_nums[dob_ind]) + ' , ' + self.listener[self.dobj_nums[dob_ind]].class_id + ' , ' + str(len(class_dvs)) + ' , ' + str(self.retraced_steps[dob_ind]))
                next_dvs.append(class_dvs[0])
                self.retraced_steps[dob_ind] = self.len_dvecs[dob_ind] + 1  # plus one because it needs to be greater than to delete
                continue
            else:
                dvec, ind = next_bc[0], next_bc[1]
                
            
            self.retraced_steps[dob_ind] += ind 
            next_dvs.append(dvec)

        nearest_breadcrumb_id = self.find_nearest_breadcrumb(next_dvs)
        # it is possible we might still want to visit the ones we found that were distant 
        # but were not the closest distant dvector
        # updated retraced_steps accordingly.
        self.cur_id = nearest_breadcrumb_id
        self.cur_dobj = self.dobj_nums[nearest_breadcrumb_id]
        self.cur_num = self.retraced_steps[nearest_breadcrumb_id]
        self.cur_class = self.listener[self.cur_dobj].class_id
        return next_dvs[nearest_breadcrumb_id].sub_pt


    def execute(self, userdata):
        self.toggle_waypoint_control(False)
        self.cuprint(u"executing Rétrace".encode("utf-8"))
        dobj_dict = self.listener.query_classes(self.target_class_ids)
        
        # this is just to print/report the nice totals of detections for each class, not dobj
        len_class_ids = {i: 0 for i in self.target_class_ids}
        for target in self.target_class_ids:
            nums = dobj_dict[target]
            if self.allow_duplicate_dobjects:
                # Take all dobjects we queried.
                for dob in nums:
                    self.dobj_nums.append(dob)
                    len_class_ids[target] += len(self.listener[dob].dvectors)
                    
            elif len(nums) > 1:
                # shouldn't have multiple dobjects, choose the one with most dvectors
                max_num = max([len(self.listener.dobjects[i].dvectors) for i in nums])
                self.dobj_nums.append(self.listener.dobjects.index(max_num))
                len_class_ids[target] += len(self.listener[max_num].dvectors)
            else:
                self.dobj_nums.append(nums[0])
                len_class_ids[target] += len(self.listener[nums[0]].dvectors)   

        #make sure we clear any preexisting flags.
        self.listener.clear_new_dv_flags(self.dobj_nums)

        #loop variables
        count = 0
        #keep track of each dobject's retraced steps
        self.retraced_steps = [1 for i in self.dobj_nums]
        self.len_dvecs = [len(self.listener[id].dvectors) for id in self.dobj_nums]
        self.cuprint("Len Dvecs: " + str(self.len_dvecs))
        for targ in self.target_class_ids:
            string = "Total dvectors found for " + bcolors.HEADER + bcolors.BOLD + targ  + bcolors.ENDC
            self.cuprint(string + ": " + bcolors.OKBLUE + str(len_class_ids[targ]) + bcolors.ENDC)

        #recent_dvectors will be the indeces of each target_class' dvectors that we need to go back to.
        #recent_dvectors = [self.find_distant_breadcrumb(self.retrace_dist_min, revrsd_dvecs[id]) for id in range(len(rev_dvecs))]  
        # recent_dvectors = [self.listener[dobj_nums[id]][len_dvecs[id]-self.retraced_steps[id]] for id in range(len(self.target_class_ids))]
        # nearest_breadcrumb_id = self.find_nearest_breadcrumb(recent_dvectors)
        # last_sub_pt = recent_dvectors[nearest_breadcrumb_id].sub_pt
        super_dvectors = [self.listener[dob].dvectors for dob in self.dobj_nums]
        # find_next_breadcrumb finds the next point to go to and updates current id (class str)
        # it also updates cur_num, which is the dvector index we are currently 
        # at (can't be larger than length of dvectors of corresponding dobject)
        last_sub_pt = self.find_next_breadcrumb(super_dvectors, self.retrace_dist_min)
        last_pose = Pose(last_sub_pt, self.cur_pose.orientation)
        #Start Retrace: Set first waypoint
        self.go_to_pose_non_blocking(last_pose, move_mode="strafe")
        # necessary for same-line printing
        print("")
        while not rospy.is_shutdown():
            if self.listener.check_new_dvs(self.dobj_nums) != None and count < self.retrace_hit_cnt: 
                #found object again
                count += 1
                if count >= self.retrace_hit_cnt:
                    self.cancel_way_client_goal()
                    return "found"
            else:  
                dist = round(self.get_distance(self.cur_pose.position, last_pose.position),3)
                self.cuprint("BC_ID " + bcolors.HEADER + self.cur_class +bcolors.ENDC + " DObj # " +bcolors.HEADER+str(self.cur_dobj) + bcolors.ENDC+ " Step # " + bcolors.HEADER + str(self.cur_num)+ bcolors.ENDC + " dist: " + bcolors.OKBLUE + str(dist) + bcolors.ENDC, print_prev_line=True)
                if self.check_reached_pose(last_pose):
                    self.cuprint("Cur Dobj:" + str(self.cur_dobj))
                    if (self.retraced_steps[self.cur_id] > self.len_dvecs[self.cur_id]):
                        self.dobj_nums.pop(self.cur_id)     # class list is local to this state so pop() is ok?
                        if len(self.dobj_nums) == 0:
                            #means we retraces all our steps. We're lost.
                            return "not_found"
                    # Goto Last dvector
                    # get last dvector sub_pt
                    # recent_dvectors = [self.listener[dobj_nums[id]][len_dvecs[id]-self.retraced_steps[id]] for id in range(len(self.target_class_ids))]
                    # nearest_breadcrumb_id = self.find_nearest_breadcrumb(recent_dvectors)
                    # last_sub_pt = recent_dvectors[nearest_breadcrumb_id].sub_pt
                    last_sub_pt = self.find_next_breadcrumb(super_dvectors, self.retrace_dist_min)
                    last_pose = Pose(last_sub_pt, self.cur_pose.orientation)
            
                    #set waypoint to this point. Give some distance to account for error in bearing and sub_pt
                    # Set waypoint
                    self.go_to_pose_non_blocking(last_pose, move_mode="strafe",log_print=False)

            if userdata.timeout_obj.timed_out:
                self.cancel_way_client_goal()
                userdata.outcome = "timed_out"
                return "not_found"
            rospy.sleep(.5)

        #clean up if we are killed
        return "not_found"