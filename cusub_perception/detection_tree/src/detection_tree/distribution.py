"""
Distribution of Dvectors
Groups dvectors of the same object

Attributes
- dvector_list : list of dvectors
"""

class DvectorDistribution:

    def __init__(self, initial_dvector):
        self.class_id = initial_dvector.class_id
        self.dvectors  = [initial_dvector]

    def add_dvector(self, dvector):
        if dvector.class_id != self.class_id:
            raise("Dvector's class id does not match distribution's class_id: " + dvector.class_id+" and " + self.class_id)
        self.dvectors.append(dvector)

    # function to determine probability of dvector being part of distribution