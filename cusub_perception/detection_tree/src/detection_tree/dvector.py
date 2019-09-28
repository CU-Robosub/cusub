"""
Detection Vector / Dvector
consolidated way to represent a detection hit of a class

Attributes
- class_id
- azimuth & elevation of detection
- timestamp in header.stamp
- frame_id of original camera in header.frame_id
"""

class Dvector:

    def __init__(self, class_id, azimuth, elevation, header, sub_pose):
        """
        Params
        ------
        class_id : str
        azimuth : float
        elevation : float
        header : std_msgs/Header
            frame_id of original camera
            timestamp of detection + pose synchronized
        sub_pose : geometry_msgs/Pose

        """
        
        self.class_id = class_id
        self.azimuth = azimuth
        self.elevation = elevation
        self.header = header
        self.sub_pose = sub_pose