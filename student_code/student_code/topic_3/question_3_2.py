from geometry_msgs.msg import Pose
import numpy as np

class TutorialTopic_3_2:
    def __init__(self):
        ## TODO: Q3.2.a Linear Coordinate Conventions
        self.red_buoy_location = Pose()

        self.yellow_buoy_location = Pose()

        ## TODO: Q3.2.b Angular Coordinate Convention
        self.red_buoy_angle = 0.0
        self.yellow_buoy_angle= 0.0

        ### STUDENT CODE HERE
        self.red_buoy_location.position.x = 2.0
        self.red_buoy_location.position.y = 3.0

        self.yellow_buoy_location.position.x = -3.0
        self.yellow_buoy_location.position.y = 1.0

        self.red_buoy_angle = np.arctan2(self.red_buoy_location.position.y, self.red_buoy_location.position.x)
        self.yellow_buoy_angle = np.arctan2(self.yellow_buoy_location.position.y, self.yellow_buoy_location.position.x)
        ### END STUDENT CODE
