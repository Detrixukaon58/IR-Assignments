from re import T
import turtle
from geometry_msgs.msg import Pose, PoseArray, Quaternion
from . pf_base import PFLocaliserBase
import math
import rospy

from . util import rotateQuaternion, getHeading
from random import random, gauss

from time import time


class PFLocaliser(PFLocaliserBase):
       
    def __init__(self):
        # ----- Call the superclass constructor
        super(PFLocaliser, self).__init__()
        
        # -----  Odometry Parameters
        self.ODOM_ROTATION_NOISE = 0 		# Odometry model rotation noise
        self.ODOM_TRANSLATION_NOISE = 0 	# Odometry x axis (forward) noise
        self.ODOM_DRIFT_NOISE = 0 			# Odometry y axis (side-side) noise

        # ----- Set motion model parameters
        self.NUMBER_OF_PARTICALS=200

        self.INITIAL_NOISE_X=0.3
        self.INITIAL_NOISE_Y=0.3
        self.INITIAL_NOISE_THETA=180
        
        # ----- Sensor model parameters
        self.NUMBER_PREDICTED_READINGS = 20     # Number of readings to predict
        self.OCCUPANCY_THRESHOLD=70

    def map_position_checker(self,x,y):
        cell_x=int(x/self.occupancy_map.info.resolution)
        cell_y=y/self.occupancy_map.info.resolution
        width=self.occupancy_map.info.width
        height=self.occupancy_map.info.height

        if cell_x>width or cell_x<0 or cell_y>height or cell_y <0:
            return False
        if self.occupancy_map.data[cell_x+cell_y*width] > self.OCCUPANCY_THRESHOLD:
            return False
        return True


    def initialise_particle_cloud(self, initialpose):
        """
        Set particle cloud to initialpose plus noise

        Called whenever an initialpose message is received (to change the
        starting location of the robot), or a new occupancy_map is received.
        self.particlecloud can be initialised here. Initial pose of the robot
        is also set here.
        
        :Args:
            | initialpose: the initial pose estimate
        :Return:
            | (geometry_msgs.msg.PoseArray) poses of the particles
        """
        #adding my code here
        
        initial_x=initialpose.pose.pose.position.x
        initial_y=initialpose.pose.pose.position.y
        initial_theta=initialpose.pose.pose.orientation

        pose_array=PoseArray()
        for i in range(self.NUMBER_OF_PARTICALS):
            new_partical=Pose()
            while(True):
                new_partical.position.x=initial_x+gauss(0,1)*self.INITIAL_NOISE_X
                new_partical.position.y=initial_y+gauss(0,1)*self.INITIAL_NOISE_Y
                if self.map_position_checker(new_partical.position.x,new_partical.position.y):
                    break
        
            new_partical.orientation=rotateQuaternion(initial_theta, gauss(0,1)*self.INITIAL_NOISE_THETA*math.pi/180)
            pose_array.poses.append(new_partical)
        
        
        return pose_array



        #pass

 
    
    def update_particle_cloud(self, scan):
        """
        This should use the supplied laser scan to update the current
        particle cloud. i.e. self.particlecloud should be updated.
        
        :Args:
            | scan (sensor_msgs.msg.LaserScan): laser scan to use for update

         """
        pass

    def estimate_pose(self):
        """
        This should calculate and return an updated robot pose estimate based
        on the particle cloud (self.particlecloud).
        
        Create new estimated pose, given particle cloud
        E.g. just average the location and orientation values of each of
        the particles and return this.
        
        Better approximations could be made by doing some simple clustering,
        e.g. taking the average location of half the particles after 
        throwing away any which are outliers

        :Return:
            | (geometry_msgs.msg.Pose) robot's estimated pose.
         """
        pass
