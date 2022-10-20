from pickle import TRUE
from re import T
import turtle
from geometry_msgs.msg import Pose, PoseArray, Quaternion
from . pf_base import PFLocaliserBase
import math
import rospy

from . util import rotateQuaternion, getHeading
from random import random, gauss,uniform

from time import time

import matplotlib.pyplot as plt


class PFLocaliser(PFLocaliserBase):
       
    def __init__(self):
        # ----- Call the superclass constructor
        super(PFLocaliser, self).__init__()
        
        # -----  Odometry Parameters
        self.ODOM_ROTATION_NOISE = 0.000001 		# Odometry model rotation noise
        self.ODOM_TRANSLATION_NOISE = 0.000002 	# Odometry x axis (forward) noise
        self.ODOM_DRIFT_NOISE = 0.00000003 			# Odometry y axis (side-side) noise

        # ----- Set motion model parameters
        self.NUMBER_OF_PARTICALS=200           # NUmber of particals in the starting this parameter is dynamic

        #------- Set percentage of random particals
        self.RANDOM_PARTICAL_PERCENTAGE=5           # value between 0 to 100

        #--------Set initial noise for partical cloud generation
        self.INITIAL_NOISE_X=0.3        # maximum noise in X direction
        self.INITIAL_NOISE_Y=0.3        # maximum noise in Y direction
        self.INITIAL_NOISE_THETA=180    # angle in degree
        
        # ----- Sensor model
        self.NUMBER_PREDICTED_READINGS = 20     # Number of readings to predict
        
        # ----- Map occupancy threshold
        self.OCCUPANCY_THRESHOLD=70              # value between 0 to 100

        # ------ RESAMPLING NOISE
        self.RESAMPLING_NOISE_X=0.1           
        self.RESAMPLING_NOISE_Y=0.1            
        self.RESAMPLING_NOISE_THETA=10

        # --- parameters for advance dynamic partical 
        self.MAX_NUMBER_OF_PARTICALS=300
        self.MIN_NUMBER_OF_PARTICALS=50
        self.MAX_RANDOM_PARTICAL_PERCENTAGE=20
        self.MIN_RANDOM_PARTICAL_PERCENTAGE=5

        #-- weights array for estimate pose function
        self.weights_array=[]

        #-- parameters for dynamic number of partical change
        self.threshold_for_dynamic_partical_number_change=4.5
        

    def map_position_checker(self,x,y):
        #-- converting map position to cell location
        cell_x=int(math.floor(x/self.occupancy_map.info.resolution))
        cell_y=int(math.floor(y/self.occupancy_map.info.resolution))
        width=self.occupancy_map.info.width
        height=self.occupancy_map.info.height

        # check if the particle is within the bounds of the map
        if cell_x>=width or cell_x<0 or cell_y>=height or cell_y <0: #
            return False
        if cell_x+cell_y*width>len(self.occupancy_map.data):
            return False
        # check if any obstracle is present at the location 
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
        
        #-- getting details about initial pose
        initial_x=initialpose.pose.pose.position.x
        initial_y=initialpose.pose.pose.position.y
        initial_theta=initialpose.pose.pose.orientation

        # -- creating pose array object to return
        pose_array=PoseArray()

        # -- adding guassian noise to initial pose and generating partical cloud
        for i in range(self.NUMBER_OF_PARTICALS):
            new_partical=Pose()
            while(True):
                new_partical.position.x=initial_x+gauss(0,1)*self.INITIAL_NOISE_X
                new_partical.position.y=initial_y+gauss(0,1)*self.INITIAL_NOISE_Y
                if self.map_position_checker(new_partical.position.x,new_partical.position.y):
                    break
            new_partical.orientation=rotateQuaternion(initial_theta, gauss(0,1)*self.INITIAL_NOISE_THETA*math.pi/180)
            pose_array.poses.append(new_partical)
        
        #--- return pose array object
        print("initial cloud created")
        return pose_array

 
    
    def update_particle_cloud(self, scan):
        """
        This should use the supplied laser scan to update the current
        particle cloud. i.e. self.particlecloud should be updated.
        
        :Args:
            | scan (sensor_msgs.msg.LaserScan): laser scan to use for update

         """
        #---- Getting weights for all the particals
        weights=[]
        for partical in self.particlecloud.poses:
            weights.append(self.sensor_model.get_weight(scan,partical))
        
        self.weights_array=weights

        #--  Sum of all the weights
        sum_of_weight=sum(weights)

        #---- Dynamic changes in the number of particals and random partical percentage
        if((sum_of_weight/self.NUMBER_OF_PARTICALS) > self.threshold_for_dynamic_partical_number_change):
            if self.NUMBER_OF_PARTICALS > self.MIN_NUMBER_OF_PARTICALS:
                self.NUMBER_OF_PARTICALS-=1
            if self.RANDOM_PARTICAL_PERCENTAGE > self.MIN_RANDOM_PARTICAL_PERCENTAGE:
                self.RANDOM_PARTICAL_PERCENTAGE-=1
        else:
            if self.NUMBER_OF_PARTICALS < self.MAX_NUMBER_OF_PARTICALS:
                self.NUMBER_OF_PARTICALS+=1
            if self.RANDOM_PARTICAL_PERCENTAGE < self.MAX_RANDOM_PARTICAL_PERCENTAGE:
                self.RANDOM_PARTICAL_PERCENTAGE+=1



        print("particals=%d "%self.NUMBER_OF_PARTICALS)
        print("random percentage=%d"%self.RANDOM_PARTICAL_PERCENTAGE)
        
        #---- Normalising weights
        weights=[i/sum_of_weight for i in weights]

        #-- cumulitive density function list
        cdf=[]
        cdf.append(weights[0])
        for i in range(1,len(weights)):
            cdf.append(cdf[i-1]+weights[i])

        #-- Resampling algorithm
        M=self.NUMBER_OF_PARTICALS-int(self.NUMBER_OF_PARTICALS*self.RANDOM_PARTICAL_PERCENTAGE/100)
        inverse_of_M=1/M
        resampled=[]
        U=0
        while U==0:
            U=uniform(0,inverse_of_M)
        K=0
        for j in range(M):
            while(U>cdf[K]):
                K+=1
            resampled.append(self.particlecloud.poses[K])
            U=U+inverse_of_M
        
        #--- adding gaussian noise to each partical
        update_array=[]
        for i in range(len(resampled)):
            noise_added_partical=Pose()
            while(True):
                noise_added_partical.position.x=(resampled[i].position.x+gauss(0,1)*self.RESAMPLING_NOISE_X)
                noise_added_partical.position.y=(resampled[i].position.y+gauss(0,1)*self.RESAMPLING_NOISE_Y)
                if self.map_position_checker(noise_added_partical.position.x,noise_added_partical.position.y):
                    break
            noise_added_partical.orientation=rotateQuaternion(resampled[i].orientation, gauss(0,1)*self.RESAMPLING_NOISE_THETA*math.pi/180)
            update_array.append(noise_added_partical)      


        #-- adding the random particals all across the map
        for i in range(int(self.NUMBER_OF_PARTICALS*self.RANDOM_PARTICAL_PERCENTAGE/100)):
            random_partical=Pose()
            while True:
                random_partical.position.x=uniform(0,self.occupancy_map.info.width)
                random_partical.position.y=uniform(0,self.occupancy_map.info.height)
                if self.map_position_checker(random_partical.position.x,random_partical.position.y):
                    break
            random_partical.orientation.w=1
            random_partical.orientation=rotateQuaternion(random_partical.orientation,uniform(0,1)*self.INITIAL_NOISE_THETA*math.pi/180)
            update_array.append(random_partical)
        
        #--- update partical cloud
        self.particlecloud.poses=update_array
        return True



        

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


        # Simple average function for position
        def averagePose(poses):

            estimated_pose = Pose()        

            for p in poses:
                estimated_pose.position.x += p.position.x
                estimated_pose.position.y += p.position.y
                estimated_pose.orientation.x += p.orientation.x
                estimated_pose.orientation.y += p.orientation.y
                estimated_pose.orientation.z += p.orientation.z
                estimated_pose.orientation.w += p.orientation.w
                
            length_of_partical_cloud=len(poses)

            estimated_pose.position.x /= length_of_partical_cloud
            estimated_pose.position.y /= length_of_partical_cloud
            estimated_pose.orientation.x /= length_of_partical_cloud
            estimated_pose.orientation.y /=length_of_partical_cloud
            estimated_pose.orientation.z /= length_of_partical_cloud
            estimated_pose.orientation.w /= length_of_partical_cloud
            return estimated_pose
            #end
        
       # taking estimated pose from all the particals
        estimated_pose = averagePose(self.particlecloud.poses)
        
        #---measuring distance of each partical from the average partical
        distances = []        
        for i in range(len(self.particlecloud.poses)):
            p = self.particlecloud.poses[i]
            if i < len(self.weights_array):
                distances.append((p,self.weights_array[i],(p.position.x-estimated_pose.position.x)**2+(p.position.y-estimated_pose.position.y)**2))
            else:
                distances.append((p,0,(p.position.x-estimated_pose.position.x)**2+(p.position.y-estimated_pose.position.y)**2))
        distances.sort(key=lambda tup: tup[2])

        #--- taking poses which are close to main pose
        better_poses =[]
        for i in range(int(len(distances)/2)):
            better_poses.append(distances[i])
        better_poses.sort(key=lambda tup: tup[1])

        #-- estimating the main pose
        estimated_pose2 = Pose()    
        sum=0    

        for p in better_poses:
            estimated_pose2.position.x += p[0].position.x*p[1]
            estimated_pose2.position.y += p[0].position.y*p[1]
            estimated_pose2.orientation.x += p[0].orientation.x*p[1]
            estimated_pose2.orientation.y += p[0].orientation.y*p[1]
            estimated_pose2.orientation.z +=p[0].orientation.z*p[1]
            estimated_pose2.orientation.w += p[0].orientation.w*p[1]
            sum += p[1]
            
        estimated_pose2.position.x /= sum
        estimated_pose2.position.y /= sum
        estimated_pose2.orientation.x /= sum
        estimated_pose2.orientation.y /=sum
        estimated_pose2.orientation.z /= sum
        estimated_pose2.orientation.w /= sum
        
        
        return estimated_pose2
       