from geometry_msgs.msg import Pose, PoseArray, Quaternion
from pf_base import PFLocaliserBase
import math
import rospy

from util import rotateQuaternion, getHeading
from random import random

from time import time


class PFLocaliser(PFLocaliserBase):
       
    def __init__(self):
        # Call the superclass constructor
        super(PFLocaliser, self).__init__()
        
        # Set motion model parameters
 
        # Sensor model parameters
        self.NUMBER_PREDICTED_READINGS = 20 	# Number of readings to predict
        
       
    #def initialise_particle_cloud(self, initialpose):
        #print('in1')
        # Set particle cloud to initialpose plus noise
        # You'll then need to implement the following three abstract methods:
        # 4.2. initialise_particle_cloud(self, initialpose)
        # • Called whenever a new initial pose is set in rviz.
        # • This should instantiate and return a PoseArray [3] object, which contains a list 
        # of Pose objects. Each of these Poses should be set to a random position and 
        # orientation around the initial pose, e.g. by adding a Gaussian random number 
        # multiplied by a noise parameter to the initial pose.
        # • Orientation in ROS is represented as Quaternions, which are 4-dimensional 
        # representations of a 3-D rotation describing pitch, roll, and yaw ("heading"). This 
        # is much more complex than you will need as the Pioneer only rotates around the 
        # yaw axis, so to make it easier for you, you have been provided with the following
        # methods for handling rotations in pf_localisation.util:
        # o rotateQuaternion(q_orig, yaw)
        # Takes an existing Quaternion q_orig, and an angle 
        # in radians (positive or negative), and returns the Quaternion rotated 
        # around the heading axis by that angle. So, for example, you can take 
        # the Quaternion from the Pose object, rotate it by Math.PI/20 radians, 
        # and insert the resulting Quaternion back into the Pose.
        # o getHeading(q)
        # Performs the reverse conversion and gives you the heading (in
        # radians) described by a given Quaternion q.
        
    def initialise_particle_cloud(self, initialpose):
        # Number of particles
        num_particles = 100  # You can adjust this number

        # Extract the initial pose
        initial_x = initialpose.pose.pose.position.x
        initial_y = initialpose.pose.pose.position.y
        initial_orientation = initialpose.pose.pose.orientation

        # Initialize PoseArray
        particle_cloud = PoseArray()

        for _ in range(num_particles):
            # Create a new Pose object
            particle = Pose()

            # Add Gaussian noise to the position
            noise_x = np.random.normal(0, 0.5)  # Adjust the 0.5 as needed for your noise level
            noise_y = np.random.normal(0, 0.5)

            particle.position.x = initial_x + noise_x
            particle.position.y = initial_y + noise_y

            # Add Gaussian noise to the orientation
            noise_theta = np.random.normal(0, np.radians(5))  # 5 degrees of noise, adjust as needed
            noisy_orientation = rotateQuaternion(initial_orientation, noise_theta)

            particle.orientation = noisy_orientation

            # Add the particle to the PoseArray
            particle_cloud.poses.append(particle)

        # Return the PoseArray
        return particle_cloud
 
    
    def update_particle_cloud(self, scan):
        print('int2')
        # Update particlecloud, given map and laser scan
        #4.3. update_particle_cloud(self, scan)
        # • Called whenever a new LaserScan message is received. This method does the 
        # actual particle filtering.
        # • The PFLocaliserBase will already have moved each of the particles around the 
        # map approximately according to the odometry readings coming from the robot. 
        # Lesson 9 Lab sheet- assignment 2
        # Inteligent mobile robotics
        # De Montfort University
        # But odometry measurements are unreliable and noisy, so the particle filter makes 
        # use of laser readings to confirm the estimated location.
        # • Your method should get the likelihood weighting of each Pose in self.particlecloud.
        # .poses using the self.sensor_model.get_weight(scan, pose) method. This 
        # weighting refers to the likelihood that a particle in a certain location and orientation 
        # matches the observations from the laser, and is, therefore, a good estimate of the 
        # robot's pose.
        # • You should then resample the particlecloud by creating particles with a new 
        # location and orientation depending on the probabilites of the particles in the old 
        # particle cloud, for example by doing roulette-wheel selection to choose highweight particles more often than low-weight particles. (you could use the method 
        # used for random selection in RANSAC). 
        # Each new particle should have resampling noise added to it, to keep the
        # particle cloud spread out enough to keep up with any changes in the robot's
        # position.
        # • The new particle cloud should be assigned to self.particlecloud to replace the 
        # existing one.

    def estimate_pose(self):
        print('in3')
        # Create new estimated pose, given particle cloud
        # E.g. just average the location and orientation values of each of
        # the particles and return this.
        
        # Better approximations could be made by doing some simple clustering,
        # e.g. taking the average location of half the particles after 
        # throwing away any which are outliers

        # 4.4. estimate_pose()
        # • This should return the estimated position and orientation of the robot based on 
        # the current particle cloud, i.e. self.particlecloud.
        # You could do this by finding the densest cluster of particles and taking the average 
        # location and orientation, or by using just the selected 'best' particle, or any other 
        # method you prefer -- but make sure you test your method and justify it! Taking the 
        # average position of the entire particle cloud is probably not a good solution... can 
        # you think why not?
        # • To find the average orientation of a set of particles, you will encounter a problem 
        # because angles increase from 0 to pi radians then continue from -pi back to 0. 
        # For example, if you have two particles, one facing at -179 degrees and one facing 
        # at 179 degrees (only 2 degrees apart in reality), the mean orientation will be -179 
        # + 179 / 2 = 0 degrees, not 180 degrees.
        # • This situation is exactly what quaternions were created for. Instead of calculating 
        # the heading of each particle and finding the mean, you can simply take the mean 
        # of each of the x, y, z and w values directly from the Quaternions 
        # (using getW() etc.) before creating a new Quaternion with these mean values. 
        # This new quaternion represents the average heading of all the particles in your 
        # set.

    def predict_from_odometry(self, odom):
        # Set motion model parameters
        self.ODOM_ROTATION_NOISE = 0 #???? # Odometry model rotation noise
        self.ODOM_TRANSLATION_NOISE = 0 #???? # Odometry model x axis (forward) noise
        self.ODOM_DRIFT_NOISE = 0 #???? # Odometry model y axis (side-to-side) noise
        return super().predict_from_odometry(odom)