from time import time
from geometry_msgs.msg import Pose, PoseArray, Quaternion
from pf_base import PFLocaliserBase
import math
import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid, MapMetaData, Odometry

from util import rotateQuaternion, getHeading
from random import random

from time import time

class PFLocaliser(PFLocaliserBase):
       
    def __init__(self):
        # Call the superclass constructor
        super(PFLocaliser, self).__init__()

        # Subscribe to odometry data
        #rospy.Subscriber('odom', Odometry, self.estimate_pose, queue_size=1)
        
        # Set motion model parameters
        self.ODOM_ROTATION_NOISE = 0.0
        1  # Adjust as needed
        self.ODOM_TRANSLATION_NOISE = 0.02  # Adjust as needed
        self.ODOM_DRIFT_NOISE = 0.02  # Adjust as needed
        # Sensor model parameters
        self.NUMBER_PREDICTED_READINGS = 20 	# Number of readings to predict        
              
    def initialise_particle_cloud(self, initialpose):
        print('in1')
        # Number of particles
        num_particles = 100  # You can adjust this number for testing

        # Extract the initial pose
        initial_x = initialpose.pose.pose.position.x
        initial_y = initialpose.pose.pose.position.y
        initial_orientation = initialpose.pose.pose.orientation

        # Initialise PoseArray
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
        # Initialise choices as particles
        choices = resampled_particles

        # Calculate weights for each particle
        weights = [self.sensor_model.get_weight(scan, particle) for particle in self.particlecloud.poses]

        # Resample particles based on weights
        resampled_particles = choices(self.particlecloud.poses, weights, k=len(self.particlecloud.poses))

        # Add Gaussian noise for diversity in position and orientation
        for particle in resampled_particles:
            particle.position.x += np.random.normal(0, 0.1)  # Position noise
            particle.position.y += np.random.normal(0, 0.1)  # Position noise

            # Orientation noise
            yaw_noise = np.random.normal(0, np.radians(5))  # 5 degrees of noise
            particle.orientation = rotateQuaternion(particle.orientation, yaw_noise)

        # Update the particle cloud
        self.particlecloud.poses = resampled_particles
        
    def estimate_pose(self):
        print('in3')
        # Initialise sums for positions and quaternion components
        sum_x, sum_y = 0, 0
        sum_qx, sum_qy, sum_qz, sum_qw = 0, 0, 0, 0

        num_particles = len(self.particlecloud.poses)

        # Sum up all the positions and quaternion components
        for particle in self.particlecloud.poses:
            sum_x += particle.position.x
            sum_y += particle.position.y
            sum_qx += particle.orientation.x
            sum_qy += particle.orientation.y
            sum_qz += particle.orientation.z
            sum_qw += particle.orientation.w

        # Compute averages
        avg_x = sum_x / num_particles
        avg_y = sum_y / num_particles
        avg_qx = sum_qx / num_particles
        avg_qy = sum_qy / num_particles
        avg_qz = sum_qz / num_particles
        avg_qw = sum_qw / num_particles

        # Create a new pose for the estimated position and orientation
        estimated_pose = Pose()
        estimated_pose.position.x = avg_x
        estimated_pose.position.y = avg_y
        estimated_pose.orientation.x = avg_qx
        estimated_pose.orientation.y = avg_qy
        estimated_pose.orientation.z = avg_qz
        estimated_pose.orientation.w = avg_qw
        
        # Print the estimated position every second
        current_time = time.time()
        if current_time - self.last_print_time >= 1:
            #print(f'Estimated Position: ', 'x={avg_x}', 'y={avg_y}', 'time={current_time}')
            print('Estimated Position: x = ' + str(avg_x ) + 'y = ' + str(avg_y) + 'time = ' + str(current_time))  
            self.last_print_time = current_time

        return estimated_pose

    def odom_function_RENAME(self, odom):
        # Set motion model parameters
        #self.ODOM_ROTATION_NOISE = 0.01  # Adjust as needed
        #self.ODOM_TRANSLATION_NOISE = 0.02  # Adjust as needed
        #self.ODOM_DRIFT_NOISE = 0.02  # Adjust as needed

        # Incorporate noise into the particle update step
        for particle in self.particlecloud.poses:
            # Add noise to each component of the particle's pose
            particle.position.x += np.random.normal(0, self.ODOM_TRANSLATION_NOISE)
            particle.position.y += np.random.normal(0, self.ODOM_DRIFT_NOISE)
            yaw_noise = np.random.normal(0, self.ODOM_ROTATION_NOISE)
            particle.orientation = rotateQuaternion(particle.orientation, yaw_noise)

        return particle.position.y, particle.position.x, particle.orientation
        #return super(particle.position.x, particle.position.y).predict_from_odometry(odom)