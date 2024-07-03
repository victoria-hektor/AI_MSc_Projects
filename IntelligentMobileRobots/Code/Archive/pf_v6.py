import numpy as np
import rospy
import math
from geometry_msgs.msg import Pose, PoseArray, Quaternion, PoseStamped
from nav_msgs.msg import OccupancyGrid, MapMetaData, Odometry
from util import rotateQuaternion, getHeading
from pf_base import PFLocaliserBase
from random import random
from time import time

class PFLocaliser(PFLocaliserBase):
       
    def __init__(self):
        # Call the superclass constructor
        super(PFLocaliser, self).__init__()
        
        # Publishers
        self._cloud_publisher = rospy.Publisher("/particlecloud", PoseArray, queue_size=10)
        self._pose_publisher = rospy.Publisher("/estimatedpose", PoseStamped, queue_size=10)

        # Set motion model parameters
        self.ODOM_ROTATION_NOISE = 0.01  # Adjust as needed
        self.ODOM_TRANSLATION_NOISE = 0.02  # Adjust as needed
        self.ODOM_DRIFT_NOISE = 0.02  # Adjust as needed
        
        # Sensor model parameters
        self.NUMBER_PREDICTED_READINGS = 20 	# Number of readings to predict  
        
        # Initialise the last print time
        self.last_print_time = rospy.get_time()  

    def roulette_wheel_selection(self, particles, weights):
        total_weight = sum(weights)
        cumulative_weights = [sum(weights[:i+1]) for i in range(len(weights))]
        resampled_particles = []
        for _ in range(len(particles)):
            rand = random() * total_weight
            for i, cumulative_weight in enumerate(cumulative_weights):
                if rand < cumulative_weight:
                    resampled_particles.append(particles[i])
                    break
        return resampled_particles
        
        rospy.loginfo('Roulette wheel selection executed')#debug
              
    def initialise_particle_cloud(self, initialpose):
        rospy.loginfo('Initialising particle cloud') #debug
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
            
            #self.particlecloud = self.initialise_particle_cloud(initialpose)
            #self._cloud_publisher.publish(self.particlecloud)
            
        # Return the PoseArray
        return particle_cloud
        
        # Publication of the particle cloud
        self.particlecloud = particle_cloud
        self._cloud_publisher.publish(self.particlecloud)
        
        #debug
        rospy.loginfo('Particle cloud initialised with {} particles'.format(len(particle_cloud.poses)))
         
    def update_particle_cloud(self, scan):
        rospy.loginfo('Updating particle cloud')#debug
        # Initialise choices as particles
        resampled_particles = []

        # Calculate weights for each particle
        weights = [self.sensor_model.get_weight(scan, particle) for particle in self.particlecloud.poses]

        # Resample particles based on weights
        resampled_particles = self.roulette_wheel_selection(self.particlecloud.poses, weights)

        # Add Gaussian noise for diversity in position and orientation
        for particle in resampled_particles:
            particle.position.x += np.random.normal(0, 0.1)  # Position noise
            particle.position.y += np.random.normal(0, 0.1)  # Position noise

            # Orientation noise
            yaw_noise = np.random.normal(0, np.radians(5))  # 5 degrees of noise
            particle.orientation = rotateQuaternion(particle.orientation, yaw_noise)

        # Update the particle cloud
        self.particlecloud.poses = resampled_particles
        self._cloud_publisher.publish(self.particlecloud)  # Publish the updated particle cloud
        
        estimated_pose = self.estimate_pose()  # Estimate the new pose
        rospy.loginfo('Particle cloud updated with {} particles'.format(len(self.particlecloud.poses)))
        
    def estimate_pose(self):
        rospy.loginfo('Estimating pose')
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
        
        # Create PoseStamped message
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.header.frame_id = "map"
        pose_stamped.pose = estimated_pose
        
        # Publish the estimated pose
        self._pose_publisher.publish(pose_stamped)
        
        # Print the estimated position every second
        current_time = rospy.get_time()
        
        if current_time - self.last_print_time >= 60:  # Check if a minute has passed
            rospy.loginfo('Estimated Position: x = {:.2f}, y = {:.2f}, time = {:.2f}'.format(avg_x, avg_y, current_time))
            self.last_print_time = current_time  # Update the last print time
            print('estimated_pose')
            
        return estimated_pose
        
        rospy.loginfo('Estimated pose published: x = {:.2f}, y = {:.2f}'.format(avg_x, avg_y))
        
    if __name__ == '__main__':
    rospy.init_node("pf_localisation")
    rospy.loginfo('Particle filter localisation node started')
    try:
        node = ParticleFilterLocalisationNode()
    except rospy.ROSInterruptException as e:
        rospy.logerr('Particle filter localisation node interrupted: {}'.format(e))
    rospy.spin()
        