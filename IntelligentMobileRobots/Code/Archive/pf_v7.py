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
        self._pose_publisher = rospy.Publisher("/estimated_pose", PoseStamped, queue_size=10)        
        
        # Set motion model parameters
        self.ODOM_ROTATION_NOISE = 0.01  # Adjust as needed
        self.ODOM_TRANSLATION_NOISE = 0.02  # Adjust as needed
        self.ODOM_DRIFT_NOISE = 0.02  # Adjust as needed
        
        # Sensor model parameters
        self.NUMBER_PREDICTED_READINGS = 20 	# Number of readings to predict  
        
        # Initialise the last print time
        self.last_print_time = rospy.get_time()  

    def roulette_wheel_selection(self, particles, weights):
        print('in0')
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

              
    def initialise_particle_cloud(self, initialpose):
        print('in1')

        # No. of particles
        num_particles = 100

        # Extract the initial pose
        initial_x = initialpose.pose.pose.position.x
        initial_y = initialpose.pose.pose.position.y
        initial_orientation = initialpose.pose.pose.orientation

        # Initialise PoseArray
        particle_cloud = PoseArray()

        for _ in range(num_particles):
            # Create a new Pose object
            particlecloud = Pose()

            # Add Gaussian noise to the position
            noise_x = np.random.normal(0, 0.5)  # Adjust the 0.5 as needed for your noise level
            noise_y = np.random.normal(0, 0.5)

            particlecloud.position.x = initial_x + noise_x
            particlecloud.position.y = initial_y + noise_y

            # Add Gaussian noise to the orientation
            noise_theta = np.random.normal(0, np.radians(5))  # 5 degrees of noise, adjust as needed
            noisy_orientation = rotateQuaternion(initial_orientation, noise_theta)

            particlecloud.orientation = noisy_orientation

            # Add the particle to the PoseArray
            particle_cloud.poses.append(particlecloud)
            
            #self.particlecloud = self.initialise_particle_cloud(initialpose)
            #self._cloud_publisher.publish(self.particlecloud)

            print('X Position = ', particlecloud.position.x)
            print('Y Position = ', particlecloud.position.y)
            print('orientation = ', particlecloud.orientation.z)
                    
        # Publication of the particle cloud
        self.particlecloud = particle_cloud
        self._cloud_publisher.publish(self.particlecloud)

        # Return the PoseArray
        return particle_cloud
         
    def update_particle_cloud(self, scan):
        print('int2')
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
        print('Estimated pose is: ', estimated_pose)
        
    def estimate_pose(self):
        # Estimate the pose by clustering the particles
        num_particles = len(self.particlecloud.poses)
        if num_particles == 0:
            return Pose()  # Return a default pose if no particles

        # Convert particle poses to numpy arrays for easier processing
        positions = np.array([[p.position.x, p.position.y] for p in self.particlecloud.poses])
        orientations = np.array([rotateQuaternion(p.orientation, 0) for p in self.particlecloud.poses])

        # Use k-means clustering to find clusters, here we choose 2 clusters for simplicity
        k = 2
        kmeans = KMeans(n_clusters=k)
        kmeans.fit(positions)
        centroids = kmeans.cluster_centers_
        
        # Find the largest cluster
        cluster_sizes = [np.sum(kmeans.labels_ == i) for i in range(k)]
        largest_cluster_index = np.argmax(cluster_sizes)
        largest_cluster_centroid = centroids[largest_cluster_index]

        # Select particles belonging to the largest cluster
        cluster_particles = [self.particlecloud.poses[i] for i in range(num_particles) if kmeans.labels_[i] == largest_cluster_index]

        # Average positions and orientations of the particles in the largest cluster
        sum_x, sum_y = 0, 0
        sum_qx, sum_qy, sum_qz, sum_qw = 0, 0, 0, 0

        for particle in cluster_particles:
            sum_x += particle.position.x
            sum_y += particle.position.y
            sum_qx += particle.orientation.x
            sum_qy += particle.orientation.y
            sum_qz += particle.orientation.z
            sum_qw += particle.orientation.w

        estimated_pose = Pose()
        estimated_pose.position.x = sum_x / len(cluster_particles)
        estimated_pose.position.y = sum_y / len(cluster_particles)
        estimated_pose.orientation.x = sum_qx / len(cluster_particles)
        estimated_pose.orientation.y = sum_qy / len(cluster_particles)
        estimated_pose.orientation.z = sum_qz / len(cluster_particles)
        estimated_pose.orientation.w = sum_qw / len(cluster_particles)

        # Publish the estimated pose
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.header.frame_id = "map"
        pose_stamped.pose = estimated_pose
        self._pose_publisher.publish(pose_stamped)

        return estimated_pose
