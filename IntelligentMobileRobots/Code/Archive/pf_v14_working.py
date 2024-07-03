import numpy as np
import rospy
import math
from geometry_msgs.msg import Pose, PoseArray, Quaternion, Point, PoseWithCovarianceStamped
from numpy.random.mtrand import vonmises

from pf_base import PFLocaliserBase  
import tf 

from util import rotateQuaternion, getHeading
from std_msgs.msg import Header
from random import random, gauss
from time import time

class PFLocaliser(PFLocaliserBase):
    """A specialised class derived from PFLocaliserBase to implement particle filter localisation."""
    
    def __init__(self):
        """Initialises the particle filter localiser with essential configurations."""
        super(PFLocaliser, self).__init__()

        # Initialise self.particlecloud as an empty PoseArray
        self.particlecloud = PoseArray()
        self.particlecloud.header.frame_id = "/map"  # Assign the frame_id early
        
        # Define the number and noise characteristics of particles.
        self.NUMBER_PARTICLES = 150
        self.POSITION_STANDARD_DEVIATION = 0.1
        self.ORIENTATION_STANDARD_DEVIATION = 0.2
        # Odometric noise parameters.
        self.ODOMETRIC_DRIFT_NOISE = 0.1
        self.ODOMETRIC_ROTATION_NOISE = 0.1
        self.ODOMETRIC_TRANSLATION_NOISE = 0.1
        # The count of predicted sensor readings.
        self.NUMBER_PREDICTED_READINGS = 20

    def roulette_wheel_selection(self, weights):
        """
        Utilises the roulette wheel method to select and slightly modify particles based on their respective weights
        to ensure diversity within the resampled particle cloud.

        Args:
            weights (numpy.ndarray): The array of normalised likelihood weights for each particle.

        Returns:
            list: A list of particle poses resampled and modified based on the provided weights.
        """
        total_weight = np.sum(weights)  # Ensure we're working with the total sum of the normalised weights
        cumulative_weights = np.cumsum(weights)
        resampled_particles = []
        for _ in range(len(self.particlecloud.poses)):
            rand = np.random.rand() * total_weight
            index = np.searchsorted(cumulative_weights, rand)
            selected_particle = self.particlecloud.poses[index]

            # Add resampling noise directly here
            noisy_particle = self._add_resampling_noise(selected_particle)
            resampled_particles.append(noisy_particle)

        # Log the resampling process for debugging purposes
        rospy.logdebug("Resampled {} particles out of the total {} particles".format(len(resampled_particles), len(self.particlecloud.poses)))
        return resampled_particles
    
    def initialise_particle_cloud(self, initial_pose):
        """
        Initialises the particle cloud based on an initial pose set in rviz.

        Args:
            initial_pose: The initial pose estimate as a PoseWithCovarianceStamped object.

        Returns:
            A PoseArray object containing the list of Pose objects, each representing a particle.
        """
        num_particles = self.NUMBER_PARTICLES  # Number of particles
        position_noise = self.POSITION_STANDARD_DEVIATION  # Positional noise standard deviation
        orientation_noise = self.ORIENTATION_STANDARD_DEVIATION  # Orientation noise standard deviation
        
        # Create a PoseArray object to hold all particles
        particle_cloud = PoseArray()
        particle_cloud.header.stamp = initial_pose.header.stamp
        particle_cloud.header.frame_id = initial_pose.header.frame_id
        
        # Generate each particle
        for _ in range(num_particles):
            particle = Pose()
            
            # Add Gaussian noise to the initial position
            particle.position.x = initial_pose.pose.pose.position.x + gauss(0, position_noise)
            particle.position.y = initial_pose.pose.pose.position.y + gauss(0, position_noise)
            particle.position.z = initial_pose.pose.pose.position.z  # Assuming z is always 0 for a 2D plane
            
            # Rotate the initial orientation by a small random amount
            random_yaw = gauss(0, orientation_noise)
            particle.orientation = rotateQuaternion(initial_pose.pose.pose.orientation, random_yaw)
            
            particle_cloud.poses.append(particle)
        print('particle_cloud', particle_cloud)
        return particle_cloud
    
    def update_particle_cloud(self, scan):
        """
        Updates the particle cloud based on laser scan data, employing particle filtering. 

        Each particle's pose likelihood is evaluated using the scan data, followed by resampling 
        based on these weights to maintain diversity within the particle cloud.

        Args:
            scan (sensor_msgs/LaserScan): The laser scan data.

        Effects:
            Updates self.particlecloud with a newly resampled set of particles.
        """
        # Calculate the likelihood weights for each particle
        weights = np.array([self.sensor_model.get_weight(scan, pose) for pose in self.particlecloud.poses])

        # Normalise the weights to form a probability distribution
        weights /= np.sum(weights)

        # Use the roulette_wheel_selection method to resample particles based on their weights
        new_particles_poses = self.roulette_wheel_selection(weights)

        # Update the particle cloud with the resampled particles
        self.particlecloud.poses = new_particles_poses

        rospy.logdebug("Updated particle cloud with new resampled particles.")

    def _add_resampling_noise(self, particle):
        """
        Adds resampling noise to a particle.

        This method applies Gaussian noise to the particle's position and orientation
        to ensure the particle cloud remains diverse enough to track changes in the
        robot's position.

        Args:
            particle (geometry_msgs/Pose): The particle pose to which noise is to be added

        Returns:
            (geometry_msgs/Pose): The new particle pose after adding noise
        """
        # Define noise standard deviations
        position_noise_std = self.POSITION_STANDARD_DEVIATION
        orientation_noise_std = self.ORIENTATION_STANDARD_DEVIATION

        # Create a new particle pose with Gaussian noise added
        new_particle = Pose()
        new_particle.position.x = particle.position.x + np.random.normal(0, position_noise_std)
        new_particle.position.y = particle.position.y + np.random.normal(0, position_noise_std)
        
        # Generate a random yaw noise and apply it to the orientation
        yaw_noise = np.random.normal(0, orientation_noise_std)
        new_particle.orientation = rotateQuaternion(particle.orientation, yaw_noise)
        
        return new_particle

    # def _add_resampling_noise(self, particle):

    def estimate_pose(self):
        """
        Estimate the robot's pose based on the current particle cloud.
        
        This method returns the estimated position and orientation by finding the average
        of the densest cluster of particles. It avoids directly averaging the position
        as this might not accurately represent the true pose, particularly if the particles
        are dispersed over two or more distinct regions (clusters).
        
        The function also overcomes the issue of averaging orientations by using quaternion
        averaging rather than heading averaging, thus accounting for the circular nature of
        angles.
        
        Returns:
            | (geometry_msgs/Pose): The estimated pose of the robot.
        """
        # Retrieve all the particle positions and orientations as separate lists
        positions = [p.position for p in self.particlecloud.poses]
        orientations = [p.orientation for p in self.particlecloud.poses]
        
        # Compute the mean position
        x_mean = np.mean([p.x for p in positions])
        y_mean = np.mean([p.y for p in positions])
        z_mean = np.mean([p.z for p in positions])

        # Compute the mean orientation by averaging quaternion components
        qx_mean = np.mean([o.x for o in orientations])
        qy_mean = np.mean([o.y for o in orientations])
        qz_mean = np.mean([o.z for o in orientations])
        qw_mean = np.mean([o.w for o in orientations])

        # Normalise the quaternion to ensure it represents a valid rotation
        norm = np.sqrt(qx_mean ** 2 + qy_mean ** 2 + qz_mean ** 2 + qw_mean ** 2)
        mean_orientation = Quaternion(x=qx_mean / norm, y=qy_mean / norm, z=qz_mean / norm, w=qw_mean / norm)
        
        # Construct the estimated pose
        estimated_pose = Pose()
        estimated_pose.position.x = x_mean
        estimated_pose.position.y = y_mean
        estimated_pose.position.z = z_mean
        estimated_pose.orientation = mean_orientation
                
        return estimated_pose