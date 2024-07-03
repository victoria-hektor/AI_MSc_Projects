import numpy as np
import rospy
import math
from geometry_msgs.msg import Pose, PoseArray, Quaternion, Point, PoseWithCovarianceStamped
from numpy.random.mtrand import vonmises

from pf_base import PFLocaliserBase  # Ensure correct import paths
import tf 

from util import rotateQuaternion, getHeading  # Ensure correct import paths
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
        self.NUMBER_PARTICLES = 100
        self.POSITION_STANDARD_DEVIATION = 0.1
        self.ORIENTATION_STANDARD_DEVIATION = 0.15
        # Odometric noise parameters.
        self.ODOMETRIC_DRIFT_NOISE = 0
        self.ODOMETRIC_ROTATION_NOISE = 0
        self.ODOMETRIC_TRANSLATION_NOISE = 0
        # The count of predicted sensor readings.
        self.NUMBER_PREDICTED_READINGS = 20

    def set_map(self, occupancy_map):
        if self.particlecloud is None:
            self.particlecloud = PoseArray()
            self.particlecloud.header = Header() 
        self.particlecloud.header.frame_id = "/map"
        self.occupancy_map = occupancy_map  # Store the occupancy map data

    def initialise_particle_cloud(self, initial_pose):
        # Initialises the particle cloud around the given pose with added noise
        num_particles = self.NUMBER_PARTICLES  # Use the class attribute
        particle_cloud = PoseArray()
        particle_cloud.header.stamp = rospy.Time.now()  # Set the current time as the timestamp
        particle_cloud.header.frame_id = "/map"  # The frame ID where the particle cloud is relevant
    
        # Generate each particle
        for _ in range(num_particles):
            particle = self._gauss_pose_generator(initial_pose)
            particle_cloud.poses.append(particle)
        
        self.particlecloud = particle_cloud  # Update the particle cloud

    def set_initial_pose(self, initial_pose):
        self.initialise_particle_cloud(initial_pose)
        # Now you can publish the initialised particle cloud
        self.publish_particle_cloud()

    def publish_particle_cloud(self):
        if hasattr(self, '_cloud_publisher'):
            self._cloud_publisher.publish(self.particlecloud)
        else:
            rospy.logwarn("Particle cloud publisher is not initialised yet.")

    def _gauss_pose_generator(self, initial_pose):
        """Generates a single particle pose based on the initial pose with Gaussian noise."""
        p = Pose()
        p.position.x = gauss(initial_pose.pose.pose.position.x, self.POSITION_STANDARD_DEVIATION)
        p.position.y = gauss(initial_pose.pose.pose.position.y, self.POSITION_STANDARD_DEVIATION)
        p.position.z = 0.0
        rotation_angle = vonmises(0.0, 1 / self.ORIENTATION_STANDARD_DEVIATION ** 2)
        p.orientation = rotateQuaternion(initial_pose.pose.pose.orientation, rotation_angle)
        return p

    def update_particle_cloud(self, scan):
        """Updates the particle cloud based on the received laser scan data."""
        self._add_noise_to_particles()
        likelihoods = np.array([self.sensor_model.get_weight(scan, p) for p in self.particlecloud.poses])
        weights = likelihoods / likelihoods.sum()
        self.particlecloud.poses = self._resample_particles(weights)
        # Optionally, add more noise to particles after resampling for diversity.
        self._add_noise_to_particles()

    def _add_noise_to_particles(self):
        """Adds Gaussian noise to each particle for simulating prediction uncertainty."""
        for p in self.particlecloud.poses:
            p.position.x += gauss(0, self.POSITION_STANDARD_DEVIATION)
            p.position.y += gauss(0, self.POSITION_STANDARD_DEVIATION)
            rotation_noise = vonmises(0.0, 1 / self.ORIENTATION_STANDARD_DEVIATION ** 2)
            p.orientation = rotateQuaternion(p.orientation, rotation_noise)

    def _resample_particles(self, weights):
        """Resamples the particles based on their weights to focus on high-likelihood areas."""
        cumulative_weights = np.cumsum(weights)
        resampled_poses = []
        for _ in range(self.NUMBER_PARTICLES):
            index = np.searchsorted(cumulative_weights, random())
            resampled_poses.append(self.particlecloud.poses[index])
        return resampled_poses

    def estimate_pose(self):
        """Estimates the robot's pose by averaging the positions and orientations of the particles."""
        x_mean = np.mean([p.position.x for p in self.particlecloud.poses])
        y_mean = np.mean([p.position.y for p in self.particlecloud.poses])
        orientation_vectors = [self._orientation_to_vector(p.orientation) for p in self.particlecloud.poses]
        mean_orientation = self._vector_to_orientation(np.mean(orientation_vectors, axis=0))
        estimated_pose = Pose(position=Point(x=x_mean, y=y_mean, z=0), orientation=mean_orientation)
        
        # Periodically log the estimated pose to the console.
        current_time = time()
        if current_time - self.last_print_time >= 1:
            rospy.loginfo('Estimated Position: x = {estimated_pose.position.x:.2f}, y = {estimated_pose.position.y:.2f}')
            self.last_print_time = current_time
        return estimated_pose

    def _orientation_to_vector(self, orientation):
        """Converts a quaternion orientation to a vector representation."""
        return np.array([orientation.x, orientation.y, orientation.z, orientation.w])

    def _vector_to_orientation(self, vec):
        """Converts a vector back to a quaternion orientation."""
        return Quaternion(x=vec[0], y=vec[1], z=vec[2], w=vec[3])