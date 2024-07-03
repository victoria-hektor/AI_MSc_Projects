import numpy as np
import rospy
import math
from geometry_msgs.msg import Pose, PoseArray, Quaternion, PoseStamped
from util import rotateQuaternion, getHeading
from pf_base import PFLocaliserBase
from random import random
from time import time

class PFLocaliser(PFLocaliserBase):
       
    def __init__(self):
        # Initialises the particle filter localiser with necessary ROS publishers and parameters.
        super(PFLocaliser, self).__init__()  # Calls the superclass constructor

        # Initialise self.particlecloud as an empty PoseArray
        self.particlecloud = PoseArray()
        self.particlecloud.header.frame_id = "/map"  # Assign the frame_id early
        
        self._cloud_publisher = rospy.Publisher("/particlecloud", PoseArray, queue_size=10)
        self._pose_publisher = rospy.Publisher("/estimated_pose", PoseStamped, queue_size=10)        
        
        # Motion model parameters for simulating robot movement.
        self.ODOM_ROTATION_NOISE = 0.01
        self.ODOM_TRANSLATION_NOISE = 0.02
        self.ODOM_DRIFT_NOISE = 0.02

        # Sensor model parameter for the number of predicted sensor readings.
        self.NUMBER_PREDICTED_READINGS = 20
        
        # Additional parameters for noise standard deviations.
        self.POSITION_STANDARD_DEVIATION = 0.1
        self.ORIENTATION_STANDARD_DEVIATION = 0.15
        
        # Tracks the last time a pose was printed for throttled logging.
        self.last_print_time = rospy.get_time()

    def set_map(self, occupancy_map):
        if self.particlecloud is None:
            self.particlecloud = PoseArray()
            self.particlecloud.header.frame_id = "/map"

    def roulette_wheel_selection(self, particles, weights):
        # Selects particles based on their weight using a roulette wheel method.
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

    def add_noise_to_particles(self, particle_cloud, position_noise_std, orientation_noise_std):
        # Applies Gaussian noise to each particle's position and orientation.
        for particle in particle_cloud.poses:
            particle.position.x += np.random.normal(0, position_noise_std)
            particle.position.y += np.random.normal(0, position_noise_std)
            yaw_noise = np.random.normal(0, orientation_noise_std)
            particle.orientation = rotateQuaternion(particle.orientation, yaw_noise)

    def initialise_particle_cloud(self, initialpose):
        # Initialises the particle cloud around the given pose with added noise.
        num_particles = 100
        particle_cloud = PoseArray()
        for _ in range(num_particles):
            particle = Pose()
            particle.position.x = initialpose.pose.pose.position.x + np.random.normal(0, self.POSITION_STANDARD_DEVIATION)
            particle.position.y = initialpose.pose.pose.position.y + np.random.normal(0, self.POSITION_STANDARD_DEVIATION)
            noise_theta = np.random.normal(0, self.ORIENTATION_STANDARD_DEVIATION)
            particle.orientation = rotateQuaternion(initialpose.pose.pose.orientation, noise_theta)
            particle_cloud.poses.append(particle)

        self.particlecloud = particle_cloud  # It seems there was a mistake in the original snippet here
        self.add_noise_to_particles(self.particlecloud, self.POSITION_STANDARD_DEVIATION, self.ORIENTATION_STANDARD_DEVIATION)
        self._cloud_publisher.publish(self.particlecloud)

    def random_position_generator(self):
        # Generates a particle with a random valid position and orientation.
        p = Pose()
        while not self.is_valid_position(p.position):
            p.position.x = random() * self.occupancy_map.info.width
            p.position.y = random() * self.occupancy_map.info.height
            p.position.z = 0.0
            p.orientation = rotateQuaternion(Quaternion(x=0.0, y=0.0, z=0.0, w=1.0), 2 * math.pi * random())
        return p

    def update_particle_cloud(self, scan):
        # Updates the particle cloud based on the latest sensor scan.
        self.add_noise_to_particles(self.particlecloud, self.POSITION_STANDARD_DEVIATION, self.ORIENTATION_STANDARD_DEVIATION)
        weights = [self.sensor_model.get_weight(scan, particle) for particle in self.particlecloud.poses]
        resampled_particles = self.roulette_wheel_selection(self.particlecloud.poses, weights)
        num_random_particles = int(0.1 * len(resampled_particles))
        for _ in range(num_random_particles):
            resampled_particles.append(self.random_position_generator())
        self.particlecloud.poses = resampled_particles
        self._cloud_publisher.publish(self.particlecloud)

    def estimate_pose(self):
        # Averages the positions and orientations of all particles to estimate the pose.
        num_particles = len(self.particlecloud.poses)
        sum_x, sum_y, sum_qx, sum_qy, sum_qz, sum_qw = 0, 0, 0, 0, 0, 0
        for particle in self.particlecloud.poses:
            sum_x += particle.position.x
            sum_y += particle.position.y
            sum_qx += particle.orientation.x
            sum_qy += particle.orientation.y
            sum_qz += particle.orientation.z
            sum_qw += particle.orientation.w
        estimated_pose = Pose()
        estimated_pose.position.x = sum_x / num_particles
        estimated_pose.position.y = sum_y / num_particles
        estimated_pose.orientation.x = sum_qx / num_particles
        estimated_pose.orientation.y = sum_qy / num_particles
        estimated_pose.orientation.z = sum_qz / num_particles
        estimated_pose.orientation.w = sum_qw / num_particles

        # Logging and publishing the estimated pose.
        current_time = time()
        if current_time - self.last_print_time >= 1:
            rospy.loginfo('Estimated Position: x = {:.2f}, y = {:.2f}, time = {:.2f}'.format(estimated_pose.position.x, estimated_pose.position.y, current_time))
            self.last_print_time = current_time
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.header.frame_id = "map"
        pose_stamped.pose = estimated_pose
        self._pose_publisher.publish(pose_stamped)

        return estimated_pose