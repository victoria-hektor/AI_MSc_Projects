
from geometry_msgs.msg import Pose, PoseArray, Quaternion, Point, PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from util import rotateQuaternion, getHeading

import matplotlib # doesn't work in Melodic, poss in newer distro's, keeping for reference
matplotlib.use('Agg')  # Use a non-interactive backend
import matplotlib.pyplot as plt # doesn't work in Melodic, poss in newer distro's, keeping for reference

from pf_base import PFLocaliserBase
from random import random, gauss
from std_msgs.msg import Header
from time import time
import numpy as np
import rospy
import math

class PFLocaliser(PFLocaliserBase):
    """A specialised class derived from PFLocaliserBase to implement particle filter localisation."""
    
    def __init__(self):
        """

        Initialises the PFLocaliser class, 
        setting up essential configurations for particle filter localisation. 
        It subscribes to the "/base_pose_ground_truth" topic to receive the ground truth pose. 

        The method also defines the particle cloud's frame_id as "/map," establishing the spatial context. 
        Key parameters include the number of particles, their positional and orientational noise characteristics, 
        measurement noise, and odometric noise parameters for drift, rotation, and translation. 

        Additionally, it specifies the number of predicted sensor readings, 

        """
        super(PFLocaliser, self).__init__()

        # Subscribe to the ground truth pose topic
        rospy.Subscriber("/base_pose_ground_truth", PoseWithCovarianceStamped, self.ground_truth_callback)
        self.ground_truth_poses = []
        self.particlecloud.header.frame_id = "/map"  # Assign the frame_id early
        
        # Define the number and noise characteristics of particles.
        self.NUMBER_PARTICLES = 150
        self.POSITION_STANDARD_DEVIATION = 0.1
        self.ORIENTATION_STANDARD_DEVIATION = 0.2
        # additional visualisation params needed (standard deviation):
        self.measurement_noise = 0.5
        # Odometric noise parameters.
        self.ODOMETRIC_DRIFT_NOISE = 0.1
        self.ODOMETRIC_ROTATION_NOISE = 0.1
        self.ODOMETRIC_TRANSLATION_NOISE = 0.1
        # The count of predicted sensor readings.
        self.NUMBER_PREDICTED_READINGS = 20

    def ground_truth_callback(self, msg):
        # Callback function to store the ground truth pose
        self.ground_truth_poses.append(msg.pose.pose)

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
        
        # After updating the particle cloud, estimate the pose
        # estimated_pose = self.estimate_pose()
        # self.estimated_poses.append(estimated_pose)
        
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

    def numpy_array_to_quaternion(self, arr):
        """
        Converts a numpy array to a Quaternion object.

        Args:
            arr (numpy.ndarray): Numpy array with elements [x, y, z, w].

        Returns:
            Quaternion: Quaternion object with the same values as the numpy array.
        """
        return Quaternion(x=arr[0], y=arr[1], z=arr[2], w=arr[3])

    def estimate_pose(self):
        """
        Estimate the robot's pose based on the current particle cloud. This method now includes
        an option to use either a min/max approach for both position and orientation, or a mean average approach.
        
        The position is estimated by finding the mid-point of the minimum and maximum coordinates
        of all particles, which provides a bounding box approximation of the robot's position.
        
        For orientation, the function converts quaternions to Euler angles to apply a min/max or mean average
        approach more effectively, then converts back to quaternion for the final orientation estimation.
        """
        # Extract all particle positions and orientations
        positions = [p.position for p in self.particlecloud.poses]
        orientations = [p.orientation for p in self.particlecloud.poses]

        # Calculate min and max for positions
        x_min = min(p.x for p in positions)
        x_max = max(p.x for p in positions)
        y_min = min(p.y for p in positions)
        y_max = max(p.y for p in positions)
        z_min = min(p.z for p in positions)
        z_max = max(p.z for p in positions)

        # Calculate mid-point for position (for Min/Max approach)
        x_mid = (x_min + x_max) / 2
        y_mid = (y_min + y_max) / 2
        z_mid = (z_min + z_max) / 2

        # Convert quaternions to Euler angles
        eulers = [euler_from_quaternion([o.x, o.y, o.z, o.w]) for o in orientations]

        # For orientation, choose between Min/Max approach and Mean Average approach
        # Uncomment the desired block and comment out the other
        # Use ctrl+q to comment out multiple lines of code

        # Min/Max Approach for Orientation (comment out if using Mean Average)
        roll_min, roll_max = min(e[0] for e in eulers), max(e[0] for e in eulers)
        pitch_min, pitch_max = min(e[1] for e in eulers), max(e[1] for e in eulers)
        yaw_min, yaw_max = min(e[2] for e in eulers), max(e[2] for e in eulers)
        roll_mid = (roll_min + roll_max) / 2
        pitch_mid = (pitch_min + pitch_max) / 2
        yaw_mid = (yaw_min + yaw_max) / 2
        mean_orientation = quaternion_from_euler(roll_mid, pitch_mid, yaw_mid)

        # Mean Average Approach for Orientation (comment out if using Min/Max)
        # qx_mean = np.mean([o.x for o in orientations])
        # qy_mean = np.mean([o.y for o in orientations])
        # qz_mean = np.mean([o.z for o in orientations])
        # qw_mean = np.mean([o.w for o in orientations])
        # norm = np.sqrt(qx_mean ** 2 + qy_mean ** 2 + qz_mean ** 2 + qw_mean ** 2)
        # mean_orientation = Quaternion(x=qx_mean / norm, y=qy_mean / norm, z=qz_mean / norm, w=qw_mean / norm)

        # Construct the estimated pose
        estimated_pose = Pose()
        estimated_pose.position.x = x_mid  # Use mid-point for Min/Max approach
        estimated_pose.position.y = y_mid
        estimated_pose.position.z = z_mid
        estimated_pose.orientation = mean_orientation  # Result from either Min/Max or Mean Average

        # convert it back to a Quaternion object if it's a numpy array
        if isinstance(mean_orientation, np.ndarray):
            mean_orientation = self.numpy_array_to_quaternion(mean_orientation)

        estimated_pose.orientation = mean_orientation

        return estimated_pose
        
    def generate_probability_heatmap(self, sensor_model):
        obs_ranges = np.linspace(0, sensor_model.scan_range_max, 100)
        map_ranges = np.linspace(0, sensor_model.scan_range_max, 100)
        probabilities = np.array([[self.sensor_model.predict(obs, map_r) for map_r in map_ranges] for obs in obs_ranges])
        plt.imshow(probabilities, cmap='hot', interpolation='nearest')
        plt.savefig('/catkin_ws/src/pf_localisation0/plots/heat_map.png')

    
    def print_localisation_error(self):
        if not self.ground_truth_poses or not self.estimated_poses:
            rospy.loginfo("Ground truth or estimated poses are empty.")
            return

        # Assuming the latest ground truth and estimated pose are used for comparison
        ground_truth_pose = self.ground_truth_poses[-1]
        estimated_pose = self.estimated_poses[-1]

        # Convert quaternions to Euler angles
        _, _, yaw_ground_truth = euler_from_quaternion([ground_truth_pose.orientation.x,
                                                        ground_truth_pose.orientation.y,
                                                        ground_truth_pose.orientation.z,
                                                        ground_truth_pose.orientation.w])
        _, _, yaw_estimated = euler_from_quaternion([estimated_pose.orientation.x,
                                                    estimated_pose.orientation.y,
                                                    estimated_pose.orientation.z,
                                                    estimated_pose.orientation.w])

        # Calculate positional error
        position_error = math.sqrt((estimated_pose.position.x - ground_truth_pose.position.x) ** 2 +
                                (estimated_pose.position.y - ground_truth_pose.position.y) ** 2)

        # Calculate orientation error (in yaw)
        orientation_error = abs(yaw_estimated - yaw_ground_truth)

        rospy.loginfo("Localisation error - Position: {position_error} meters, Orientation: {orientation_error} radians")

    # def calculate_and_plot_location_error(self):        
    #     # Calculate location errors
    #     errors = [self.calculate_distance(estimated, ground_truth) for estimated, ground_truth in zip(self.estimated_poses, self.ground_truth_poses)]
        
    #     # Plot errors
    #     plt.figure()
    #     plt.plot(errors, label='Location Error')
    #     plt.xlabel('Time Step')
    #     plt.ylabel('Error (meters)')
    #     plt.title('Location Error Over Time')
    #     plt.legend()
    #     plt.show()

    #     # Save the plot to a file
    #     plt.savefig('/catkin_ws/src/pf_localisation0/plots/plot.png')

    #     # Close the plot if not displaying it
    #     #plt.close()

    def calculate_distance(self, pose1, pose2):
        # Simple Euclidean distance for 2D
        return math.sqrt((pose1.position.x - pose2.position.x)**2 + (pose1.position.y - pose2.position.y)**2)