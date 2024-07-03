    def initialise_particle_cloud(self, initialpose):
        """
        Initialize the particle cloud around the given initial pose.
        Args:
            initialpose (PoseWithCovarianceStamped): The initial pose received from rviz.
        Returns:
            PoseArray: A PoseArray containing randomly perturbed poses around the initial pose.
        """
        num_particles = 100  # You can adjust the number of particles as needed
        particle_cloud = PoseArray()

        for _ in range(num_particles):
            # Extract the pose from PoseWithCovarianceStamped
            pose = initialpose.pose.pose

            # Randomly perturb the initial pose with noise
            noisy_pose = self.add_noise_to_pose(pose)

            # Append the noisy pose to the particle cloud
            particle_cloud.poses.append(noisy_pose)

        return particle_cloud

    def add_noise_to_pose(self, pose):
        """
        Add noise to the given pose.
        Args:
            pose (Pose): The pose to add noise to.
        Returns:
            Pose: The pose with added noise.
        """
        # Extract yaw (heading) from the pose using getHeading method
        yaw = getHeading(pose.orientation)

        # Add noise to yaw (heading) component
        noisy_yaw = yaw + np.random.normal(0, self.ODOM_ROTATION_NOISE)

        # Create a new Quaternion with the noisy yaw
        noisy_orientation = rotateQuaternion(pose.orientation, noisy_yaw)

        # Add noise to position components
        noisy_x = pose.position.x + np.random.normal(0, self.ODOM_TRANSLATION_NOISE)
        noisy_y = pose.position.y + np.random.normal(0, self.ODOM_DRIFT_NOISE)

        # Create a new Pose with noisy position and orientation
        noisy_pose = Pose()
        noisy_pose.position.x = noisy_x
        noisy_pose.position.y = noisy_y
        noisy_pose.orientation = noisy_orientation

        return noisy_pose