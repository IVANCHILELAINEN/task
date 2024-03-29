#!/usr/bin/env python
import numpy as np
from utils import decompose, distance, wrap_to_pi


class PathFollower(object):
    """Implements path following using feedback control.
    """

    def __init__(self, pos, vel, time, goal_threshold=0.2):
        """Initialize the PathFollower.

        Args:
            pos (ndarray): Positions size (n,2)
            vel (ndarray): Velocities size (n,2)
            time (ndarray): Time since start size (n,)
            goal_threshold (float): Threshold in meters for when the goal is
                deemed reached. Default 0.2.
        """
        super(PathFollower, self).__init__()
        self._goal_reached = False
        self._pos = pos
        self._vel = vel
        self._time = time
        self._goal_threshold = goal_threshold
        self._curvature = map(self._get_curvature, range(len(time)))
        # Start indexing from 1, because velocity at index 0 is 0
        self._idx = 1

    def _get_curvature(self, idx):
        """Get the curvature of the path at the given index.

        Args:
            idx (int): Index of point in path at which to get curvature

        Returns:
            float: Curvature at the given index
        """
        try:
            # TODO
            velo = self._vel[idx].reshape((-1, 1))
            v = np.sqrt(np.power(velo[0], 2) + np.power(velo[1], 2))  # calculate magnitude
            that = velo / v  # get direction
            nhat = np.array([[-velo[1]], [velo[0]]]) / v  # get the unit vector normal
            # get the direction for next point
            vel_next = self._vel[idx + 1].reshape((-1, 1))
            that_next = vel_next / np.sqrt(np.power(vel_next[0], 2) + np.power(vel_next[1], 2))

            td_hat = (that_next - that) / (self._time[idx + 1] - self._time[idx])  # calculate the chage

            curvature = np.dot(td_hat.reshape(-1), nhat) / v
            return curvature
        except IndexError:
            return 0.

    def _get_nearest(self, pose, window=2):
        """Get the index of the path point closest to the given pose. Updates
        the current running index and whether the goal has been reached.

        Args:
            pose (ndarray): Pose with size (3,) or a position with size (2,)
            window (int, optional): Window size for how many indeces are
                considered ahead the current running index. Default 2.

        Returns:
            int: The nearest index
        """
        min_idx = self._idx
        max_idx = np.minimum(self._pos.shape[0], self._idx + window)
        idx = range(min_idx, max_idx)
        nearest = idx[np.argmin(distance(pose[:2], self._pos[idx]))]
        self._idx = nearest

        # Check if goal has been reached
        if distance(pose[:2], self._pos[-1]) <= self._goal_threshold:
            self._goal_reached = True

        return nearest

    def _get_desired_pose(self, idx):
        """Get the desired pose at the given path index. The desired heading
        is calculated from the velocity.

        Args:
            idx (int): Index of point in path

        Returns:
            ndarray: The desired pose at the given index
        """
        # TODO
        desired_pose = self._pos[idx]
        heading = np.arctan2(self._vel[idx, 1], self._vel[idx, 0])
        return desired_pose, heading

    def _get_transform(self, idx):
        """Get a matrix to transform the error into the coordinate system of
        the desired pose.

        Args:
            idx (int): The index of the desired point in path

        Returns:
            ndarray: The transformation matrix, size (3,3)
        """
        heading = np.arctan2(self._vel[idx, 1], self._vel[idx, 0])
        sin_h = np.sin(heading)
        cos_h = np.cos(heading)
        return np.array([[cos_h, sin_h, 0],
                         [-sin_h, cos_h, 0],
                         [0, 0, 1]])

    @property
    def goal_reached(self):
        """Determine whether the goal has been reached.

        Returns:
            bool: Returns True if the goal has been reached
        """
        return self._goal_reached

    def get_control(self, pose):
        """Calculate the values for the feedback control.

        Args:
            pose (ndarray): The current pose of the mobile base

        Returns:
            tuple: Linear and angular velocity
        """
        # TODO
        nearest = self._get_nearest(self, pose)  # to get nearest pos at path
        desired_pos, heading = self._get_desired_pose(nearest)  # get desired pose and linear velocity
        trans_matrix = self._get_transform(nearest)
        curvature = self._get_curvature(nearest)

        qd = np.array([desired_pos[0], desired_pos[1], heading])  # the desired state vector

        e = np.dot(trans_matrix, (pose - qd).resize((-1, 1)))

        lamda = -3  # design a desired behaviour
        k2 = lamda * lamda
        k3 = -2 * lamda

        v = np.sqrt(np.power(desired_pos[0], 2) + np.power(desired_pos[1], 2))  # calculate magnitude


        linear = v
        angular = (-k2 * e[1] - k3 * e[2]) * v + curvature * (self._time[nearest + 1] - self._time[nearest])

        return linear, angular