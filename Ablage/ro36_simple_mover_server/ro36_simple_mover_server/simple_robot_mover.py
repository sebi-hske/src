# SimpleRobotMover calculates the velocity commands to reach a desired position
# from a starting position.

from enum import Enum
import numpy as np
import math


class _State(Enum):
    """State of the robot."""
    IDLING = 0
    ROTATINGTOTARGET = 1
    MOVING = 2
    ROTATINGFINAL = 3


def normalize_angle(angle):
    """Normalize an angle to the interval [-pi, pi]."""
    while angle > math.pi:
        angle -= 2*math.pi
    while angle < -math.pi:
        angle += 2*math.pi
    return angle


class SimpleRobotMover:
    """
    A class for controlling the movement of a simple robot.

    The robot can be commanded to move to a target pose, which consists of an x-coordinate,
    a y-coordinate, and an orientation. The robot will rotate to face the target pose, then
    move to the target pose, and finally rotate to the final orientation.
    """

    _omega = 0.63  # Commanded angular velocity for turning commands rad/s
    _v = 0.2  # Commanded linear velocity for moving commands m/s
    _angle_threshold = 0.01  # Threshold up to which the orientation will be corrected rad  !!!War auf 0.5 wegen zu langer wartezeit
    # Longitudinal distance threshold up to which the robot will move towards the goal m
    _distance_threshold = 0.01

    def __init__(self) -> None:
        """Initialize the SimpleRobotMover to state IDLING."""
        self._state = _State.IDLING
        self._target_pose = None

    def set_target_pose(self, x, y, theta):
        """
        Set the target pose for the robot to move to.

        :param x: The x-coordinate of the target pose.
        :type x: float
        :param y: The y-coordinate of the target pose.
        :type y: float
        :param theta: The orientation of the target pose.
        :type theta: float
        """
        try:
            self._target_pose = [float(x), float(y), float(theta)]
        except:
            raise ValueError("Target pose values must be convertible to floats.")
        self._state = _State.ROTATINGTOTARGET

    def cancel_target(self):
        """Cancel the target and set the state to IDLING."""
        self._target_pose = None
        self._state = _State.IDLING

    def get_velocity_command(self, x, y, theta):
        """
        Return the velocity commands to reach the target pose from the
        current pose.

        Returns None if the target pose has not been set or has been reached.

        :param x: The x-coordinate of the current pose.
        :type x: float
        :param y: The y-coordinate of the current pose.
        :type y: float
        :param theta: The orientation of the current pose.
        :type theta: float

        :return: Longitudinal and angular velocity commands to reach the target pose.
        :rtype: tuple(float, float)
        """
        
        
        current_pose = [x, y, theta]
        if self._state == _State.IDLING:
            return None
        elif self._state == _State.ROTATINGTOTARGET:            
            return self._rotate_to_target(current_pose)
        elif self._state == _State.MOVING:
            return self._move_to_target(current_pose)
        elif self._state == _State.ROTATINGFINAL:
            return self._rotate_final(current_pose)
        else:
            return None

    def _rotate_to_target(self, current_pose):
        """Return the velocity commands to rotate to the target pose from the current pose."""
        vec_to_target = [self._target_pose[0] - current_pose[0],
                         self._target_pose[1] - current_pose[1]]
        dist_to_target = np.linalg.norm(vec_to_target)                  #numpy fehler mit np.norm
        if dist_to_target < SimpleRobotMover._distance_threshold :
            self._state = _State.ROTATINGFINAL
            return self._move_to_target(current_pose)

        angle_to_target = math.atan2(vec_to_target[1], vec_to_target[0])
        angle_diff = normalize_angle(angle_to_target - current_pose[2])
        if abs(angle_diff) < SimpleRobotMover._angle_threshold:
            self._state = _State.MOVING
            return self._move_to_target(current_pose)

        if angle_diff > 0:
            return [0., SimpleRobotMover._omega]
        else:
            return [0., -SimpleRobotMover._omega]

    def _move_to_target(self, current_pose):
        """
        Return the velocity commands to move forward until the dsitance to
        the target position is less than SimpleRobotMover._distance_threshold
        in longitudinal direction of the robot.
        """
        robot_dir_vec = [np.cos(current_pose[2]), np.sin(current_pose[2])]
        long_dist = np.dot([self._target_pose[0] - current_pose[0],
                           self._target_pose[1] - current_pose[1]], robot_dir_vec)
        if long_dist < SimpleRobotMover._distance_threshold:
            self._state = _State.ROTATINGFINAL
            return self._rotate_final(current_pose)
        else:
            return [SimpleRobotMover._v, 0.]

    def _rotate_final(self, current_pose):
        """
        Return the velocity commands to rotate to the target orientation
        from the current orientation.
        """
        angle_diff = self._target_pose[2] - current_pose[2]
        if abs(angle_diff) < SimpleRobotMover._angle_threshold:
            self._state = _State.IDLING
            return None
        else:
            angle_diff = normalize_angle(angle_diff)
            if angle_diff > 0:
                return [0., SimpleRobotMover._omega]
            else:
                return [0., -SimpleRobotMover._omega]
