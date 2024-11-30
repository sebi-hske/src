import pytest
import numpy as np
from ro36_simple_mover_server.simple_robot_mover import SimpleRobotMover

"""Given a new SimpleRobotMover,
when the target pose is not explicitly set and the current pose is [0, 0, 0],
then get_velocity_command() returns None."""
def test_get_velocity_command_no_target_pose_0_0_0_return_none():
    mover = SimpleRobotMover()
    assert mover.get_velocity_command(0, 0, 0) == None

"""Given a new SimpleRobotMover,
when the target pose is not explicitly set and the current pose is [1, 2, 3],
then get_velocity_command() returns None."""
def test_get_velocity_command_no_target_pose_1_2_3_return_none():
    mover = SimpleRobotMover()
    assert mover.get_velocity_command(1, 1, 0) == None

"""Given a new SimpleRobotMover,
when the target pose is set and then cancelled,
then get_velocity_command() returns None."""
def test_get_velocity_command_target_pose_set_and_cancelled_return_none():
    mover = SimpleRobotMover()
    mover.set_target_pose(1, 2, 3)
    mover.cancel_target()
    assert mover.get_velocity_command(4, 5, 6) == None

"""Given a new SimpleRobotMover,
when the current pose is [0 0 0] and the target pose is set to [1, 1, 0]
then get_velocity_command() returns [0, 0, SimpleRobotMover._omega]."""
def test_get_velocity_command_cur_pose_0_0_0_target_pose_1_1_0_return_turn_left():
    mover = SimpleRobotMover()
    mover.set_target_pose(1, 1, 0)
    assert mover.get_velocity_command(0, 0, 0) == [0, SimpleRobotMover._omega]

"""Given a new SimpleRobotMover,
when the current pose is [0 0 0] and the target pose is set to [1, -1, 0]
then get_velocity_command() returns [0, 0, -SimpleRobotMover._omega]."""
def test_get_velocity_command_cur_pose_0_0_0_target_pose_1_minus_1_0_return_turn_right():
    mover = SimpleRobotMover()
    mover.set_target_pose(1, -1, 0)
    assert mover.get_velocity_command(0, 0, 0) == [0, -SimpleRobotMover._omega]

"""Given a new SimpleRobotMover,
when the current pose is [0 0 0] and the target pose is set to [1, 0, 0],
then get_velocity_command() returns [SimpleRobotMover._v, 0]."""
def test_get_velocity_command_cur_pose_0_0_0_target_pose_1_0_0_return_move_forward():
    mover = SimpleRobotMover()
    mover.set_target_pose(1, 0, 0)
    assert mover.get_velocity_command(0, 0, 0) == [SimpleRobotMover._v, 0]

"""Given a new SimpleRobotMover,
when the current pose is [1 0 0] and the target pose is set to [1, 0, 1],
then get_velocity_command() returns [0, SimpleRobotMover._omega]."""
def test_get_velocity_command_cur_pose_0_0_0_target_pose_1_0_1_return_turn_left():
    mover = SimpleRobotMover()
    mover.set_target_pose(1, 0, 1)
    assert mover.get_velocity_command(1, 0, 0) == [0, SimpleRobotMover._omega]

"""Given a new SimpleRobotMover,
when the current pose is [1 0 0] and the target pose is set to [1, 0, -1],
then get_velocity_command() returns [0, -SimpleRobotMover._omega]."""
def test_get_velocity_command_cur_pose_1_0_0_target_pose_1_0_minus_1_return_turn_right():
    mover = SimpleRobotMover()
    mover.set_target_pose(1, 0, -1)
    assert mover.get_velocity_command(1, 0, 0) == [0, -SimpleRobotMover._omega]

"""Given a new SimpleRobotMover,
when the current pose is [1 0 0] and the target pose is set to [2, 0, 0],
then get_velocity_command() returns [SimpleRobotMover._v, 0]."""
def test_get_velocity_command_cur_pose_1_0_0_target_pose_2_0_0_return_move_forward():
    mover = SimpleRobotMover()
    mover.set_target_pose(2, 0, 0)
    assert mover.get_velocity_command(1, 0, 0) == [SimpleRobotMover._v, 0]

"""Given a new SimpleRobotMover,
when the current pose is [1 2 pi/4] and the target pose is set to [42, 1, 0],
then get_velocity_command() returns [0, -SimpleRobotMover._omega]."""
def test_get_velocity_command_cur_pose_1_2_pi4_target_pose_42_1_0_return_turn_right():
    mover = SimpleRobotMover()
    mover.set_target_pose(42, 1, 0)
    assert mover.get_velocity_command(1, 2, np.pi/4) == [0, -SimpleRobotMover._omega]