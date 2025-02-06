import pytest
from geometry_msgs.msg import Twist
from ar_pipe_server.drive import DriveNode
from ar_pipe_server.turn import TurningNode
from ar_pipe_server.follow import FollowerNode


# Driving Node gets None as Input Data and should return None
def test_drive_no_data_return_none():
    drive = DriveNode()
    cmd = Twist()
    cmd.linear.x = 0.0
    cmd.angular.z = 0.0
    result = True
    assert drive.drive(None, 0.0) == (cmd, result, 1)

#Driving Node gets Input with Distance too high, returns driving command with positive linear velocity
def test_drive_return_cmd_vel():
    drive = DriveNode()
    print(drive.drive((1.0, -0.5, 4.0), 0.1))

#Driving Node gets Input with Distance within threshold, returns driving command with linear velocity 0.0
def test_drive_dst_reached():
    drive = DriveNode()
    print(drive.drive((1.0, -0.35, 1.9), 0.1))

#Turning Node sets target angle with Input + pi
def test_turn_set_target():
    turn = TurningNode()
    print("Expected -2.14...: " + str(turn.set_target_angle(1.0)))
    print("Expected 2.14...: " + str(turn.set_target_angle(-1.0)))

#Turning Node angular error is set to 0, returns True for goal reached
def test_turn_perform_turn_angle_reached():
    turn = TurningNode()
    print(turn.perform_turning(1.0, 1.0))

#Turning Node returns angular movement command to turn towards target
def test_turn_return_cmd_vel():
    turn = TurningNode()
    print(turn.perform_turning(1.0, 2.0))

#Follower Node gets Input None, returns None
def test_follow_return_none():
    follow = FollowerNode()
    print(follow.follow_target(None))

#Follower Node gets Input with Distance higher than threshold, returns movement command with positive linear speed
def test_follow_drive_forwards():
    follow = FollowerNode()
    print(follow.follow_target((1.0, -0.35, 4.0)))

#Follower Node gets Input with Distance lower than threshold, returnst movement command with negative linear speed
def test_follow_drive_backwards():
    follow = FollowerNode()
    print(follow.follow_target((1.0, -0.35, 1.5)))