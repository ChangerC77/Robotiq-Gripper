import gripper

"""
close the gripper
"""

gripper = gripper.RobotiqGripper()
# gripper.close_gripper()
gripper.move(position=205, speed=255, force=20) # with tactile
