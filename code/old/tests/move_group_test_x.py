#!/usr/bin/env python

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import tf
import geometry_msgs
from math import pi,sqrt
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from matplotlib.patches import Circle 
from createCircle import createCircle

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

robot = moveit_commander.RobotCommander()

scene = moveit_commander.PlanningSceneInterface()

group_name = "manipulator"
move_group = moveit_commander.MoveGroupCommander(group_name)

display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)

# We can get the name of the reference frame for this robot:
planning_frame = move_group.get_planning_frame()
# print "============ Planning frame: %s" % planning_frame

# # We can also print the name of the end-effector link for this group:
# eef_link = move_group.get_end_effector_link()
# print "============ End effector link: %s" % eef_link

# # We can get a list of all the groups in the robot:
# group_names = robot.get_group_names()
# print "============ Available Planning Groups:", robot.get_group_names()

# Sometimes for debugging it is useful to print the entire state of the
# robot:
# print "============ Printing robot state"
# print robot.get_current_state()
# print ""
exit = 'n'
goal = [0.6, 0.6, 0.6]
flange = [0.7, 0.72, 0.728]
while (exit != 'y'):
#dont use n too large, or the the robot simulation moves pretty much randomly
    values = createCircle(goal, flange, n=25, freix=True, freiy=False, freiz=False)

    for i in values:
        # print(aux)
        euler_x = i[3]
        euler_y = i[4]
        euler_z = i[5]
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.position.x = i[0]
        pose_goal.position.y = i[1]
        pose_goal.position.z = i[2]
        quaternion = tf.transformations.quaternion_from_euler(euler_x, euler_y, euler_z)
        pose_goal.orientation.x = quaternion[0]
        pose_goal.orientation.y = quaternion[1]
        pose_goal.orientation.z = quaternion[2]
        pose_goal.orientation.w = quaternion[3]
        move_group.set_pose_target(pose_goal)
        plan = move_group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        move_group.clear_pose_targets()
    
    exit = raw_input("Exit? (y/n)")