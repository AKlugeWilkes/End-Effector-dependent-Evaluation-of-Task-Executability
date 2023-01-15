from scipy.spatial.transform import Rotation
import numpy as np
from screwdriver2flange import calcFlangePos


class Task():
	def __init__(self, pose):
		"""	Defines a task pose

		Parameters:
		pose (np.array): Task coordinates and euler angles ([x,y,z,a,b,c])
		"""
		self.pose = pose

	def transform_coodinates_to_robot(self, pose_robot):
		"""Performs a change of coordinate system from global to robot

		Parameters:
		pose_robot (np.array): Robots global pose (coordinates and rotation relative to global coordinate system) ([x,y,z,a,b,c])
		"""

		for i in range(6):
			self.pose[i] -= pose_robot[i]


def euler_to_quaternion(euler_angles):
	rot = Rotation.from_euler('xyz', euler_angles, degrees=True)
	return rot.as_quat()

def calculate_task_poses(tasks, pose_robot):
	"""Creates a list of Task instances and transforms them to the robots coordinate system

	Parameters:
	tasks ((np.array)): List of task poses (([x,y,z,a,b,c]))
	"""

	task_instances_list = []
	for i in range(len(tasks)):
		task_instances_list.append(Task(tasks[i]))
		task_instances_list[i].transform_coodinates_to_robot(pose_robot)

	return task_instances_list

def calculate_flange_positions(tasks, tool_offset, csv):
	"""Calculates the flange locations for each Task in the list and writes them to .csv or returns them
	use calculate_task_poses as helper function to create the necessary task objects

	Parameters:
	tasks ((np.array)): List of task instances ((Task))
	tool_offset (var): offset of the tool in mm
	"""

	flangePos = []
	for i in range(len(tasks)):
		task_coordinates = tasks[i].pose
		task_quternion = euler_to_quaternion(tasks.pose) #given in tasks
		task_pose = [task_coordinates,task_quternion]
		pos = calcFlangePos(task_pose, tool_offset, False, csv)

	flangePos.append(pos)
	return flangePos
	
def get_falnge_poses(task_poses, robot_pose, tool_offset, csv):
	"""Method to be called by GUI, either returns a list of all task poses or writes the to csv file as required

	Parameters:
	task_poses ((np.array)): List of task poses with coordiantes and euler angels ((x,y,z,a,b,c))
	robot_pose (np.array): Pose of the robot relative to the global coordinate system (x,y,z,a,b,c)
	tool_offset (var): offset of the tool in mm
	csv (bool): True to write poses to csv file
	"""

	task_instances = calculate_task_poses(task_poses,robot_pose)
	flange_poses = calculate_flange_positions(task_instances, tool_offset, csv)
	return  flange_poses

