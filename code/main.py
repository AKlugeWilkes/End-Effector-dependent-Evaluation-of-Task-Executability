
from hashlib import new
import math
from cv2 import norm
import numpy as np
import h5py
import os
import csv
import matplotlib.pyplot as plt
# import quaternion
from scipy.spatial.transform import Rotation
import pyrr


pi = math.pi

def normalize(v, tolerance=0.00001):
    """Normalize a vector 
    Input
    : param v: vector to be normalized
      param tolerance: precision of decimal numbers 

    Output
    : new vector v normalized
    """
    mag2 = sum(n * n for n in v)
    if abs(mag2 - 1.0) > tolerance:
        mag = math.sqrt(mag2)
        v = tuple(n / mag for n in v)
    return v

def q_conjugate(q):
    """Calcule the conjucate of a quaternion
    Input
    : param q: quaternion to be conjugated

    Output
    : tuple with new elemens of the quaternion    
    """
    x, y, z, w = q
    return (-x, -y, -z, w)

def qv_mult(q1, v1):
    """Rotate a vector using quaternion
    Input
    : param q1: quaternion 
      param v1: vector

    Output
    : Rotated vector
    """
    q2 = v1 + [0.0] 
    return q_mult(q_mult(q1, q2), q_conjugate(q1))[:3]

def axisangle_to_q(v, theta):
    """
    Calcule a quaternion based on a vector and angle theta
    Input
    : param v: vector
      param theta: angle theta

    Output
    : x-, y-, z-, w-quaternion
    """
    v = normalize(v)
    x, y, z = v
    theta /= 2
    w = math.cos(theta)
    x = x * math.sin(theta)
    y = y * math.sin(theta)
    z = z * math.sin(theta)
    return x, y, z, w

def q_mult(q1, q2):
    """
    Multiply two quaternions
    Input
    : param q1 and q2: arrays representing the quaternion (x,y,z,w) 

    Output
    : x, y, z, w components of the new quaternion
    """
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 + y1 * w2 + z1 * x2 - x1 * z2
    z = w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2
    return x, y, z, w


def createDefaultCircle(tool_height, tool_width,n=20):
    """Creates flange positions in task coordinate system
    
    tool_height: (m)
    tool_width: (m)
    n: numbers of flange poses on the circle"""

    circle=[]
    # radius of the circle
    r = tool_width
    # calculate x-cord and y-cord for n points on the circle	
    values = [((math.cos(2*pi/n*x)*r),(math.sin(2*pi/n*x)*r),(2*pi/n*x)) for x in range(0,n)]
    for i in values:
    # i0,i1, tool_height: x,y,z Coordinates. 0,0,i2: Euler angle
        circle.append((i[0],i[1],tool_height,0,0,i[2]))
    return circle


def euler_to_quaternion(pose):
    """
     Helper function to return any poses' rotation as quaternion
    """
    euler_angles = [pose[3], pose[4], pose[5]]
    rot = Rotation.from_euler('xyz', euler_angles, degrees=True)
    quaternion = np.quaternion(rot.as_quat()[0],rot.as_quat()[1],rot.as_quat()[2],rot.as_quat()[3])
    return quaternion


def createDefaultCircle_quat(tool_height, tool_width,n=20):
    """Same as createDefaultCircle but in quaternions
    
    Creates flange positions in task coordinate system
    

    tool_height: (m)
    tool_width: (m)
    n: numbers of flange poses on the circle"""
    circle=[]
    # radius of the circle
    r = tool_width
    # calculate x-cord and y-cord for n points on the circle	
    values = [((math.cos(2*pi/n*x)*r),(math.sin(2*pi/n*x)*r),(2*pi/n*x)) for x in range(0,n)]
    for i in values:
    # i0,i1, tool_height: x,y,z Coordinates. 0,0,i2: Euler angle
        v = [i[0],i[1],tool_height]
        new_quaternion = euler_to_quaternion([i[0],i[1],tool_height,0,0,i[2]])
        circle.append((i[0],i[1],tool_height, new_quaternion.x, new_quaternion.y, new_quaternion.z, new_quaternion.w))
    return circle    


def transform(point, new_coordinate_axis):
    """get location of a point in the new coordinate system
     new_coordinate_axis: the axis of new coordinate system, measured from the original coordinate system
     point: np.array (x,y,z,x_euler,y_euler,z_euler) - a point in the original coordinate system
    """
    # Calculate the rotational matrix R from old frame to new frame:
    x_euler = new_coordinate_axis[3]
    y_euler = new_coordinate_axis[4]
    z_euler = new_coordinate_axis[5]

    rotX = np.array([[1,0,0],[0,math.cos(x_euler),-math.sin(x_euler)],[0,math.sin(x_euler),math.cos(x_euler)]], np.float64)

    rotY = np.array([[math.cos(y_euler),0,math.sin(y_euler)],[0,1,0],[-math.sin(y_euler),0,math.cos(y_euler)]], np.float64)
  
    rotZ = np.array([[math.cos(z_euler),-math.sin(z_euler),0],[math.sin(z_euler),math.cos(z_euler),0],[0,0,1]], np.float64)

    R = np.matmul(rotY,rotZ)
    R = np.matmul(R,rotX)
    R = np.transpose(R)

    # x-,y-,z-coordinates of point in original coordinate system
    point_xyz = np.array([point[0],point[1],point[2]])
    # x-,y-,z-euler angles of point in original coordinate system   
    point_abc = np.array([point[3],point[4],point[5]])
    
    rotated_xyz = np.matmul(R,point_xyz)
    rotated_abc = np.matmul(R,point_abc)

    # point measured from the new coordinate system
    point_newView = np.array([rotated_xyz[0]-new_coordinate_axis[0],rotated_xyz[1]-new_coordinate_axis[1],rotated_xyz[2]-new_coordinate_axis[2],rotated_abc[0],rotated_abc[1],rotated_abc[2]])
    return point_newView

def transform_quat(point, new_coordinate_axis):
    """same as transform but in Quaternions

    get location of a point in the new coordinate system (aka: view a point of original 
    coordinate system in the new coordinate system)

    new_coordinate_axis: the axis of new coordinate system, measured from the original coordinate system
    point: np.array (x, y, z, x_quat, y_quat, z_quat, w_quat) - a point in the original coordinate system

    more info: https://stackoverflow.com/questions/4870393/rotating-coordinate-system-via-a-quaternion

    """
    
     # x-,y-,z-coordinates of point in original coordinate system
    point_coord = [point[0],point[1],point[2]]
    point_quat = normalize([point[3],point[4],point[5], point[6]])
    # x-,y-,z-,w-quaternion of point in new coordinate system   
    axis_quat = normalize([new_coordinate_axis[3],new_coordinate_axis[4],new_coordinate_axis[5],new_coordinate_axis[6]])

    rotated_xyz = qv_mult(pyrr.quaternion.inverse(axis_quat), point_coord)
    rotated_quat = q_mult(point_quat, pyrr.quaternion.inverse(axis_quat))
    # point measured from the new coordinate system
    point_newView = np.array([rotated_xyz[0]-new_coordinate_axis[0],rotated_xyz[1]-new_coordinate_axis[1], rotated_xyz[2]-new_coordinate_axis[2],rotated_quat[0],rotated_quat[1],
                                rotated_quat[2],rotated_quat[3]])
    
    return  point_newView


def getFlangePos(robot_pose, task_pose_global, tool_height, tool_width, n=20):
    """Get flange positions for a task (in robot coordinate system)

    Execution-sequence: 
    1.get view of task position in robot coordinate system (from view world to view robot)
    2.get view of flange positions in robot coordinate system (from view task to view robot)
    
    robot_pose: in global coordinate system (m)
    task_pose_global: in global coordinate system (m)
    tool_height: (m)
    tool_width: (m) 
    n:numbers of flange poses for a task"""

    task_from_robot_view = transform(task_pose_global,robot_pose)
    circle = createDefaultCircle(tool_height,tool_width,n)
    flange_poses_from_robot_view = np.zeros(shape=(len(circle),6),dtype='float64')
    for i in range(len(circle)):
        flange_pos = circle[i]
        robot_from_task_view = np.array([-task_from_robot_view[0],-task_from_robot_view[1],-task_from_robot_view[2],
                                        -task_from_robot_view[3],-task_from_robot_view[4],-task_from_robot_view[5]])
        flange_poses_from_robot_view[i] = transform(flange_pos,robot_from_task_view)
    return flange_poses_from_robot_view


def getFlangePos_quat(robot_pose, task_pose_global, tool_height, tool_width, n=20):
    """TODO: same as getFlangePos but in Quaternion

    Get flange positions for a task (in robot coordinate system)

    Execution-sequence: 
    1.get view of task position in robot coordinate system (from view world to view robot) - use method transform_quat
    2.get view of flange positions in robot coordinate system (from view task to view robot) - use method transform_quat
    
    robot_pose: in global coordinate system (m)
    task_pose_global: in global coordinate system (m)
    tool_height: (m)
    tool_width: (m) 
    n:numbers of flange poses for a task
    
    """
    task_from_robot_view = transform_quat(task_pose_global,robot_pose)
    # print("task")
    # print()
    circle = createDefaultCircle_quat(tool_height,tool_width,n)
    flange_poses_from_robot_view = np.zeros(shape=(len(circle),7),dtype='float64')
    for i in range(len(circle)):
        flange_pos = circle[i]
        robot_from_task_view = np.array([-task_from_robot_view[0],-task_from_robot_view[1],-task_from_robot_view[2],
                                         -task_from_robot_view[3],-task_from_robot_view[4],-task_from_robot_view[5],
                                         -task_from_robot_view[6]])

        flange_poses_from_robot_view[i] = transform_quat(flange_pos,robot_from_task_view)
    return flange_poses_from_robot_view


def getReachabilityScore(file, flangePose,tolerance=0.11):
        """Find the reachability score of a pose from the reachability map 'file'

        Parameteres:
        file: The reachability map (.h5 file) location: in folder "maps"
        pose: an np.array representing a pose (in robot coordinate system) (m)
        tolerance: error deviation (0.11m for reachability map with recommended resolution 0.12)

        Return: The reachability score (type float64)
        """
        file_dir = os.path.abspath(os.path.join(os.pardir,"./maps",file))
        f = h5py.File(file_dir,'r')
        spheres = f['Spheres']
        spheres_data = spheres['sphere_dataset']
        for i in range(spheres_data.len()):
            if math.isclose(flangePose[0],spheres_data[i,0],abs_tol=tolerance) and math.isclose(flangePose[1],spheres_data[i,1],abs_tol=tolerance) and math.isclose(flangePose[2],spheres_data[i,2],abs_tol=tolerance):
                return spheres_data[i,3]
        return 0


def exportCSV(result, inEuler, export_file='../result/result.csv'):
    """Method to export the flange poses + their reachability scores to a csv file. Default: result.csv in folder result
    
    result: an np.array of 7-ary np.arrays comprises of flange poses and their reachability scores
    export_file: the desired export file, user can explicitly create their file (in result folder)
    else the result will be exported to result.csv"""
    csv_dir = os.path.abspath(os.path.join(os.pardir,"./result", export_file))
    f = open(csv_dir,'w')
    writer = csv.writer(f)
    if inEuler:
        writer.writerow(['x','y','z','x-euler','y-euler','z-euler','Reachability score'])
    else:
        writer.writerow(['x','y','z','x-quaternion','y-quaternion','z-quaternion','w-quaternion','Reachability score'])

    for row in result:
        writer.writerow(row)
    
    f.close()

def plot(result, inEuler):
    Xreach1 = []
    Xreach2 = []
    Xreach3 = []
    Xreach4 = []
    Xreach5 = []

    Yreach1 = []
    Yreach2 = []
    Yreach3 = []
    Yreach4 = []
    Yreach5 = []

    Zreach1 = []
    Zreach2 = []
    Zreach3 = []
    Zreach4 = []
    Zreach5 = []

### Filter the data according to its reachability score

    for row in result:
        if inEuler:
            reachabilityScore = row[6]
        else:
            reachabilityScore = row[7]

        if reachabilityScore<=20:
            Xreach1.append(row[0])
            Yreach1.append(row[1])
            Zreach1.append(row[2])
        elif reachabilityScore>20 and reachabilityScore<=40:
            Xreach2.append(row[0])
            Yreach2.append(row[1])
            Zreach2.append(row[2])
        elif reachabilityScore>40 and reachabilityScore<=60:
            Xreach3.append(row[0])
            Yreach3.append(row[1])
            Zreach3.append(row[2])
        elif reachabilityScore>60 and reachabilityScore<=80:
            Xreach4.append(row[0])
            Yreach4.append(row[1])
            Zreach4.append(row[2])
        else:
            Xreach5.append(row[0])
            Yreach5.append(row[1])
            Zreach5.append(row[2])

    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')

    # plot points with color according to its reachability score
    ax.scatter(Xreach1, Yreach1, Zreach1, c='red')
    ax.scatter(Xreach2, Yreach2, Zreach2, c='yellow')
    ax.scatter(Xreach3, Yreach3, Zreach3, c='green')
    ax.scatter(Xreach4, Yreach4, Zreach4, c='lightblue')
    ax.scatter(Xreach5, Yreach5, Zreach5, c='blue')

    plt.savefig(os.path.abspath(os.path.join(os.pardir,"./result", 'result.svg')))


def calcForAllTasks(file, robot_pose,task_poses_global,tool_height,tool_width,n=20,export=True, export_file='/result/result.csv',tolerance=0.11, inEuler=True):
    """Calculate and return flange positions + reachability scores for a list of tasks (in robot coordinate system)

    ATTENTION!!!!: the length measurements are in meter, the angle measurement are in radians (for Euler)

    Parameters:
    file: name of the reachability map (in maps folder)
    robot_pose (np.array): Pose of the robot relative to the global coordinate system (x,y,z,a,b,c) (m)
    task_poses_global ((np.array)): List of task poses with coordinates and euler angels ((x,y,z,a,b,c)) relative to global coordinate system (m)
    tool_height: (m)
    tool_width: (m)
    n: number of desired flange poses for each task
    export: True to export results to .csv file & create .svg file to plot points
    export_file: name of the .csv file to export to (in maps folder)
    tolerance: maximal distance to the "next point" on reachability maps
    """
    counter = 0
    if inEuler==False:
        ### generate result array for quaternions
        result = np.zeros(shape=(len(task_poses_global)*n,8),dtype='float64')
    else:
        ### generate result array for eulers
        result = np.zeros(shape=(len(task_poses_global)*n,7),dtype='float64')
     
    for task_pos in task_poses_global:
        if inEuler:
            flange_pos = getFlangePos(robot_pose,task_pos,tool_height,tool_width,n) 
        else:
            flange_pos = getFlangePos_quat(robot_pose,task_pos,tool_height,tool_width,n) 

        for pos in flange_pos:
            reachabilityIndex = getReachabilityScore(file,np.array([pos[0],pos[1],pos[2]]),tolerance)
            if inEuler:
                result[counter] = np.array([pos[0],pos[1],pos[2],pos[3],pos[4],pos[5],reachabilityIndex])  
            else:
                result[counter] = np.array([pos[0],pos[1],pos[2],pos[3],pos[4],pos[5],pos[6],reachabilityIndex]) 
            counter+=1
    
    if export==True:
        exportCSV(result,inEuler,export_file)
        plot(result, inEuler)

    return result
