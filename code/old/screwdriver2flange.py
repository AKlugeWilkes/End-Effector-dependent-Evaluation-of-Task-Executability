import math
from re import X
import numpy as np
import quaternion
import sympy as sp
from sympy.utilities.lambdify import lambdify
import matplotlib.pyplot as plt
import csv


def calcFlangePos(target_pose, tool_offset, plot=True, export=True):
    """Calculates, visualizes and exports possible flange points as csv file

    Parameters:
    target_pose ((np.array, np.quaternion)): TCP pose ([x,y,z], [w,x,y,z])
    tool_offset (np.array): Tool dimension ([x,y,z])
    plot (bool): True to enable 3D plot
    export (bool): True to enable csv export
    """
    try:
        # define standard vector base (world coordinate system)
        basis_x = np.array([1, 0, 0])
        basis_y = np.array([0, 1, 0])
        basis_z = np.array([0, 0, 1])

        # calculate direction of tool basis vectors (world coordinate system)
        basis_tool_x = quaternion.rotate_vectors(target_pose[1], basis_x)
        basis_tool_y = quaternion.rotate_vectors(target_pose[1], basis_y)
        basis_tool_z = quaternion.rotate_vectors(target_pose[1], basis_z)

        # calculate tool offset in screw direction only (world coordinate system)
        screw_offset_x = basis_tool_x.dot(np.array([0, 0, tool_offset[2]])) ########## 
        screw_offset_y = basis_tool_y.dot(np.array([0, 0, tool_offset[2]])) ##########            
        screw_offset_z = basis_tool_z.dot(np.array([0, 0, tool_offset[2]])) ########## why does the offset only depend on z-coordinate of the tool dimension? ** IN SCREW DIRECTION** --> that is only z
        screw_offset = np.array([screw_offset_x, screw_offset_y, screw_offset_z])  # vector from flange to TCP in screw direction only ########## Can screw direction be more complex than just parallel with the z-axis? ** no, otherwise you would screw around an angle**
        screw_vec = screw_offset / np.linalg.norm(screw_offset)   # normalized vector (unit vector)

        # calculate tool radius around flange center
        rad = np.sqrt(tool_offset[0]**2 + tool_offset[1]**2)

        # calculate two normal vectors (unit vectors) to the tool offset vector in screw direction
        x, y, z = sp.symbols('x, y, z')
        eq = sp.Eq(screw_vec[0] * x + screw_vec[1] * y + screw_vec[2] * z, 0)     # dot product between screw vector and wanted orthogonal vector is equal to 0
        sol = sp.solve(eq, (x, y, z))   # symbolic solution for any normal vector
        expr_x = sol[0][0]
        expr_y = sol[0][1]
        expr_z = sol[0][2]
        eval_x = lambdify((x, y, z), expr_x, "numpy")   # functions to evaluate the vector via substitution of dependant variables
        eval_y = lambdify((x, y, z), expr_y, "numpy")
        eval_z = lambdify((x, y, z), expr_z, "numpy")
        vec_1 = np.array([eval_x(1, 1, 1), eval_y(1, 1, 1), eval_z(1, 1, 1)])   # first normal vector (substitution of x=1, y=1, z=1)
        vec_1 = vec_1 / np.linalg.norm(vec_1)   # normalized vector (unit vector)
        vec_2 = np.cross(screw_vec, vec_1)      # second normal vector is calculated with the cross product between the screw vector and the first vector
        vec_2 = vec_2 / np.linalg.norm(vec_2)   # normalized vector (unit vector)

        # describe the location of possible flange points mathematically
        print(f"\nP = S + r * cos(alpha) * U + r * sin(alpha) * V")   # parameter representation of a circle in 3D
        print(f"S = {target_pose[0] - np.array([screw_offset_x, screw_offset_y, screw_offset_z])}")
        print(f"r = {rad}")
        print(f"U = {vec_1}")
        print(f"V = {vec_2}")
        sol_X, sol_Y, sol_Z = calcCirclePoints(target_pose[0] - screw_offset, rad, vec_1, vec_2, 10)

        if plot:
            plotPoints(sol_X, sol_Y, sol_Z)

        if export:
            exportPoints(sol_X, sol_Y, sol_Z, target_pose[1]) ### TODO different orientation for each point!!!! - array instead of one orientation for all

    except Exception as e:
        # print("Please check the input format. Example use: \ncalcFlangePos((np.array([0, 0, 0]), np.quaternion(1, 0, 0, 0)), np.array([10, 10, 10]))")
        print(e)
    return (sol_X, sol_Y, sol_Z, target_pose[1])


def calcCirclePoints(S, r, U, V, incr = 5):
    """Returns calculated coordinates of points given the parametric representation of a circle in 3D

    Parameters:
    S (np.array): support vector ([x,y,z])
    r (float): radius
    U (np.array): first normal vector (unit vector)
    V (np.array): second normal vector (unit vector)
    inc (float): increments in which points are calculated on the circumference in degree

    Returns:
    X, Y, Z (list, list, list): 3D positions split into X, Y, Z coordinates
    """
    X = []
    Y = []
    Z = []

    for alpha in range(0, 360, incr):
        point = S + r * math.cos(math.radians(alpha)) * U + r * math.sin(math.radians(alpha)) * V
        X.append(point[0])
        Y.append(point[1])
        Z.append(point[2])

    return X, Y, Z


def plotPoints(X_cord, Y_cord, Z_cord):
    """Visualizes given points in a 3D scatter plot.

    Parameters:
    X_cord (list): X-coordinates
    Y_cord (list): Y-coordinates
    Z_cord (list): Z-coordinates
    """
    fig = plt.figure()
    ax = plt.axes(projection ='3d')
    ax.scatter(X_cord, Y_cord, Z_cord, c="r", marker="o")
    plt.show()


def exportPoints(X_cord, Y_cord, Z_cord, orientation, filename="flange_poses"):
    """Exports poses into a csv file.

    Parameters:
    X_cord (list): X-coordinates
    Y_cord (list): Y-coordinates
    Z_cord (list): Z-coordinates
    orientation (np.quaternion): TCP orientation ([w,x,q,z])
    """
    with open(f'{filename}.csv', 'w', newline = '') as csvfile:
        writer = csv.writer(csvfile, delimiter = ',')
        for i in range(len(X_cord)):
            writer.writerow([X_cord[i], Y_cord[i], Z_cord[i], orientation.w, orientation.x, orientation.y, orientation.z])





if __name__ == '__main__':

    # help(calcFlangePos)
    # help(calcCirclePoints)
    # help(plotPoints)
    # help(exportPoints)

    # print("\nNo translation, no rotation:")
    # calcFlangePos((np.array([0, 0, 0]), np.quaternion(1, 0, 0, 0)), np.array([20, 10, 100]))

    # print("\nNo translation, 180 degree rotation around world X-axis:")
    # calcFlangePos((np.array([0, 0, 0]), np.quaternion(0, 1, 0, 0)), np.array([20, 10, 100]))

    print("\nTranslation, 45 degree rotation around world X-axis:")
    calcFlangePos((np.array([0.1, 0.2, 0.3]), np.quaternion(0.924,  0.383, 0, 0)), np.array([10, 10, 100]))



