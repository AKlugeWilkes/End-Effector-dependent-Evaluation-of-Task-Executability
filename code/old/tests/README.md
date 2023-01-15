
#HOW TO TEST FLANGE POSITIONS:
# Prerequisites:

1. Install Moveit Package according to the [link](http://docs.ros.org/en/melodic/api/moveit_tutorials/html/doc/getting_started/getting_started.html
)

2. Create Moveit configuration of the desired robot or install existing configurations from Github.

 * [For Universal Robots](https://github.com/ros-industrial/universal_robot)

 * [Here for UR10](https://github.com/ros-industrial/universal_robot/tree/melodic-devel/ur10_moveit_config)

# Application: 
Remember to source the catkin workspace in terminals 2 and 3
    ```
    source ~/catkin_ws/devel/setup.bash
    ```

1. In the terminal 1 start:
    ```
    roscore
    ```
2. In the terminal 2, start the launch file of the Moveit configuration of the robot. Here for the UR10:
    ```
    roslaunch ur10_moveit_config demo.launch
    ```

3. Execute code in terminal 3 (for rotational degree of freedom around Oz):
    ```
    python move_group_test_z.py
    ```
=> If a solution is found, the robot moves to the target pose.

=> If no solution is found, the following text appears in the terminal:

"ABORTED: No motion plan found. No execution attempted."

