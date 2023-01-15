# End-Effector-dependent-Evaluation-of-Task-Executability


Git project for End-Effector-dependent Evaluation of Task Executability by Mobile Robots in Changeable Assembly Stations. 
This project is funded by Deutsche Forschungsgemeinschaft (DFG, German Research Foundation) under Germanys Excellence Strategy – EXC-2023 Internet of Production – 390621612.
The authors are responsible for the content.

#Goal

  The goal of this project is calculate possible poses of a robot based on the tools measurements, the goal pose and the reachability map of the robot. Using the map, is possible to find the best pose of the robot by the collection of reachable poses and also the reachability index of each of them.

 \page prerequisites Prerequisites 


# How to Use the Application: 

1. On the last line of main code, you need to change the input based on your test.

2. Build the docker:
```
docker build -t spp-maa .
```
3. Run it on terminal:
```
docker run spp-maa
```
4. Export the file from Docker to Host:

    1. Find out the ID of the Docker:

    ```
    docker ps -a
    ```
    2. Copy the file using:
    ```
    sudo docker cp <Docker_ID>:/result/result.csv /path/of/the/host
    ```

\page createMap Create Map

# Create a reachability map for your robot:
1. Run roscore in background and open a new terminal and navigate to catkin_ws:
```
cd catkin_ws
source devel/setup.bash
```

2. Navigate to the directory storing the .xacro file of your robot and convert this .xacro file into .urdf file (skip this step if you already have the .urdf file):
```
cd (the path to the urdf file)
rosrun xacro xacro.py <your_robot>.xacro > <your_robot>.urdf
```

3. Create the collada file using .urdf file:
```
rosrun collada_urdf urdf_to_collada <your_robot>.urdf <your_robot>.dae
```

4. Generate IK solver:
```
python `openrave-config --python-dir`/openravepy/_openravepy_/ikfast.py --robot=<your_robot>.dae --iktype=transform6d --baselink=1 --eelink=8 --savefile=<ikfast_output_path> 
```
On <ikfast_output_path> you should put the path + name of output file + ".cpp" (e.g folder-path/kr6r700sixx.cpp)

5. If the .cpp file is not created, please fix the bug in the ikfast.py by using the solution in the following link:
(https://github.com/ros-planning/moveit_tutorials/issues/417#issuecomment-673459896)

- The file is located at '/usr/local/lib/python2.7/dist-packages/openravepy/_openravepy_0_9/ikfast.py'

- In order to have the right for changes, open a terminal in the directory containing this file and type:
```
sudo chmod 777 ikfast.py
```

6. Go to /catkin_ws/src/reuleaux-melodic/map_creator/include/map_creator and paste the .cpp file which we just created
Please open kinematics.h which is in the same folder and change the name of .cpp file to the current one.

7. Save the kinematics.h file, then go to your catkin workspace and run catkin build:
```
cd catkin_ws
catkin build
```
8. In the same terminal, where we navigated to the robot urdf file, create the reachability map using:
```
rosrun map_creator create_reachability_map 0.05 <your_robot>.h5
```
You can change the resolution from 0.05 to what ever you want. The recommended resolution is 0.12.
Then you can find the .h5 file in /catkin_ws/src/reuleaux-melodic/map_creator/maps

# Interface
Unfortunately, The GUI is not working. Although, you can run main.py instead. Then, before you build the docker, change the parameters on it. If you want to use the interface, please change the branch to "Interface".



# Contact:

A.Kluge-Wilkes@wzl-mq.rwth-aachen.de

Laboratory for Machine Tools and Production Engineering WZL of RWTH Aachen 

Chair of Production Metrology and Quality Management

Department Model-based Systems

Group Model-based Assembly Automation


