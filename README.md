# Laboratory 5-7 ARIAC 2019 Competition

This submission marks the conclusion of the ARIAC competition. The success of this laboratory project is contingent upon the groundwork laid in Lab 6, where solutions for inverse kinematics were computed.

# Problems with the current node
# Lab5 

In lab5 all the functions work!

# Lab7
Now the robot's robotic arm is able to move to any specified position that has a SOLUTION, I think I'm correct with my IK, but I'm a little confused as to where this robotic arm is getting the end position from. 

# Install Simulation Environment

```
mkdir -p ~/ariac_ws/src
cd ~/ariac_ws/src
git clone https://github.com/cwru-eecs-373/cwru_ariac_2019.git
rosdep install --from-paths cwru_ariac_2019 --ignore-src -r -y
cd ../
sudo -- /bin/bash -c "source /opt/ros/noetic/setup.bash; catkin_make -DCMAKE_INSTALL_PREFIX=/opt/ros/noetic install"
```

The ARIAC Simulation does not work on its own at this time. It must be invoked through another package. That package should be built in its own workspace so that it is not conflated with the simulator itself.


# Fixing bug

When executing the launch file for the initial time, it may encounter issues due to a bug in the "empy" module for Python3. There are two alternative solutions to address this problem. First, you can resolve it by employing the following method:

```
roslaunch ecse_373_ariac ecse_373_ariac.launch python:=false
```

However, the first method is inefficient, so we can use the second method instead:

```
sudo patch /usr/lib/python3/dist-packages/em.py < `rospack find ecse_373_ariac`/patches/empy.patch
```

This approach involves applying a patch to the "em.py" file within the Python3 empy module. This corrective action ensures a thorough and permanent resolution to the existing bug.

# Launch Competition Environment

We can use this command to launch ARIAC 2019

```
roslaunch ariac_entry lab5.launch
```

we can use this command to kill the launch properly and quickly

```
killall gzserver gzclient roslaunch
```


The environment does not initialize correctly upon launch; it requires manual intervention by pressing the play button located at the bottom-middle left of the screen to initiate proper startup.

# Run the ARIAC 2019 Competition Node

We can use this command to run the competition
```
roslaunch ariac_entry lab5.launch
```
