# Ros Documentation 

### Creating a catkin workspace 

If you just installed ROS noetic from apt on Ubuntu then you will have setup.*sh files in '/opt/ros/noetic/', and you could source them like
```bash
source /opt/ros/noetic/setup.bash
```
You will need to run this command on every new shell you open to have access to the ROS commands

```bash
 mkdir -p ~/catkin_ws/src   
 cd ~/catkin_ws/     
 catkin_make   
```

This creates additional build and devel folder in your current directory
This also creates a Cmakelist.txt file in your src folder when you run this command for the first time

```bash
source devel/setup.bash
```
Run this command to use the catkin commands

### Ros packages

The simplest possible package has a structure which looks like this:
* CMakeLists.txt
* package.xml

A trivial workspace might look like this

```
workspace_folder/        -- WORKSPACE
  src/                   -- SOURCE SPACE
    CMakeLists.txt       -- 'Toplevel' CMake file, provided by catkin
    package_1/
      CMakeLists.txt     -- CMakeLists.txt file for package_1
      package.xml        -- Package manifest for package_1
    ...
    package_n/
      CMakeLists.txt     -- CMakeLists.txt file for package_n
      package.xml        -- Package manifest for package_n
```
You can create a package by running the following command

```bash
$ catkin_create_pkg <package_name> [depend1] [depend2] [depend3]
```
creating a new package called 'beginner_tutorials' which depends on std_msgs, roscpp, and rospy:
```bash
catkin_create_pkg beginner_tutorials std_msgs rospy roscpp
```

Building the package
```bash
cd ~/catkin_ws
catkin_make
```






