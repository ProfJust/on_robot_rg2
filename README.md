# Roboterprogrammierung mit ROS und Python - Wintersemester 2019/20
- ROS integration of Universal Robots UR3 with attached OnRobot RG2
- Motion planning with MoveIT using [Trac_ik Inverse Kinematics Solver](https://docs.ros.org/melodic/api/moveit_tutorials/html/doc/trac_ik/trac_ik_tutorial.html)
- Visualisation in RVIZ
- Simulation in Gazebo without functionality of gripper
- Object detection and hand to eye transformation with RealSense D435, VISP and OpenCV2
- Execution Control via PyQT Userinterface

# Acknowledgement

This Implementation uses the https://github.com/UniversalRobots/Universal_Robots_ROS_Driver and the https://github.com/fmauch/universal_robot calibration-devel branch.



# Setup and Running
## Motion
### Installation:
```bash
$ cd ~/catkin_ws
$ git clone https://github.com/GeraldHebinck/Universal_Robots_ROS_Driver.git src/Universal_Robots_ROS_Driver
$ git clone -b calibration_devel https://github.com/GeraldHebinck/universal_robot.git src/fmauch_universal_robot
$ git clone https://github.com/GeraldHebinck/rop_ws1920_hht src/rop_ws1920_hht

$ sudo apt install ros-melodic-moveit
$ sudo apt install ros-melodic-trac-ik-kinematics-plugin
$ rosdep update
$ rosdep install -y --from-paths . --ignore-src --rosdistro melodic
$ catkin_make
```
### Execution:
```bash
$ # real Robot: start the Universal Robots ROS Driver and ROS-master
$ roslaunch rop_ws1920_hht ur3_bringup.launch
$ # Simualtion:
$ roslaunch rop_ws1920_hht ur3_gazebo.launch


$ # start MoveIt
$ roslaunch rop_ws1920_hht ur3_moveit_planning_execution.launch
$ # start rviz (optional)
$ roslaunch rop_ws1920_hht moveit_rviz.launch

```
## vision
### Installation:
For the Camera follow instructions on https://github.com/IntelRealSense/realsense-ros

*) Disable Secure Boot in BIOS

Additionally VISP is needed
```bash
$ cd ~/catkin_ws
$ sudo apt-get install ros-melodic-visp
$ git clone -b melodic https://github.com/lagadic/vision_visp.git src/vision_visp

    in File ~/catkin_ws/src/vision_visp/visp_auto_tracker/launch/tracklive_usb.launch 
    in Z21 und Z22 die USB-Camera deaktivieren
        vorher:
        <remap from="/visp_auto_tracker/camera_info" to="/usb_cam/camera_info"/>
        <remap from="/visp_auto_tracker/image_raw" to="/usb_cam/image_raw"/>

        korrigiert:
        <remap from="/visp_auto_tracker/camera_info" to="/camera/color/camera_info"/>
        <remap from="/visp_auto_tracker/image_raw" to="/camera/color/image_raw"/>
        
    in File vision_master_wfT.py Z52 den korrekten Usernamen einfügen
    ... ["/home/oj/catkin_ws/src/vision_visp/visp_auto_tracker/launch/tracklive_usb.launch"])

```

### Execution:
```bash
$ # start RealSense Camera Stream
$ roslaunch rop_ws1920_hht rs_aligned_depth.launch

    ### OJ 2.3.20
    Bei Fehlermeldung
        [ERROR] [1583139268.782813191]: Skipped loading plugin with error: XML Document '/opt/ros/melodic/share/nerian_stereo/nodelet_plugins.xml' has no Root Element. This likely means the XML is malformed or missing..

    die Datei nodelet_plugins.xml in den ROS-Pfad /opt/ros/melodic/share/nerian_stereo kopieren 

$ # start object detection
$ roslaunch rop_ws1920_hht hht_visionmaster.launch
```

## User Interface
### Installation:
```bash
$ pip install PyQt5
```

### Execution
```bash
$ # start controlinterface
$ # Verbindung der ROS-Topics mit den Callbacks des Qt-GUI-hht_interface
$ roslaunch rop_ws1920_hht hht_ur3control.launch

$ # start Qt-UserInterface
$ roslaunch rop_ws1920_hht hht_interface.launch

    ## OJ 2.3.20
    Bild kommt nicht, beim Klick auf Starten
    => Z36 und Z42 in videostream.py auskommentieren 

    ## FS 3.3.20
    Position finden funktioniert nicht mit Fehlermeldung
        [ INFO] [1583229121.559630228]: Starting '/camera/image_raw' (/dev/video0) at 640x480 via mmap (yuyv) at 30 FPS
        [ERROR] [1583229121.559871826]: VIDIOC_S_FMT error 16, Device or resource busy

     ## OJ 3.3.20
    Loesung: in tracklive_usb.launch Z21 und Z22 die USB-Camera deaktivieren
        vorher:
        <remap from="/visp_auto_tracker/camera_info" to="/usb_cam/camera_info"/>
        <remap from="/visp_auto_tracker/image_raw" to="/usb_cam/image_raw"/>

        korrigiert:
        <remap from="/visp_auto_tracker/camera_info" to="/camera/color/camera_info"/>
        <remap from="/visp_auto_tracker/image_raw" to="/camera/color/image_raw"/>
```


# Inspired by
- https://github.com/sharathrjtr/ur10_rg2_ros
- https://github.com/SintefManufacturing/python-urx
