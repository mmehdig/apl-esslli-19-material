# Hands-on Robot Operating System (ROS)

This is a quick tutorial for ROS and plactical example on how to setup environment to run python applications inlcuding image classification. 

## Requirements

* [Virtual Box (latest version) + extension pack](https://www.virtualbox.org/wiki/Downloads)
* Download and run [Ubuntu-18.04-ROS-Melodic.ova](https://gubox.box.com/shared/static/1d4h3alkkx8i9noo7jx4xsy1gjdd0l5a.ova)

Or

* Ubuntu 16.04 or 18.04
* ROS Kinetik or Melodic

## 1. Dive Into ROS

In this section, we provide essential information and examples to build a ros worksace, and run python programs in ROS. 

Most part of this section is also covered in more details in [ROS official tutorials](http://wiki.ros.org/ROS/Tutorials).

Follow these steps:

#### 1.1. Manage the shell environment

Run this:

```
printenv | grep ROS
```

The result must be something like this:

```
ROS_ROOT=/opt/ros/kinetic/share/ros
ROS_PACKAGE_PATH=/opt/ros/kinetic/share
ROS_MASTER_URI=http://localhost:11311
ROSLISP_PACKAGE_DIRECTORIES=
ROS_DISTRO=kinetic
ROS_ETC_DIR=/opt/ros/kinetic/etc/ros
```

Run `source /opt/ros/kinetic/setup.bash` if the environment was not ready.


#### 1.2. Create a Workspace
'catkin' is a build system based on CMake and extended with Python.
most famouse IDEs have their own build system to maintain processes 
related to maintainig packages in a project and run them for test.
catkin prepare dependencies and the code to be build and run for ros.
This is how you can create a catkin workspace which you can write python 
codes for ros:
```
mkdir -p ~/ros_ws/src
cd ~/ros_ws/
catkin_make
```

Now that the folder is ready to be used as a ros workspace, you must source
this workspace for shell environment:
```
source devel/setup.bash
```

In order to check if this is correct:
```
echo $ROS_PACKAGE_PATH
```

The workspace forlder is part of the `ROS_PACKAGE_PATH` for example
`/home/<username>/ros_ws/src:/opt/ros/kinetic/share`

In order to understand the ROS file system and folders follow [the ROS tutorial](http://wiki.ros.org/ROS/Tutorials/NavigatingTheFilesystem). 

#### 1.3. Create and build a package

Within the workspace for your projects, you can have several packages. 
each package is a folder containing the minimum specifications of its 
CMake build and the required packages.

Using the command line tool `catkin_create_pkg` is an easy way to initialize
a new package: `catkin_create_pkg <package_name> [depend1] [depend2] [depend3]`
As an example, we create a package for this tutorial as following:
```
cd ~/ros_ws/src
catkin_create_pkg this_tutorial std_msgs rospy
```

Now you need to rebuild the workspace, and source it again:
```
cd ~/ros_ws/
catkin_make
. ~/ros_ws/devel/setup.bash
```

For more details you can read the documentations and follow more advanced
tutorials in the [wiki.ros.org](wiki.ros.org).

#### 1.4. Running ROS and your programs in ROS
ROS needs its core program runing in background. The easiest way to run it is as following:
```
roscore
```

`roscore` starts the master process with one node. We can see list of nodes 
with the following command:
```
rosnode list
```

*HINTS: you can run commands with nohup in background: `nohup roscore &`
but in this tutorial, it is easier to open each process in separate terminals*

The next step is to run a python code in ros.
here we show how to write a python code in the package folder and run it.
then we will expand this small program to become a node in ROS.

First, let's run a hello world for ros:
```
echo '#!/usr/bin/env python
print("hello rosrun!")
' > ~/ros_ws/src/this_tutorial/src/hello.py
```

It needs to be executable as well: 
```
chmod +x ~/ros_ws/src/this_tutorial/src/hello.py
```

This is how you can run it:
```
rosrun this_tutorial hello.py
```

*HINT: you can kill nodes with `ctrl+c`*

Next step is to make it persistent.
In order make it a valid ros node we just need to assign it a name,
and tell the ros master.
```
echo '#!/usr/bin/env python
import rospy
rospy.init_node("greeter")
rate = rospy.Rate(1) # it means 1hz
while not rospy.is_shutdown():
   print("hello world {0}".format(rospy.get_time()))
   rate.sleep()
' > ~/ros_ws/src/this_tutorial/src/hello.py
```

Now, if you run it again it stays open:
```
rosrun this_tutorial hello.py
```

You can check its info, in another terminal:
```
rosnode info \greeter
```

Nodes alone cannot communicate with eachother,
messages must be communicated under a ros topic.
you can list all topics in command line:
```
rostopic list
```

We can start a new topic and send signals there:
```
echo '#!/usr/bin/env python
import rospy
from std_msgs.msg import String
rospy.init_node("greeter")
rate = rospy.Rate(1) # it means 1hz
pub = rospy.Publisher("greeting_topic", String, queue_size=1)
while not rospy.is_shutdown():
   pub.publish("hello world {0}".format(rospy.get_time()))
   rate.sleep()
' > ~/ros_ws/src/this_tutorial/src/hello.py
```

Now, when we run this node there is no sign of input in this terminal.
```
rosrun this_tutorial hello.py
```

In a seperate terminal, we can check if the new topic is active.
```
rostopic list
```

Then, if the /greeting_topic is in the list, we can get the detailed
information about the publisher and its datatype:
```
rostopic info /greeting_topic
```

We can echo the content that are published in the topic:
```
rostopic echo /greeting_topic
```

Similar to the echo action for rostopic, we can also write a node
which process the published content and then publishes a new materrial.
```
echo '#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def callback(string):
   pub.publish("I heared \"{0}\"".format(string.data))

rospy.init_node("feedbacker")
pub = rospy.Publisher("feedback_topic", String, queue_size=1)
sub = rospy.Subscriber("greeting_topic", String, callback)
rospy.spin()
' > ~/ros_ws/src/this_tutorial/src/feedback.py
chmod +x ~/ros_ws/src/this_tutorial/src/feedback.py
```

Notice that we don't need infinit loop to keep the node running now. And the frequency of publishing content in feedback_topic depends on the how frequent the subscriber calls the callback function.


Now, you can run both of them in two terminals:

**Terminal 1:**
```
. ~/ros_ws/devel/setup.bash
rosrun this_tutorial hello.py
```

**Terminal 2:**
```
. ~/ros_ws/devel/setup.bash
rosrun this_tutorial feedback.py
```

#### 1.5. Record topics (optional)

We often need to record all topics at once to be able to replicate them later. ROS provide this with `rosbag`. First, you can create a folder for saving it on a file:

```
mkdir ~/bagfiles
```

While two nodes are running, we can open another terminal and record them all in one file:

```
cd ~/bagfiles
rosbag record -a
```

Later, you can just play the nodes without running them:

```
rosbag play <your bagfile>
```

#### 1.6. Explore ROS tutorials

It is very common to have several nodes running which depend on each other at the same time. The command for running them at once can be packed into a `roslaunch` script. You can read about this among other related topics in [ROS tutorials](http://wiki.ros.org/ROS/Tutorials). 


## 2. Image Classification With Tensorflow On ROS

Download `keras-application.py`.

We are going to use an object classifier implemented in Keras as a node in ROS which can capture images from camera nodes.

#### 2.1. Publish static images

If you don't have a camera, you can publish an image or an avi movie with `image_publisher`: 
```
rosrun image_publisher image_publisher <path_to_an_image>
```

In another terminal, you can check if the relevat topics are published. List all topics:
```
rostopic list
```

If the node is running properly, there should be several topics related to `/image_publisher`. For example, `/image_publisher_<random-number>/image_raw`. You can try to read the data signal with echo command:
```
rostopic echo /image_publisher_<random-number>/image_raw
```

However, this is going to echo numerical representation of the image. 


#### 2.2. Visualisation with rviz

A simple way to see an image signal is to use `image_view`. For example, you can view the image in `image_publisher` topic:
```
rosrun image_view image_view image:=/image_publisher_<random-number>/image_raw
```

You can also visualise images and other sensors using `rviz`. This tool provides graphical interface for visualising robots and sensors. Open `rviz` with the following command:
```
rosrun rviz rviz
```
Then add a new sensor and configure the related topic for it. When running ROS in virtual box, using rviz is not recommended becuase it requires a lot of memory and processing time.

### 2.3. Publish from webcam

You can capture most webcams using `usb_cam` in ROS:
```
rosrun usb_cam usb_cam_node
```

### 2.4. Run `keras-application.py` node

After downloading the code `keras-application.py`, try to understand the code. 

Place the file in `~/ros_ws/src/this_tutorial/src/keras-application.py`, make sure it is executable with `chmod +x keras-application.py` similar to other codes. This node is going to read image signals as a ROS sensor message. Then, it provides simple interface for classifyinf the image content.

Before running the node, you need to make sure that this node is subscribed to the correct topic. Modify last two lines in `keras-application.py`. Change the topic name in `sub = rospy.Subscriber("/camera/rgb/image_color", Image, callback)`  according to the topic which you are publishing images/video.

```
rosrun this_tutorial keras-application.py
```

OpenCV is going to open a window from camera content. Pressing `r` will run the recognition process and `q` exits the window. In another terminal you can read the published messages as before with running the feedback node:

```
rosrun this_tutorial feedback.py
```


#### Contacts

Contact Mehdi Ghanimifard (mehdi.ghanimifard@gu.se) for questions and help.

