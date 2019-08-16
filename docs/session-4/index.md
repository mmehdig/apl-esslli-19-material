# Hands-on Robot Operating System (ROS)

This is a quick tutorial for ROS 
With some plactical examples. 
On the first section,
you are going to learn 
how to setup the workspace,
how to run nodes and 
how to write nodes with python.
Then on second section,
you are going to run a simple keras object recognition
as a node which can can take images from camera sensors. 

## Requirements

* [Virtual Box (latest version) + extension pack](https://www.virtualbox.org/wiki/Downloads)
* Download and run [Ubuntu-18.04-ROS-Melodic.ova](https://gubox.box.com/shared/static/1d4h3alkkx8i9noo7jx4xsy1gjdd0l5a.ova)
(`apluser`: `aplgu`)


Or


* Ubuntu 16.04 or 18.04
* ROS Kinetic or Melodic

## 1. Dive Into ROS

In this section, we provide essential information and examples to build a ros worksace, and run python programs in ROS. 

Most part of this section is also covered in more details in [ROS official tutorials](http://wiki.ros.org/ROS/Tutorials).

Follow these steps:

#### 1.1. Manage the shell environment

Open a terminal, then run this:

```
printenv | grep ROS
```

The result must be something like:

```
ROS_ROOT=/opt/ros/melodic/share/ros
ROS_PACKAGE_PATH=/opt/ros/melodic/share
ROS_MASTER_URI=http://localhost:11311
ROSLISP_PACKAGE_DIRECTORIES=
ROS_DISTRO=melodic
ROS_ETC_DIR=/opt/ros/melodic/etc/ros
```

If you can't find these settings, run `source /opt/ros/melodic/setup.bash`.

The default installation of ROS sets up these parameters however, it is important to understand that these settings are critiacl for running ROS on terminal environment. 


#### 1.2. Create a Workspace

After setting up terminal environment, you need to have a workspace to be able to run and execute a process.
For exmaple, most famouse IDEs have their own build system to maintain processes 
regarding package management, maintaing all project files and run tests and delivering the finalised package.
To manage these processes for ROS, we use `catkin`.  
Catkin is a build system based on CMake that is extended for Python as well.
With catkin we can manage dependencies, build and run programs for ROS.

Here is a quick way to create a catkin workspace, then we can move on to write a python 
codes for ROS:

1. Open a terminal window
2. Create a directory for your workspace (e.g. `~/ros_ws`)
3. Run the shell command to make the workspace:
```
mkdir -p ~/ros_ws/src
cd ~/ros_ws/
catkin_make
```

After you built the workspace, you must source this workspace for shell environment:
```
source devel/setup.bash
```

You can check if it has correctly added the workspace in your repository:
```
echo $ROS_PACKAGE_PATH
```

This must show that the workspace directory is part of the `ROS_PACKAGE_PATH`:
`/home/<username>/ros_ws/src:/opt/ros/melodic/share`

In order to understand the ROS file system and folders follow [the ROS tutorial](http://wiki.ros.org/ROS/Tutorials/NavigatingTheFilesystem). 

#### 1.3. Create and build a package

In a workspace for your projects, you can have several packages. 
Each package is a directory in `src`, containing the minimum specifications of its 
CMake build and the dependencies with required packages.

You can use terminal command `catkin_create_pkg` to initialize
a package with its dependencies: `catkin_create_pkg <package_name> [depend1] [depend2] [depend3]`
For this tutorial, we create a package as following:
```
cd ~/ros_ws/src
catkin_create_pkg this_tutorial std_msgs rospy
```

After adding a new package, you need to rebuild the workspace, and source it again:
```
cd ~/ros_ws/
catkin_make
. ~/ros_ws/devel/setup.bash
```

For more details you can read the documentations and follow more advanced
tutorials in the [wiki.ros.org](wiki.ros.org).

#### 1.4. Running ROS and your programs in ROS
ROS is a message passing framework, which depends on a master core program.
The easiest way to run the master core is to open a terminal and run the following:
```
roscore
```

`roscore` starts the master process. We can always list all nodes 
with the following command:
```
rosnode list
```

*HINTS: you can run commands with nohup in background: `nohup roscore &`
but in this tutorial, it is easier if we open each process in separate terminals.*

The next step is to run a python code in ROS.
Here is how to write a python code, in your package folder.
We will expand this small program to become a node in ROS.

First, let's write and run a python hello world for ROS:
```
echo '#!/usr/bin/env python
print("hello rosrun!")
' > ~/ros_ws/src/this_tutorial/src/hello.py
```

Second, the python code must be executable to be able to work with ROS: 
```
chmod +x ~/ros_ws/src/this_tutorial/src/hello.py
```

This is how you can run it:
```
rosrun this_tutorial hello.py
```

*HINT: you can kill nodes with `ctrl+c`*

Next step is to make it persistent.
In order make it a valid ROS node, we just need to assign it a name and initiate it as a Node (e.g. `greeter`).
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

Now, if we run it again, this time it shoudl stay open:
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

Download `keras-application.py` from [the hands-on repository.](https://github.com/mmehdig/apl-esslli-19-material/)

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

