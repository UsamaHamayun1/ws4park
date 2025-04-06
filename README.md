
ROS Package Documentation: visual parking setup using april tags
How to used it/Prep work: 

StepA- Create workspace or clone the give package in your workspace/src folder

StepB- build packge in main workspace using “colcon build” source it “source install/setup.bash”

StepC- Connect your camera 
Install your ros2 drivers for usb_cam  “sudo apt install ros-humble-usb_cam”

Verify The connected Device 
	v4l2-ctl --list-devices
Output : 
USB Camera (usb-0000:00:14.0-11):
    /dev/video0

Now Stream the camera or all the connected /dev/video*  to re-verify it : 
ffplay -f v4l2 -input_format mjpeg -framerate 30 -video_size 1280x720 /dev/video*

Note : Once you verify the video of your desired camera , Replace “/dev/video4” in launch file with your camera name



Step3- Calibration

Copy the Calibration info of your camera (provided by vendor) into cfg/camera_info.yaml. If the calibration info is not available do the next Step from here and follow the instructions

Download the april tags from here 

How to run this package 



Terminal 1 : ros2 launch apriltag_ros usb_cam_apriltag.launch.yml


Terminal 2 : ros2 run apriltag_ros box_visualizer.py 
Rviz (not mandatory)

Terminal3 : ros2 run apriltag_ros 2d_square_tracker.py






1. Introduction
This document describes a ROS 2 package designed to track the 2D positions of AprilTags, calculate the distance between a specified tag and the center of a square formed by other tags, and determine if the specified tag is inside the square. It also provides real-time visualization and logging of this data.
This package utilizes the apriltag_ros package for AprilTag detection and the tf2_ros package for transform management.
apriltag_ros is a ROS package that provides an interface for detecting AprilTags in images. AprilTags are visual fiducial markers, which are useful for various robotics applications such as localization, navigation, and object tracking. The apriltag_ros package uses the AprilTag library to detect these tags in a camera's image stream and then publishes the detected tag information, including their position and orientation, as ROS 2 messages. This allows other ROS 2 nodes to easily use AprilTag detections for their specific tasks.
2. Package Structure
The core functionality is implemented in the Python script square_distance_tracker.py.
2.1 square_distance_tracker.py
square_distance_tracker.py: This script is the main component of the package. It creates a ROS 2 node that:
Receives AprilTag detection data from the apriltag_ros package.
Calculates the 2D distance between a specified tag and the center of a square formed by other tags.
Determines if the specified tag is inside the square.
Publishes the calculated distance and inside/outside status as ROS 2 messages.
Visualizes the tag positions and distance in real-time using matplotlib.
Manages coordinate frame transformations using tf2_ros.
2.2 Dependencies
rclpy: ROS 2 Python client library, providing core ROS 2 functionalities for Python. This library allows the script to interact with the ROS 2 system, including creating nodes, subscribing to topics, publishing messages, and managing node lifecycles.
apriltag_msgs: ROS 2 message definitions for AprilTag detections, defining the structure of the data published by apriltag_ros. These messages contain the information about detected AprilTags, such as their ID, position, and orientation.
geometry_msgs: ROS 2 message definitions for common geometric data types, such as TransformStamped (used for TF transformations). TransformStamped is the standard message type for representing coordinate frame transformations in ROS 2.
std_msgs: ROS 2 message definitions for standard data types, such as Float64 and Bool, used for publishing the calculated distance and inside/outside status. These message types are used for simple numerical and logical data.
matplotlib: Python library for creating plots and visualizations. It is a powerful tool for creating informative visual representations of data.
numpy: Python library for numerical computations, used for calculating distances and positions. numpy provides efficient array operations and mathematical functions.
math: Python standard library module providing mathematical functions. It provides access to a wide range of mathematical functions.
tf2_ros: ROS 2 package for working with the tf2 transformation library, enabling the management of coordinate frames and transformations. It provides the ROS 2 bindings for tf2.
os: Python standard library module providing operating system interaction functions. This module is used for interacting with the operating system, such as creating directories and checking file paths.
datetime: Python standard library module providing classes for working with dates and times. This module is used for timestamping data and logging.

2.3 Package Structure of apriltag_ros
src/
apriltag_ros/
  -- cfg/
    -- camera_info.yaml
    -- tags_16h5.yaml
  -- launch/
    -- usb_cam_apriltag.launch.yaml
src/apriltag_ros/
  -- 2d_square_tracker.py
  -- box_visualizer.py
  -- AprilTagNode.cpp
  -- src/conversion.cpp
  -- src/pose_estimation.cpp
  -- src/pose_estimation.hpp
  -- src/tag_functions.cpp
  -- src/tag_functions.hpp
  -- CMakeLists.txt
  -- package.xml


cfg/: This directory contains configuration files for the apriltag_ros node.
camera_info.yaml: Contains camera calibration parameters, which are essential for accurate pose estimation of the detected AprilTags.
tags_16h5.yaml: Contains specific tag configurations, such as tag IDs, sizes, and frame names, for the 16h5 AprilTag family.
launch/: This directory contains launch files for starting the apriltag_ros node.
usb_cam_apriltag.launch.yaml: Launch file to run apriltag detection with a USB camera.
src/apriltag_ros/: This directory contains the source code for the apriltag_ros package.
2d_square_tracker.py: Python script to track 2D AprilTag positions and calculate distances.
box_visualizer.py: Python script for visualizing the square formed by AprilTags.



src/
src/AprilTagNode.cpp: Main C++ ROS 2 node for AprilTag detection.
src/conversion.cpp: Source file for coordinate conversion functions.
src/pose_estimation.cpp: Source file for AprilTag pose estimation functions.
src/pose_estimation.hpp: Header file for AprilTag pose estimation functions.
src/tag_functions.cpp: Source file for general AprilTag utility functions.
src/tag_functions.hpp: Header file for general AprilTag utility functions.

CMakeLists.txt: This file contains the build instructions for the apriltag_ros package.
package.xml: This file contains the package metadata, such as dependencies and version information.

2.4 ROS Node Parameters
The apriltag_ros node accepts the following parameters, which can be set in a YAML file or via the command line. These parameters configure the AprilTag detection process.
Here's a detailed explanation of each parameter, including the criteria for changing them:
apriltag:
This is simply the namespace for the parameters, indicating they belong to the apriltag node. You wouldn't typically change this unless you've renamed your node.
ros__parameters:
This is a required section in ROS 2 parameter files, indicating that the following entries are ROS 2 parameters. Do not change it.
Setup Parameters:
image_transport:
Specifies how image data is transmitted.
raw: Transmits the image data as is. This is the most direct, but can use a lot of bandwidth, especially for high-resolution images.
compressed: Transmits the image data in a compressed format (e.g., JPEG). This reduces bandwidth usage, which is beneficial for networked systems or when dealing with high-resolution images. However, it adds computational overhead for compression and decompression.
Criteria for changing:
Use raw if:
You have sufficient bandwidth.
Minimal latency is critical.
Your application needs the highest possible image quality.
Use compressed if:
You have limited bandwidth.
You are transmitting images over a network.
You need to reduce the amount of data being processed.
family:
Specifies the AprilTag family being used (e.g., 36h11, 16h5, 25h9). AprilTags come in different families, which define the tag's structure and encoding.
Criteria for changing:
You must set this to match the family of the AprilTags you are physically using. If you print out AprilTags from a specific generator, ensure this parameter matches that family. Mismatched families will result in detection failures.
size:
The edge length of the square AprilTag in meters.
Criteria for changing:
This parameter is crucial for accurate pose estimation (determining the tag's 3D position and orientation).
You must change this to reflect the actual size of the printed AprilTags. Measure the side of your printed tag accurately and enter that value.
profile:
A boolean flag. If true, the node will print profiling information to the standard output, which can be helpful for debugging and performance analysis.
Criteria for changing:
Set to true if you are trying to:
Diagnose performance bottlenecks.
Understand how long the detection process is taking.
Optimize the node's performance.
Set to false (the default) for normal operation.
Tuning Parameters:
max_hamming:
The maximum number of bit errors allowed when decoding a tag. The Hamming distance is the number of bit positions in which two codewords differ.
Criteria for changing:
A value of 0 means that only perfect matches are accepted (no bit errors). This is the most robust but may miss tags if there is noise or distortion.
Increase this value if:
Your tags are often partially obscured or damaged.
You are operating in a noisy environment (e.g., with poor lighting or a lot of motion blur).
You are detecting tags at a distance.
Be cautious when increasing this value, as it increases the risk of false positives (detecting something that isn't an AprilTag).
detector:
This section groups parameters that control the AprilTag detection algorithm itself.
threads:
The number of CPU threads the detector uses.
Criteria for changing:
Increase this to utilize multiple CPU cores and potentially speed up detection, especially on high-resolution images. A good starting point is the number of physical CPU cores in your system.
If your system is heavily loaded or you are running other CPU-intensive tasks, increasing this too much might not provide a significant benefit or could even hurt performance due to context switching overhead.
decimate:
A factor by which the input image is downsampled before tag detection. For example, decimate: 2.0 means the image's width and height are halved.
Criteria for changing:
Increase this value to speed up detection, especially with high-resolution images. Downsampling reduces the number of pixels the detector needs to process.
Decrease this value (towards 1.0) if you are having trouble detecting small or distant tags. Downsampling reduces the effective resolution, making it harder to detect fine details. A value of 1.0 means no downsampling.
blur:
The standard deviation of a Gaussian blur applied to the image before tag detection. Blurring reduces image noise.
Criteria for changing:
Increase this value (e.g., to 0.8 or 1.0) if your images are noisy. Blurring can smooth out noise and improve tag detection.
Set to 0.0 to disable blurring. If your images are clean, blurring might make the tags slightly less sharp.
refine:
A boolean flag (typically 0 or 1) that controls edge refinement. If enabled, the detector tries to precisely locate tag edges by snapping them to strong gradients in the image.
Criteria for changing:
Set to 1 (enabled) to improve the accuracy of tag corner detection, which leads to more accurate pose estimation. This is generally recommended, especially when using decimation.
Set to 0 to disable edge refinement. You might disable this in very specific, low-processing-power situations, but it's rarely beneficial to do so.
sharpening:
Controls the amount of sharpening applied to the decoded tag image.
Criteria for changing:
Values greater than 0.0 will sharpen the image, which can sometimes improve decoding, especially if the tags are slightly blurry.
Values close to 0.0 mean no sharpening.
Negative values will blur the image.
Adjust this value with caution; excessive sharpening can increase noise.
debug:
An integer that controls the level of debugging information. If greater than 0, the detector may write intermediate images to disk.
Criteria for changing:
Set to a non-zero value (e.g., 1) if you are debugging the AprilTag detection process and want to see the intermediate steps. This can help you understand why tags are not being detected correctly.
Leave at 0 for normal operation.
pose_estimation_method:
Selects the algorithm used to estimate the 3D pose of the tag. Commonly "pnp" (Perspective-n-Point).
Criteria for changing:
"pnp" is a standard and generally good choice. You would only change this if you have a very specific reason, such as needing a different algorithm for a particular camera model or optimization goal. Other methods might exist in the underlying AprilTag library, but "pnp" is very common.
Tag-Specific Parameters:
tag:
This section allows you to override parameters for specific tags. This is useful if you have tags of different sizes or need to assign specific frame IDs.
ids:
A list of the integer IDs of the tags you want to configure.
Criteria for changing:
List the IDs of the tags for which you want to provide custom settings. For example, if you only want to publish the transforms of tags with IDs 1 and 2, you would list those here.
frames:
A list of frame names to use for the corresponding tags in the ids list.
Criteria for changing:
By default, tags are often published with frame names like tag36h11:0 (family and ID). Use this parameter to assign more meaningful names (e.g., camera_link, object_1). The order must correspond to the order of the IDs.
sizes:
A list of tag sizes (in meters) for the corresponding tags in the ids list.
Criteria for changing:
If you have tags of different physical sizes, use this to specify the correct size for each tag ID. This overrides the global size parameter. The order must correspond to the order of the IDs.
