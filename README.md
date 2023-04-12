# ROS PCL Object Detection


### This code is a part of experimental setup of ZED2 steareo camera for its point cloud processing .


```bash
├── objectrecognition
    ├── include
    │   ├── objecrrecognition
    │       ├──box.h
    │       ├──ros_control.h
    │       ├──ros_pointprocess.h
    │
    ├── msg
    │   ├──BoundingBox3d.msg
    │   ├──BoundingBoxes3d.msg
    │   ├──DetectedObject.msg
    │   ├──DetectedObjectsArray.msg
    │   ├──GetCloud.msg
    │   ├──vectorpointcloud.msg
    ├── script
    │   ├──point_cloud.py
    │   ├──recognition.py
    │   ├──train_svm.py
    │   ├──model.sav
    ├── src
    │   ├──point_cloud.py
    │      ├──objectrecognition
    │      ├──__init__.py
    │      ├──features.py
    │      ├──marker_tools.py
    │      ├──pcl_helper.py
    │   ├──ransac.cpp
    │   ├──ros_control.cpp
    │   ├──ros_node.cpp
    │   ├──ros_pointcloud.cpp
    ├── Tranning_Data
    │   ├── training_set_new.sav
    ├── srv
    │   ├──GetNormals.srv
    └── launch
    │   ├──segmentation.launch
    │   ├──Trainning.launch
    ├──.vscode
    ├──CMakeLists.txt
    ├──package.xml
    ├──setup.py
```
If you do not have an active ROS workspace, you can create one by:
```sh
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin_make
```

Now that you have a workspace, clone or download this repo into the **src** directory of your workspace:
```sh
$ cd ~/catkin_ws/src
$ git clone https://github.com/BADAL244/ros-pcl-ml.git
```
Now from a terminal window:  
```sh
$ cd ~/catkin_ws
$ catkin_make
```

Add following to your .bashrc file
```
source ~/catkin_ws/devel/setup.bash
```


1.first launch zed_ros_wrapper launch file .
[More information](https://www.stereolabs.com/documentation/guides/using-zed-with-ros/introduction.html)

ZED2 camera:

    $ roslaunch zed_wrapper zed2.launch
    
if you have any other source of ROS_msg sensor_msgs::PointCloud2 
then :

in **include/objectrecognition** 
ros_control.h file 
#59 change 

const std::string point_cloud name to your point cloud2 topic name for detection purpose
```
const std::string point_cloud = "your PointCloud2 Topic Name "
```
<b>ros_pointcloud.cpp include code for</b>:
```
1.filtering 
2.segmentation based on Ransac
3.clustering 
4.Bounding Box around the cluster 
5.surface Normal calcualtion 
6.conversion form ros_msg to pcl_msg and vice -versa
```
```
$ roslaunch objectrecognition segmentation.launch
```

this provide a crop BOx filter based Point Cloud based on setting in X Y Z direction for range selection 

```
$ roslaunch objectrecogntion Trainning.launch 
```
used for Trainnig Differnt model like Stair , chair and other Indoor object based on their colour histrogram and surface normal histogram for SVM-SVC Trainnig for creation of a Trainned Model to estimate the correct label for each object .

installing Bazel and MediaPipe 

https://www.forecr.io/blogs/installation/how-to-install-bazel

https://www.forecr.io/blogs/ai-algorithms/how-to-download-build-mediapipe-on-nvidia-jetson-xavier-nx
https://docs.nvidia.com/deeplearning/frameworks/install-pytorch-jetson-platform/index.html
https://towardsdatascience.com/create-virtual-environment-using-virtualenv-and-add-it-to-jupyter-notebook-6e1bf4e03415


https://developer.nvidia.com/cuda-gpus
https://docs.nvidia.com/deeplearning/frameworks/install-pytorch-jetson-platform/index.html
