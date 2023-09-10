# DC-VSLAM
This is the software related to the publication: Multicamera Visual SLAM For Vineyard Inspection, submited at the CASE 2023.

DC-VSLAM is a multicamera visual SLAM designed for vineyard inspection. To address the challenge of homogeneous environments, loop closures are detected using AprilTags.

DC-VSLAM has been tested with OpenCV 4.2.0, Eigen 3.3.7 on Ubuntu 20.04 with ROS Noetic.

## Results

Dual Camera on Vineyard :

![til](https://media.giphy.com/media/v1.Y2lkPTc5MGI3NjExMjlkYTQ3YmQwMjJmOTAyOTljNGVmZmRmNGJlN2ZkZDQ3MGU2YWM4NiZjdD1n/wi9GiM3Pfev54cKcEs/giphy.gif)

# Installation

DC-VSLAM has many dependencies and all can be downloaded using a script provided in this repo. ROS Noetic is needed for the installation. Instructions on how to install ROS Noetic can be found [here](http://wiki.ros.org/noetic/Installation).

We recommend the users to create an empty workspace. Clone the package on the catkin workspace and run the build script. Python 3 has to be set as default for Pangolin installation.

```
cd ${WORKSPACE_PATH}/src
git clone https://ChristosKokas@bitbucket.org/csl_legged/dc-vslam-case2023.git
cd dc-vslam-case2023
chmod +x build.sh
./build.sh
```

# Quick Start

Several launch files are provided. The RT denotes real-time and the AT denotes the use of AprilTag Loop Closure. Change the launch files to match the config file name and the topic of the image msgs for AprilTag detection.

DC-VSLAM can run both with images and with rosbags. Images need to be provided as presented below ( the bullets are folders ): 

- dc-vslam-case2023
  - images
    - dataset_name
      - left
        1. 000000.jpg(.png)
        2. 000001.jpg(.png)
        3. ...
      - right
        1. 000000.jpg(.png)
        2. 000001.jpg(.png)
        3. ...
      - leftBack
        1. 000000.jpg(.png)
        2. 000001.jpg(.png)
        3. ...
      - rightBack
        1. 000000.jpg(.png)
        2. 000001.jpg(.png)
        3. ...

And the full path to the dataset folder has to be provided in the config file.

### Replaying Recorded Experiments

Rosbags for each experiment can be downloaded from [here](https://centralntuagr-my.sharepoint.com/:f:/g/personal/amast_central_ntua_gr/Ehrrt-IUFB5DsIH4nJnh6tUBfJvtzW-FmX1IaixmhvRuSg?e=G41m63).

To launch : 

| Launch File  | Rosbag  |
| ------------- | ------------- |
| roslaunch dc_vslam DualCamRTAT.launch  | rosbag play csl_experiment.bag  |
| roslaunch dc_vslam DualCamRT.launch  | rosbag play simu_vineyard_exp.bag  |
