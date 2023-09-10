#ifndef CAMERA_H
#define CAMERA_H

#include "Settings.h"
#include <unistd.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/highgui.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/core.hpp"
#include <yaml-cpp/yaml.h>
#include <Eigen/Core>
#include <Eigen/LU>
#include <chrono>


namespace DC_VSLAM
{

class CameraPose
{
    private:
    public:
        // camera pose
        Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
        // camera reference pose
        Eigen::Matrix4d refPose = Eigen::Matrix4d::Identity();
        // camera pose inverse
        Eigen::Matrix4d poseInverse = Eigen::Matrix4d::Identity();
        std::chrono::time_point<std::chrono::high_resolution_clock> timestamp;

        CameraPose(Eigen::Matrix4d _pose = Eigen::Matrix4d::Identity(), std::chrono::time_point<std::chrono::high_resolution_clock> _timestamp = std::chrono::high_resolution_clock::now());

        // set camera Pose
        void setPose(const Eigen::Matrix4d& poseT);
        void setPose(Eigen::Matrix4d& _refPose, Eigen::Matrix4d& _keyPose);

        // get pose
        Eigen::Matrix4d getPose() const;
        Eigen::Matrix4d getInvPose() const;

        // change pose using reference psoe
        void changePose(const Eigen::Matrix4d& _keyPose);

        // set inv pose from local/global BA
        void setInvPose(const Eigen::Matrix4d poseT);
};

/**
 * @brief Camera class that contains intrinsic values
 * 
 */

class Camera
{
    private:
    public:
        // intrinsic values
        double fx {},fy {},cx {}, cy {};
        // distortion values
        double k1 {}, k2 {}, p1 {}, p2 {}, k3{};
        // camera matrices
        cv::Mat D = cv::Mat::zeros(1,5,CV_64F);
        // camera matrix
        cv::Mat K = cv::Mat::eye(3,3,CV_64F);
        cv::Mat R = cv::Mat::eye(3,3,CV_64F);
        // projection matrix
        cv::Mat P = cv::Mat::eye(3,4,CV_64F);
        // intrinsic matrix
        Eigen::Matrix<double,3,3> intrinsics = Eigen::Matrix<double,3,3>::Identity();
        Camera() = default;
        ~Camera();
        void setIntrinsicValuesUnR(const std::string& cameraPath, ConfigFile* confFile);
        void setIntrinsicValuesR(const std::string& cameraPath, ConfigFile* confFile);
};

/**
 * @brief Zed Camera class that contains 2 cameras and IMU
 * 
 */

class Zed_Camera
{
    private:
    
    public:
        // if keyframe to be added
        bool addKeyFrame {false};
        // if rectified
        bool rectified {};
        float mBaseline, mFps;
        int mWidth, mHeight;
        size_t numOfFrames {};

        Camera cameraLeft;
        Camera cameraRight;
        CameraPose cameraPose;

        ConfigFile* confFile;
        // extrinsics
        Eigen::Matrix<double,4,4> extrinsics = Eigen::Matrix<double,4,4>::Identity();
        // transformation from front camera to back camera
        Eigen::Matrix<double,4,4> TCamToCam = Eigen::Matrix<double,4,4>::Identity();
        // transformation from back camera to front camera
        Eigen::Matrix<double,4,4> TCamToCamInv = Eigen::Matrix<double,4,4>::Identity();
        Zed_Camera(ConfigFile* yamlFile);
        Zed_Camera(ConfigFile* yamlFile, bool backCamera);
        ~Zed_Camera();
        // set back camera transformation
        void setBackCameraT(const bool backCamera);
        void setCameraValues(const std::string& camPath);

};


}

#endif // CAMERA_H