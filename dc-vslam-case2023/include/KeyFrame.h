#ifndef KEYFRAME_H
#define KEYFRAME_H

#include "Settings.h"
#include "Camera.h"
#include "FeatureExtractor.h"
#include "Map.h"
#include "opencv2/core.hpp"


namespace DC_VSLAM
{

class Map;
class MapPoint;

class KeyFrame
{
    private:

    public:
        // intrinsic parameters
        double fx,fy,cx,cy;
        // intrinsic parameters back camera
        double fxb,fyb,cxb,cyb;
        CameraPose pose;
        // extrinsic parameters
        Eigen::Matrix4d extr;
        // extrinsic parameters back camera
        Eigen::Matrix4d extrB;
        // front camera to back camera transformation
        Eigen::Matrix4d TCamToCam;
        // back camera pose
        Eigen::Matrix4d backPose;
        Eigen::Matrix4d backPoseInv;
        cv::Mat leftIm, rightIm;
        cv::Mat rLeftIm;
        // vector that shows if front left keypoints are matched or not
        std::vector<int> unMatchedF;
        // vector that shows if front right keypoints are matched or not
        std::vector<int> unMatchedFR;
        // vector that shows if back left keypoints are matched or not
        std::vector<int> unMatchedFB;
        // vector that shows if back right keypoints are matched or not
        std::vector<int> unMatchedFRB;
        // scale factor for each level
        std::vector<float> scaleFactor;
        std::vector < float > sigmaFactor;
        std::vector < float > InvSigmaFactor;
        std::unordered_map<KeyFrame*, int> weightsKF;
        // sorted vector of keyframes based on connected mappoints
        std::vector<std::pair<int,KeyFrame*>> sortedKFWeights;
        // log scale factor for scale prediction
        float logScale;
        int nScaleLev;

        // last keyframe number that was used for local BA
        int LBAID {-1};
        // last keyframe number that was used for loop closure
        int LCID {-1};

        // if this keyframe is a candidate for loop closure
        bool LCCand {false};

        // keypoints with parameters
        TrackedKeys keys, keysB;
        // not used
        Eigen::MatrixXd homoPoints3D;
        // number of keyframe
        const unsigned long numb;
        // number of image
        const int frameIdx;
        // how many mappoints are tracked
        int nKeysTracked {0};
        // if this keyframe should be visualized
        bool visualize {true};
        // front left vector that holds the mappoints in the position of the feature with which they are matched
        std::vector<MapPoint*> localMapPoints;
        // front right vector that holds the mappoints in the position of the feature with which they are matched
        std::vector<MapPoint*> localMapPointsR;
        // back left vector that holds the mappoints in the position of the feature with which they are matched
        std::vector<MapPoint*> localMapPointsB;
        // back right vector that holds the mappoints in the position of the feature with which they are matched
        std::vector<MapPoint*> localMapPointsRB;
        // previous keyframe
        KeyFrame* prevKF = nullptr;
        // next keyframe
        KeyFrame* nextKF = nullptr;
        // if active
        bool active {true};
        // if actual keyframe and not just a frame
        bool keyF {false};
        // if this keyframe has been optimized
        bool LBA {false};
        // if this keyframe is fixed
        bool fixed {false};
        // if this is Dual Stereo Setup
        bool backCam {false};

        // update pose after local BA or loop closure
        void updatePose(const Eigen::Matrix4d& keyPose);

        // calculate connections between keyframes
        void calcConnections();


        void setBackPose(const Eigen::Matrix4d& _backPose);
        // erase mappoint connection
        void eraseMPConnection(const int mpPos);
        void eraseMPConnectionB(const int mpPos);
        void eraseMPConnection(const std::pair<int,int>& mpPos);
        void eraseMPConnectionB(const std::pair<int,int>& mpPos);
        void eraseMPConnectionR(const int mpPos);
        void eraseMPConnectionRB(const int mpPos);
        KeyFrame(Eigen::Matrix4d _pose, cv::Mat& _leftIm, cv::Mat& rLIm, const int _numb, const int _frameIdx);
        KeyFrame(const Eigen::Matrix4d& _refPose, const Eigen::Matrix4d& realPose, cv::Mat& _leftIm, cv::Mat& rLIm, const int _numb, const int _frameIdx);
        KeyFrame(const Zed_Camera* _zedCam, const Eigen::Matrix4d& _refPose, const Eigen::Matrix4d& realPose, cv::Mat& _leftIm, cv::Mat& rLIm, const int _numb, const int _frameIdx);
        KeyFrame(const Zed_Camera* _zedCam, const Zed_Camera* _zedCamB, const Eigen::Matrix4d& _refPose, const Eigen::Matrix4d& realPose, cv::Mat& _leftIm, cv::Mat& rLIm, const int _numb, const int _frameIdx);
        Eigen::Vector4d getWorldPosition(int idx);
        // get connected keyframes for local BA
        void getConnectedKFs(std::vector<KeyFrame*>& activeKF, const int N);
        // get connected keyframes for loop closure
        void getConnectedKFsLC(const Map* map, std::vector<KeyFrame*>& activeKF);

        Eigen::Matrix4d getPose();
};

} // namespace DC_VSLAM

#endif // KEYFRAME_H