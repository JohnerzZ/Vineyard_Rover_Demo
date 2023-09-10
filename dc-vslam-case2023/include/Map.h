#ifndef MAP_H
#define MAP_H

#include "Camera.h"
#include "KeyFrame.h"
#include "FeatureMatcher.h"
#include "Settings.h"
#include "Optimizer.h"
#include <fstream>
#include <string>
#include <iostream>
#include <random>
#include <unordered_map>

namespace DC_VSLAM
{

class KeyFrame;

class MapPoint
{
    private:

    public:
        // world position 4d
        Eigen::Vector4d wp;
        // world position 3d
        Eigen::Vector3d wp3d;
        // how many times continously this mappoint was not matched
        int unMCnt {0};
        std::vector<cv::KeyPoint> obs;
        // the last left feature that observed this mappoint
        cv::KeyPoint lastObsL;
        // the last right feature that observed this mappoint
        cv::KeyPoint lastObsR;
        // the last keyframe that observed this mappoint
        KeyFrame* lastObsKF;
        // descriptor
        cv::Mat desc;
        // the keyframes that are matched with this mappoint
        std::unordered_map<KeyFrame*, std::pair<int,int>> kFMatches;
        // the keyframes that are matched with this mappoint in the back camera
        std::unordered_map<KeyFrame*, std::pair<int,int>> kFMatchesB;


        // last keyframe number that was used for local BA
        int LBAID {-1};
        // last keyframe number that was used for loop closure
        int LCID {-1};

        // max-min scale distance for scale prediction
        float maxScaleDist, minScaleDist;

        // if the mappoint is active (can be seen by current frame)
        bool isActive {true};

        bool inFrame {true};
        bool inFrameR {true};
        bool isOutlier {false};
        bool added {false};

        // predicted position on the image frame
        cv::Point2f predL, predR;
        // predicted angle on the image frame
        float predAngleL, predAngleR;
        // last scale level that matched with the mappoint
        int scaleLevel {0};
        // predicted scale level
        int prdScaleLevel {0};
        // scale level of left feature
        int scaleLevelL {0};
        // scale level of right feature
        int scaleLevelR {0};

        // last keyframe number observed this mappoint
        int keyFrameNb {0};
        // keyframe number that created the mappoint
        const unsigned long kdx;
        // index of the mappoint
        const unsigned long idx;

        // update the mappoint (position, predictions, decsriptor, etc.)
        void update(KeyFrame* kF);
        void update(KeyFrame* kF, const bool back);
        int predictScale(float dist);
        // add keyframe connection
        void addConnection(KeyFrame* kF, const std::pair<int,int>& keyPos);
        void addConnectionB(KeyFrame* kF, const std::pair<int,int>& keyPos);

        // erase keyframe connection
        void eraseKFConnection(KeyFrame* kF);
        void eraseKFConnectionB(KeyFrame* kF);
        void setActive(bool act);
        void SetInFrame(bool infr);
        void SetIsOutlier(bool isOut);
        bool getActive() const;
        bool GetIsOutlier() const;
        bool GetInFrame() const;
        void calcDescriptor();
        MapPoint(const Eigen::Vector4d& p, const cv::Mat& _desc, const cv::KeyPoint& obsK, const unsigned long _kdx, const unsigned long _idx);

        Eigen::Vector4d getWordPose4d() const;
        Eigen::Vector3d getWordPose3d() const;
        void updatePos(const Eigen::Vector3d& newPos, const Zed_Camera* zedPtr);
        void setWordPose4d(const Eigen::Vector4d& p);
        void setWordPose3d(const Eigen::Vector3d& p);
};

class Map
{
    private:

    public:
        // if apriltag has been detected
        bool aprilTagDetected {false};

        // if the end of frames has been reached
        bool endOfFrames {false};

        // all keyframes
        std::unordered_map<unsigned long, KeyFrame*> keyFrames;
        // all mappoints
        std::unordered_map<unsigned long, MapPoint*> mapPoints;
        // all active mappoints front
        std::vector<MapPoint*> activeMapPoints;
        // all active mappoints back
        std::vector<MapPoint*> activeMapPointsB;
        // all frames poses
        std::vector<KeyFrame*> allFramesPoses;
        // keyframe index
        unsigned long kIdx {0};
        // mappoint index
        unsigned long pIdx {0};
        
        // if keyframe has been added so that local BA can initiate
        bool keyFrameAdded {false};
        bool keyFrameAddedMain {false};
        bool frameAdded {false};
        // if local BA has been done
        bool LBADone {false};
        // which keyframe index was the last to be optimized
        int endLBAIdx {0};

        // the pose calculated from the apriltag detection (for loop closure)
        Eigen::Matrix4d LCPose = Eigen::Matrix4d::Identity();
        // if loop closure has been done
        bool LCDone {false};
        // if loop closure is to be initiated
        bool LCStart {false};
        // the keyframe candidate for loop closure
        int LCCandIdx {-1};
        // which keyframe index was the last to be optimized for loop closure
        int endLCIdx {0};


        Map(){};
        void addMapPoint(MapPoint* mp);
        void addKeyFrame(KeyFrame* kF);
        void removeKeyFrame(int idx);
        mutable std::mutex mapMutex;

    protected:
};

} // namespace DC_VSLAM

#endif // MAP_H