#ifndef FEATUREEXTRACTOR_H
#define FEATUREEXTRACTOR_H

#include "Settings.h"
#include <iostream>
#include <opencv2/calib3d.hpp>
#include "opencv2/highgui.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <future>
#include <numeric>

namespace DC_VSLAM
{

struct TrackedKeys
{
    std::vector<cv::KeyPoint> keyPoints;
    std::vector<cv::KeyPoint> rightKeyPoints;
    // left features assigned to grid
    std::vector<std::vector<std::vector<int>>> lkeyGrid;
    // right features assigned to grid
    std::vector<std::vector<std::vector<int>>> rkeyGrid;
    // index of right feature corresponding to left feature
    std::vector<int> rightIdxs;
    // index of left feature corresponding to right feature
    std::vector<int> leftIdxs;
    // estimated depth of stereo matches
    std::vector<float> estimatedDepth;
    // whether the feature is close enough to be considered a stereo match
    std::vector<bool> close;
    // descriptors 
    cv::Mat Desc, rightDesc;
    // multipliers for grid
    float xMult, yMult;
    // number of grids
    int xGrids, yGrids;

    // copy constructor
    void getKeys(const TrackedKeys& keysToClone)
    {
        keyPoints = keysToClone.keyPoints;
        rightKeyPoints = keysToClone.rightKeyPoints;
        lkeyGrid = keysToClone.lkeyGrid;
        rkeyGrid = keysToClone.rkeyGrid;
        rightIdxs = keysToClone.rightIdxs;
        leftIdxs = keysToClone.leftIdxs;
        Desc = keysToClone.Desc.clone();
        rightDesc = keysToClone.rightDesc.clone();
        close = keysToClone.close;
        estimatedDepth = keysToClone.estimatedDepth;
        xMult = keysToClone.xMult;
        yMult = keysToClone.yMult;
        xGrids = keysToClone.xGrids;
        yGrids = keysToClone.yGrids;
    }
};

class FeatureExtractor
{
    // ORB parameters
    const int nFeatures;
    const int edgeThreshold;
    const int patchSize;
    const int halfPatchSize {15};
    const int maxFastThreshold;
    const int minFastThreshold;
    
    // ORB pattern
    std::vector<cv::Point> pattern;
    
    // ORB scale pyramid
    std::vector<int> umax;

    public:
        // scale pyramid parameters
        const float imScale;
        const int nLevels;

        // pyramid of images
        std::vector <cv::Mat> imagePyramid;
        std::vector<int>scaledPatchSize;
        // needed parameters for scale pyramid
        std::vector < float > scalePyramid;
        std::vector < float > scaleInvPyramid;
        std::vector < float > sigmaFactor;
        std::vector < float > InvSigmaFactor;
        std::vector < int > featurePerLevel;
        
        
        FeatureExtractor(const int _nfeatures = 2000, const int _nLevels = 8, const float _imScale = 1.2f, const int _edgeThreshold = 19, const int _patchSize = 31, const int _maxFastThreshold = 20, const int _minFastThreshold = 7);

        // ssc from https://github.com/BAILOOL/ANMS-Codes
        std::vector<cv::KeyPoint> ssc(std::vector<cv::KeyPoint> keyPoints, int numRetPoints,
                         float tolerance, int cols, int rows);

        // extract keypoints
        void extractKeysNew(cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors);
        void computeKeypointsORBNew(cv::Mat& image, std::vector<std::vector<cv::KeyPoint>>& allKeys);

        // compute orientations for rotation invariance
        void computeAllOrientations(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints);
        float computeOrientation(const cv::Mat& image, const cv::Point2f& point);

        // compute pyramid of image for scale invariance
        void computePyramid(const cv::Mat& image);
    
};

} // namespace DC_VSLAM

#endif // FEATUREEXTRACTOR_H