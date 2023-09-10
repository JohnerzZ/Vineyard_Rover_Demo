#ifndef OPTIMIZER_H
#define OPTIMIZER_H

#include "Settings.h"
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include "opencv2/core.hpp"

namespace DC_VSLAM
{

class OptimizePose
{
public:

    OptimizePose(const Eigen::Matrix3d& K, const Eigen::Vector3d& point, const Eigen::Vector2d& observation, const double weight)
        : K_(K), point_(point), observation_(observation), weight_(weight)
    {
    }

    template<typename T>
    bool operator()(const T* const cameraT, const T* const cameraR, T* residuals_ptr) const
    {

        Eigen::Map<const Eigen::Matrix<T, 3, 1>> p_frame(cameraT);
        Eigen::Map<const Eigen::Quaternion<T>> q_frame(cameraR);

        Eigen::Matrix<T, 3, 1> p_cp =
        q_frame * point_.template cast<T>() + p_frame;
        // Compute the map point pose in pixel frame.
        Eigen::Matrix<T, 3, 1> projected = K_ * p_cp;

        // Compute the residuals.
        // [ undistorted - projected ]
        Eigen::Map<Eigen::Matrix<T, 2, 1>> residuals(residuals_ptr);
        residuals[0] =
            observation_.template cast<T>()[0] - projected[0] / projected[2];
        residuals[1] =
            observation_.template cast<T>()[1] - projected[1] / projected[2];

        residuals[0] = T(weight_) * residuals[0];
        residuals[1] = T(weight_) * residuals[1];

        return true;
    }

    static ceres::CostFunction* Create(const Eigen::Matrix3d& K, const Eigen::Vector3d& point, const Eigen::Vector2d& observation, const double weight)
    {
        return (new ceres::AutoDiffCostFunction<OptimizePose, 2, 3, 4>(
                        new OptimizePose(K, point, observation, weight)));
    }
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
    // intrinsic matrix
    const Eigen::Matrix3d& K_;
    const Eigen::Vector3d point_;
    const Eigen::Vector2d observation_;
    const double weight_;

};

// optimize pose for all cameras that are not left front (a transformation from left front to other cameras is needed)
class OptimizePoseR
{
public:

    OptimizePoseR(const Eigen::Matrix3d& K, const Eigen::Matrix<double,3,1>& tc1c2, const Eigen::Matrix3d& qc1c2, const Eigen::Vector3d& point, const Eigen::Vector2d& observation, const double weight)
        : K_(K), tc1c2_(tc1c2), qc1c2_(qc1c2), point_(point), observation_(observation), weight_(weight)
    {
    }

    template<typename T>
    bool operator()(const T* const cameraT, const T* const cameraR, T* residuals_ptr) const
    {

        Eigen::Map<const Eigen::Matrix<T, 3, 1>> p_frame(cameraT);
        Eigen::Map<const Eigen::Quaternion<T>> q_frame(cameraR);

        Eigen::Matrix<T, 3, 1> p_cp =
        q_frame * point_.template cast<T>() + p_frame;
        // transform the point from left front to the current camera
        Eigen::Matrix<T,3,1> pointc1c2 = qc1c2_ * p_cp + tc1c2_;
        // Compute the map point pose in pixel frame.
        Eigen::Matrix<T, 3, 1> projected = K_ * pointc1c2;

        // Compute the residuals.
        // [ undistorted - projected ]
        Eigen::Map<Eigen::Matrix<T, 2, 1>> residuals(residuals_ptr);
        residuals[0] =
            observation_.template cast<T>()[0] - projected[0] / projected[2];
        residuals[1] =
            observation_.template cast<T>()[1] - projected[1] / projected[2];


        residuals[0] = T(weight_) * residuals[0];
        residuals[1] = T(weight_) * residuals[1];

        return true;
    }

    static ceres::CostFunction* Create(const Eigen::Matrix3d& K, const Eigen::Matrix<double,3,1>& tc1c2, const Eigen::Matrix3d& qc1c2, const Eigen::Vector3d& point, const Eigen::Vector2d& observation, const double weight)
    {
        return (new ceres::AutoDiffCostFunction<OptimizePoseR, 2, 3, 4>(
                        new OptimizePoseR(K, tc1c2, qc1c2, point, observation, weight)));
    }
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
    // intrinsic matrix
    const Eigen::Matrix3d& K_;
    // translation
    const Eigen::Matrix<double,3,1>& tc1c2_;
    // quaternion
    const Eigen::Matrix3d& qc1c2_;
    const Eigen::Vector3d point_;
    const Eigen::Vector2d observation_;
    const double weight_;

};

// triangulate a point from multiple views
// only 3D point is optimized, poses remain fixed
class MultiViewTriang
{
public:

    MultiViewTriang(const Eigen::Matrix4d& camPose, const Eigen::Matrix<double, 3, 4>& proj, const Eigen::Vector2d& observation)
        : camPose_(camPose), proj_(proj), observation_(observation)
    {
    }

    template<typename T>
    bool operator()(const T* const point, T* residuals_ptr) const
    {

        T p[4];
        p[0] = point[0];
        p[1] = point[1];
        p[2] = point[2];
        p[3] = T(1);
        Eigen::Map<const Eigen::Matrix<T,4,1>> point4d(p);
        // transform the point from world to camera
        const Eigen::Matrix<T,4,1> pointCam = camPose_ * point4d;
        // project the point to the image plane
        const Eigen::Matrix<T, 3, 1> p_cp =
        proj_ * pointCam;

        // Compute the residuals.
        // [ undistorted - projected ]
        Eigen::Map<Eigen::Matrix<T, 2, 1>> residuals(residuals_ptr);
        residuals[0] =
            observation_.template cast<T>()[0] - p_cp[0] / p_cp[2];
        residuals[1] =
            observation_.template cast<T>()[1] - p_cp[1] / p_cp[2];

        return true;
    }

    static ceres::CostFunction* Create(const Eigen::Matrix4d& camPose, const Eigen::Matrix<double, 3, 4>& proj, const Eigen::Vector2d& observation)
    {
        return (new ceres::AutoDiffCostFunction<MultiViewTriang, 2, 3>(
                        new MultiViewTriang(camPose, proj, observation)));
    }
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
    const Eigen::Matrix4d& camPose_;
    const Eigen::Matrix<double,3,4> proj_;
    const Eigen::Vector2d observation_;

};

// the local bundle adjustment 
// 3D points and poses are optimized
class LocalBundleAdjustment
{
public:

    LocalBundleAdjustment(const Eigen::Matrix3d& K, const Eigen::Vector2d& observation, const double weight)
        : K_(K), observation_(observation), weight_(weight)
    {
    }

    template<typename T>
    bool operator()(const T* const point, const T* const cameraT, const T* const cameraR, T* residuals_ptr) const
    {

        Eigen::Map<const Eigen::Matrix<T, 3, 1>> p_frame(cameraT);
        Eigen::Map<const Eigen::Quaternion<T>> q_frame(cameraR);
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> p_point(point);

        Eigen::Matrix<T, 3, 1> p_cp =
        q_frame * p_point + p_frame;
        // Compute the map point pose in pixel frame.
        Eigen::Matrix<T, 3, 1> projected = K_ * p_cp;

        // Compute the residuals.
        // [ undistorted - projected ]
        Eigen::Map<Eigen::Matrix<T, 2, 1>> residuals(residuals_ptr);
        residuals[0] =
            observation_.template cast<T>()[0] - projected[0] / projected[2];
        residuals[1] =
            observation_.template cast<T>()[1] - projected[1] / projected[2];


        residuals[0] = T(weight_) * residuals[0];
        residuals[1] = T(weight_) * residuals[1];

        return true;
    }

    static ceres::CostFunction* Create(const Eigen::Matrix3d& K, const Eigen::Vector2d& observation, const double weight)
    {
        return (new ceres::AutoDiffCostFunction<LocalBundleAdjustment, 2, 3, 3, 4>(
                        new LocalBundleAdjustment(K, observation, weight)));
    }
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
    const Eigen::Matrix3d& K_;
    const Eigen::Vector2d observation_;
    const double weight_;

};

// the local bundle adjustment for all cameras that are not left front (a transformation from left front to other cameras is needed)
// 3D points and poses are optimized
class LocalBundleAdjustmentR
{
public:

    LocalBundleAdjustmentR(const Eigen::Matrix3d& K, const Eigen::Matrix<double,3,1>& tc1c2, const Eigen::Matrix3d& qc1c2, const Eigen::Vector2d& observation, const double weight)
        : K_(K), tc1c2_(tc1c2), qc1c2_(qc1c2), observation_(observation), weight_(weight)
    {
    }

    template<typename T>
    bool operator()(const T* const point, const T* const cameraT, const T* const cameraR, T* residuals_ptr) const
    {

        Eigen::Map<const Eigen::Matrix<T, 3, 1>> p_frame(cameraT);
        Eigen::Map<const Eigen::Quaternion<T>> q_frame(cameraR);
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> p_point(point);

        // transform the point from left front to the current camera
        Eigen::Matrix<T, 3, 1> p_cp =
        q_frame * p_point + p_frame;
        Eigen::Matrix<T,3,1> pointc1c2 = qc1c2_ * p_cp + tc1c2_;
        // Compute the map point pose in pixel frame.
        Eigen::Matrix<T, 3, 1> projected = K_ * pointc1c2;

        // Compute the residuals.
        // [ undistorted - projected ]
        Eigen::Map<Eigen::Matrix<T, 2, 1>> residuals(residuals_ptr);
        residuals[0] =
            observation_.template cast<T>()[0] - projected[0] / projected[2];
        residuals[1] =
            observation_.template cast<T>()[1] - projected[1] / projected[2];


        residuals[0] = T(weight_) * residuals[0];
        residuals[1] = T(weight_) * residuals[1];

        return true;
    }

    static ceres::CostFunction* Create(const Eigen::Matrix3d& K, const Eigen::Matrix<double,3,1>& tc1c2, const Eigen::Matrix3d& qc1c2, const Eigen::Vector2d& observation, const double weight)
    {
        return (new ceres::AutoDiffCostFunction<LocalBundleAdjustmentR, 2, 3, 3, 4>(
                        new LocalBundleAdjustmentR(K, tc1c2, qc1c2, observation, weight)));
    }
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
    const Eigen::Matrix3d& K_;
    const Eigen::Matrix<double,3,1>& tc1c2_;
    const Eigen::Matrix3d& qc1c2_;
    const Eigen::Vector2d observation_;
    const double weight_;

};

} // namespace DC_VSLAM

#endif