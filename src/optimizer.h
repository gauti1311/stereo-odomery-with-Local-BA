#pragma once
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include "visualOdometry.h"
/* 
template<int PoseBlockSize>
class PoseSE3Parameterization : public ceres::LocalParameterization {
public:
    PoseSE3Parameterization() {}
    virtual ~PoseSE3Parameterization() {}
    virtual bool Plus(const double* x,
                      const double* delta,
                      double* x_plus_delta) const;
    virtual bool ComputeJacobian(const double* x,
                                 double* jacobian) const;
    virtual int GlobalSize() const { return PoseBlockSize; }
    virtual int LocalSize() const { return 6; }
};  */

class ReprojectionError {
public:
    ReprojectionError(double _observed_x, double _observed_y, const cv::Mat& _intrinsics) :
            observed_x(_observed_x), observed_y(_observed_y), intrinsics(_intrinsics) {}

    template <typename T>
    bool operator()(const T* const camera, const T* const point, T* residuals) const
    {
        T p[3];

        //std::cout << " point " << *point <<" "<< *(point+1) << " "<< *(point+2) << std::endl; 

        ceres::AngleAxisRotatePoint(camera, point, p);
        
        p[0] += camera[3]; p[1] += camera[4]; p[2] += camera[5];
        // std::cout << " p_ " << "  "<< camera[3] <<"  " << camera[4] << "  " << camera[5] << std::endl; 
        T predicted_x = intrinsics.at<double>(0, 0) * p[0] / p[2] + intrinsics.at<double>(0, 2);
        T predicted_y = intrinsics.at<double>(1, 1)* p[1] / p[2]  + intrinsics.at<double>(1, 2);

        //std::cout << " p_ " << *p << std::endl; 
        //std::cout<< " xp: " << predicted_x << " yp : " << predicted_y << std::endl;
        // std::cout<< " obs_x: " << observed_x << " obs_y : " << observed_y << std::endl;
        
        residuals[0] = predicted_x - observed_x;
        residuals[1] = predicted_y - observed_y;
        //std::cout << " predicted : "<< predicted_x << "," << predicted_y << " vs. observed : " << observed_x << "," << observed_y << std::endl;
        //std::cout << " -------- new line ----- " << std::endl;
        return true;
        
    }

private:
    double observed_x, observed_y;
    const cv::Mat intrinsics;
};

void optimize(std::deque<cv::Mat>& points3D, std::deque<std::vector<cv::Point2f>> pointsLeft, std::deque<std::array<double,6>>& camera_poses, const cv::Mat& intrinsics);  

void solver(ceres::Solver::Options options, ceres::Problem& problem, ceres::Solver::Summary& summary)
{
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.minimizer_progress_to_stdout = true;
    options.max_num_iterations = 10;

    ceres::Solve(options, &problem, &summary);
    if(options.minimizer_progress_to_stdout)
			std::cout << summary.FullReport() << std::endl;
}