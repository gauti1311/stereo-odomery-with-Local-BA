#include<algorithm>
#include "optimizer.h"


void optimize(std::deque<cv::Mat>& points3D, std::deque<std::vector<cv::Point2f>> pointsLeft, std::deque<std::array<double,6>>& camera_poses, const cv::Mat& intrinsics)
{
    ceres::Problem problem;
    ceres::LossFunction *loss_function = new ceres::HuberLoss(1.0);
    static const int pointSize  = 3;
    static const int cameraSize = 6;
    double* camPoses = new double[6];
    std::vector<double*> allPoints;
    std::vector<double*> allPoses;

    for(int i= 0; i < pointsLeft.size(); i++ )
    {
        double* point = new double[3*pointsLeft[i].size()];
        auto points = points3D[i].col(0);     
        camPoses = &(camera_poses[i][0]);     

        for(int j= 0; j < pointsLeft[i].size(); j++ )
        {
		    point[3*j+0] = points.at<cv::Vec3f>(j,0)(0);
		    point[3*j+1] = points.at<cv::Vec3f>(j,0)(1);
		    point[3*j+2] = points.at<cv::Vec3f>(j,0)(2);
        }
        allPoints.push_back(point);
        allPoses.push_back(camPoses);
    }

    for(int i= 0; i < pointsLeft.size(); i++ )
    {
        auto pts = allPoints[i]; 
        for(int j= 0; j < pointsLeft[i].size(); j++ )
        {
            double* pt = new double[3];
		    pt[0] = pts[3*j+0];;
		    pt[1] = pts[3*j+1];
		    pt[2] = pts[3*j+2];
            //std::cout << " point_old " << point[0] << " " << point[1] << " "<<point[2] << std::endl; 
            ceres::CostFunction* costFunction = new ceres::AutoDiffCostFunction<ReprojectionError, 2, cameraSize, pointSize>(new ReprojectionError(pointsLeft[i][j].x, pointsLeft[i][j].y, intrinsics));
            problem.AddResidualBlock (costFunction, nullptr /* squared loss */,allPoses[i], pt);   
        } 
    }
    ceres::Solver::Options options;
    ceres::Solver::Summary summary;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.minimizer_progress_to_stdout = true;
    options.max_num_iterations = 30;

    ceres::Solve(options, &problem, &summary);
    if(options.minimizer_progress_to_stdout)
			std::cout << summary.FullReport() << std::endl;

    auto newPose = allPoses.back(); 

    camera_poses[camera_poses.size()-1][0] = newPose[0];
    camera_poses[camera_poses.size()-1][1] = newPose[1];
    camera_poses[camera_poses.size()-1][2] = newPose[2];
    camera_poses[camera_poses.size()-1][3] = newPose[3];
    camera_poses[camera_poses.size()-1][4] = newPose[4];
    camera_poses[camera_poses.size()-1][5] = newPose[5];

    
    /*int n = Points3D.size()* pointsLeft[i].size(); 
    double* Points3Dset = new double[];

        for(int j=0 ; j < pointsLeft[i].size() ; ++j)
        {
            Points3D[i].x    = point[pointSize*j + 0];
            Points3DSet[i].y = point[pointSize*j + 1];
            Points3DSet[i].z = point[pointSize*j + 2];

            std::cout << " does 3d point loop work ?" << std::endl;
            std::cout << " z pose: "<< Points3DSet[i].z << std::endl;
        }
	*/
}
