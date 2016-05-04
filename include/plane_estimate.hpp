#include <pcl/search/kdtree.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>



Eigen::Vector4f EstimatePlane(std::vector<cv::Point3d> Point_Near)
{
	std::vector<int> pointIdxNKNSearch;

	pcl::PointCloud<pcl::PointXYZ>::Ptr pc (new pcl::PointCloud<pcl::PointXYZ>);
	pc->points.resize(Point_Near.size());

	for(size_t i = 0; i < pc->points.size(); i++)
	{
		pc->points[i].x = Point_Near[i].x;
		pc->points[i].y = Point_Near[i].y;
		pc->points[i].z = Point_Near[i].z;

		pointIdxNKNSearch.push_back(i);
	}

	// Create the normal estimation class, and pass the input dataset to it
	Eigen::Vector4f plane_param;
	float curvatura;
  	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
   	ne.computePointNormal(*pc,pointIdxNKNSearch,plane_param,curvatura);
  
  	return plane_param;
}

void Camera::FindBottonPos3D(Eigen::Vector4f plane_param)
{
	//the plane parameters as: a, b, c, d
	float a = plane_param[0];
	float b = plane_param[1];
	float c = plane_param[2];
	float d = plane_param[3];

	Eigen::Vector2d point_temp(BottonCHosen.Botton_2frame.x, BottonCHosen.Botton_2frame.y);
		
	float first_temp = (a*(point_temp[0]-cam_cx))/cam_fx;
	float second_temp = (b*(point_temp[1]-cam_cy))/cam_fy;

	BottonCHosen.Pos3d_.z = -d/ ( first_temp + second_temp + c);
	BottonCHosen.Pos3d_.x = (( (point_temp[0]-cam_cx) * BottonCHosen.Pos3d_.z)/cam_fx);
	BottonCHosen.Pos3d_.y = (( (point_temp[1]-cam_cy) * BottonCHosen.Pos3d_.z)/cam_fy);

	
    myfile1 << BottonCHosen.Pos3d_.z << "\n" ;
    // myfile1.close();
	ROS_INFO_STREAM("IL BOTTONE E': "<<BottonCHosen.Pos3d_);
}