#include <pcl/search/kdtree.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>



Eigen::Vector4f Camera::EstimatePlane(std::vector<cv::Point3d> Point_Near)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr pc (new pcl::PointCloud<pcl::PointXYZ>);
	pc->points.resize(Point_Near.size());

	for(size_t i = 0; i < pc->points.size(); i++)
	{
		pc->points[i].x = Point_Near[i].x;
		pc->points[i].y = Point_Near[i].y;
		pc->points[i].z = Point_Near[i].z;
	}

	// pcl::search::KdTree <pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
	// tree->setInputCloud(pc);

	//find a centroid of pointcloud
	// Eigen::Vector4f centroid;
	// pcl::compute3DCentroid(*pc,centroid);

	// ROS_INFO_STREAM("centroid:"<<centroid);

	// int K = 1;	//mi basta un punto 
	std::vector<int> pointIdxNKNSearch;
 // 	std::vector<float> pointNKNSquaredDistance(K);

 	// pcl::PointXYZ pc_centroid;
 	// pc_centroid.x = centroid[0];
 	// pc_centroid.y = centroid[1];
 	// pc_centroid.z = centroid[2];
 	
 	for(int i=0; i< pc->points.size(); i++)
 		pointIdxNKNSearch.push_back(i);

	// tree->nearestKSearch(pc_centroid,K,pointIdxNKNSearch,pointNKNSquaredDistance);

	// ROS_INFO_STREAM("pointIdxNKNSearch: " <<pointIdxNKNSearch[0]);
	// ROS_INFO_STREAM("pointNKNSquaredDistance: "<<pointNKNSquaredDistance[0]);

	// Create the normal estimation class, and pass the input dataset to it
	Eigen::Vector4f plane_param;
	float curvatura;
  	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  	// ne.setInputCloud(pc);
  	// ne.setViewPoint(0,0,0);
  	// ne.setKSearch(pc->points.size());
  	ne.computePointNormal(*pc,pointIdxNKNSearch,plane_param,curvatura);
  	ROS_INFO_STREAM("plane_param: " <<plane_param);
  	ROS_INFO_STREAM("curvatura: "<<curvatura);

  	return plane_param;
}

void Camera::FindBottonPos3D(Eigen::Vector4f plane_param)
{

	//the plane parameters as: a, b, c, d
	float a = plane_param[0];
	float b = plane_param[1];
	float c = plane_param[2];
	float d = plane_param[3];

	// ROS_INFO_STREAM("plane_param: " << plane_param);
	Eigen::Vector2d point_temp(BottonCHosen.Center_.x, BottonCHosen.Center_.y);
	// Eigen::Vector3d 3d_temp;
	ROS_INFO_STREAM("BOTTONE 2S X: " << point_temp[0]);
	ROS_INFO_STREAM("BOTTONE 2S y: " << point_temp[1]);
		
	float first_temp = (a*(point_temp[0]-cam_cx))/cam_fx;
	float second_temp = (b*(point_temp[1]-cam_cy))/cam_fy;

	// ROS_INFO_STREAM("first_temp:"<< first_temp);
	// ROS_INFO_STREAM("second_temp: "<< second_temp);

	BottonCHosen.Pos3d_.z = -d/ ( first_temp + second_temp + c);
	BottonCHosen.Pos3d_.x = (( (point_temp[0]-cam_cx) * BottonCHosen.Pos3d_.z)/cam_fx);
	BottonCHosen.Pos3d_.y = (( (point_temp[1]-cam_cy) * BottonCHosen.Pos3d_.z)/cam_fy);


	ROS_INFO_STREAM("IL BOTTONE E': "<<BottonCHosen.Pos3d_);






}