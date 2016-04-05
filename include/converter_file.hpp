void FromMatToEigen(cv::Mat Mat_, Eigen::MatrixXd &Eigen1)
{

	for( int i=0; i < Mat_.rows; i++)
	{
		for( int j =0; j < Mat_.cols ; j++)
		{
			Eigen1(i,j) = Mat_.at<float>(i,j);
		}
	}

	Eigen1(3,0) = 0;
	Eigen1(3,1) = 0;
	Eigen1(3,2) = 0;
	Eigen1(3,3) = 1;
}

void FromCvPointToEigen(cv::Point3d point_, Eigen::VectorXd &vect)
{
	vect[0] = point_.x;
	vect[1] = point_.y;
	vect[2] = point_.z;
	vect[3] = 1;
}


void FromEigenVectorToCvPOint(Eigen::VectorXd Eigen, cv::Point3d &mat)
{
	mat.x = Eigen[0];
	mat.y = Eigen[1];
	mat.z = Eigen[2];
}
