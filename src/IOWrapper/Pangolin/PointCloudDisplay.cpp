
#include <pangolin/PointCloudDisplay.h>


using namespace pcl;

namespace dso
{
namespace IOWrop
{
	 void PointCloudDisplay::filterCloud(PointCloud &filter_cloud)
	 {
	 	//-------------------------------------------Radius outlier removal method ----------------//
		pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> outrem;
		outrem.setInputCloud(filter_cloud);
		outrem.setRadiusSearch(0.2);
		outrem.setMinNeighborsInRadius (6);
		outrem.filter (*filter_cloud);
		printf("---------------------using radius method remove the outliers is succeed !----------\n");

		//-------------------------------------------statistical outlier removal method------------//
		// pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
		// sor.setInputCloud(cloud);
		// sor.setMeanK(50);
		// sor.setStddevMulThresh(1.0);
		// sor.filter(*filter_cloud);

		//--------------------------------------------conditional outlier removal method ----------// 
		// PointCloud<PointNormal>::Ptr doncloud (new pcl::PointCloud<PointNormal>);
		// PointCloud<PointNormal>::Ptr doncloud_filtered (new pcl::PointCloud<PointNormal>);
		// pcl::copyPointCloud<PointXYZRGB, PointNormal>(*cloud, *doncloud);
		// float threshold = 0.05;
		// pcl::ConditionOr<PointNormal>::Ptr range_cond (new pcl::ConditionOr<PointNormal> ());
		// range_cond->addComparison (pcl::FieldComparison<PointNormal>::ConstPtr (new pcl::FieldComparison<PointNormal> ("curvature", pcl::ComparisonOps::GT, threshold)));
		// pcl::ConditionalRemoval<PointNormal> condrem (range_cond);
		// condrem.setInputCloud (doncloud);
		// condrem.filter (*doncloud_filtered);
		// std::cout << "Filtered Pointcloud: " << doncloud->points.size () << " data points." << std::endl;

		// pcl::copyPointCloud< PointNormal, PointXYZRGB>(*doncloud_filtered, *filter_cloud);

	 }
	 void PointCloudDisplay::generateMap(PointCloud &temp)
	 {
	 	//------------------------passthrough outlier removal method--------///
		pcl::PointCloud<PointXYZRGB>::Ptr pass_cloud(new pcl::PointCloud<PointXYZRGB>());
		float robot_height = 0.5;
		pcl::PassThrough<pcl::PointXYZRGB> pass;
		pass.setInputCloud(temp);
		pass.setFilterFieldName ("y");
		pass.setFilterLimits(mf(1,3),mf(1,3) + robot_height);
		pass.filter (*pass_cloud);
		printf("---------------------print out the points of pass through----------\n");
		// for (size_t i = 0; i < pass_cloud->points.size (); ++i)
	    // std::cerr << "    " << pass_cloud->points[i].x << " " 
	    //                     << pass_cloud->points[i].y << " " 
	    //                     << pass_cloud->points[i].z << std::endl;

		// float robot_height = 1.0;
		printf("---------------------using passthrought method remove outliers is succeed !----------\n");
	//----------------------------projecting the PC to plane is succeed ------------//
		pcl::PointCloud<PointXYZRGB>::Ptr project_cloud(new pcl::PointCloud<PointXYZRGB>());
		if ( pass_cloud->points.size () >0 )
		{
			pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
			coefficients->values.resize (4);
			coefficients->values[0] = 0;
			coefficients->values[1] = 1.0;
			coefficients->values[2] = 0;
			coefficients->values[3] = 0;

			pcl::ProjectInliers<pcl::PointXYZRGB> proj;
			proj.setModelType (pcl::SACMODEL_PLANE);
			proj.setInputCloud (pass_cloud);
			proj.setModelCoefficients (coefficients);
			proj.filter (*project_cloud);
		}
		printf("---------------------projecting the PC to plane is succeed !----------\n");

	 }
	 // void PointCloudDisplay::avoidingObstacle()
	 // {
	 	//------------------------------obstacles avoiding--------------------------//
	// if ( temp->points.size () >0 )
	// {
	// 	pcl_octree.setInputCloud(temp);
	// 	pcl_octree.addPointsFromInputCloud();
	// 	pcl::PointXYZRGB searchPoint;
	// 	searchPoint.x = mf(0,3);
	// 	searchPoint.y = mf(1,3);
	// 	searchPoint.z = mf(2,3);

	// 	std::vector<int>pointIdxRadiusSearch;
	// 	std::vector<float>pointRadiusSquaredDistance;

	// 	float radius = 0.2f;
	// 	std::cout<<"Neighbors within radius search at ("<<searchPoint.x
	// 	<<" "<<searchPoint.y
	// 	<<" "<<searchPoint.z
	// 	<<") with radius="<< radius <<std::endl;
	// 	int obstacle_num = pcl_octree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);
	// 	std::cout << " obstacle_num :" <<obstacle_num <<endl; 
	// 	int obstaclePointsNum = pointIdxRadiusSearch.size();
	// 	Vec3f* tmpObstacleBuffer = new Vec3f[obstaclePointsNum];
	// 	if ( obstacle_num >0)
	// 	{
	// 		for(size_t i =0 ; i < pointIdxRadiusSearch.size(); i++ )
	// 		{
	// 			std::cout<<"    "<<  temp->points[ pointIdxRadiusSearch[i] ].x 
	// 			<<" "<< temp->points[pointIdxRadiusSearch[i] ].y 
	// 			<<" "<< temp->points[pointIdxRadiusSearch[i] ].z 
	// 			<<" (squared distance: "<<pointRadiusSquaredDistance[i] <<")"<<std::endl;
	// 			tmpObstacleBuffer[i][0] = temp->points[ pointIdxRadiusSearch[i] ].x ; 
	// 			tmpObstacleBuffer[i][1] = temp->points[ pointIdxRadiusSearch[i] ].y; 
	// 			tmpObstacleBuffer[i][2] = temp->points[ pointIdxRadiusSearch[i] ].z;
	// 		} 
	// 	}
	// 	if(obstaclePointsNum == 0)
	// 	{
	// 		delete[] tmpObstacleBuffer;
	// 		return true;
	// 	}
	// 	numObstacleGoodPoints = obstaclePointsNum;
	// 	if(numObstacleGoodPoints > numObstaclePoints)
	// 	{
	// 		numObstaclePoints = obstaclePointsNum*1.3;
	// 		obstacleBuffer.Reinitialise(pangolin::GlArrayBuffer, numObstaclePoints, GL_FLOAT, 3, GL_DYNAMIC_DRAW );
	// 	}
	// 	obstacleBuffer.Upload(tmpObstacleBuffer, sizeof(float)*3*numObstacleGoodPoints, 0);
			
	// 	printf("---------------------extracting obstacle is succeed !----------\n");
	// 	obstacleValid = true;
	// 	delete[] tmpObstacleBuffer;
	// }

	 }
	void PointCloudDisplay::quickSort( float tmpPoint[], int low , int high)
	{
		if (low < high)
		{
			float privotKey = tmpPoint[low];
			int forword_ptr = low +1;
			int backword_ptr = high;
			float middleKey =0.0;
			while(forword_ptr < backword_ptr)
			{
				while(forword_ptr < backword_ptr && tmpPoint[backword_ptr] >= privotKey) backword_ptr--;
				if (forword_ptr < backword_ptr)
					tmpPoint[forword_ptr++] = tmpPoint[backword_ptr];
				while(forword_ptr < backword_ptr && tmpPoint[forword_ptr] < privotKey) forword_ptr++;
				if(forword_ptr <backword_ptr)
					tmpPoint[backword_ptr--] = tmpPoint[forword_ptr];
			}
			tmpPoint[forword_ptr] = privotKey; 
			quickSort(tmpPoint, low, forword_ptr - 1);
			quickSort(tmpPoint, forword_ptr + 1, high);
		}

	}


}

}
	

//---------------------------------plane segmentation------------------//
	// int number_of_Points = temp->points.size();
	// float tmpPoint[number_of_Points];
	// float average_height =0;
	// float sum = 0 ;
	// for(size_t i = number_of_Points; i > number_of_Points - 10; i--)
	// {
	// 	sum = sum+ temp->points[i].y;	
	// }
	// average_height = sum /10;
	// std::cerr << "    " << average_height << " " << std::endl;
	// for (int i = 0; i < number_of_Points; i++)
	// {
	// 	tmpPoint[i] = temp->points[i].y; 
	// }
	// printf("test the quicksort!\n");
	// std::cerr << "    " << number_of_Points << " " << std::endl;
	// if ( number_of_Points > 0 )
	// {
	// 	quickSort(tmpPoint, 0, number_of_Points-1);
	// 	for (size_t i = 0; i < number_of_Points; ++i)
	// 	{
	// 		std::cerr << "    " << tmpPoint[i] << " " << std::endl;
	// 	}
			
	// }
	// printf("---------------------------no quicksort-----------");
	// for (size_t i = 0; i < number_of_Points; ++i)
	// std::cerr << "    " << tmpPoint[i] << " " << std::endl;


