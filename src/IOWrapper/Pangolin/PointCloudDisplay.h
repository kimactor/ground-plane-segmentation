#include <stdio.h>
#include "util/settings.h"

#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>

#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/octree/octree.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/organized.h>
// #include <pcl/filters/conditional_removal.h>
// #include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/features/don.h>
#include <pcl/features/normal_3d_omp.h>


namespace dso
{
	namespace IOWrap

	class PointCloudDisplay
	{
	public:

		void PointCloudDisplay::filterCloud(PointCloud &filter_cloud);

		void PointCloudDisplay::generateMap(PointCloud &temp);

		// void PointCloudDisplay::avoidingObstacle();

		void PointCloudDisplay::quickSort( float tmpPoint[], int low , int high)


	}


}