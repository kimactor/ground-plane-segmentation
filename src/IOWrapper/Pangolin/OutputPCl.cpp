#include <iostream>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/octree/octree.h>
namespace dso
{
	class OutputPCL 
	{
	public:
		void OutputPCL::addPointsToCloud ( Vec3f* tmpVertexBuffer ,Vec3f* tmpColorBuffer)
		{
		
			pcl::PointCloud<pcl::PointXYZRGBA> cloud;

			cloud->points.x= tmpVertexBuffer[vertexBufferNumPoints][0]; 
			cloud->points.y = tmpVertexBuffer[vertexBufferNumPoints][1]; 
			cloud->points.z = tmpVertexBuffer[vertexBufferNumPoints][2];

			cloud->points.r = tmpColorBuffer[vertexBufferNumPoints][0];
			cloud->points.g = tmpColorBuffer[vertexBufferNumPoints][1];
			cloud->points.b = tmpColorBuffer[vertexBufferNumPoints][2];

			return 
			
		}





		


	}
	
}
