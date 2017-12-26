/**
* This file is part of DSO.
* 
* Copyright 2016 Technical University of Munich and Intel.
* Developed by Jakob Engel <engelj at in dot tum dot de>,
* for more information see <http://vision.in.tum.de/dso>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* DSO is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* DSO is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with DSO. If not, see <http://www.gnu.org/licenses/>.
*/



#include <stdio.h>
#include "util/settings.h"
// #include <vector> 

//#include <GL/glx.h>
//#include <GL/gl.h>
//#include <GL/glu.h>
// #include <octomap/octomap.h>
// #include <octomap/ColorOcTree.h>
// #include <octomap/math/Pose6D.h>
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

#include <pangolin/pangolin.h>
#include "KeyFrameDisplay.h"
#include "FullSystem/HessianBlocks.h"
#include "FullSystem/ImmaturePoint.h"
#include "util/FrameShell.h"
//#include "util/settings.h"
//#include "util/ParameterReader.h"
#include "util/DatasetReader.h"

// #include <Eigen/Core>
// #include <Eigen/Geometry> 
using namespace pcl;
//--------------------------------define tree------------------// 

pcl::PointCloud<PointXYZRGB>::Ptr save_cloud (new pcl::PointCloud<PointXYZRGB>);
// pcl::search::Search<PointXYZRGB>::Ptr tree;
// float resolution_of_octree = 0.02;
// pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB> pcl_octree (resolution_of_octree);
//------------------------------------------------------------//

namespace dso
{
namespace IOWrap
{

KeyFrameDisplay::KeyFrameDisplay()
{
	originalInputSparse = 0;
	numSparseBufferSize=0;
	numSparsePoints=0;

	id = 0;
	active= true;
	camToWorld = SE3();

	needRefresh=true;

	my_scaledTH =1e10;
	my_absTH = 1e10;
	my_displayMode = 1;
	my_minRelBS = 0;
	my_sparsifyFactor = 1;

	numGLBufferPoints=0;
	numMapGLBufferPoints = 0;
	// numObstaclePoints = 0 ;
	bufferValid = false;
	mapbufferValid = false;
	// obstacleValid = false;

}
void KeyFrameDisplay::setFromF(FrameShell* frame, CalibHessian* HCalib)
{
	id = frame->id;
	fx = HCalib->fxl();
	fy = HCalib->fyl();
	cx = HCalib->cxl();
	cy = HCalib->cyl();
	width = wG[0];
	height = hG[0];
	fxi = 1/fx;
	fyi = 1/fy;
	cxi = -cx / fx;
	cyi = -cy / fy;
	camToWorld = frame->camToWorld;
	needRefresh=true;
}

void KeyFrameDisplay::setFromKF(FrameHessian* fh, CalibHessian* HCalib)
{
	setFromF(fh->shell, HCalib);

	// add all traces, inlier and outlier points.
	int npoints = 	fh->immaturePoints.size() +
					fh->pointHessians.size() +
					fh->pointHessiansMarginalized.size() +
					fh->pointHessiansOut.size();

	if(numSparseBufferSize < npoints)
	{
		if(originalInputSparse != 0) delete originalInputSparse;
		numSparseBufferSize = npoints+100;
        originalInputSparse = new InputPointSparse<MAX_RES_PER_POINT>[numSparseBufferSize];
	}

    InputPointSparse<MAX_RES_PER_POINT>* pc = originalInputSparse;
	numSparsePoints=0;
	for(ImmaturePoint* p : fh->immaturePoints)
	{
		for(int i=0;i<patternNum;i++)
			pc[numSparsePoints].color[i] = p->color[i];

		pc[numSparsePoints].u = p->u;
		pc[numSparsePoints].v = p->v;
		pc[numSparsePoints].idpeth = (p->idepth_max+p->idepth_min)*0.5f;
		pc[numSparsePoints].idepth_hessian = 1000;
		pc[numSparsePoints].relObsBaseline = 0;
		pc[numSparsePoints].numGoodRes = 1;
		pc[numSparsePoints].status = 0;
		numSparsePoints++;
	}

	for(PointHessian* p : fh->pointHessians)
	{
		for(int i=0;i<patternNum;i++)
			pc[numSparsePoints].color[i] = p->color[i];
		pc[numSparsePoints].u = p->u;
		pc[numSparsePoints].v = p->v;
		pc[numSparsePoints].idpeth = p->idepth_scaled;
		pc[numSparsePoints].relObsBaseline = p->maxRelBaseline;
		pc[numSparsePoints].idepth_hessian = p->idepth_hessian;
		pc[numSparsePoints].numGoodRes =  0;
		pc[numSparsePoints].status=1;

		numSparsePoints++;
	}

	for(PointHessian* p : fh->pointHessiansMarginalized)
	{
		for(int i=0;i<patternNum;i++)
			pc[numSparsePoints].color[i] = p->color[i];
		pc[numSparsePoints].u = p->u;
		pc[numSparsePoints].v = p->v;
		pc[numSparsePoints].idpeth = p->idepth_scaled;
		pc[numSparsePoints].relObsBaseline = p->maxRelBaseline;
		pc[numSparsePoints].idepth_hessian = p->idepth_hessian;
		pc[numSparsePoints].numGoodRes =  0;
		pc[numSparsePoints].status=2;
		numSparsePoints++;
	}

	for(PointHessian* p : fh->pointHessiansOut)
	{
		for(int i=0;i<patternNum;i++)
			pc[numSparsePoints].color[i] = p->color[i];
		pc[numSparsePoints].u = p->u;
		pc[numSparsePoints].v = p->v;
		pc[numSparsePoints].idpeth = p->idepth_scaled;
		pc[numSparsePoints].relObsBaseline = p->maxRelBaseline;
		pc[numSparsePoints].idepth_hessian = p->idepth_hessian;
		pc[numSparsePoints].numGoodRes =  0;
		pc[numSparsePoints].status=3;
		numSparsePoints++;
	}
	assert(numSparsePoints <= npoints);

	camToWorld = fh->PRE_camToWorld;

	needRefresh=true;
}


KeyFrameDisplay::~KeyFrameDisplay()
{
	if(originalInputSparse != 0)
		delete[] originalInputSparse;
}

bool KeyFrameDisplay::refreshPC(bool canRefresh, float scaledTH, float absTH, int mode, float minBS, int sparsity)
{
	if(canRefresh)
	{
		needRefresh = needRefresh ||
				my_scaledTH != scaledTH ||
				my_absTH != absTH ||
				my_displayMode != mode ||
				my_minRelBS != minBS ||
				my_sparsifyFactor != sparsity;
	}

	if(!needRefresh) return false;
	needRefresh=false;

	my_scaledTH = scaledTH;
	my_absTH = absTH;
	my_displayMode = mode;
	my_minRelBS = minBS;
	my_sparsifyFactor = sparsity;


	// if there are no vertices, done!
	if(numSparsePoints == 0)
		return false;

	// make data
	Vec3f* tmpVertexBuffer = new Vec3f[numSparsePoints*patternNum];
	Vec3b* tmpColorBuffer = new Vec3b[numSparsePoints*patternNum];
	// octomap::Pointcloud cloud_octo;
	pcl::PointCloud<PointXYZRGB>::Ptr cloud (new pcl::PointCloud<PointXYZRGB>);
    
	int vertexBufferNumPoints=0;

	for(int i=0;i<numSparsePoints;i++)
	{
		/* display modes:
		 * my_displayMode==0 - all pts, color-coded
		 * my_displayMode==1 - normal points
		 * my_displayMode==2 - active only
		 * my_displayMode==3 - nothing
		 */

		if(my_displayMode==1 && originalInputSparse[i].status != 1 && originalInputSparse[i].status!= 2) continue;
		if(my_displayMode==2 && originalInputSparse[i].status != 1) continue;
		if(my_displayMode>2) continue;

		if(originalInputSparse[i].idpeth < 0) continue;


		float depth = 1.0f / originalInputSparse[i].idpeth;
		float depth4 = depth*depth; depth4*= depth4;
		float var = (1.0f / (originalInputSparse[i].idepth_hessian+0.01));

		if(var * depth4 > my_scaledTH)
			continue;

		if(var > my_absTH)
			continue;

		if(originalInputSparse[i].relObsBaseline < my_minRelBS)
			continue;


		for(int pnt=0;pnt<patternNum;pnt++)
		{
			pcl::PointXYZRGB p;
		
			if(my_sparsifyFactor > 1 && rand()%my_sparsifyFactor != 0) continue;
			int dx = patternP[pnt][0];
			int dy = patternP[pnt][1];

			tmpVertexBuffer[vertexBufferNumPoints][0] = ((originalInputSparse[i].u+dx)*fxi + cxi) * depth;
			tmpVertexBuffer[vertexBufferNumPoints][1] = ((originalInputSparse[i].v+dy)*fyi + cyi) * depth;
			tmpVertexBuffer[vertexBufferNumPoints][2] = depth*(1 + 2*fxi * (rand()/(float)RAND_MAX-0.5f));
			
			if(my_displayMode==0)
			{
				if(originalInputSparse[i].status==0)
				{
					tmpColorBuffer[vertexBufferNumPoints][0] = 0;
					tmpColorBuffer[vertexBufferNumPoints][1] = 255;
					tmpColorBuffer[vertexBufferNumPoints][2] = 255;
				}
				else if(originalInputSparse[i].status==1)
				{
					tmpColorBuffer[vertexBufferNumPoints][0] = 0;
					tmpColorBuffer[vertexBufferNumPoints][1] = 255;
					tmpColorBuffer[vertexBufferNumPoints][2] = 0;
				}
				else if(originalInputSparse[i].status==2)
				{
					tmpColorBuffer[vertexBufferNumPoints][0] = 0;
					tmpColorBuffer[vertexBufferNumPoints][1] = 0;
					tmpColorBuffer[vertexBufferNumPoints][2] = 255;
				}
				else if(originalInputSparse[i].status==3)
				{
					tmpColorBuffer[vertexBufferNumPoints][0] = 255;
					tmpColorBuffer[vertexBufferNumPoints][1] = 0;
					tmpColorBuffer[vertexBufferNumPoints][2] = 0;
				}
				else
				{
					tmpColorBuffer[vertexBufferNumPoints][0] = 255;
					tmpColorBuffer[vertexBufferNumPoints][1] = 255;
					tmpColorBuffer[vertexBufferNumPoints][2] = 255;
				}

			}
			else
			{
				tmpColorBuffer[vertexBufferNumPoints][0] = originalInputSparse[i].color[pnt];
				tmpColorBuffer[vertexBufferNumPoints][1] = originalInputSparse[i].color[pnt];
				tmpColorBuffer[vertexBufferNumPoints][2] = originalInputSparse[i].color[pnt];
			}

			p.x = tmpVertexBuffer[vertexBufferNumPoints][0]; 
			p.y = tmpVertexBuffer[vertexBufferNumPoints][1]; 
			p.z = tmpVertexBuffer[vertexBufferNumPoints][2];
			p.r = tmpColorBuffer[vertexBufferNumPoints][0];
			p.g = tmpColorBuffer[vertexBufferNumPoints][1];
			p.b = tmpColorBuffer[vertexBufferNumPoints][2];
			cloud->points.push_back( p ); 



			vertexBufferNumPoints++;
			assert(vertexBufferNumPoints <= numSparsePoints*patternNum);
		}
	}


	// pcl_octree.setInputCloud (temp);
	// pcl_octree.addPointsFromInputCloud ();
//----------------------------------tree reset----------------------------------//
	// if (cloud->isOrganized ())
	// {
	// 	tree.reset (new pcl::search::OrganizedNeighbor<PointXYZRGB> ());
	// }
	// else
	// {
	// 	tree.reset (new pcl::search::KdTree<PointXYZRGB> (false));
	// }
	// tree->setInputCloud(cloud);

//-------------------------------------------statistical outlier removal method------------//
	// pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
	// sor.setInputCloud(cloud);
	// sor.setMeanK(50);
	// sor.setStddevMulThresh(1.0);
	// sor.filter(*filter_cloud);

//-------------------------------------------Radius outlier removal method ----------------//
	pcl::PointCloud<PointXYZRGB>::Ptr filter_cloud(new pcl::PointCloud<PointXYZRGB>());
	pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> outrem;
	outrem.setInputCloud(cloud);
	outrem.setRadiusSearch(0.2);
	outrem.setMinNeighborsInRadius (6);
	outrem.filter (*filter_cloud);
	printf("---------------------using radius method remove the outliers is succeed !----------\n");
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
	
//----------------------------------transform the point cloud to world coodiration------------------//
	
	pcl::PointCloud<PointXYZRGB>::Ptr temp(new pcl::PointCloud<PointXYZRGB>());
	Sophus::Matrix4f mf = camToWorld.matrix().cast<float>();
	pcl::transformPointCloud(*filter_cloud, *temp, camToWorld.matrix().cast<float>());
	printf("---------------------transformation is succeed !----------\n");
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
//-----------------------------save the point cloud-------------------//
	for (auto p:project_cloud->points)
	{
		save_cloud->points.push_back( p );	
	}
	// pcl::copyPointCloud< PointXYZRGB, PointXYZRGB>(*pass_cloud, *save_cloud);
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

//------------------------------save pc in vertexbuffer-------------------------//
	vertexBufferNumPoints = filter_cloud->points.size();
	// std::cerr<<" vertexBufferNumPoints:"<< vertexBufferNumPoints <<"   "<<endl;
	for (size_t i = 0; i < vertexBufferNumPoints; ++i)
	{
		tmpVertexBuffer[i][0] = filter_cloud->points[i].x; 
		tmpVertexBuffer[i][1] = filter_cloud->points[i].y; 
		tmpVertexBuffer[i][2] = filter_cloud->points[i].z;
		tmpColorBuffer[i][0] = filter_cloud->points[i].r;
		tmpColorBuffer[i][1] = filter_cloud->points[i].g;
		tmpColorBuffer[i][2] = filter_cloud->points[i].b;
	}
		
	if(vertexBufferNumPoints==0)
	{
		delete[] tmpColorBuffer;
		delete[] tmpVertexBuffer;
		return true;
	}
	numGLBufferGoodPoints = vertexBufferNumPoints;
	if(numGLBufferGoodPoints > numGLBufferPoints)
	{
		numGLBufferPoints = vertexBufferNumPoints*1.3;
		vertexBuffer.Reinitialise(pangolin::GlArrayBuffer, numGLBufferPoints, GL_FLOAT, 3, GL_DYNAMIC_DRAW );
		colorBuffer.Reinitialise(pangolin::GlArrayBuffer, numGLBufferPoints, GL_UNSIGNED_BYTE, 3, GL_DYNAMIC_DRAW );
	}
	vertexBuffer.Upload(tmpVertexBuffer, sizeof(float)*3*numGLBufferGoodPoints, 0);
	colorBuffer.Upload(tmpColorBuffer, sizeof(unsigned char)*3*numGLBufferGoodPoints, 0);
	printf("---------------------save pc in vertexbuffer is succeed !----------\n");
//---------------------------------- save map in mapvertexbuffer ---------------------------------//

	int mapvertexBufferNumPoints = project_cloud->points.size();
	Vec3f* tmpMapVertexBuffer = new Vec3f[mapvertexBufferNumPoints];
	Vec3f* tmpMapColorBuffer = new Vec3f[mapvertexBufferNumPoints];
	// std::cerr<<" mapvertexBufferNumPoints:"<< mapvertexBufferNumPoints <<"   "<<endl;
	for (size_t i = 0; i < mapvertexBufferNumPoints; ++i)
	{
		tmpMapVertexBuffer[i][0] = project_cloud->points[i].x;
		tmpMapVertexBuffer[i][1] = project_cloud->points[i].y;
		tmpMapVertexBuffer[i][2] = project_cloud->points[i].z;
		tmpMapColorBuffer[i][0] = project_cloud->points[i].r;
		tmpMapColorBuffer[i][1] = project_cloud->points[i].g;
		tmpMapColorBuffer[i][2] = project_cloud->points[i].b;				
	}

	if(mapvertexBufferNumPoints == 0)
	{
		delete[] tmpMapColorBuffer;
		delete[] tmpMapVertexBuffer;
		return true;
	}

	numMapGLBufferGoodPoints = mapvertexBufferNumPoints;
	if(numMapGLBufferGoodPoints > numMapGLBufferPoints)
	{
		numMapGLBufferPoints = mapvertexBufferNumPoints*1.3;
		mapvertexBuffer.Reinitialise(pangolin::GlArrayBuffer, numMapGLBufferPoints, GL_FLOAT, 3, GL_DYNAMIC_DRAW );
		mapcolorBuffer.Reinitialise(pangolin::GlArrayBuffer, numMapGLBufferPoints, GL_UNSIGNED_BYTE, 3, GL_DYNAMIC_DRAW );
	}
	mapvertexBuffer.Upload(tmpMapVertexBuffer, sizeof(float)*3*numMapGLBufferGoodPoints, 0);
	mapcolorBuffer.Upload(tmpMapColorBuffer, sizeof(unsigned char)*3*numMapGLBufferGoodPoints, 0);
	printf("---------------------save map in mapvertexbuffer is succeed !----------\n");

	bufferValid = true;
	mapbufferValid = true;
	
	
	delete[] tmpColorBuffer;
	delete[] tmpVertexBuffer;
	delete[] tmpMapVertexBuffer;
	delete[] tmpMapColorBuffer;

	return true;
}

void KeyFrameDisplay::quickSort( float tmpPoint[], int low , int high)
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
// void KeyFrameDisplay::generateMap(Vec3f* tmpVertexBuffer, Vec3b* tmpColorBuffer)
// {

// }

void KeyFrameDisplay::savePC (bool setting_savepcdfileRequested ) 
{
	ParameterReader pard;
	if ( setting_savepcdfileRequested )
	{

		std::string filename = pard.getData( "filename" );
		std::string path_of_savePCDdir =  filename + "/PCD_results/";
		if (NULL == opendir( path_of_savePCDdir.c_str()))
		{
			mkdir( path_of_savePCDdir.c_str(), 0755 );
			printf( " mkdir path of save PCDdir succeed!");
		}
		std::string path_of_savePCDFile = filename + "/PCD_results/" + filename.substr( filename.find_last_of( "/\\" ) + 1 ) + ".pcd";
		printf( "start saving the points cloud to PCD file!\n" );
		pcl::io::savePCDFileBinary( path_of_savePCDFile , *save_cloud );
		printf("Finished save Points cloud !\n");
		setting_savepcdfileRequested = false;
	}
}



void KeyFrameDisplay::drawCam(float lineWidth, float* color, float sizeFactor)
{
	if(width == 0)
		return;

	float sz=sizeFactor;

	glPushMatrix();

		Sophus::Matrix4f m = camToWorld.matrix().cast<float>();
		glMultMatrixf((GLfloat*)m.data());

		if(color == 0)
		{
			glColor3f(1,0,0);
		}
		else
			glColor3f(color[0],color[1],color[2]);

		glLineWidth(lineWidth);
		glBegin(GL_LINES);
		glVertex3f(0,0,0);
		glVertex3f(sz*(0-cx)/fx,sz*(0-cy)/fy,sz);
		glVertex3f(0,0,0);
		glVertex3f(sz*(0-cx)/fx,sz*(height-1-cy)/fy,sz);
		glVertex3f(0,0,0);
		glVertex3f(sz*(width-1-cx)/fx,sz*(height-1-cy)/fy,sz);
		glVertex3f(0,0,0);
		glVertex3f(sz*(width-1-cx)/fx,sz*(0-cy)/fy,sz);

		glVertex3f(sz*(width-1-cx)/fx,sz*(0-cy)/fy,sz);
		glVertex3f(sz*(width-1-cx)/fx,sz*(height-1-cy)/fy,sz);

		glVertex3f(sz*(width-1-cx)/fx,sz*(height-1-cy)/fy,sz);
		glVertex3f(sz*(0-cx)/fx,sz*(height-1-cy)/fy,sz);

		glVertex3f(sz*(0-cx)/fx,sz*(height-1-cy)/fy,sz);
		glVertex3f(sz*(0-cx)/fx,sz*(0-cy)/fy,sz);

		glVertex3f(sz*(0-cx)/fx,sz*(0-cy)/fy,sz);
		glVertex3f(sz*(width-1-cx)/fx,sz*(0-cy)/fy,sz);

		glEnd();
	glPopMatrix();
}


void KeyFrameDisplay::drawPC(float pointSize)
{

	if(!bufferValid || numGLBufferGoodPoints==0)
		return;


	glDisable(GL_LIGHTING);

	glPushMatrix();

		Sophus::Matrix4f m = camToWorld.matrix().cast<float>();
		glMultMatrixf((GLfloat*)m.data());

		glPointSize(pointSize);

		colorBuffer.Bind();
		glColorPointer(colorBuffer.count_per_element, colorBuffer.datatype, 0, 0);
		glEnableClientState(GL_COLOR_ARRAY);

		vertexBuffer.Bind();
		glVertexPointer(vertexBuffer.count_per_element, vertexBuffer.datatype, 0, 0);
		glEnableClientState(GL_VERTEX_ARRAY);
		glDrawArrays(GL_POINTS, 0, numGLBufferGoodPoints);
		
		glDisableClientState(GL_VERTEX_ARRAY);
		vertexBuffer.Unbind();

		glDisableClientState(GL_COLOR_ARRAY);
		colorBuffer.Unbind();

	glPopMatrix();
}

void KeyFrameDisplay::drawMap(float pointSize)
{

	if(!mapbufferValid || numMapGLBufferGoodPoints==0 )
		return;


	glDisable(GL_LIGHTING);

	glPushMatrix();

		// Sophus::Matrix4f m = camToWorld.matrix().cast<float>();
		// glMultMatrixf((GLfloat*)m.data());

		glPointSize(pointSize);

		colorBuffer.Bind();
		glColorPointer(colorBuffer.count_per_element, colorBuffer.datatype, 0, 0);
		glEnableClientState(GL_COLOR_ARRAY);

		mapvertexBuffer.Bind();
		glVertexPointer(mapvertexBuffer.count_per_element, mapvertexBuffer.datatype, 0, 0);
		glEnableClientState(GL_VERTEX_ARRAY);
		glDrawArrays(GL_POINTS, 0, numMapGLBufferGoodPoints);
		
		glDisableClientState(GL_VERTEX_ARRAY);
		mapvertexBuffer.Unbind();

		glDisableClientState(GL_COLOR_ARRAY);
		colorBuffer.Unbind();

	glPopMatrix();
}

// void KeyFrameDisplay::drawObstacle(float pointSize)
// {

// 	if(!obstacleValid || numObstacleGoodPoints==0 )
// 		return;


// 	glDisable(GL_LIGHTING);

// 	glPushMatrix();

// 		// Sophus::Matrix4f m = camToWorld.matrix().cast<float>();
// 		// glMultMatrixf((GLfloat*)m.data());

// 		glPointSize(pointSize);

// 		colorBuffer.Bind();
// 		glColorPointer(colorBuffer.count_per_element, colorBuffer.datatype, 0, 0);
// 		glEnableClientState(GL_COLOR_ARRAY);

// 		obstacleBuffer.Bind();
// 		glVertexPointer(obstacleBuffer.count_per_element, obstacleBuffer.datatype, 0, 0);
// 		glEnableClientState(GL_VERTEX_ARRAY);
// 		glDrawArrays(GL_POINTS, 0, numObstacleGoodPoints);
		
// 		glDisableClientState(GL_VERTEX_ARRAY);
// 		obstacleBuffer.Unbind();

// 		glDisableClientState(GL_COLOR_ARRAY);
// 		colorBuffer.Unbind();

// 	glPopMatrix();
// }


}
}
