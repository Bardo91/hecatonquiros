//---------------------------------------------------------------------------------------------------------------------
//  HECATONQUIROS
//---------------------------------------------------------------------------------------------------------------------
//  Copyright 2018 ViGUS University of Seville
//---------------------------------------------------------------------------------------------------------------------
//  Permission is hereby granted, free of charge, to any person obtaining a copy of this software 
//  and associated documentation files (the "Software"), to deal in the Software without restriction, 
//  including without limitation the rights to use, copy, modify, merge, publish, distribute, 
//  sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is 
//  furnished to do so, subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all copies or substantial 
//  portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING 
//  BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND 
//  NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES 
//  OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN 
//  CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//---------------------------------------------------------------------------------------------------------------------


#ifdef ENABLE_PCL

#include <hecatonquiros/visualization/PositionerVis.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/PCLPointCloud2.h>


namespace hecatonquiros{
	bool PositionerVis::init(pcl::visualization::PCLVisualizer * _viewer, std::string _folderFiles) {
		mViewer = _viewer;
		if (pcl::io::loadPolygonFileSTL(_folderFiles+"/bar0.stl", mBarMesh0) == 0) {
			PCL_ERROR("Failed to load STL file\n");
			return false;
		}
		if (pcl::io::loadPolygonFileSTL(_folderFiles + "/bar1.stl", mBarMesh1) == 0) {
			PCL_ERROR("Failed to load STL file\n");
			return false;
		}
		if (pcl::io::loadPolygonFileSTL(_folderFiles + "/bar2.stl", mBarMesh2) == 0) {
			PCL_ERROR("Failed to load STL file\n");
			return false;
		}
		if (pcl::io::loadPolygonFileSTL(_folderFiles + "/bar3.stl", mBarMesh3) == 0) {
			PCL_ERROR("Failed to load STL file\n");
			return false;
		}
		if (pcl::io::loadPolygonFileSTL(_folderFiles + "/bar4.stl", mBarMesh4) == 0) {
			PCL_ERROR("Failed to load STL file\n");
			return false;
		}
		if (pcl::io::loadPolygonFileSTL(_folderFiles + "/bar5.stl", mBarMesh5) == 0) {
			PCL_ERROR("Failed to load STL file\n");
			return false;
		}

		return true;
	}

	void PositionerVis::draw(Eigen::Matrix4f  _t0, Eigen::Matrix4f _t1, Eigen::Matrix4f  _t2, Eigen::Matrix4f  _t3, Eigen::Matrix4f  _t4, Eigen::Matrix4f  _t5) {
		pcl::PointCloud<pcl::PointXYZ> pctbar0, pctbar1, pctbar2, pctbar3, pctbar4, pctbar5;
		pcl::fromPCLPointCloud2(mBarMesh0.cloud, pctbar0);
		pcl::fromPCLPointCloud2(mBarMesh1.cloud, pctbar1);
		pcl::fromPCLPointCloud2(mBarMesh2.cloud, pctbar2);
		pcl::fromPCLPointCloud2(mBarMesh3.cloud, pctbar3);
		pcl::fromPCLPointCloud2(mBarMesh4.cloud, pctbar4);
		pcl::fromPCLPointCloud2(mBarMesh5.cloud, pctbar5);

		//std::cout << _t4 << std::endl;
		pcl::transformPointCloud(pctbar0, pctbar0, _t0);
		pcl::transformPointCloud(pctbar1, pctbar1, _t1);
		pcl::transformPointCloud(pctbar2, pctbar2, _t2);
		pcl::transformPointCloud(pctbar3, pctbar3, _t3);
		pcl::transformPointCloud(pctbar4, pctbar4, _t4);
		pcl::transformPointCloud(pctbar5, pctbar5, _t5);


		mViewer->addPolygonMesh<pcl::PointXYZ>(pctbar0.makeShared(), mBarMesh0.polygons, "positioner_bar0", 0);
		mViewer->addPolygonMesh<pcl::PointXYZ>(pctbar1.makeShared(), mBarMesh1.polygons, "positioner_bar1", 0);
		mViewer->addPolygonMesh<pcl::PointXYZ>(pctbar2.makeShared(), mBarMesh2.polygons, "positioner_bar2", 0);
		mViewer->addPolygonMesh<pcl::PointXYZ>(pctbar3.makeShared(), mBarMesh3.polygons, "positioner_bar3", 0);
		mViewer->addPolygonMesh<pcl::PointXYZ>(pctbar4.makeShared(), mBarMesh4.polygons, "positioner_bar4", 0);
		mViewer->addPolygonMesh<pcl::PointXYZ>(pctbar5.makeShared(), mBarMesh5.polygons, "positioner_bar5", 0);

	}
}

#endif
