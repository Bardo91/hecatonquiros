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


#include <arm_controller/visualization/ArmVis.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/PCLPointCloud2.h>

namespace hecatonquiros{
	bool ArmVis::init(pcl::visualization::PCLVisualizer * _viewer, std::string _folderFiles) {
		mViewer = _viewer;
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

		return true;
	}

	void ArmVis::draw(Eigen::Matrix4f  _t1, Eigen::Matrix4f  _t2, Eigen::Matrix4f _t3) {

		pcl::PointCloud<pcl::PointXYZ> pctbar1, pctbar2, pctbar3;
		pcl::fromPCLPointCloud2(mBarMesh1.cloud, pctbar1);
		for(auto &p: pctbar1) { p.x /=1000; p.y /=1000;  p.z /=1000;}
		pcl::fromPCLPointCloud2(mBarMesh2.cloud, pctbar2);
		for(auto &p: pctbar2) { p.x /=1000; p.y /=1000;  p.z /=1000;}
		pcl::fromPCLPointCloud2(mBarMesh3.cloud, pctbar3);
		for(auto &p: pctbar3) { p.x /=1000; p.y /=1000;  p.z /=1000;}

		pcl::transformPointCloud(pctbar1, pctbar1, _t1);
		pcl::transformPointCloud(pctbar2, pctbar2, _t2);
		pcl::transformPointCloud(pctbar3, pctbar3, _t3);

		mViewer->addPolygonMesh<pcl::PointXYZ>(pctbar1.makeShared(), mBarMesh1.polygons, "arm_bar1", 0);
		mViewer->addPolygonMesh<pcl::PointXYZ>(pctbar2.makeShared(), mBarMesh2.polygons, "arm_bar2", 0);
		mViewer->addPolygonMesh<pcl::PointXYZ>(pctbar3.makeShared(), mBarMesh3.polygons, "arm_bar3", 0);
	}
}