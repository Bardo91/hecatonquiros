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


#include <arm_controller/visualization/UavVis.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/PCLPointCloud2.h>

namespace hecatonquiros{
	bool UavVis::init(pcl::visualization::PCLVisualizer * _viewer, std::string _folderFiles) {
		mViewer = _viewer;
		if (pcl::io::loadPolygonFileSTL(_folderFiles + "/uav.stl", mUavMesh) == 0) {
			PCL_ERROR("Failed to load STL file\n");
			return false;
		}

		return true;
	}

	void UavVis::draw(Eigen::Matrix4f _t1) {
		pcl::PointCloud<pcl::PointXYZ> pcuav;
		pcl::fromPCLPointCloud2(mUavMesh.cloud, pcuav);

		pcl::transformPointCloud(pcuav, pcuav, _t1);

		mViewer->addPolygonMesh<pcl::PointXYZ>(pcuav.makeShared(), mUavMesh.polygons, "uav", 0);
		
	}
}