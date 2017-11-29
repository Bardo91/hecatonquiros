//
//
//
//

#include <arm_controller/visualization/UavVis.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/PCLPointCloud2.h>

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
