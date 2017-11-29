//
//
//
//

#include <arm_controller/visualization/ArmVis.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/PCLPointCloud2.h>

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
	pcl::fromPCLPointCloud2(mBarMesh2.cloud, pctbar2);
	pcl::fromPCLPointCloud2(mBarMesh3.cloud, pctbar3);

    pcl::transformPointCloud(pctbar1, pctbar1, _t1);
	pcl::transformPointCloud(pctbar2, pctbar2, _t2);
	pcl::transformPointCloud(pctbar3, pctbar3, _t3);

    mViewer->addPolygonMesh<pcl::PointXYZ>(pctbar1.makeShared(), mBarMesh1.polygons, "arm_bar1", 0);
    mViewer->addPolygonMesh<pcl::PointXYZ>(pctbar2.makeShared(), mBarMesh2.polygons, "arm_bar2", 0);
    mViewer->addPolygonMesh<pcl::PointXYZ>(pctbar3.makeShared(), mBarMesh3.polygons, "arm_bar3", 0);
}
