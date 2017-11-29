//
//
//
//
//

#include <Eigen/Eigen>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/PolygonMesh.h>

class PositionerVis {
public:
	bool init(pcl::visualization::PCLVisualizer *_viewer, std::string _folderFiles);
	void draw(Eigen::Matrix4f _t0, Eigen::Matrix4f _t1, Eigen::Matrix4f _t2, Eigen::Matrix4f _t3, Eigen::Matrix4f _t4, Eigen::Matrix4f _t5);

private:
	pcl::visualization::PCLVisualizer *mViewer;
	pcl::PolygonMesh mBarMesh0, mBarMesh1, mBarMesh2, mBarMesh3, mBarMesh4, mBarMesh5;
	pcl::PolygonMesh mFinalMesh;
};