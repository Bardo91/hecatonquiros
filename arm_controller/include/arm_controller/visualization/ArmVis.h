//
//
//
//
//

#include <Eigen/Eigen>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/PolygonMesh.h>

class ArmVis {
public:
	bool init(pcl::visualization::PCLVisualizer *_viewer, std::string _folderFiles);
	void draw(Eigen::Matrix4f _t1, Eigen::Matrix4f _t2, Eigen::Matrix4f _t3);

private:
	pcl::visualization::PCLVisualizer *mViewer;
	pcl::PolygonMesh mBarMesh1, mBarMesh2, mBarMesh3;
};