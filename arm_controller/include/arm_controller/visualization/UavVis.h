//
//
//
//


#include <Eigen/Eigen>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/PolygonMesh.h>

class UavVis {
public:
	bool init(pcl::visualization::PCLVisualizer *_viewer, std::string _folderFiles);
	void draw(Eigen::Matrix4f _t1);
	
private:
	pcl::visualization::PCLVisualizer *mViewer;
	pcl::PolygonMesh mUavMesh;
};