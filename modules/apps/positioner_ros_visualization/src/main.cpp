//---------------------------------------------------------------------------------------------------------------------
//  Copyright 2018 Pablo Ramon Soria (a.k.a. Bardo91) pabramsor@gmail.com
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


#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <hecatonquiros/visualization/PositionerVis.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <tf_conversions/tf_eigen.h>

int main(int _argc, char **_argv) {
	ros::init(_argc, _argv, "positioner_visualization_ros");

	pcl::visualization::PCLVisualizer viewer;
	hecatonquiros::PositionerVis dockingToolVisualization;
	dockingToolVisualization.init(&viewer, "src/hecatonquiros/tools/meshes");	// 666 hum... there should be a kind of ROS path selector

	tf::TransformListener listener;	
	ros::Rate rate(20.0);
	while (ros::ok()){
		tf::StampedTransform transform;
		Eigen::Affine3d t01,t12,t23,t34,t4f;
		try{
			listener.lookupTransform("/base_docking", "/dt_t01",	ros::Time(0), transform);
			tf::transformTFToEigen(transform, t01);

			listener.lookupTransform("/base_docking", "/dt_t12",	ros::Time(0), transform);
			tf::transformTFToEigen(transform, t12);

			listener.lookupTransform("/base_docking", "/dt_t23",	ros::Time(0), transform);
			tf::transformTFToEigen(transform, t23);

			listener.lookupTransform("/base_docking", "/dt_t34",	ros::Time(0), transform);
			tf::transformTFToEigen(transform, t34);

			listener.lookupTransform("/base_docking", "/dt_t4f",	ros::Time(0), transform);
			tf::transformTFToEigen(transform, t4f);

			dockingToolVisualization.draw(	Eigen::Matrix4f::Identity(),
											((Eigen::Matrix4d)t01.matrix()).cast<float>(),
											((Eigen::Matrix4d)t12.matrix()).cast<float>(),
											((Eigen::Matrix4d)t23.matrix()).cast<float>(),
											((Eigen::Matrix4d)t34.matrix()).cast<float>(),
											((Eigen::Matrix4d)t4f.matrix()).cast<float>());
			viewer.spinOnce(20);
		}
		catch (tf::TransformException ex){
			ROS_ERROR("%s",ex.what());
			ros::Duration(1.0).sleep();
		}
		rate.sleep();
	}
}
