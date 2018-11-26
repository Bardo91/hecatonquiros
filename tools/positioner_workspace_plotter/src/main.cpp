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


#include <hecatonquiros/Positioner.h>
#include <iostream>
#include <chrono>
#include <ros/ros.h>

#include <std_srvs/Trigger.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <tf/transform_broadcaster.h>

#include <pcl/visualization/pcl_visualizer.h>

bool getHeatMapColor(float value, float *red, float *green, float *blue)
{
  const int NUM_COLORS = 4;
  static float color[NUM_COLORS][3] = { {0,0,1}, {0,1,0}, {1,1,0}, {1,0,0} };
    // A static array of 4 colors:  (blue,   green,  yellow,  red) using {r,g,b} for each.
 
  int idx1;        // |-- Our desired color will be between these two indexes in "color".
  int idx2;        // |
  float fractBetween = 0;  // Fraction between "idx1" and "idx2" where our value is.
 
  if(value <= 0)      {  idx1 = idx2 = 0;            }    // accounts for an input <=0
  else if(value >= 1)  {  idx1 = idx2 = NUM_COLORS-1; }    // accounts for an input >=0
  else
  {
    value = value * (NUM_COLORS-1);        // Will multiply value by 3.
    idx1  = floor(value);                  // Our desired color will be after this index.
    idx2  = idx1+1;                        // ... and before this index (inclusive).
    fractBetween = value - float(idx1);    // Distance between the two indexes (0-1).
  }
 
  *red   = (color[idx2][0] - color[idx1][0])*fractBetween + color[idx1][0];
  *green = (color[idx2][1] - color[idx1][1])*fractBetween + color[idx1][1];
  *blue  = (color[idx2][2] - color[idx1][2])*fractBetween + color[idx1][2];
}


int main(int _argc, char **_argv) {
	pcl::visualization::PCLVisualizer viewer;

	hecatonquiros::Positioner dockingTool("/dev/ttyACM0", 115200);

	pcl::PointCloud<pcl::PointXYZRGB> cloud, cloudFiltered;
	// Create the filtering object
	pcl::VoxelGrid<pcl::PointXYZRGB> sor;
	sor.setLeafSize (0.05f, 0.05f, 0.05f);

	Eigen::Matrix4f cs = Eigen::Matrix4f::Identity();
	Eigen::Matrix3f rot;
	rot = Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitZ());
	cs.block<3,3>(0,0) = rot;
	cs(0,3) = -0.05;



	viewer.addCoordinateSystem(0.2, Eigen::Affine3f(cs));
	viewer.addPointCloud(cloudFiltered.makeShared(), "cloud");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cloud");
	viewer.setBackgroundColor(1,1,1);
	for(;;){
		auto angles = dockingTool.angles();
		auto accel = dockingTool.accelerationVector();
		Eigen::Matrix4f baseToHand, handToBase;
		dockingTool.baseToHand(baseToHand);
		dockingTool.handToBase(handToBase);

		Eigen::Matrix4f t01,t12,t23,t34,t4f;
		dockingTool.lastTransforms(t01,t12,t23,t34,t4f);
		
		float val = 1;
		for(auto angle:angles){
			float valInter = 1 - fabs(angle)/(M_PI/2);
			val *= valInter;
		}
		float r,g,b;
		getHeatMapColor(1 - val, &r, &g, &b);
		pcl::PointXYZRGB p(r*255,g*255,b*255);

		p.x = baseToHand(0,3);
		p.y = baseToHand(1,3);
		p.z = baseToHand(2,3);

		cloud.push_back(p);

		sor.setInputCloud (cloud.makeShared());
		sor.filter (cloudFiltered);
		viewer.updatePointCloud(cloudFiltered.makeShared(), "cloud");
		viewer.spinOnce(30, true);
	}
}