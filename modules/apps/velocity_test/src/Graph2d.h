//---------------------------------------------------------------------------------------------------------------------
//  RGBD_TOOLS
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


#ifndef GRAPH2D_H_
#define GRAPH2D_H_

#include <string>
#include <opencv2/opencv.hpp>

namespace rgbd{
	class Graph2d {
	public:		// Public interface
		enum eDrawType { Points = 0, Lines, Circles, FilledCircles, Cross };

		struct Graph {
			std::vector<double> mX;
			std::vector<double> mY;
			cv::Scalar mColor;
			eDrawType mType;
		};

		/// Create a new instance of graph
		/// \param _name: Name to be displayed on the figure.
		Graph2d(std::string _name);

		/// Clean graph
		void clean();

		/// Show last render
		void show();

		/// Draw new data
		void draw(const std::vector<double> &_x, const std::vector<double> &_y, unsigned char _r, unsigned char _g, unsigned _b, eDrawType _type = eDrawType::FilledCircles);
		void draw(const std::vector<double> &_y, unsigned char _r, unsigned char _g, unsigned _b, eDrawType _type = eDrawType::Circles);

	private:	// Private methods
		void drawAxis();
		void drawGraph(const Graph &_graph);
		void drawPoints(const Graph &_graph);
		void drawCircles(const Graph &_graph);
		void drawLines(const Graph &_graph);

		void cleanGraph();

	private:	// Members

		std::string			mWindowName;
		cv::Size2i			mWindowSize = { 1200, 1200 };	// Fixed resolution by now.
		const unsigned		cOffsetHorizontal = 100;			// Fixed by now.
		const unsigned		cOffsetVertical = 100;			// Fixed by now.
		const unsigned		cHorizontalDivisions = 10;			// Fixed by now.
		const unsigned		cVerticalDivisions = 10;			// Fixed by now.

		cv::Mat				mLastRender;
		std::vector<Graph>	mGraphs;

		double mMinX = 999999999999, mMinY = 9999999999999, mMaxX = -9999999999, mMaxY = -9999999999;

	};	// class Graph2d.
}	//	namespace rgbd

#endif	//	GRAPH2D_H_
