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


#include "Graph2d.h"

using namespace cv;
using namespace std;

namespace rgbd {
	//---------------------------------------------------------------------------------------------------------------------
	Graph2d::Graph2d(std::string _name) {
		mWindowName = _name;
		namedWindow(mWindowName, CV_WINDOW_FREERATIO);
		clean();
	}

	//---------------------------------------------------------------------------------------------------------------------
	void Graph2d::clean() {
		mGraphs.clear();
		cleanGraph();
	}

	//---------------------------------------------------------------------------------------------------------------------
	void Graph2d::show() {
		imshow(mWindowName, mLastRender);
	}

	//---------------------------------------------------------------------------------------------------------------------
	void Graph2d::draw(const std::vector<double>& _x, const std::vector<double>& _y, unsigned char _r, unsigned char _g, unsigned _b, eDrawType _type) {
		if (_x.size() != _y.size() || _x.size() == 0 || _y.size() == 0) {
			std::cerr << "Bad input arguments" << std::endl;
			return;
		}

		double minX = *min_element(_x.begin(), _x.end());
		double maxX = *max_element(_x.begin(), _x.end());
		double minY = *min_element(_y.begin(), _y.end());
		double maxY = *max_element(_y.begin(), _y.end());

		Graph graph = { _x, _y, Scalar(_b, _g, _r), _type };
		mGraphs.push_back(graph);

		if (minX < mMinX || maxX > mMaxX || minY < mMinY || maxY > mMaxY) {
			mMinX = minX < mMinX ? minX : mMinX;
			mMaxX = maxX > mMaxX ? maxX : mMaxX;
			mMinY = minY < mMinY ? minY : mMinY;
			mMaxY = maxY > mMaxY ? maxY : mMaxY;

			cleanGraph();
			drawAxis();
			// Redraw all graph
			for (Graph graph2 : mGraphs) {
				drawGraph(graph2);
			}

		}
		else {
			drawGraph(graph);
		}

		show();
	}

	//---------------------------------------------------------------------------------------------------------------------
	void Graph2d::draw(const std::vector<double>& _y, unsigned char _r, unsigned char _g, unsigned _b, eDrawType _type) {
		vector<double> x;
		for (unsigned i = 0; i < _y.size(); i++) {
			x.push_back(i);
		}
		draw(x, _y, _r, _g, _b, _type);
	}

	//---------------------------------------------------------------------------------------------------------------------
	void Graph2d::drawAxis() {
		double xStepD = (mWindowSize.width - cOffsetHorizontal * 2) / cHorizontalDivisions;
		double yStepD = (mWindowSize.height - cOffsetVertical * 2) / cVerticalDivisions;
		double xStep = (mMaxX - mMinX) / cHorizontalDivisions;
		double yStep = (mMaxY - mMinY) / cVerticalDivisions;

		// Draw horizontal axi
		Point2i p1(cOffsetHorizontal, mWindowSize.height - cOffsetVertical);
		Point2i p2(cOffsetHorizontal, mWindowSize.height - int(cOffsetVertical*0.75));
		for (unsigned i = 0; i <= cHorizontalDivisions; i++) {
			line(mLastRender, p1, p2, Scalar(0, 0, 0), 2);
			putText(mLastRender, to_string(mMinX + xStep*i).substr(0, 5), Point2i(p1.x - cOffsetHorizontal / 2, mWindowSize.height - int(cOffsetVertical*0.5)), CV_FONT_HERSHEY_PLAIN, 1.5, Scalar(0, 0, 0), 2);
			p1.x += int(xStepD);
			p2.x += int(xStepD);
		}

		// Draw vertical axi
		p1 = Point2i(int(cOffsetHorizontal*0.75), mWindowSize.height - cOffsetVertical);
		p2 = Point2i(cOffsetHorizontal, mWindowSize.height - cOffsetVertical);
		for (unsigned i = 0; i <= cHorizontalDivisions; i++) {
			line(mLastRender, p1, p2, Scalar(0, 0, 0), 2);
			putText(mLastRender, to_string(mMinY + yStep*i).substr(0, 5), Point2i(0, p2.y), CV_FONT_HERSHEY_PLAIN, 1.5, Scalar(0, 0, 0), 2);
			p1.y -= int(yStepD);
			p2.y -= int(yStepD);
		}
	}

	//---------------------------------------------------------------------------------------------------------------------
	void Graph2d::drawGraph(const Graph & _graph) {
		switch (_graph.mType) {
		case eDrawType::FilledCircles:
			drawPoints(_graph);
			break;
		case eDrawType::Lines:
			drawLines(_graph);
			break;
		case eDrawType::Circles:
			drawCircles(_graph);
			break;
		default:
			std::cout << "Graph type not defined" << std::endl;
			break;
		}

	}

	//---------------------------------------------------------------------------------------------------------------------
	void Graph2d::drawPoints(const Graph &_graph) {
		for (unsigned i = 0; i < _graph.mX.size(); i++) {
			int x = int(cOffsetHorizontal + (_graph.mX[i] > mMinX ? _graph.mX[i] - mMinX : mMinX - _graph.mX[i]) / (mMaxX - mMinX)*(mWindowSize.width - cOffsetHorizontal * 2));
			int y = int((mWindowSize.height - cOffsetVertical) - (_graph.mY[i] > mMinY ? _graph.mY[i] - mMinY : mMinY - _graph.mY[i]) / (mMaxY - mMinY)*(mWindowSize.height - cOffsetVertical * 2));
			Point2i point(x, y);
			circle(mLastRender, point, 5, _graph.mColor, CV_FILLED);
		}
	}

	//---------------------------------------------------------------------------------------------------------------------
	void Graph2d::drawCircles(const Graph &_graph) {
		for (unsigned i = 0; i < _graph.mX.size(); i++) {
			int x = int(cOffsetHorizontal + (_graph.mX[i] > mMinX ? _graph.mX[i] - mMinX : mMinX - _graph.mX[i]) / (mMaxX - mMinX)*(mWindowSize.width - cOffsetHorizontal * 2));
			int y = int((mWindowSize.height - cOffsetVertical) - (_graph.mY[i] > mMinY ? _graph.mY[i] - mMinY : mMinY - _graph.mY[i]) / (mMaxY - mMinY)*(mWindowSize.height - cOffsetVertical * 2));
			Point2i point(x, y);
			circle(mLastRender, point, 5, _graph.mColor);
		}
	}


	//---------------------------------------------------------------------------------------------------------------------
	void Graph2d::drawLines(const Graph & _graph) {
		for (unsigned i = 0; i < _graph.mX.size() - 1; i++) {
			int x1 = int(cOffsetHorizontal + (_graph.mX[i] > mMinX ? _graph.mX[i] - mMinX : mMinX - _graph.mX[i]) / (mMaxX - mMinX)*(mWindowSize.width - cOffsetHorizontal * 2));
			int y1 = int((mWindowSize.height - cOffsetVertical) - (_graph.mY[i] > mMinY ? _graph.mY[i] - mMinY : mMinY - _graph.mY[i]) / (mMaxY - mMinY)*(mWindowSize.height - cOffsetVertical * 2));
			Point2i p1(x1, y1);

			int x2 = int(cOffsetHorizontal + (_graph.mX[i + 1] > mMinX ? _graph.mX[i + 1] - mMinX : mMinX - _graph.mX[i + 1]) / (mMaxX - mMinX)*(mWindowSize.width - cOffsetHorizontal * 2));
			int y2 = int((mWindowSize.height - cOffsetVertical) - (_graph.mY[i + 1] > mMinY ? _graph.mY[i + 1] - mMinY : mMinY - _graph.mY[i + 1]) / (mMaxY - mMinY)*(mWindowSize.height - cOffsetVertical * 2));
			Point2i p2(x2, y2);

			line(mLastRender, p1, p2, _graph.mColor, 2);
		}
	}

	//---------------------------------------------------------------------------------------------------------------------
	void Graph2d::cleanGraph() {
		mLastRender = Mat(mWindowSize, CV_8UC3, Scalar(150, 150, 150));
		rectangle(mLastRender, Point2i(cOffsetHorizontal, cOffsetVertical), Point2i(mWindowSize.width - cOffsetHorizontal, mWindowSize.height - cOffsetVertical), Scalar(255, 255, 255), CV_FILLED);
		rectangle(mLastRender, Point2i(cOffsetHorizontal, cOffsetVertical), Point2i(mWindowSize.width - cOffsetHorizontal, mWindowSize.height - cOffsetVertical), Scalar(0, 0, 0), 2);
	}
}	//	namespace rgbd
