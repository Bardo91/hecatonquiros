//
//
//
//
//
//

#include "PlotterJointsGui.h"
#include <QTime>
#include <std_srvs/Trigger.h>

PlotterJointsGui::PlotterJointsGui(std::string _refTopic, std::string _valTopic, QWidget *parent): QMainWindow(parent) {
    mCentralWidget = new QWidget();
    mMainLayout = new QVBoxLayout();
    mCentralWidget->setLayout(mMainLayout);
    setCentralWidget(mCentralWidget);
    this->setWindowTitle("Plotter Joints Gui");

    // Add text entries for topics     
    mButtonsLayout = new QVBoxLayout();

    QHBoxLayout *refLayout = new QHBoxLayout();
    mButtonsLayout->addLayout(refLayout);
    mRefEdit = new QLineEdit(_refTopic.c_str());
    mRefEdit->setMaximumWidth(300);
    auto refText = new QLineEdit("Ref. joints");
    refText->setMaximumWidth(100);
    refText->setEnabled(false);
    refLayout->addWidget(refText);
    refLayout->addWidget(mRefEdit);

    QHBoxLayout *valLayout = new QHBoxLayout();
    mButtonsLayout->addLayout(valLayout);
    mMainLayout->addLayout(mButtonsLayout); 
    mJointEdit = new QLineEdit(_valTopic.c_str());
    mJointEdit->setMaximumWidth(300);
    auto posText = new QLineEdit("Current joints");
    posText->setMaximumWidth(100);
    posText->setEnabled(false);
    valLayout->addWidget(posText);
    valLayout->addWidget(mJointEdit);

    mButtonsOptionsLayout = new QHBoxLayout();
    mButtonsLayout->addLayout(mButtonsOptionsLayout);
    mChangeSubscribers = new QPushButton("Change subscribers");
    mButtonsOptionsLayout->addWidget(mChangeSubscribers);
    connect(mChangeSubscribers, SIGNAL (released()), this, SLOT (connectTopics()));

    mTargetJoints.resize(6);
    mStateJoints.resize(6);
    for(unsigned i = 0; i < mTargetJoints.size(); i++){
        mTargetJoints[i] = 0;
        mStateJoints[i] = 0;
    }
    
    // JOINT 0
    mGraphLayout = new QVBoxLayout();
    mGraphJoints = new QCustomPlot();
    mGraphLayout->addWidget(mGraphJoints);
    mMainLayout->addLayout(mGraphLayout);

    mGraphJoints->addGraph(); // red line joint 0
    QPen pen;
    pen.setWidthF(2);
    pen.setColor(QColor(255, 0, 0));
    mGraphJoints->graph(0)->setPen(pen);

    mGraphJoints->addGraph(); // red line joint 0 ref
    pen.setColor(QColor(255, 0, 0));
    pen.setStyle(Qt::DotLine);
    mGraphJoints->graph(1)->setPen(pen);
    
    QSharedPointer<QCPAxisTickerTime> mTimeTicker(new QCPAxisTickerTime);
    mTimeTicker->setTimeFormat("%h:%m:%s");
    mGraphJoints->xAxis->setTicker(mTimeTicker);
    mGraphJoints->yAxis->setRange(-0.5, 0.5);

    // JOINT 1
    mGraphJoints->addGraph(); // green line joint 1
    pen.setColor(QColor(0, 255, 0));
    pen.setStyle(Qt::SolidLine);
    mGraphJoints->graph(2)->setPen(pen);

    mGraphJoints->addGraph(); // green line joint 1 ref
    pen.setColor(QColor(0, 255, 0));
    pen.setStyle(Qt::DotLine);
    mGraphJoints->graph(3)->setPen(pen);

    mGraphJoints->xAxis->setTicker(mTimeTicker);
    mGraphJoints->yAxis->setRange(-0.5, 0.5);

    // JOINT 2
    mGraphJoints->addGraph(); // blue joint line 2
    pen.setColor(QColor(0, 0, 255));
    pen.setStyle(Qt::SolidLine);
    mGraphJoints->graph(4)->setPen(pen);

    mGraphJoints->addGraph(); // blue line joint 2 ref
    pen.setColor(QColor(0, 0, 255));
    pen.setStyle(Qt::DotLine);
    mGraphJoints->graph(5)->setPen(pen);

    mGraphJoints->xAxis->setTicker(mTimeTicker);
    mGraphJoints->yAxis->setRange(-0.5, 0.5);

    // JOINT 3
    mGraphJoints->addGraph(); // purple line joint 3
    pen.setColor(QColor(255, 0, 255));
    pen.setStyle(Qt::SolidLine);
    mGraphJoints->graph(6)->setPen(pen);

    mGraphJoints->addGraph(); // purple line joint 3 ref
    pen.setColor(QColor(255, 0, 255));
    pen.setStyle(Qt::DotLine);
    mGraphJoints->graph(7)->setPen(pen);

    mGraphJoints->xAxis->setTicker(mTimeTicker);
    mGraphJoints->yAxis->setRange(-0.5, 0.5);
    
    connect(mGraphJoints->xAxis, SIGNAL(rangeChanged(QCPRange)), mGraphJoints->xAxis2, SLOT(setRange(QCPRange)));
    connect(mGraphJoints->yAxis, SIGNAL(rangeChanged(QCPRange)), mGraphJoints->yAxis2, SLOT(setRange(QCPRange)));

    mDataTimer = new QTimer(this);
    connect(mDataTimer, SIGNAL(timeout()), this, SLOT(realTimePlot()));
    mDataTimer->start(0);
}


void PlotterJointsGui::closeEvent(QCloseEvent *event) {

}


void PlotterJointsGui::realTimePlot(){
    static QTime time(QTime::currentTime());
    // calculate two new data points:
    double key = time.elapsed()/1000.0; // time elapsed since start of demo, in seconds
    static double lastPointKey = 0;
    if (key-lastPointKey > 0.005) { // at most add point every 2 ms
      // add data to lines:
      mGraphJoints->graph(0)->addData(key, mStateJoints[0]);
      mGraphJoints->graph(1)->addData(key, mTargetJoints[0]);
      
      mGraphJoints->graph(2)->addData(key, mStateJoints[1]);
      mGraphJoints->graph(3)->addData(key, mTargetJoints[1]);

      mGraphJoints->graph(4)->addData(key, mStateJoints[2]);
      mGraphJoints->graph(5)->addData(key, mTargetJoints[2]);

      mGraphJoints->graph(6)->addData(key, mStateJoints[3]);
      mGraphJoints->graph(7)->addData(key, mTargetJoints[3]);

      lastPointKey = key;
    }
    // make key axis range scroll with the data (at a constant range size of 8):
    
    double min = mLast-2 < mLastRef - 2? mLast -2: mLastRef -2;
    double max = mLast+2 > mLastRef + 2? mLast +2: mLastRef +2;
    mGraphJoints->yAxis->setRange(min, max);

    mGraphJoints->xAxis->setRange(key, 8, Qt::AlignRight);
    mGraphJoints->replot();
}

void PlotterJointsGui::connectTopics(){
    // other
    ros::NodeHandle n;
    mSubJoint = n.subscribe<sensor_msgs::JointState>(mJointEdit->text().toStdString(),1, &PlotterJointsGui::jointsCallback, this);
    mSubRef = n.subscribe<sensor_msgs::JointState>(mRefEdit->text().toStdString(), 1,&PlotterJointsGui::referenceCallback, this);

}

void PlotterJointsGui::jointsCallback(const sensor_msgs::JointState::ConstPtr &_data){
    std::vector<float> joints;
    mStateJoints.resize(_data->position.size());
    for(auto j:_data->position){
        joints.push_back(j);
    }
    mStateJoints = joints; // 666 Thread safe?
    mLast = mStateJoints[0];
}

void PlotterJointsGui::referenceCallback(const sensor_msgs::JointState::ConstPtr &_data){
    std::vector<float> joints;
    mTargetJoints.resize(_data->position.size());
    for(auto j:_data->position){
        joints.push_back(j);
    }
    mTargetJoints = joints; // 666 Thread safe?
    mLastRef = mTargetJoints[0];
}
