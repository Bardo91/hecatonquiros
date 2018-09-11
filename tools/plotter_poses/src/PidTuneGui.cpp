//
//
//
//
//
//

#include "PidTuneGui.h"
#include <QTime>
#include <std_srvs/Trigger.h>

PidTuneGui::PidTuneGui(std::string _refTopic, std::string _valTopic, std::vector<std::pair<std::string, std::string>> _pidBaseTopics, QWidget *parent): QMainWindow(parent) {
    mCentralWidget = new QWidget();
    mMainLayout = new QVBoxLayout();
    mCentralWidget->setLayout(mMainLayout);
    setCentralWidget(mCentralWidget);
    this->setWindowTitle("PID Tune Gui");

    // Add text entries for topics     
    mButtonsLayout = new QVBoxLayout();

    QHBoxLayout *refLayout = new QHBoxLayout();
    mButtonsLayout->addLayout(refLayout);
    mRefEdit = new QLineEdit(_valTopic.c_str());
    mRefEdit->setMaximumWidth(300);
    auto refText = new QLineEdit("Ref. position");
    refText->setMaximumWidth(100);
    refText->setEnabled(false);
    refLayout->addWidget(refText);
    refLayout->addWidget(mRefEdit);

    QHBoxLayout *valLayout = new QHBoxLayout();
    mButtonsLayout->addLayout(valLayout);
    mMainLayout->addLayout(mButtonsLayout); 
    mPosEdit = new QLineEdit(_refTopic.c_str());
    mPosEdit->setMaximumWidth(300);
    auto posText = new QLineEdit("Current position");
    posText->setMaximumWidth(100);
    posText->setEnabled(false);
    valLayout->addWidget(posText);
    valLayout->addWidget(mPosEdit);

    mButtonsOptionsLayout = new QHBoxLayout();
    mButtonsLayout->addLayout(mButtonsOptionsLayout);
    mChangeSubscribers = new QPushButton("Change subscribers");
    mButtonsOptionsLayout->addWidget(mChangeSubscribers);
    connect(mChangeSubscribers, SIGNAL (released()), this, SLOT (connectTopics()));
    
    /// GRAPH X
    mGraphLayout = new QVBoxLayout();
    mGraphPosition = new QCustomPlot();
    mGraphLayout->addWidget(mGraphPosition);
    mMainLayout->addLayout(mGraphLayout);

    mGraphPosition->addGraph(); // red line X
    QPen pen;
    pen.setWidthF(4);
    pen.setColor(QColor(255, 0, 0));
    mGraphPosition->graph(0)->setPen(pen);

    mGraphPosition->addGraph(); // red line X ref
    pen.setColor(QColor(255, 0, 0));
    pen.setStyle(Qt::DotLine);
    mGraphPosition->graph(1)->setPen(pen);
    
    QSharedPointer<QCPAxisTickerTime> mTimeTicker(new QCPAxisTickerTime);
    mTimeTicker->setTimeFormat("%h:%m:%s");
    mGraphPosition->xAxis->setTicker(mTimeTicker);
    mGraphPosition->yAxis->setRange(-1.0, 1.0);

    // GRAPH Y
    mGraphPosition->addGraph(); // green line Y
    pen.setColor(QColor(0, 255, 0));
    pen.setStyle(Qt::SolidLine);
    mGraphPosition->graph(2)->setPen(pen);

    mGraphPosition->addGraph(); // green line Y ref
    pen.setColor(QColor(0, 255, 0));
    pen.setStyle(Qt::DotLine);
    mGraphPosition->graph(3)->setPen(pen);

    mGraphPosition->xAxis->setTicker(mTimeTicker);
    mGraphPosition->yAxis->setRange(-1.0, 1.0);

    // GRAPH Z
    mGraphPosition->addGraph(); // blue line Z
    pen.setColor(QColor(0, 0, 255));
    pen.setStyle(Qt::SolidLine);
    mGraphPosition->graph(4)->setPen(pen);

    mGraphPosition->addGraph(); // blue line Z ref
    pen.setColor(QColor(0, 0, 255));
    pen.setStyle(Qt::DotLine);
    mGraphPosition->graph(5)->setPen(pen);

    mGraphPosition->xAxis->setTicker(mTimeTicker);
    mGraphPosition->yAxis->setRange(-1.0, 1.0);
    
    connect(mGraphPosition->xAxis, SIGNAL(rangeChanged(QCPRange)), mGraphPosition->xAxis2, SLOT(setRange(QCPRange)));
    connect(mGraphPosition->yAxis, SIGNAL(rangeChanged(QCPRange)), mGraphPosition->yAxis2, SLOT(setRange(QCPRange)));

    mDataTimer = new QTimer(this);
    connect(mDataTimer, SIGNAL(timeout()), this, SLOT(realTimePlot()));
    mDataTimer->start(0);
}


void PidTuneGui::closeEvent(QCloseEvent *event) {

}


void PidTuneGui::realTimePlot(){
    static QTime time(QTime::currentTime());
    // calculate two new data points:
    double key = time.elapsed()/1000.0; // time elapsed since start of demo, in seconds
    static double lastPointKey = 0;
    if (key-lastPointKey > 0.005) { // at most add point every 2 ms
      // add data to lines:
      mGraphPosition->graph(0)->addData(key, mLastX);
      mGraphPosition->graph(1)->addData(key, mLastRefX);
      
      mGraphPosition->graph(2)->addData(key, mLastY);
      mGraphPosition->graph(3)->addData(key, mLastRefY);

      mGraphPosition->graph(4)->addData(key, mLastZ);
      mGraphPosition->graph(5)->addData(key, mLastRefZ);

      lastPointKey = key;
    }
    // make key axis range scroll with the data (at a constant range size of 8):
    
    double min = mLastX-1 < mLastRefX - 1? mLastX -1: mLastRefX -1;
    double max = mLastX+1 > mLastRefX + 1? mLastX +1: mLastRefX +1;
    mGraphPosition->yAxis->setRange(min, max);

    mGraphPosition->xAxis->setRange(key, 8, Qt::AlignRight);
    mGraphPosition->replot();
}

void PidTuneGui::connectTopics(){
    // other
    ros::NodeHandle n;
    mSubPos = n.subscribe<geometry_msgs::PoseStamped>(mPosEdit->text().toStdString(),1, &PidTuneGui::positionCallback, this);
    mSubRef = n.subscribe<geometry_msgs::PoseStamped>(mRefEdit->text().toStdString(), 1,&PidTuneGui::referenceCallback, this);
    //mSubXKp = n.subscribe<std_msgs::Float32>("/PID_x/kp", 1,&PidTuneGui::changePIDCallback, this);

}

//void PidTuneGui::changePIDCallback(const std_msgs::Float32::ConstPtr &_data){}



void PidTuneGui::positionCallback(const geometry_msgs::PoseStamped::ConstPtr &_data){
    mLastX = _data->pose.position.x;
    mLastY = _data->pose.position.y;
    mLastZ = _data->pose.position.z;
}

void PidTuneGui::referenceCallback(const geometry_msgs::PoseStamped::ConstPtr &_data){
    mLastRefX = _data->pose.position.x;
    mLastRefY = _data->pose.position.y;
    mLastRefZ = _data->pose.position.z;
}
