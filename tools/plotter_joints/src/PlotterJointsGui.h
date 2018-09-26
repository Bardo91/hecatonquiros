//
//
//
//
//

#ifndef PLOTTERJOINTSGUI_H_
#define PLOTTERJOINTSGUI_H_

    #include <ros/ros.h>
    #include <QHBoxLayout>
    #include <QVBoxLayout>
    #include <QMainWindow>
    #include <QLineEdit>
    #include <QTextEdit>
    #include <map>
    #include <sensor_msgs/JointState.h>
    #include "qcustomplot.h"
    #include <vector>

    class PlotterJointsGui : public QMainWindow {
        Q_OBJECT
    public:
        PlotterJointsGui(std::string _refTopic, std::string _valTopic, QWidget *parent = 0);
        virtual ~PlotterJointsGui(){};

    protected:
        void closeEvent(QCloseEvent *event) override;
        
    private slots:
        void realTimePlot();
        void connectTopics();

    private:
        void referenceCallback(const sensor_msgs::JointState::ConstPtr &_data);
        void jointsCallback(const sensor_msgs::JointState::ConstPtr &_data);
        
    private:
        QVBoxLayout *mMainLayout;
        QHBoxLayout *mButtonsOptionsLayout;
        
        QVBoxLayout *mButtonsLayout, *mGraphLayout;
        QWidget     *mCentralWidget;
        QLineEdit *mJointEdit, *mRefEdit;
        QPushButton *mChangeSubscribers;
       
        QCustomPlot *mGraphJoints;
        QTimer *mDataTimer;
        ros::Subscriber mSubJoint, mSubRef;
        std::vector<float> mTargetJoints;
        std::vector<float> mStateJoints; 

    };

#endif
