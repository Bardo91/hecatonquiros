//
//
//
//
//

#ifndef PLOTTERPOSESGUI_H_
#define PLOTTERPOSESGUI_H_

    #include <ros/ros.h>
    #include <QHBoxLayout>
    #include <QVBoxLayout>
    #include <QMainWindow>
    #include <QLineEdit>
    #include <QTextEdit>
    #include <map>
    #include <geometry_msgs/PoseStamped.h>
    #include "qcustomplot.h"
    #include <vector>

    class PlotterPosesGui : public QMainWindow {
        Q_OBJECT
    public:
        PlotterPosesGui(std::string _refTopic, std::string _valTopic, QWidget *parent = 0);
        virtual ~PlotterPosesGui(){};

    protected:
        void closeEvent(QCloseEvent *event) override;
        
    private slots:
        void realTimePlot();
        void connectTopics();

    private:
        void referenceCallback(const geometry_msgs::PoseStamped::ConstPtr &_data);
        void positionCallback(const geometry_msgs::PoseStamped::ConstPtr &_data);
        
    private:
        QVBoxLayout *mMainLayout;
        QHBoxLayout *mButtonsOptionsLayout;
        
        QVBoxLayout *mButtonsLayout, *mGraphLayout;
        QWidget     *mCentralWidget;
        QLineEdit *mPosEdit, *mRefEdit;
        QPushButton *mChangeSubscribers;
       
        QCustomPlot *mGraphPosition;
        QTimer *mDataTimer;
        ros::Subscriber mSubPos, mSubRef;
        float mLastX = 0, mLastY = 0, mLastZ = 0, mLastRefX = 0, mLastRefY = 0, mLastRefZ = 0;

    };

#endif
