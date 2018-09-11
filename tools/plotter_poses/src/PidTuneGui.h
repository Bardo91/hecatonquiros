//
//
//
//
//

#ifndef PIDTUNEGUI_H_
#define PIDTUNEGUI_H_

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

    class PidTuneGui : public QMainWindow {
        Q_OBJECT
    public:
        PidTuneGui(std::string _refTopic, std::string _valTopic,  std::vector<std::pair<std::string, std::string>> _pidBaseTopics, QWidget *parent = 0);
        virtual ~PidTuneGui(){};

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
        
        QVBoxLayout *mButtonsModifySat, *mButtonsModifyWP, *mButtonsModifyXK, *mButtonsModifyYK, *mButtonsModifyZK, *mButtonsModifyRef, *mButtonsTime, *mAnalysisTimeError, *mAnalysisTimePercentage, *mAnalysisTimeMarkT0, *mAnalysisTimeMarkT1, *mAnalysisTimeMarkT2;
        QHBoxLayout *mButtonsLayoutFilaK, *mButtonsLayoutFilaSW, *mButtonsLayoutFilaRef, *mLayoutFilaErr, *mLayoutFilaTimeMarks;

        QVBoxLayout *mButtonsLayout, *mGraphLayout;
        QWidget     *mCentralWidget;
        QLineEdit *mPosEdit, *mRefEdit;
        QLineEdit *mNameError, *mNamePercentage, *mTextErrorX, *mTextErrorY, *mTextErrorZ, *mTextPercentageX, *mTextPercentageY, *mTextPercentageZ, *mNameMarkXT0, *mNameMarkYT0, *mNameMarkZT0, *mNameMarkXT1, *mNameMarkYT1, *mNameMarkZT1, *mNameMarkXT2, *mNameMarkYT2, *mNameMarkZT2, *mTextMarkXT0, *mTextMarkYT0, *mTextMarkZT0, *mTextMarkXT1, *mTextMarkYT1, *mTextMarkZT1, *mTextMarkXT2, *mTextMarkYT2, *mTextMarkZT2;
        QPushButton *mChangeSubscribers, *mTimeMarkX, *mTimeMarkY, *mTimeMarkZ;
       

        QCustomPlot *mGraphPosition;
        QTimer *mDataTimer, *mDataTimerMark;
        ros::Subscriber mSubPos, mSubRef, mSubXKp;
        float mLastX = 0, mLastY = 0, mLastZ = 0, mLastRefX = 0, mLastRefY = 0, mLastRefZ = 0;

    };

#endif
