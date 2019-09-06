//---------------------------------------------------------------------------------------------------------------------
//  Copyright 2018 ViGUS University of Seville
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

#ifndef FINALDEMOAEROARMS_TOPICWATCHDOG_H_
#define FINALDEMOAEROARMS_TOPICWATCHDOG_H_

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <functional>
#include <mutex>
#include <type_traits>
#include <thread>
#include <atomic> 

template<typename T_>
struct SubscriberTrait{
    typedef typename std::conditional<  std::is_same<T_, sensor_msgs::Image>::value,
                                        image_transport::Subscriber,
                                        ros::Subscriber>::type Subscriber;
    
    Subscriber sub;
};

template<typename T_, int QueueSize_ = 10>
class TopicWatchDog{
public:
    typedef std::function<void(const typename T_::ConstPtr &)> Callback;

    TopicWatchDog(std::string _topic, float _maxTimeout = 0.03);

    bool isValid(){
        return delay() < mMaxTimeout;
    }

    float delay(){
        float accum = 0;
        for(unsigned i = 0; i < QueueSize_; i++){
            accum += mDiffTimeQueue[i];
        }
        return accum/QueueSize_;
    }

    float hz(){
        return 1/delay();
    }

    T_ lastData(){
        return mLastData;
    }

    int attachCallback(std::function<void(const typename T_::ConstPtr &)> &_callback){
        int id = mCustomCallbacks.size();
        mCustomCallbacks.push_back(_callback);
    }

    void eraseCallback(int _i){
        mBulldog.lock();
        mCustomCallbacks.erase(mCustomCallbacks.begin()+_i);
        mBulldog.unlock();
    }

    int getNumPublishers(){
        return mSubscriber.sub.getNumPublishers();
    }

private: 
    void dog(const typename T_::ConstPtr &_msg);
    void initTimer();

private:
    SubscriberTrait<T_> mSubscriber;
    std::mutex mBulldog;
    std::vector<std::function<void(const typename T_::ConstPtr &)>> mCustomCallbacks;

    std::thread mTimerThread;
    bool newMeasure = false;

    float       mMaxTimeout;

    T_          mLastData;
    ros::Time   mLastTime;
    float       mDiffTimeQueue[QueueSize_];
    int         mLastTimeIndex = 0;
};

//---------------------------------------------------------------------------------------------------------------------
template<typename T_,  int QueueSize_>
inline void TopicWatchDog<T_, QueueSize_>::initTimer(){
    mLastTime = ros::Time::now();
    for(int i = 0; i < QueueSize_; i++) mDiffTimeQueue[i] = std::numeric_limits<float>::max();  // .... why dont u use memset?

    mTimerThread = std::thread([&](){
        while(ros::ok()){
            // Update hz.
            if(newMeasure){ // 666 Meh, this method limits the maximum speed but at least it updates the time when there is a huge delay.
                mLastTimeIndex = mLastTimeIndex < QueueSize_-1? mLastTimeIndex + 1: 0;
                newMeasure=false;
            }
            ros::Duration diff = ros::Time::now()-mLastTime;
            mDiffTimeQueue[mLastTimeIndex] = diff.toSec();
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    });
}

//---------------------------------------------------------------------------------------------------------------------
template<typename T_,  int QueueSize_>
inline TopicWatchDog<T_, QueueSize_>::TopicWatchDog(std::string _topic, float _maxTimeout){
    mMaxTimeout = _maxTimeout;
    initTimer();
    
    ros::NodeHandle n;
    mSubscriber.sub = n.subscribe<T_>(_topic, 1, [&](const typename T_::ConstPtr &_msg) {
        // Call callbacks
        mLastTime = ros::Time::now();
        newMeasure = true;
        mLastData = *_msg;
        mBulldog.lock();
        for (auto &_c : mCustomCallbacks)
            _c(_msg);
        mBulldog.unlock();
    });
}

//---------------------------------------------------------------------------------------------------------------------
template <>
inline TopicWatchDog<sensor_msgs::Image>::TopicWatchDog(std::string _topic, float _maxTimeout){
    mMaxTimeout = _maxTimeout;
    initTimer();

    ros::NodeHandle n;
    image_transport::ImageTransport it(n);
    mSubscriber.sub = it.subscribe(_topic, 1, [&](const typename sensor_msgs::Image::ConstPtr &_msg) {
        // Call callbacks
        mLastTime = ros::Time::now();
        newMeasure = true;
        mLastData = *_msg;
        mBulldog.lock();
        for (auto &_c : mCustomCallbacks)
            _c(_msg);
        mBulldog.unlock();
    });
}


#endif
