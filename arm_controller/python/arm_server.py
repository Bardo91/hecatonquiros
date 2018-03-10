#---------------------------------------------------------------------------------------------------------------------
#  HECATONQUIROS
#---------------------------------------------------------------------------------------------------------------------
#  Copyright 2018 ViGUS University of Seville
#---------------------------------------------------------------------------------------------------------------------
#  Permission is hereby granted, free of charge, to any person obtaining a copy of this software 
#  and associated documentation files (the "Software"), to deal in the Software without restriction, 
#  including without limitation the rights to use, copy, modify, merge, publish, distribute, 
#  sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is 
#  furnished to do so, subject to the following conditions:
#
#  The above copyright notice and this permission notice shall be included in all copies or substantial 
#  portions of the Software.
#
#  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING 
#  BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND 
#  NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES 
#  OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN 
#  CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
#---------------------------------------------------------------------------------------------------------------------

from openravepy import *
import time
import IPython
import numpy as np
import rospy
from arm_controller.srv import SetJoints, SetPose
from geometry_msgs.msg  import *
from sensor_msgs.msg    import *
import threading

class ArmServer:
    ### Initialize robot, ROS interface and multithreads
    ### \param env: initialized enfironment
    ### \param filePath: path to robot file if not loaded before in the enviroment
    ### \param name: custom name of the robot, if loadded before in the environment robot to reference
    ### \param offset: if defined offset of the robot in the environment
    def __init__(self, env, filePath, name, offset):
        # Init services and publishers
        self.mServiceSetJoints          = rospy.Service     (name + '/set_joints', SetJoints, self.handlerSetJoints)
        self.mServiceSetPose            = rospy.Service     (name + '/set_pose', SetPose, self.handlerSetPose)
        self.mJointsPublisher           = rospy.Publisher   (name + '/joints', JointState, queue_size=1)
        self.mPosePublisher             = rospy.Publisher   (name + '/pose', PoseStamped, queue_size=1)
        self.mJointsTransformPublisher  = rospy.Publisher   (name + '/joints_transform', PoseArray, queue_size=1)

        # Init openrave "things"
        self.mRobot = env.ReadRobotXMLFile(filePath)
        self.mRobot.SetName(name)
        initTransform = np.array(   [   [1,0,0,0],
                                        [0,1,0,0],
                                        [0,0,1,0],
                                        [0,0,0,1]   ])
        self.mRobot.SetTransform(initTransform)
        env.Add(self.mRobot)

        # Init threading
        self.mRun = True
        self.mPosePubThread                 = threading.Thread(target=self.runPosePublisher,             name='PosePublisher')
        self.mJointsPubThread               = threading.Thread(target=self.runJointsPublisher,           name='JointsPublisher')
        self.mJointsTransformsPubThread     = threading.Thread(target=self.runJointsTransformPublisher,  name='JointsTransformPublisher')
        self.mPosePubThread.start()
        self.mJointsPubThread.start()
        self.mJointsTransformsPubThread.start()
        

    ### Stop threads, ROS interface and uninitialize OpenRAVE 
    def stop(self):
        self.mRun = False
        # sleep a moment and close threads
        self.mPosePubThread.join(500)
        self.mJointsPubThread.join(500)
        self.mJointsTransformsPubThread.join(500)

    ### Handler of "SetPose" service
    def handlerSetPose(self, req):
        return false

    ### Handler of "SetJoints" service
    def handlerSetJoints(self, req):
        return false

    ### Body pose publisher thread
    def runPosePublisher(self):
        rate = rospy.Rate(30) # 10hz
        while(self.mRun):
            pose = PoseStamped()
            self.mPosePublisher.publish()
            rate.sleep()

    
    ### Body joint publisher thread
    def runJointsPublisher(self):
        rate = rospy.Rate(30) # 10hz
        while(self.mRun):
            joints = JointState()
            self.mJointsPublisher.publish(joints)
            rate.sleep()

    ### Body joints transform thread
    def runJointsTransformPublisher(self):
        rate = rospy.Rate(30) # 10hz
        while(self.mRun):
            jointsTransform = PoseArray()
            self.mJointsTransformPublisher.publish(jointsTransform)
            rate.sleep()
