#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.msg import ModelState
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import time
# check website http://www.theconstructsim.com/ros-qa-135-how-to-rotate-a-robot-to-a-desired-heading-using-feedback-from-odometry/

class TestClass:
    def __init__(self):

        self.pitchAngle = -100
##        sub = rospy.Subscriber("/gazebo/model_states", ModelStates, self.getPitchAngle, queue_size = 1)
        

    def getPitchAngle(self):
        print "waiting for message"
        msg = rospy.wait_for_message("/gazebo/model_states", ModelStates)
        print "received message"
        orientation_msg = msg.pose[1].orientation
        orientationsList = [orientation_msg.x, orientation_msg.y, orientation_msg.z, orientation_msg.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientationsList)
    ##    lWheelPosition = msg.position[0]
    ##    lwpDegrees = lWheelPosition * 57.3
    ##    rWheelPosition = msg.position[1]
    ##    rwpDegrees = rWheelPosition * 57.3

    ##    rospy.loginfo('lwp: {}, rwp: {}'.format(lWheelPosition, rWheelPosition))
    ##    rospy.loginfo('lwpDegrees: {}, rwpDegrees: {}'.format(lwpDegrees, rwpDegrees))

        self.pitchAngle = pitch*57.3

        pubLeft = rospy.Publisher('/teeterbot/left_motor_voltage', Float64, queue_size = 1)
        pubRight = rospy.Publisher('/teeterbot/right_motor_voltage', Float64, queue_size = 1)

        #msg.name = "teeterbot"

        newMsg = ModelState()
        newMsg.model_name = "teeterbot"
        newMsg.pose = msg.pose[1]
        newMsg.twist = msg.twist[1]
##        newMsg['pose'] = {}
##        newMsg['pose'] = msg.pose[1]#.position
##        #newMsg['pose']['orientation'] = msg.pose[1].position
##        newMsg['twist'] = msg.twist[1]
        #print msg.pose[1].orientation

##        print newMsg
        for i in range(10,-10,-2):

            print "i: "+str(i)
            pubLeft.publish(i)
            pubRight.publish(i)
            time.sleep(3)

        pubLeft.publish(0.0)
        pubRight.publish(0.0)

        pubLeft.unregister()
        pubRight.unregister()
##
##            #i = -1
##            print "i: "+str(i)
##            
##            pitch = (i)/57.3
##            
##            print str(pitch)
##
##            q = quaternion_from_euler(roll, pitch, yaw)
####            print q
##            newMsg.pose.orientation.x = q[0]
##            newMsg.pose.orientation.y = q[1]
##            newMsg.pose.orientation.z = q[2]
##            newMsg.pose.orientation.w = q[3]
##
####            print newMsg
##            pub.publish(newMsg)
##            time.sleep(1)
##                        
            
            
        #print "pitchAngle in method: "+str(self.pitchAngle)

        
    


rospy.init_node('getJointPosition')
testNode = TestClass()
#testNode.start()

print "pitchAngle before loop: "+str(testNode.pitchAngle)
time.sleep(0.1)

testNode.getPitchAngle()

##for i in range(50):
##    print str(i+1)
##    print "pitchAngle main: "+str(testNode.pitchAngle)
##    print "sleeping main"
##    #self.loop_rate.sleep()
##    time.sleep(0.001)
##    print "finished sleeping main"

    #rospy.Subscriber("/teeterbot/joint_states", JointState, callbackFunc)

##    print "sleeping"
##    time.sleep(2)
##    print "finished sleeping"
    #pitch = getPitchAngle()
##    print "pitchAngle before loop: "+str(testNode.pitchAngle)
##
##    for i in range(100):
##        print str(i+1)
##        print "pitchAngle: "+str(testNode.pitchAngle)

    ##rospy.loginfo(pitch)
    ##rospy.loginfo("pitch degrees: {}".format(pitch*57.3))

    ##print "Before Spin"
    ##rospy.spin()
    ##print "After Spin"


