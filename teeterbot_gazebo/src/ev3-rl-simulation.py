#!/usr/bin/env python
##from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_D, SpeedPercent, MoveSteering
##from ev3dev2.sensor import INPUT_2
##from ev3dev2.sensor.lego import GyroSensor
##from ev3dev2.button import Button
##from ev3dev2.power import PowerSupply
##from ev3dev2.sound import Sound
from random import *
import time
import math
import ev3_rl_classes
import json
import sys
import pickle

## Imports for ROS-Gazebo Simulation
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from gazebo_msgs.msg import ModelStates
from tf.transformations import euler_from_quaternion, quaternion_from_euler
# check website http://www.theconstructsim.com/ros-qa-135-how-to-rotate-a-robot-to-a-desired-heading-using-feedback-from-odometry/
from std_srvs.srv import Empty

## Helper Methods
# Function to set the duty cycle of the motors
def SetMotorDutyCycle(pub, duty):
    # Compansate for nominal voltage and round the input
    dutyInt = int(round(duty))

    # Add or subtract offset and clamp the value between -100 and 100
    if dutyInt > 0:
        dutyInt = min(100, dutyInt + frictionOffset)
    elif dutyInt < 0:
        dutyInt = max(-100, dutyInt - frictionOffset)

    # Apply the signal to the motor
##    writeValue(motorDutyFileHandle, dutyInt)
    pub.publish(dutyInt)    

def assignReward(gyro_state, ideal_state, newDifference, oldAngleDifference):
    if (gyro_state == ideal_state):#(new_gyro_state >= (ideal_state-2)) and (new_gyro_state <= (ideal_state+2)):
        calcReward = 100
    else:
        if (gyro_state >= (ideal_state-8)) and (gyro_state <= (ideal_state+8)):
            calcReward = 5
            if abs(newDifference) < abs(oldAngleDifference):
                calcReward = 2*calcReward
        elif (gyro_state >= (ideal_state-40)) and (gyro_state <= (ideal_state+40)):
            calcReward = 2
        else:
            if gyro_state > ideal_state:
                if newDifference >= 0:
                    calcReward = -5
                else:
                    calcReward = 1
            else:
                if newDifference <= 0:
                    calcReward = -5
                else:
                    calcReward = 1
    return calcReward


class SubscriberClass:
    def __init__(self):

        self.pitchAngle = -100
        self.sub = rospy.Subscriber("/gazebo/model_states", ModelStates, self.getPitchAngle, queue_size = 1)
        

    def getPitchAngle(self, msg):

        orientation_msg = msg.pose[1].orientation
        orientationsList = [orientation_msg.x, orientation_msg.y, orientation_msg.z, orientation_msg.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientationsList)
        self.pitchAngle = int(round(pitch*57.3))
        #print "pitchAngle in method: "+str(self.pitchAngle)

    def __del__(self):
        self.sub.unregister()



if __name__ == '__main__':

    print "Initialising Node"
    rospy.init_node('RLnode')
    time.sleep(0.1)

    subscriber = SubscriberClass()
    time.sleep(0.1)
    
    ## defining actions and states
    print "Initialise actions and states"
    action_duty_cycle = [0]
    states = [0]

    for i in range(60,0,-1):
        states.append(-i)
        states.append(i)

    for i in range(100,0,-1):
        action_duty_cycle.append(-i)
        action_duty_cycle.append(i)

    ##Initialise robot

    ## Initialise Q-matrix

    print "Initialise Q-matrix"
    try:
        f = open("QMatrix_gamma0.4_alpha0.8 changedReward.pkl","rb")
        q_matrix = pickle.load(f)
        f.close()
    except  IOError:
        q_matrix = []

        for state in states:
            q_matrix_row = []
            for action in action_duty_cycle:
                q_matrix_row.append(0)
            q_matrix.append(q_matrix_row)

    ideal_state = 0 #or 22


    num_iterations = 2000
    num_episodes = 2000


    epsilon = 0.7#2.00564812e-7#0.9
    epsilon_decay = 0.99
    min_epsilon = 0.01
    max_epsilon = 1.0
    alpha = 0.8#0.7
    gamma = 0.4#0.999
    total_reward = 0
    max_reward = 10000
    seed()


    print "Initialising Publishing and services"
    pubLeft = rospy.Publisher('/teeterbot/left_motor_voltage', Float64, queue_size = 1)
    pubRight = rospy.Publisher('/teeterbot/right_motor_voltage', Float64, queue_size = 1)

    rospy.wait_for_service('/gazebo/reset_simulation')
    rospy.wait_for_service('/gazebo/pause_physics')
    rospy.wait_for_service('/gazebo/unpause_physics')

    reset_simulation = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
    pauseSim = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
    unpauseSim = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)

    print "Completed Publishers and Services initialisation"

    try:
        f = open("RLdatalog-10am.pkl","rb")
        datalog = pickle.load(f)
        f.close()
    except IOError:
        datalog = {
            'timeStart' : time.strftime("UTC: %Y-%m-%d-%H:%M:%S"),
            #'loopStartTimes' : [],
            #'episodeStartTimes' : [], 
            'gyro_angle_states' : [],
            'motorDutyCycleActions' : [], 
            'angleDifferences' : [],
            'rewards' : [],
            'totalRewards' : []
        }


    buttonPressed = False
    indexOutOfBoundFlag = 0
    print "Starting episodes"
    try:
        for i in range(num_episodes):

            #reset simulation
            print "resetting simulation"
            reset_simulation()


            #################### Below section of code taken from laurensvalk/segway ######################################

            powerClass = ev3_rl_classes.Power()
            gyroClass  = ev3_rl_classes.Gyro()
            timingClass = ev3_rl_classes.Timing()

            # Define Math constants and conversions
            radiansPerDegree               = 3.14159/180                                                # The number of radians in a degree.
            radiansPerSecondPerRawGyroUnit = gyroClass.degPerSecondPerRawGyroUnit * radiansPerDegree    # Rate in radians/sec per gyro output unit

            # Offset to limit friction deadlock
            frictionOffset = int(round(powerClass.frictionOffsetNominal))

            #Timing settings for the program
            loopTimeSec             = timingClass.loopTimeMiliSec / 1000  # Time of each loop, measured in seconds.
            loopCount               = 0                            # Loop counter, starting at 0

            # The rate at which we'll update the gyro offset (precise definition given in docs)
            gyroDriftCompensationRate      = timingClass.gyroDriftCompensationFactor*loopTimeSec*radiansPerSecondPerRawGyroUnit

            #################### End of code taken from laurensvalk/segway ######################################

            #loopStartTimesPerEpisode = []
            gyro_angle_states_perEpisode = []
            motorDutyCycleActionsPerEpisode = []
            angleDifferencesPerEpisode = []
            rewardsPerEpisode = []

            #wait for one second (according to Rahman et al.), but considering just 10ms
            time.sleep(0.01)
            
            #pause simulation - paused for 100ms
            print "pausing simulation"
            pauseSim()

            time.sleep(0.001)
            old_state = subscriber.pitchAngle
##            print "old state: "+str(old_state)

            #unpause simulation - how?
            print "unpausing simulation"
            unpauseSim()
##            print "unpaused simulation"

            episodeStartTime = time.time()
            
            total_reward = 0
            oldAngleDifference = -1

            #datalog['episodeStartTimes'].append(time.asctime(time.localtime(episodeStartTime)))

            gyro_angle_states_perEpisode.append(old_state)

            print "starting iterations"
            for j in range(num_iterations):
                
                loopStartTime = time.time() - episodeStartTime
                print "episode: "+str(i+1)
                print "iteration: "+str(j+1)
                print "old_state: "+str(old_state)

                randomNum = uniform(0,1)
                if randomNum < epsilon:
                    actionIndex = randint(0,len(action_duty_cycle)-1)
                    action = action_duty_cycle[actionIndex]
                    SetMotorDutyCycle(pubLeft,action)
                    SetMotorDutyCycle(pubRight,action)
                else:
                    try:
                        actionIndex = q_matrix[old_state].index(max(q_matrix[old_state]))
                        action = action_duty_cycle[actionIndex]
                        SetMotorDutyCycle(pubLeft,action)
                        SetMotorDutyCycle(pubRight,action)
                    except IndexError:
                        reset_simulation()
                        #indexOutOfBoundFlag = 1
                        break
                    
                print "action: "+str(action)
                    
                #reading the new angle of gyro (new state)
                time.sleep(0.001)
                new_gyro_state = subscriber.pitchAngle
                
                #pause simulation
                pauseSim()
                
                newDifference = (new_gyro_state - old_state)

                reward = assignReward(new_gyro_state, ideal_state, newDifference, oldAngleDifference)
                total_reward += reward

                #loopStartTimesPerEpisode.append(round(loopStartTime,6))
                gyro_angle_states_perEpisode.append(new_gyro_state)
                motorDutyCycleActionsPerEpisode.append(action)
                angleDifferencesPerEpisode.append(newDifference)
                rewardsPerEpisode.append(reward)

                print "new_gyro_state: "+str(new_gyro_state)

                try:
                    max_qvalue = max(q_matrix[new_gyro_state])
                    print "reward: "+str(reward)
                    delta = reward + (gamma * max_qvalue) - q_matrix[old_state][actionIndex]
                    q_matrix[old_state][actionIndex] = q_matrix[old_state][actionIndex] + (alpha * delta)
                except IndexError:
                    #indexOutOfBoundFlag = 1
                    reset_simulation()
                    unpauseSim()
                    break
                

                old_state = new_gyro_state
                oldAngleDifference = newDifference

                if old_state > 60 or old_state < -60:
                    reset_simulation()

                while(time.time()- episodeStartTime - loopStartTime <  loopTimeSec):
                    time.sleep(0.0001)

                print ""
                unpauseSim()

            if(total_reward >= max_reward):
                print "Passed episode "+str(i+1)+" with reward: "+str(total_reward)
                
            else:
                print "Failed episode "+str(i+1)+" with reward: "+str(total_reward)
       
            #datalog['loopStartTimes'].append(loopStartTimesPerEpisode)
            datalog['gyro_angle_states'].append(gyro_angle_states_perEpisode)
            datalog['motorDutyCycleActions'].append(motorDutyCycleActionsPerEpisode)
            datalog['angleDifferences'].append(angleDifferencesPerEpisode)
            datalog['rewards'].append(rewardsPerEpisode)
            datalog['totalRewards'].append(total_reward)
            
            if indexOutOfBoundFlag == 1:
                print "Failed episode "+str(i+1)+" with reward: "+str(total_reward)
                reset_simulation()
            #epsilon = min_epsilon + (max_epsilon - min_epsilon) * exp(-0.1 * epsilon)
            epsilon = epsilon * epsilon_decay
            if epsilon < min_epsilon:
                epsilon = min_epsilon

            if (i+1)%200 == 0:
                print "Writing to files...."
                f = open('RLdatalog_gamma0.4_alpha0.8 changedReward.pkl', 'wb')
                pickle.dump(datalog,f)
                f.close()

                f = open('RLdatalog_gamma0.4_alpha0.8 changedReward.txt', 'w')
                f.write(json.dumps(datalog))
                f.close()

                f = open('QMatrix_gamma0.4_alpha0.8 changedReward.pkl','wb')
                pickle.dump(q_matrix,f)
                f.close()

                f = open('QMatrix_gamma0.4_alpha0.8 changedReward.txt','w')
                f.write(json.dumps(q_matrix))
                f.close()

                f = open('RL totalRewards_gamma0.4_alpha0.8 changedReward.txt', 'w')
                f.write(json.dumps(datalog['totalRewards']))
                f.close()

            print ""
            print "************************************************************"
            print ""

    except KeyboardInterrupt:
        #datalog['loopStartTimes'].append(loopStartTimesPerEpisode)
        datalog['gyro_angle_states'].append(gyro_angle_states_perEpisode)
        datalog['motorDutyCycleActions'].append(motorDutyCycleActionsPerEpisode)
        datalog['angleDifferences'].append(angleDifferencesPerEpisode)
        datalog['rewards'].append(rewardsPerEpisode)
        datalog['totalRewards'].append(total_reward)

    datalog['endTime'] = time.strftime("UTC: %Y-%m-%d-%H:%M:%S")
    SetMotorDutyCycle(pubLeft,0)
    SetMotorDutyCycle(pubRight,0)
    pubLeft.unregister()
    pubRight.unregister()

    f = open('RLdatalog_gamma0.4_alpha0.8 changedReward.pkl', 'wb')
    pickle.dump(datalog,f)
    f.close()

    f = open('RLdatalog_gamma0.4_alpha0.8 changedReward.txt', 'w')
    f.write(json.dumps(datalog))
    f.close()

    f = open('QMatrix_gamma0.4_alpha0.8 changedReward.pkl','wb')
    pickle.dump(q_matrix,f)
    f.close()

    f = open('QMatrix_gamma0.4_alpha0.8 changedReward.txt','w')
    f.write(json.dumps(q_matrix))
    f.close()

    f = open('RL totalRewards_gamma0.4_alpha0.8 changedReward.txt', 'w')
    f.write(json.dumps(datalog['totalRewards']))
    f.close()
