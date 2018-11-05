#!/usr/bin/env python
 
 
 
 # Copyright (c) 2017, Robot Control and Pattern Recognition Group,
 # Institute of Control and Computation Engineering
 # Warsaw University of Technology
 #
 # All rights reserved.
 # 
 # Redistribution and use in source and binary forms, with or without
 # modification, are permitted provided that the following conditions are met:
 #     * Redistributions of source code must retain the above copyright
 #       notice, this list of conditions and the following disclaimer.
 #     * Redistributions in binary form must reproduce the above copyright
 #       notice, this list of conditions and the following disclaimer in the
 #       documentation and/or other materials provided with the distribution.
 #     * Neither the name of the Warsaw University of Technology nor the
 #       names of its contributors may be used to endorse or promote products
 #       derived from this software without specific prior written permission.
 # 
 # THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 # ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 # WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 # DISCLAIMED. IN NO EVENT SHALL <COPYright HOLDER> BE LIABLE FOR ANY
 # DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 # (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 # LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 # ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 # (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 # SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 #
 # Author: Dawid Seredynski
 #
 
import roslib; roslib.load_manifest('velma_task_cs_ros_interface')
 
import rospy
import copy
 
from velma_common import *
from rcprg_ros_utils import exitError
 
if __name__ == "__main__":

     q_map_starting = {'torso_0_joint':0, 'right_arm_0_joint':-0.3, 'right_arm_1_joint':-1.8,
         'right_arm_2_joint':1.25, 'right_arm_3_joint':0.85, 'right_arm_4_joint':0, 'right_arm_5_joint':-0.5,
         'right_arm_6_joint':0, 'left_arm_0_joint':0.3, 'left_arm_1_joint':1.8, 'left_arm_2_joint':-1.25,
         'left_arm_3_joint':-0.85, 'left_arm_4_joint':0, 'left_arm_5_joint':0.5, 'left_arm_6_joint':0 }
     q_map_1 = {'torso_0_joint':-1.56, 'right_arm_0_joint':-0.3, 'right_arm_1_joint':-1.8,
         'right_arm_2_joint':1.25, 'right_arm_3_joint':0.85, 'right_arm_4_joint':0, 'right_arm_5_joint':-0.5,
         'right_arm_6_joint':0, 'left_arm_0_joint':0.3, 'left_arm_1_joint':1.8, 'left_arm_2_joint':-1.25,
         'left_arm_3_joint':-0.85, 'left_arm_4_joint':0, 'left_arm_5_joint':0.5, 'left_arm_6_joint':0 }
     q_map_2 = {'torso_0_joint':1.56, 'right_arm_0_joint':-0.3, 'right_arm_1_joint':-1.8,
         'right_arm_2_joint':1.25, 'right_arm_3_joint':0.85, 'right_arm_4_joint':0, 'right_arm_5_joint':-0.5,
         'right_arm_6_joint':0, 'left_arm_0_joint':0.3, 'left_arm_1_joint':1.8, 'left_arm_2_joint':-1.25,
         'left_arm_3_joint':-0.85, 'left_arm_4_joint':0, 'left_arm_5_joint':0.5, 'left_arm_6_joint':0 }

 
     rospy.init_node('head_test', anonymous=False)
 
     rospy.sleep(0.5)
 
     print "This test/tutorial executes simple motions"\
         " of head."
 
     print "Running python interface for Velma..."
     velma = VelmaInterface()
     print "Waiting for VelmaInterface initialization..."
     if not velma.waitForInit(timeout_s=10.0):
         print "Could not initialize VelmaInterface\n"
         exitError(1)
     print "Initialization ok!\n"

     print "Motors must be enabled every time after the robot enters safe state."
     print "If the motors are already enabled, enabling them has no effect."
     print "Enabling motors..."
     if velma.enableMotors() != 0:
         exitError(2)
 
     diag = velma.getCoreCsDiag()
     if not diag.motorsReady():
         print "Motors must be homed and ready to use for this test."
         exitError(1)
 
     print "Powering up! Portals online!"
     velma.moveJoint(q_map_starting, 5.0, start_time=0.5, position_tol=15.0/180.0*math.pi)
     error = velma.waitForJoint()
     if error != 0:
         print "The action should have ended with PATH_TOLERANCE_VIOLATED error status, but the error code is", error
         exitError(11)
 
 
     print "moving head to position: 0"
     q_dest = (0,-0.5)
     velma.moveHead(q_dest, 3.0, start_time=0.5)
     if velma.waitForHead() != 0:
         exitError(4)
     rospy.sleep(0.5)
     if not isHeadConfigurationClose( velma.getHeadCurrentConfiguration(), q_dest, 0.1 ):
         exitError(5)

 
     print "moving head to position: left"
     q_dest = (1.0, -0.5)
     velma.moveHead(q_dest, 3.0, start_time=0.5)
     if velma.waitForHead() != 0:
         exitError(6)
     rospy.sleep(0.5)
     if not isHeadConfigurationClose( velma.getHeadCurrentConfiguration(), q_dest, 0.1 ):
         exitError(7)

     print "moving head to position: right down"
     q_dest = (-1.0, 1.3)
     velma.moveHead(q_dest, 5.0, start_time=0.5)
     if velma.waitForHead() != 0:
         exitError(8)
     rospy.sleep(0.5)
     if not isHeadConfigurationClose( velma.getHeadCurrentConfiguration(), q_dest, 0.1 ):
         exitError(9)
 
     print "moving head to position: left down"
     q_dest = (1.0, 1.3)
     velma.moveHead(q_dest, 2.0, start_time=0.5)
     if velma.waitForHead() != 0:
         exitError(6)
     rospy.sleep(0.5)
     if not isHeadConfigurationClose( velma.getHeadCurrentConfiguration(), q_dest, 0.1 ):
         exitError(7)
 
     print "moving head to position: right"
     q_dest = (-1.0, -0.5)
     velma.moveHead(q_dest, 2.0, start_time=0.5)
     if velma.waitForHead() != 0:
         exitError(8)
     rospy.sleep(0.5)
     if not isHeadConfigurationClose( velma.getHeadCurrentConfiguration(), q_dest, 0.1 ):
         exitError(9)
 
     print "moving head to position: 0"
     q_dest = (0,0)
     velma.moveHead(q_dest, 3.0, start_time=0.5)
     if velma.waitForHead() != 0:
         exitError(14)
     if not isHeadConfigurationClose( velma.getHeadCurrentConfiguration(), q_dest, 0.1 ):
         exitError(15)

     print "Spinning"
     velma.moveJoint(q_map_1, 5.0, start_time=0.5, position_tol=15.0/180.0*math.pi)
     error = velma.waitForJoint()
     if error != 0:
         print "The action should have ended with PATH_TOLERANCE_VIOLATED error status, but the error code is", error
         exitError(11)

     print "moving head to position: down"
     q_dest = (0, 1.3)
     velma.moveHead(q_dest, 1.0, start_time=0.5)
     if velma.waitForHead() != 0:
         exitError(4)
     rospy.sleep(0.5)
     if not isHeadConfigurationClose( velma.getHeadCurrentConfiguration(), q_dest, 0.1 ):
         exitError(5)
 
 
     print "moving head to position: right down"
     q_dest = (-1.0, 1.3)
     velma.moveHead(q_dest, 5.0, start_time=0.5)
     if velma.waitForHead() != 0:
         exitError(8)
     rospy.sleep(0.5)
     if not isHeadConfigurationClose( velma.getHeadCurrentConfiguration(), q_dest, 0.1 ):
         exitError(9)
 
     print "moving head to position: right"
     q_dest = (-1.0, -0.5)
     velma.moveHead(q_dest, 2.0, start_time=0.5)
     if velma.waitForHead() != 0:
         exitError(8)
     rospy.sleep(0.5)
     if not isHeadConfigurationClose( velma.getHeadCurrentConfiguration(), q_dest, 0.1 ):
         exitError(9)
 
     print "moving head to position: 0"
     q_dest = (0,0)
     velma.moveHead(q_dest, 3.0, start_time=0.5)
     if velma.waitForHead() != 0:
         exitError(14)
     if not isHeadConfigurationClose( velma.getHeadCurrentConfiguration(), q_dest, 0.1 ):
         exitError(15)


     print "Spinning"
     velma.moveJoint(q_map_2, 5.0, start_time=0.5, position_tol=15.0/180.0*math.pi)
     error = velma.waitForJoint()
     if error != 0:
         print "The action should have ended with PATH_TOLERANCE_VIOLATED error status, but the error code is", error
         exitError(11)

     print "moving head to position: down"
     q_dest = (0, 1.3)
     velma.moveHead(q_dest, 1.0, start_time=0.5)
     if velma.waitForHead() != 0:
         exitError(4)
     rospy.sleep(0.5)
     if not isHeadConfigurationClose( velma.getHeadCurrentConfiguration(), q_dest, 0.1 ):
         exitError(5)
 
     print "moving head to position: left"
     q_dest = (1.0, -0.5)
     velma.moveHead(q_dest, 3.0, start_time=0.5)
     if velma.waitForHead() != 0:
         exitError(6)
     rospy.sleep(0.5)
     if not isHeadConfigurationClose( velma.getHeadCurrentConfiguration(), q_dest, 0.1 ):
         exitError(7)
 
     print "moving head to position: left down"
     q_dest = (1.0, 1.3)
     velma.moveHead(q_dest, 2.0, start_time=0.5)
     if velma.waitForHead() != 0:
         exitError(6)
     rospy.sleep(0.5)
     if not isHeadConfigurationClose( velma.getHeadCurrentConfiguration(), q_dest, 0.1 ):
         exitError(7)
 
 
     print "moving head to position: 0"
     q_dest = (0,0)
     velma.moveHead(q_dest, 3.0, start_time=0.5)
     if velma.waitForHead() != 0:
         exitError(14)
     if not isHeadConfigurationClose( velma.getHeadCurrentConfiguration(), q_dest, 0.1 ):
         exitError(15)


     print "Spinning"
     velma.moveJoint(q_map_starting, 5.0, start_time=0.5, position_tol=15.0/180.0*math.pi)
     error = velma.waitForJoint()
     if error != 0:
         print "The action should have ended with PATH_TOLERANCE_VIOLATED error status, but the error code is", error
         exitError(11)
     
 
     exitError(0)