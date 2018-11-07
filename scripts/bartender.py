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
import PyKDL
import math

from velma_common import *
from rcprg_planner import *
from rcprg_ros_utils import exitError
h_stolu=1
h_puszki=0.23
a_stolu=1.5
b_stolu=0.8

q_map_1 = {'torso_0_joint':0,
        'right_arm_0_joint': 0.,   'left_arm_0_joint':0.3,
        'right_arm_1_joint': 0,   'left_arm_1_joint':1.8,
        'right_arm_2_joint': 0,   'left_arm_2_joint':-1.25,
        'right_arm_3_joint': 0,   'left_arm_3_joint':-0.85,
        'right_arm_4_joint': 0,      'left_arm_4_joint':0,
        'right_arm_5_joint': 0,   'left_arm_5_joint':0.5,
'right_arm_6_joint': 0, 'left_arm_6_joint':0 }

q_default_position = {'torso_0_joint':0,
        'right_arm_0_joint':-0.3,   'left_arm_0_joint':0.3,
        'right_arm_1_joint':-1.8,   'left_arm_1_joint':1.8,
        'right_arm_2_joint':1.25,   'left_arm_2_joint':-1.25,
        'right_arm_3_joint':0.85,   'left_arm_3_joint':-0.85,
        'right_arm_4_joint':0,      'left_arm_4_joint':0,
        'right_arm_5_joint':-0.5,   'left_arm_5_joint':0.5,
        'right_arm_6_joint':0,      'left_arm_6_joint':0 }

def highFive(torso_angle):
		if(torso_angle>1.56):
			 torso_angle=1.56
		if(torso_angle<-1.56):
			 torso_angle=-1.56
		q_map_to_spin = {'torso_0_joint':torso_angle, 
        'right_arm_0_joint':1,   'left_arm_0_joint':0.3,
        'right_arm_1_joint':-1.2,   'left_arm_1_joint':1.8,
        'right_arm_2_joint':1.25,   'left_arm_2_joint':-1.25,
        'right_arm_3_joint':2,   'left_arm_3_joint':-0.85,
        'right_arm_4_joint':0,      'left_arm_4_joint':0,
        'right_arm_5_joint':-0.5,   'left_arm_5_joint':0.5,
        'right_arm_6_joint':0,      'left_arm_6_joint':0 }
		print "Spinning and going up"
		planAndExecute(q_map_to_spin)

      
def grabRight():
	modeImp()
        dest_q = [70.0/180.0*math.pi,70.0/180.0*math.pi,70.0/180.0*math.pi,0]
        print "Taking a hold"
        velma.moveHandRight(dest_q, [1,1,1,1], [2000,2000,2000,2000], 1000, hold=True)
        if velma.waitForHandRight() != 0:
            exitError(8)
        rospy.sleep(0.5)
        if not isHandConfigurationClose( velma.getHandRightCurrentConfiguration(), dest_q):
		print "Failure: Cannot take a hold. Returning to starting position"
		releaseRight()
		moveRight(0.6*beerFrame.p[0],0.6*beerFrame.p[1],0.4*h_puszki+beerFrame.p[2],beerAngle)
		planAndExecute(q_default_position)
        	exitError(9)
    
def releaseRight():
	modeImp()
        dest_q = [0,0,0,0]
        print "Releasing"
        velma.moveHandRight(dest_q, [1,1,1,1], [2000,2000,2000,2000], 1000, hold=True)
        if velma.waitForHandRight() != 0:
            exitError(8)
        rospy.sleep(0.5)
        if not isHandConfigurationClose( velma.getHandRightCurrentConfiguration(), dest_q):
            exitError(9)
 	         
def locateObject(object):
 
        objectFrame = velma.getTf("B", object) #odebranie pozycji i orientacji obiektu	
        objectAngle=math.atan2(objectFrame.p[1],objectFrame.p[0])
	print "Coordinates of beer:", objectFrame.p[0], objectFrame.p[1], "\n"
        return objectFrame, objectAngle
     
def moveRight(x,y,z,theta):
	modeCart()
        print "Moving right wrist to position:",x,y,z,"\n"
        T_B_Trd = PyKDL.Frame(PyKDL.Rotation.RPY( 0.0 , 0.0 , theta), PyKDL.Vector( x , y , z ))
        if not velma.moveCartImpRight([T_B_Trd], [3.0], None, None, None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5):
            exitError(8)
        if velma.waitForEffectorRight() != 0:
            exitError(9)
        rospy.sleep(0.5)
        print "calculating difference between desiread and reached pose..."
        T_B_T_diff = PyKDL.diff(T_B_Trd, velma.getTf("B", "Tr"), 1.0)
        print T_B_T_diff
        if T_B_T_diff.vel.Norm() > 0.05 or T_B_T_diff.rot.Norm() > 0.05:
            exitError(10)

def planAndExecute(q_dest):
	modeImp()
        print "Planning motion to the goal position using set of all joints..."
        print "Moving to valid position, using planned trajectory."
        goal_constraint = qMapToConstraints(q_dest, 0.01, group=velma.getJointGroup("impedance_joints"))
        for i in range(5):
            rospy.sleep(0.5)
            js = velma.getLastJointState()
            print "Planning (try", i, ")..."
            traj = p.plan(js[1], [goal_constraint], "impedance_joints", max_velocity_scaling_factor=0.15, planner_id="RRTConnect")
            if traj == None:
                continue
            print "Executing trajectory..."
            if not velma.moveJointTraj(traj, start_time=0.5):
                exitError(5)
            if velma.waitForJoint() == 0:
                break
            else:
                print "The trajectory could not be completed, retrying..."
                continue
        rospy.sleep(0.5)
        js = velma.getLastJointState()
        if not isConfigurationClose(q_dest, js[1]):
            exitError(6)

def modeImp():
		print "Switch to jnt_imp mode (no trajectory)..."
		velma.moveJointImpToCurrentPos(start_time=0.2)
		error = velma.waitForJoint()
		if error != 0:
			print "The action should have ended without error, but the error code is", error
			exitError(3)
 
		rospy.sleep(0.5)
		diag = velma.getCoreCsDiag()
		if not diag.inStateJntImp():
			print "The core_cs should be in jnt_imp state, but it is not"
			exitError(3)


def modeCart():
		print "Switch to cart_imp mode (no trajectory)..."
		if not velma.moveCartImpRightCurrentPos(start_time=0.2):
			exitError(8)
		if velma.waitForEffectorRight() != 0:
			exitError(9)
		rospy.sleep(0.5)
		diag = velma.getCoreCsDiag()
		if not diag.inStateCartImp():
			print "The core_cs should be in cart_imp state, but it is not"
			exitError(3)


def findDest(object):
        #szukanie wierzcholkow stolu
        objectFrame = velma.getTf("B", object) #odebranie pozycji i orientacji obiektu
	objectAngle=objectFrame.M
        (a,b,alfa)=objectAngle.GetRPY() #kat obrotu stolu wokol polozenia rownowagi
	alfa=0.785-alfa
        beta=math.atan2(b_stolu,a_stolu)   #kat miedzy przekatna stolu a jego dlugoscia
        d=math.sqrt(math.pow(a_stolu,2)+math.pow(b_stolu,2))  #przekatna stolu
        
    #    w_sr=(objectFrame.p[0],objectFrame.p[1],objectFrame.p[2])
     #   w1=w_sr+(0.5*d*math.sin(alfa+beta),0.5*d*math.cos(alfa+beta),0)
    #    w3=w_sr+(-0.5*d*math.sin(alfa+beta),-0.5*d*math.cos(alfa+beta),0)
    #    w2=w_sr+(0.5*d*math.cos(alfa-beta),0.5*d*math.sin(alfa-beta),0)
    #    w4=w_sr+(-0.5*d*math.cos(alfa-beta),-0.5*d*math.sin(alfa-beta),0)

        w_sr=(objectFrame.p[0],objectFrame.p[1],objectFrame.p[2])
        w1=(w_sr[0]+0.5*d*math.sin(alfa+beta),w_sr[1]+0.5*d*math.cos(alfa+beta),w_sr[2])
        w3=(w_sr[0]-0.5*d*math.sin(alfa+beta),w_sr[1]-0.5*d*math.cos(alfa+beta),w_sr[2])
        w2=(w_sr[0]-0.5*d*math.cos(alfa-beta),w_sr[1]+0.5*d*math.sin(alfa-beta),w_sr[2])
        w4=(w_sr[0]+0.5*d*math.cos(alfa-beta),w_sr[1]-0.5*d*math.sin(alfa-beta),w_sr[2])
	
        wd1=math.sqrt(w1[0]*w1[0]+w1[1]*w1[1])
        wd2=math.sqrt(w2[0]*w2[0]+w2[1]*w2[1])
        wd3=math.sqrt(w3[0]*w3[0]+w3[1]*w3[1])
        wd4=math.sqrt(w4[0]*w4[0]+w4[1]*w4[1])
        
	#Sprawdzamy ktory wierzcholek stolu jest najblizej velmy - w jego poblizu bedziemy stawiac puszke
        if(wd1>wd2): 
             w1=w2
             wd1=wd2
        if(wd1>wd3):
             w1=w3
             wd1=wd3
        if(wd1>wd4):
             w1=w4
             wd1=wd4
	w1=(0.9*w1[0]+0.1*w_sr[0],0.9*w1[1]+0.1*w_sr[1],w1[2])
	print "Coordinates to drop:", w1[0], w1[1], "\n"
        th=math.atan2(w1[1],w1[0])
        return (w1[0],w1[1],w1[2],th)

if __name__ == "__main__":
    # define some configurations
        
    #Step 1: zaciskamy palce
    #Step 2: zblizamy sie do puszki
    #Step 3: otwieramy palce
    #Step 4: podjezdzamy pod puszke
    #Step 5: zaciskamy palce(chwytamy)
    #Step 6: jedziemy nad punkt docelowy
    #Step 7: upuszczamy puszke
    #Step 8: podnosimy chwytak nad puszke, aby jej nie potracic
    #Step 9: wracamy do pozycji domyslnej
    rospy.init_node('test_cimp_pose')
    rospy.sleep(0.5)
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

    print "waiting for Planner init..."
    p = Planner(velma.maxJointTrajLen())
    if not p.waitForInit():
        print "could not initialize PLanner"
        exitError(2)
 
    diag = velma.getCoreCsDiag()
    if not diag.motorsReady():
        print "Motors must be homed and ready to use for this test."
        exitError(1)
 
    oml = OctomapListener("/octomap_binary")
    rospy.sleep(1.0)
    octomap = oml.getOctomap(timeout_s=5.0)
    p.processWorld(octomap)
    # planning...
    print "Planner init ok"
  
grabRight()
(beerFrame,beerAngle)=locateObject("beer")
highFive(beerAngle+0.15)
releaseRight() 
moveRight(0.6*beerFrame.p[0],0.6*beerFrame.p[1],0.4*h_puszki+beerFrame.p[2],beerAngle)
moveRight(beerFrame.p[0]-0.25*math.cos(beerAngle),beerFrame.p[1]-0.25*math.sin(beerAngle),0.4*h_puszki+beerFrame.p[2],beerAngle)
rospy.sleep(0.5)
grabRight()
highFive(beerAngle+0.15)

dest=findDest("table2")
highFive(dest[3]+0.15)
moveRight(dest[0],dest[1],dest[2]+0.05+h_stolu,dest[3])
releaseRight()
    
(beer_frame,beer_angle)=locateObject("beer")
moveRight(beer_frame.p[0],beer_frame.p[1],0.5*h_puszki+beer_frame.p[2]+0.2,beer_angle) #podnosimy reke zeby nie potracic puszki
  
print "Assuming resting position"
planAndExecute(q_default_position)
              


