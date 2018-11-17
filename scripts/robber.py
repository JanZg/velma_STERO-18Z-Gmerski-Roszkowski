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
import math
from velma_common import *
from rcprg_ros_utils import exitError

#przelaczanie trybow

cabinetH=0.7
cabinetW=0.6
cabinetD=0.3
tableH=1

def locateObject(object):
	return objectFrame = velma.getTf("cabinet", object) #odebranie pozycji i orientacji obiektu	
#	objectAngle=math.atan2(objectFrame.p[1],objectFrame.p[0])
#	print "Coordinates of cabinet:", objectFrame.p[0], objectFrame.p[1], "\n"
#	return objectFrame, objectAngle

def getCabinetFrame(objectFrame, objectAngle)
    x=objectFrame.p.x()
    y=objectFrame.p.y()
    z=objectFrame.p.z()
    th=objectAngle
    rot=PyKDL.Rotation(0,0,th)
    vect=PyKDL.Vector(x,y,tableH+0.5*cabinetH)      # frame przeksztalcenia szafki wzgledem srodka ukladu kartezjanskiego
    cab_Frame=(rot,vect)
    return cab_Frame


def cartIntoCab(positionCart,cabinetFrame)
    return cabinetFrame*positionCart

def cabIntoCart(positionCab,cabinetFrame)
    cabinetFrame=cabinetFrame.inverse()
    return cabinetFrame*positionCab
    


    
    
     



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

def release():
	modeImp()
	dest_q = [0,0,0,180.0/180.0*math.pi]
	print "Moving right hand fingers:", dest_q
	velma.moveHandRight(dest_q, [1,1,1,1], [2000,2000,2000,2000], 1000, hold=True)
	if velma.waitForHandRight() != 0:
		exitError(10)
	rospy.sleep(0.5)
	if not isHandConfigurationClose( velma.getHandRightCurrentConfiguration(), dest_q):
		exitError(11)

def hold():
	modeImp()
	dest_q = [80.0/180.0*math.pi,80.0/180.0*math.pi,80.0/180.0*math.pi,180.0/180.0*math.pi]
	print "Moving right hand fingers:", dest_q
	velma.moveHandRight(dest_q, [1,1,1,1], [2000,2000,2000,2000], 1000, hold=True)
	if velma.waitForHandRight() != 0:
		exitError(10)
	rospy.sleep(0.5)
	if not isHandConfigurationClose( velma.getHandRightCurrentConfiguration(), dest_q):
		exitError(11)

def setImp (x,y,z,xT,yT,zT):
     print "Setting impendance"
     imp_list = [makeWrench(1000,1000,1000,150,150,150),
                 makeWrench((1000+x)/2,(1000+x)/2,(1000+x)/2,(150+xT)/2,(150+yT)/2,(150+zT)/2),
                 makeWrench(x,y,z,xT,yT,zT)]
     if not velma.moveCartImpRight(None, None, None, None, imp_list, [0.5,1.0,1.5], PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5):
         exitError(16)
     if velma.waitForEffectorRight() != 0:
     exitError(17)

def moveImp(q_map):
	print "Moving to set position (jimp)"
	velma.moveJoint(q_map_0, 5, start_time=0.5, position_tol=0, velocity_tol=0)
	error = velma.waitForJoint()
	if error != FollowJointTrajectoryResult.PATH_TOLERANCE_VIOLATED:
		print "The action should have ended with PATH_TOLERANCE_VIOLATED error status, but the error code is", error
		exitError(4)


def moveCart(x,y,z,theta):
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

def initRobot():
     rospy.sleep(1)
 
	velma = VelmaInterface()
	print "waiting for init..."
 
	print "Running python interface for Velma..."
	velma = VelmaInterface()
	print "Waiting for VelmaInterface initialization..."
	if not velma.waitForInit(timeout_s=10.0):
		print "Could not initialize VelmaInterface\n"
		exitError(1)
	print "Initialization ok!\n"
 
	diag = velma.getCoreCsDiag()
	if not diag.motorsReady():
		print "Motors must be homed and ready to use for this test."
		exitError(1)
 
	if velma.enableMotors() != 0:
		exitError(14)

 
if __name__ == "__main__":
 
    initRobot()
    release()
    setImp()
    (cabinetPosition,cabinetAngle)=locateObject('cabinet_door')
    cabinetFrame=getCabinetFrame(cabinetPosition,cabinetAngle)

    

