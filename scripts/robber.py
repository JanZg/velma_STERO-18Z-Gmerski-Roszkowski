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

q_default_position = {'torso_0_joint':0,
        'right_arm_0_joint':-0.3,   'left_arm_0_joint':0.3,
        'right_arm_1_joint':-1.8,   'left_arm_1_joint':1.8,
        'right_arm_2_joint':1.25,   'left_arm_2_joint':-1.25,
        'right_arm_3_joint':0.85,   'left_arm_3_joint':-0.85,
        'right_arm_4_joint':0,      'left_arm_4_joint':0,
        'right_arm_5_joint':-0.5,   'left_arm_5_joint':0.5,
        'right_arm_6_joint':0,      'left_arm_6_joint':0 }

def locateObject(object):
	objectFrame = velma.getTf("B", object) #odebranie pozycji i orientacji obiektu	
	objectAngle=math.atan2(objectFrame.p[1],objectFrame.p[0])
	print "Coordinates of cabinet:", objectFrame.p[0], objectFrame.p[1], objectFrame.p[2], "\n"
	return objectFrame, objectAngle

def cartToCab(positionCart,cabinetFrame):
    return cabinetFrame*positionCart

def cartToCabFrame(frameCart,cabinetFrame):
    return cabinetFrame*frameCart

def cabToCart(positionCab,cabinetFrame):
    cabinetFrame=cabinetFrame.Inverse()
    return cabinetFrame*positionCab

def cabToCartFrame(frameCab,cabinetFrame):
    cabinetFrame=cabinetFrame.Inverse()
    return cabinetFrame*frameCab

def estimateCircle(p,t,q,u,s,z): # x1,y1,x2,y2,x3,y3										    
    x, y, z = complex(p,t),complex(q,u),complex(s,z)
    w = z-x
    w /= y-x
    c = (x-y)*(w-abs(w)**2)/2j/w.imag-x	
    x0,y0,r=c.real,c.imag,abs(c+x)												    
    return x0,y0,r      
												    
def estimateSetPoint(x0,y0,r,current_position): #pracujemy na parametrycznej postaci okregu x=r*cos(alfa)+x0, y=r*sin(alfa)+y0
    x=current_position.p.x()
    y=current_position.p.y()
    alfa=acos((x-x0)/r)                        #liczymy obecna wartosc parametru alfa
    alfa=alfa+math.pi*1.0/180.0
    x=r*cos(alfa)+x0
    y=r*sin(alfa)+y0
    return x,y,alfa-math.pi*90.0/180.0
    												    
    
    
def modeJnt():
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
	modeJnt()
	dest_q = [0,0,0,180.0/180.0*math.pi]
	print "Moving right hand fingers:", dest_q
	velma.moveHandRight(dest_q, [1,1,1,1], [2000,2000,2000,2000], 1000, hold=True)
	if velma.waitForHandRight() != 0:
		exitError(10)
	rospy.sleep(0.5)
	if not isHandConfigurationClose( velma.getHandRightCurrentConfiguration(), dest_q):
		exitError(11)

def hold():
	modeJnt()
	dest_q = [80.0/180.0*math.pi,80.0/180.0*math.pi,80.0/180.0*math.pi,180.0/180.0*math.pi]
	print "Moving right hand fingers:", dest_q
	velma.moveHandRight(dest_q, [1,1,1,1], [2000,2000,2000,2000], 1000, hold=True)
	if velma.waitForHandRight() != 0:
		exitError(10)
	rospy.sleep(0.5)
	if not isHandConfigurationClose( velma.getHandRightCurrentConfiguration(), dest_q):
		exitError(11)

def setImp (x,y,z,xT,yT,zT):
	modeCart()
	print "Setting impendance"
	if not velma.moveCartImpRight(None, None, None, None, [PyKDL.Wrench(PyKDL.Vector(x,y,z), PyKDL.Vector(xT,yT,zT))], [2], PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5))):
		exitError(16)
	if velma.waitForEffectorRight() != 0:
		exitError(17)

def moveJnt(q_map):
	modeJnt()
	print "Moving to set position (jimp)"
	velma.moveJoint(q_map, 1, start_time=0.5, position_tol=0, velocity_tol=0)
	error = velma.waitForJoint()
	if error != 0:
		print "The action should have ended without error, but the error code is", error
		exitError(6)


def moveCart(x,y,z,theta):
	modeCart()
	print "Moving right wrist to position:",x,y,z,"\n"
	T_B_Trd = PyKDL.Frame(PyKDL.Rotation.RPY( 0.0 , 0.0 , theta), PyKDL.Vector( x , y , z ))
	if not velma.moveCartImpRight([T_B_Trd], [2.0], None, None, None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5):
		exitError(8)
	if velma.waitForEffectorRight() != 0:
		exitError(9)
	rospy.sleep(0.5)
	print "calculating difference between desiread and reached pose..."
	T_B_T_diff = PyKDL.diff(T_B_Trd, velma.getTf("B", "Tr"), 1.0)
	print T_B_T_diff
	if T_B_T_diff.vel.Norm() > 0.05 or T_B_T_diff.rot.Norm() > 0.05:
		exitError(10)
		
		
def moveMyCart(x,y,z,theta,tolerance):	#polecane 0.04 jako tolerance
	modeCart()
	print "Moving right wrist to position:",x,y,z,"\n"
	T_B_Trd = PyKDL.Frame(PyKDL.Rotation.RPY( 0.0 , 0.0 , theta), PyKDL.Vector( x , y , z ))
	if not velma.moveCartImpRight([T_B_Trd], [3.0], None, None, None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5, path_tol=PyKDL.Twist(PyKDL.Vector(tolerance,tolerance,tolerance), PyKDL.Vector(tolerance,tolerance,tolerance))):
		exitError(13)
	if velma.waitForEffectorRight() != 0:
		return 1
	return 0

def getCurrentTfCab(cabinetFrame):                        #pobieranie obecnej pozycji we wspolrzednych szafki
    position=velma.getTf("B", "Tr")	
    position=cabinetFrame*position
    print "Current position (cabinet POV): ",position.p[0],position.p[1]
    return position

 
if __name__ == "__main__":

    rospy.init_node('test_init', anonymous=False)
 
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
        exitError(14)
 
    print "Also, head motors must be homed after start-up of the robot."
    print "Sending head pan motor START_HOMING command..."
    velma.startHomingHP()
    if velma.waitForHP() != 0:
	exitError(14)
    print "Head pan motor homing successful."
 
    print "Sending head tilt motor START_HOMING command..."
    velma.startHomingHT()
    if velma.waitForHT() != 0:
	exitError(15)
    print "Head tilt motor homing successful.\n"
    
    #ruch w jednej plaszczyznie ustawiamy stale z
    h=tableH+0.5*cabinetH+0.15		#srodek z jakiegos powodu nie jest na srodku (no i chwytak jest nizej od koncowki)
 
	#pozycja startowa
    moveJnt(q_default_position)
    hold()

	#gdzie jest cabinet
    (cabinetFrame,cabinetVelmaAngle)=locateObject('cabinet_door')
    (useless, who_cares, cabinetAngle)=cabinetFrame.M.GetRPY()

    cabinetAngle=cabinetAngle+math.pi
    if(cabinetAngle>math.pi):
        cabinetAngle=math.pi-cabinetAngle
    print "cabinetAngle: ", cabinetAngle

	#ruszamy
    position=cabToCart(PyKDL.Vector(cabinetD/2+0.2,cabinetW/4,0),cabinetFrame)	
    moveCart(position[0]/2, position[1],h-0.2,cabinetAngle)		#podniesienie reki na dwa razy by velma sie nie zabila ruszajac reka w gore
    moveCart(position[0]/2, position[1],h,cabinetAngle)
    setImp(500,1000,1000,150,150,150)

	#kurs kolizyjny
    if not moveMyCart(position[0],position[1],h,cabinetAngle,0.04):	#CHARGE FORWARD
        print "Expected contact, but there isn't one :( Returning"
        moveCart(position[0]/2, position[1],h-0.2,cabinetAngle)
        moveJnt(q_default_position)
        print "Shutting down"
        exitError(1)

    print "Enemy Contact! Succesfull engagement (not romantical one tho) \n"

    position=getCurrentTfCab(cabinetFrame)			#uderzenie potwierdzone, sprawdzamy gdzie
    position.p[0]=position.p[0]+0.02
    position=cabToCart(PyKDL.Vector(position.p[0],position.p[1],0),cabinetFrame)	#odpuszczamy
    moveCart(position[0],position[1],h,cabinetAngle)

	#do chwytaka
    setImp(1000,500,1000,150,150,150)
    position=getCurrentTfCab(cabinetFrame)
    position.p[1]=position.p[1]-cabinetW/2+0.05
    position=cabToCart(PyKDL.Vector(position.p[0],position.p[1],0),cabinetFrame)	#w lewo az zlapiem uchwyta
    if not moveMyCart(position[0],position[1],h,cabinetAngle,0.04):
        print "Expected contact, but there isn't one :( Returning"
        moveCart(position[0]/2, position[1],h-0.2,cabinetAngle)
        moveJnt(q_default_position)
        print "Shutting down"
        exitError(1)

    print "Hah, caught one! \n"

    print "Aborting, rest of this thing is truly unfinished"
    exitError("TODO")
	
	
   
    
# Pierwsza faza ruchu: ciagniemy do tylu
    starting_position=getCurrentTfCab(cabinetFrame) 
	
    pos_2=starting_position	
    
    dest_1=(0, 0.5*cabinetD+0.2,h)
    dest_1=cabToCart(dest_1,cabinetFrame)
    	
    moveMyCart(dest_1[0],dest_1[1],h,cabinetAngle,0.04)
	
    pos_1=getCurrentTfCab(cabinetFrame)
    
    dest_2=PyKDL.Vector(pos_1.p.x(),pos_2.p.y()+0.2,h)
    dest_2=cabToCart(dest_2,cabinetFrame)
    
    moveMyCart(dest_2[0],dest_2[1],dest_2[2],cabinetAngle,0.04)
	
    pos0=getCurrentTfCab(cabinetFrame)

    cabinetDoorAngle=0
    
# Druga faza ruchu: ciagniemy po torze
	
    while cabinetDoorAngle<math.pi*90.0/180.0:
        
        

        
        x0,y0,r,angle=estimateCircle(pos_2.p.x(),pos_2.p.y(),pos_1.p.x(),pos_1.p.y(),pos_0.p.x(),pos_0.p.y())
        x,y=estimateSetPoint(x0,y0,r,pos_0)
        
        dest=PyKDL.Vector(x,y,h)
        dest=cabToCart(dest,cabinetFrame)
       
        moveMyCart(dest[0],dest[1],dest[2],cabinetAngle+angle,tolerance=None)
        
        #estymacja na podstawie trzech ostatnich punktow
        pos_2=pos_1
        pos_1=pos_0
        pos_0=getCurrentTfCab(cabinetFrame)
        (a,b,current_angle)=current_position.M.GetRPY()
        cabinetDoorAngle=math.pi-current_angle

    

