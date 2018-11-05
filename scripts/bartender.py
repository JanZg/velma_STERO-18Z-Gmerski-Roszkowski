
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
 # Author: Teodor Teodorowicz
 #
 
import roslib; roslib.load_manifest('velma_task_cs_ros_interface')
import rospy

from velma_common import *
from rcprg_planner import *
from rcprg_ros_utils import exitError
 
if __name__ == "__main__":
    # define some configurations
    def initAll():
    # zweryfikowac z gazebo
        h_stolu=0.53
        h_puszki=0.23
        a_stolu=1.5
        b_stolu=0.8        
        q_default_position = {'torso_0_joint':0,
            'right_arm_0_joint':-0.3,   'left_arm_0_joint':0.3,
            'right_arm_1_joint':-1.8,   'left_arm_1_joint':1.8,
            'right_arm_2_joint':1.25,   'left_arm_2_joint':-1.25,
            'right_arm_3_joint':0.85,   'left_arm_3_joint':-0.85,
            'right_arm_4_joint':0,      'left_arm_4_joint':0,
            'right_arm_5_joint':-0.5,   'left_arm_5_joint':0.5,
            'right_arm_6_joint':0,      'left_arm_6_joint':0 }
 
        rospy.init_node('test_cimp_pose')
        rospy.sleep(0.5)
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
 
        print "waiting for Planner init..."
        p = Planner(velma.maxJointTrajLen())
        if not p.waitForInit():
            print "could not initialize PLanner"
            exitError(2)
        oml = OctomapListener("/octomap_binary")
        rospy.sleep(1.0)
        octomap = oml.getOctomap(timeout_s=5.0)
        p.processWorld(octomap)
        # planning...
        print "Planner init ok"
 
        if velma.enableMotors() != 0:
            exitError(14)
 
    def grabRight():
        dest_q = [80.0/180.0*math.pi,80.0/180.0*math.pi,80.0/180.0*math.pi,0]
        print "Taking a hold"
        velma.moveHandRight(dest_q, [1,1,1,1], [2000,2000,2000,2000], 1000, hold=True)
        if velma.waitForHandRight() != 0:
            exitError(8)
        rospy.sleep(0.5)
        if not isHandConfigurationClose( velma.getHandRightCurrentConfiguration(), dest_q):
            exitError(9)
    
    def grabLeft():
        dest_q = [80.0/180.0*math.pi,80.0/180.0*math.pi,80.0/180.0*math.pi,0]
        print "Taking a hold"
        velma.moveHandLeft(dest_q, [1,1,1,1], [2000,2000,2000,2000], 1000, hold=True)
        if velma.waitForLeftRight() != 0:
            exitError(8)
        rospy.sleep(0.5)
        if not isHandConfigurationClose( velma.getHandLeftCurrentConfiguration(), dest_q):
            exitError(9)
    
    def releaseRight():
        dest_q = [0,0,0,0]
        print "Taking a hold"
        velma.moveHandRight(dest_q, [1,1,1,1], [2000,2000,2000,2000], 1000, hold=True)
        if velma.waitForHandRight() != 0:
            exitError(8)
        rospy.sleep(0.5)
        if not isHandConfigurationClose( velma.getHandRightCurrentConfiguration(), dest_q):
            exitError(9)
    
    def releaseLeft():
        dest_q = [0,0,0,0]
        print "Taking a hold"
        velma.moveHandLeft(dest_q, [1,1,1,1], [2000,2000,2000,2000], 1000, hold=True)
        if velma.waitForLeftRight() != 0:
            exitError(8)
        rospy.sleep(0.5)
        if not isHandConfigurationClose( velma.getHandLeftCurrentConfiguration(), dest_q):
            exitError(9)
 	         
    def locateObject(object):
 
        objectFrame = velma.getTf("B", object) #odebranie pozycji i orientacji obiektu	
        objectAngle=math.atan2(Object.p[1],Object.p[0])
        return objectFrame, objectAngle
     
    def moveRight(x,y,z,theta):

        print "Moving right wrist to pose defined in world frame..."
        T_B_Trd = PyKDL.Frame(PyKDL.Rotation( 0.0 , 0.0 , theta), PyKDL.Vector( 0.9*x , 0.9*y , z+0.115 ))
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

    def moveLeft(x,y,z,theta):

        print "Moving left wrist to pose defined in world frame..."
        T_B_Trd = PyKDL.Frame(PyKDL.Rotation( 0.0 , 0.0 , theta), PyKDL.Vector( 0.9*x , 0.9*y , z+0.115 ))
        if not velma.moveCartImpLeft([T_B_Trd], [3.0], None, None, None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5):
            exitError(8)
        if velma.waitForEffectorLeft() != 0:
            exitError(9)
        rospy.sleep(0.5)
        print "calculating difference between desiread and reached pose..."
        T_B_T_diff = PyKDL.diff(T_B_Trd, velma.getTf("B", "Tr"), 1.0)
        print T_B_T_diff
        if T_B_T_diff.vel.Norm() > 0.05 or T_B_T_diff.rot.Norm() > 0.05:
            exitError(10)


     # define a function for frequently used routine in this test
    def planAndExecute(q_dest):
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
 
        
    def switchMode():
        diag = velma.getCoreCsDiag()
        if  diag.inStateCartImp():
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
        else: 
            # get initial configuration
            js_init = velma.getLastJointState()
 
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
          


