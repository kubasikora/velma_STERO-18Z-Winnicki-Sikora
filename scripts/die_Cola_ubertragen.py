#!/usr/bin/env python2

import roslib; roslib.load_manifest('velma_task_cs_ros_interface')
import rospy
import copy

from velma_common import *
from rcprg_planner import *
from rcprg_ros_utils import exitError

 # define a function for frequently used routine in this test
def planAndExecute(velma, q_dest):
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

def moveInCartImpMode(velma, T_B_dest):
    if not velma.moveCartImpRight([T_B_dest], [5.0], None, None, None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5):
        exitError(8)
    if velma.waitForEffectorRight() != 0:
        exitError(9)
    rospy.sleep(0.5)

def moveInCartMode(velma, T_B_dest):
    moveInCartImpMode(velma, T_B_dest)
    T_B_T_diff = PyKDL.diff(T_B_dest, velma.getTf("B", "Gr"), 1.0)
    if T_B_T_diff.vel.Norm() > 0.05 or T_B_T_diff.rot.Norm() > 0.05:
        exitError(10)

def prepareForGrip(velma, torso_angle):
    executable_q_map = copy.deepcopy(q_map_acquiring)
    executable_q_map['torso_0_joint'] = torso_angle
    planAndExecute(velma, executable_q_map)

def moveToInteractiveCursor(velma):
    T_Wo_test = velma.getTf("Wo", "example_frame")
    if not velma.moveCartImpRightCurrentPos(start_time=0.2):
        exitError(8)
    if velma.waitForEffectorRight() != 0:
        exitError(9)
    if not velma.moveCartImpRight([T_Wo_test], [3.0], None, None, None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5):
        exitError(8)
    if velma.waitForEffectorRight() != 0:
        exitError(9)
    rospy.sleep(0.5)

    T_B_T_diff = PyKDL.diff(T_Wo_test, velma.getTf("B", "Tr"), 1.0)
    if T_B_T_diff.vel.Norm() > 0.05 or T_B_T_diff.rot.Norm() > 0.05:
        exitError(10)

def hideRightHand(velma):
    dest_q = [0.5*math.pi, 0.5*math.pi, 0.5*math.pi, math.pi]
    velma.moveHandRight(dest_q, [1,1,1,1], [2000,2000,2000,2000], 1000, hold=True)
    if velma.waitForHandRight() != 0:
        exitError(10)
    rospy.sleep(0.5)

def hideLeftHand(velma):
    dest_q = [0.5*math.pi, 0.5*math.pi, 0.5*math.pi, math.pi]
    velma.moveHandLeft(dest_q, [1,1,1,1], [2000,2000,2000,2000], 1000, hold=True)
    if velma.waitForHandLeft() != 0:
        exitError(10)
    rospy.sleep(0.5)
    if not isHandConfigurationClose( velma.getHandLeftCurrentConfiguration(), dest_q):
        exitError(11)

def grabWithRightHand(velma):
    dest_q = [76.0/180.0*math.pi, 76.0/180.0*math.pi, 76.0/180.0*math.pi, 0]
    velma.moveHandRight(dest_q, [1,1,1,1], [2000,2000,2000,2000], 1000, hold=True)
    if velma.waitForHandRight() != 0:
        exitError(10)
    rospy.sleep(0.5)

def hideBothHands(velma):
    hideLeftHand(velma)
    hideRightHand(velma)


def moveToPositionZero(velma):
    print "Moving to position 0"
    print "Switch to jnt_imp mode."
    velma.moveJointImpToCurrentPos(start_time=0.5)
    error = velma.waitForJoint()

    print "Moving body to position 0"
    planAndExecute(velma, q_map_starting)

    print "Moving head to position: 0"
    q_dest = (0,0)
    velma.moveHead(q_dest, 3.0, start_time=0.5)
    if velma.waitForHead() != 0:
	    exitError(5)
    rospy.sleep(0.5)
    if not isHeadConfigurationClose( velma.getHeadCurrentConfiguration(), q_dest, 0.1 ):
        exitError(6)

def findCanOnTable(table0_tf, table1_tf, can_tf):
    [t0_x, t0_y, t0_z] = table0_tf.p
    [t1_x, t1_y, t1_z] = table1_tf.p
    [c_x, c_y, c_z] = can_tf.p

    can_to_t0 = (c_x - t0_x)**2 + (c_y - t0_y)**2 + (c_z - t0_z)**2
    can_to_t1 = (c_x - t1_x)**2 + (c_y - t1_y)**2 + (c_z - t1_z)**2

    return "table0" if can_to_t0 < can_to_t1 else "table1"

def switchToJntMode(velma):
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

def switchToCartMode(velma):
    if not velma.moveCartImpRightCurrentPos(start_time=0.2):
        print "Cannot moveCartImpRightCurrentPos"
        exitError(9)

    if velma.waitForEffectorRight() != 0:
        print "waitForEffectorright error"
        exitError(8)

    if not velma.moveCartImpLeftCurrentPos(start_time=0.2):
        print "Cannot moveCartImpLeftCurrentPos"
        exitError(9)

    if velma.waitForEffectorLeft() != 0:
        print "waitForEffectorLeft error"
        exitError(8)

    rospy.sleep(0.5) 
    diag = velma.getCoreCsDiag()
    if not diag.inStateCartImp():
        print "The core_cs should be in cart_imp state, but it is not"
        exitError(3)

def openRightHand(velma):
    dest_q = [0, 0, 0, 0]
    velma.moveHandRight(dest_q, [1,1,1,1], [2000,2000,2000,2000], 1000, hold=True)
    if velma.waitForHandRight() != 0:
        exitError(10)
    rospy.sleep(0.5)
    if not isHandConfigurationClose( velma.getHandRightCurrentConfiguration(), dest_q):
        exitError(11)



if __name__ == "__main__":
    # define some configurations
    q_map_starting = {'torso_0_joint':0,
        'right_arm_0_joint':-0.3,   'left_arm_0_joint':0.3,
        'right_arm_1_joint':-1.8,   'left_arm_1_joint':1.8,
        'right_arm_2_joint':1.25,   'left_arm_2_joint':-1.25,
        'right_arm_3_joint':0.85,   'left_arm_3_joint':-0.85,
        'right_arm_4_joint':0,      'left_arm_4_joint':0,
        'right_arm_5_joint':-0.5,   'left_arm_5_joint':0.5,
        'right_arm_6_joint':0,      'left_arm_6_joint':0 }

    q_map_acquiring = {'torso_0_joint': 0, 
        'right_arm_0_joint': 0.2176005457580103,   'left_arm_0_joint': 0.2345004080527655,
        'right_arm_1_joint': -1.9107791904878497,  'left_arm_1_joint': 1.8034769374904756,
        'right_arm_2_joint': 1.2409542924753767,   'left_arm_2_joint': -1.1982341925787994,
        'right_arm_3_joint': 1.4842204142092719,   'left_arm_3_joint': -0.8278483633253793, 
        'right_arm_4_joint': 0.2525831592128146,   'left_arm_4_joint': 0.07257063733648089,
        'right_arm_5_joint': -1.5390250000127208,  'left_arm_5_joint': 0.4699180006050142,
        'right_arm_6_joint': -0.21426825617036566, 'left_arm_6_joint': -0.0703725749418421,            
    }

    rospy.init_node('proj1_executor')
    rospy.sleep(0.5)

    print "Initializing robot..."
    velma = VelmaInterface()
    if not velma.waitForInit(timeout_s=10.0):
        print "Could not initialize VelmaInterface\n"
        exitError(1)
    
    if velma.enableMotors() != 0:
        exitError(14)

    diag = velma.getCoreCsDiag()
    if not diag.motorsReady():
        print "Motors must be homed and ready to use for this test."
        exitError(1)

    p = Planner(velma.maxJointTrajLen())
    if not p.waitForInit():
        print "could not initialize Planner"
        exitError(2)
    oml = OctomapListener("/octomap_binary")
    rospy.sleep(1.0)
    octomap = oml.getOctomap(timeout_s=5.0)
    p.processWorld(octomap)

    print "Switching to jnt_mode..."
    switchToJntMode(velma) 



    print "Moving to position zero"
    moveToPositionZero(velma)

    print "Hiding both hands"
    hideBothHands(velma)

    print "Rotating robot..."
    # can position
    T_Wo_Can = velma.getTf("Wo", "target") 
    T_Wo_Table_0 = velma.getTf("Wo", "table0") 
    T_Wo_Table_1 = velma.getTf("Wo", "table1")

    target_table = findCanOnTable(T_Wo_Table_0, T_Wo_Table_1, T_Wo_Can) # na ktorym stoliku znajduje sie puszka

    Can_x = T_Wo_Can.p[0]   
    Can_y = T_Wo_Can.p[1]
    Can_z = T_Wo_Can.p[2]

    torso_angle = math.atan2(Can_y, Can_x)
    prepareForGrip(velma, torso_angle)
    
    switchToCartMode(velma)
    openRightHand(velma)


    print "Moving the right tool and equilibrium pose from 'wrist' to 'grip' frame..."
    T_B_Wr = velma.getTf("B", "Wr")
    T_Wr_Gr = velma.getTf("Wr", "Gr")
    if not velma.moveCartImpRight([T_B_Wr*T_Wr_Gr], [0.1], [T_Wr_Gr], [0.1], None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5):
        exitError(18)
    if velma.waitForEffectorRight() != 0:
        exitError(19)
    print "The right tool is now in 'grip' pose"
    rospy.sleep(0.5)

    print "Moving grip to can..."
    arm_frame = velma.getTf("Wo", "Gr")
    frame_nearby_can = PyKDL.Frame(arm_frame.M, T_Wo_Can.p+PyKDL.Vector(0, 0, 0.15))
    moveInCartImpMode(velma, frame_nearby_can)

    print "Grabbing the can..."
    grabWithRightHand(velma)

    switchToCartMode(velma)
    T_B_Wr = velma.getTf("B", "Wr")
    T_Wr_Gr = velma.getTf("Wr", "Gr")
    if not velma.moveCartImpRight([T_B_Wr*T_Wr_Gr], [0.1], [T_Wr_Gr], [0.1], None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5):
        exitError(18)   
    if velma.waitForEffectorRight() != 0:
        exitError(19)
    print "The right tool is now in 'grip' pose"
    rospy.sleep(0.5)
	
    print "Moving right gripper up..."	
    T_B_Trd = PyKDL.Frame(arm_frame.M, T_Wo_Can.p + PyKDL.Vector(0, 0, 0.2))
    if not velma.moveCartImpRight([T_B_Trd], [3.0], None, None, None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5):
        exitError(8)    
    if velma.waitForEffectorRight() != 0:
        exitError(9)
    rospy.sleep(0.5)

    #PyKDL.Vector(0.2*math.cos(torso_angle), 0.2*math.sin(torso_angle), -0.13)



    

    


    