#!/usr/bin/env python2

import roslib; 
roslib.load_manifest('velma_task_cs_ros_interface')
import rospy
import copy
import math

from velma_common import *
#from rcprg_planner import *
import PyKDL

from rcprg_ros_utils import MarkerPublisher, exitError



 # define a function for frequently used routine in this test
def planAndExecute(velma, q_dest, pos_tol=10.0/180.0*math.pi):
    print "Moving to valid position"
    velma.moveJoint(q_dest, 2, start_time=0.5, position_tol=pos_tol)
    if velma.waitForJoint() != 0:
        exitError(4)

    rospy.sleep(0.5)
    js = velma.getLastJointState()
    if not isConfigurationClose(q_dest, js[1]):
        exitError(6)  

#funkcja realizujaca zmiane impedancji
def setImpedanceRight(velma, imp_p_x, imp_p_y, imp_p_z, imp_r_x, imp_r_y, imp_r_z):

    if not velma.moveCartImpRight(None, None, None, None, [PyKDL.Wrench(PyKDL.Vector(imp_p_x, imp_p_y, imp_p_z), PyKDL.Vector(imp_r_x, imp_r_y, imp_r_z))], [2], PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5):
        exitError(101)
    if velma.waitForEffectorRight() != 0:
        exitError(102)
    rospy.sleep(1)

def relativePosition(Transfer, deltaX, deltaY, deltaZ):
    (rotX, rotY, rotZ) = Transfer.M.GetRPY()
    posX = Transfer.p.x() + math.cos(rotZ)*deltaX - math.sin(rotZ)*deltaY
    posY = Transfer.p.y() + math.sin(rotZ)*deltaX + math.cos(rotZ)*deltaY
    posZ = Transfer.p.z() + deltaZ 
    angle = rotZ - math.pi
    if angle < -math.pi:
        angle = 2*math.pi + angle
    return [posX, posY, posZ, angle]

def moveInCartImpMode(velma, T_B_dest, tol = 10):
    if not velma.moveCartImpRight([T_B_dest], [5.0], None, None, None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.1, path_tol=PyKDL.Twist(PyKDL.Vector(tol, tol, tol), PyKDL.Vector(tol, tol, tol))):
        exitError(8)
    if velma.waitForEffectorRight() != 0:
        if not velma.moveCartImpRightCurrentPos(start_time=0.01):
            exitError(9)
        return False
    else:
        return True

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
    dest_q = [0.7*math.pi, 0.7*math.pi, 0.7*math.pi, math.pi]
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
    velma.moveJoint(q_map_starting, 3, start_time=0.5, position_tol=10.0/180.0*math.pi)
    if velma.waitForJoint() != 0:
        exitError(4)
    print "Moving head to position: 0"
    q_dest = (0,0)
    velma.moveHead(q_dest, 3.0, start_time=0.5)
    if velma.waitForHead() != 0:
	    exitError(5)
    rospy.sleep(0.5)
    if not isHeadConfigurationClose( velma.getHeadCurrentConfiguration(), q_dest, 0.1 ):
        exitError(6)

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

def openLeftHand(velma):
    dest_q = [0, 0, 0, 0]
    velma.moveHandLeft(dest_q, [1,1,1,1], [2000,2000,2000,2000], 1000, hold=True)
    if velma.waitForHandLeft() != 0:
        exitError(10)
    rospy.sleep(0.5)
    if not isHandConfigurationClose( velma.getHandLeftCurrentConfiguration(), dest_q):
        exitError(11)

def normalizeTorsoAngle(torso_angle):
    if torso_angle>math.pi/2:
        return math.pi/2-0.1
    elif torso_angle<-math.pi/2:
        return -math.pi/2+0.1
    else:
        return torso_angle

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

    rospy.init_node('proj2_executor')
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
    
    print "Switching to jnt_mode..."
    switchToJntMode(velma) 
    
    print "Moving to position zero"
    moveToPositionZero(velma)

    print "Hiding both hands"
    hideBothHands(velma)

    print "Rotating robot..."
    # cabinet position
    
    T_Wo_Cabinet = velma.getTf("Wo", "cabinet_door") 
    T_B_Cabinet = velma.getTf("B", "cabinet_door") 
    # do wyboru:
    # "cabinet_door"
    # "cabinet_door_right"
    # "cabinet_door_left"

    Cabinet_x = T_Wo_Cabinet.p[0]   
    Cabinet_y = T_Wo_Cabinet.p[1]
    Cabinet_z = T_Wo_Cabinet.p[2]

    torso_angle = normalizeTorsoAngle(math.atan2(Cabinet_y, Cabinet_x))
    
    print "torso angle: "
    print torso_angle
    prepareForGrip(velma, torso_angle)
    
    print "switching to cart mode..."
    switchToCartMode(velma)
        
    print "Moving gripper near to door..."
    (x, y, z, yaw) = relativePosition(T_B_Cabinet, 0.45, 0.1, 0.1)
    targetFrame = PyKDL.Frame(PyKDL.Rotation.RPY(0, 0, yaw), PyKDL.Vector(x, y, z))
    moveInCartImpMode(velma, targetFrame)
    
    print "Try to find door"
    setImpedanceRight(velma, 100, 100, 900, 200, 200, 200)

    (x, y, z, yaw) = relativePosition(T_B_Cabinet, 0.3, 0.1, 0.1)
    targetFrame = PyKDL.Frame(PyKDL.Rotation.RPY(0, 0, yaw), PyKDL.Vector(x, y, z))
    if moveInCartImpMode(velma, targetFrame, 0.1)==True:
        print "Door not found"
        exitError(404)
    else:
        print "Door found"
    

    switchToCartMode(velma)

    print "Move back after door found"
    (x, y, z, yaw) = relativePosition(T_B_Cabinet, 0.4, 0.1, 0.1)
    targetFrame = PyKDL.Frame(PyKDL.Rotation.RPY(0, 0, yaw), PyKDL.Vector(x, y, z))
    moveInCartImpMode(velma, targetFrame, 10)


    print "Move right hand a little bit to the left"
    (x, y, z, yaw) = relativePosition(T_B_Cabinet, 0.4, 0.0, 0.1)
    targetFrame = PyKDL.Frame(PyKDL.Rotation.RPY(0, 0, yaw), PyKDL.Vector(x, y, z))
    moveInCartImpMode(velma, targetFrame, 10)

    #target_pos = targetFrame.p
    #(target_x, target_y, target_z) = target_pos.p

    start_pos = velma.getTf("B", "Gr")
    (start_x, start_y, start_z) = start_pos.p


    print "changing impedance"
    setImpedanceRight(velma, 100, 100, 200, 100, 100, 100)
    
    #print "close right hand"
    #grabWithRightHand(velma)

    # Part 1 - jazda po prostej
    print "Part 1: pulling hand back"
    (stpt_x, stpt_y, stpt_z, yaw) = relativePosition(T_B_Cabinet, 0.7, 0, 0.15)
    targetFrame = PyKDL.Frame(PyKDL.Rotation.RPY(0, 0, yaw), PyKDL.Vector(stpt_x, stpt_y, stpt_z))
    moveInCartImpMode(velma, targetFrame, 10)

    act_pos = velma.getTf("B", "Gr")
    (act_x, act_y, act_z) = act_pos.p


    act_to_start = math.sqrt((act_x - start_x) ** 2 + (act_y - start_y) ** 2)
    act_to_stpt = math.sqrt((act_x - stpt_x) ** 2 + (act_y - stpt_y) ** 2)
    start_to_stpt = math.sqrt((start_x - stpt_x) ** 2 + (start_y - stpt_y) ** 2)

    cabinet_radius = start_to_stpt * act_to_start / act_to_stpt

    print cabinet_radius
    

    print "Part 2: open the door wider "
    print "go to point 1"
    (stpt_x, stpt_y, stpt_z, yaw) = relativePosition(T_B_Cabinet, 0.75, cabinet_radius/2, 0.15)
    targetFrame = PyKDL.Frame(PyKDL.Rotation.RPY(0, 0, yaw+45.0/180*math.pi), PyKDL.Vector(stpt_x, stpt_y, stpt_z))
    moveInCartImpMode(velma, targetFrame, 10)

    print "go to point 2"
    (stpt_x, stpt_y, stpt_z, yaw) = relativePosition(T_B_Cabinet, 0.75, cabinet_radius, 0.15)
    targetFrame = PyKDL.Frame(PyKDL.Rotation.RPY(0, 0, yaw+60.0/180*math.pi), PyKDL.Vector(stpt_x, stpt_y, stpt_z))
    moveInCartImpMode(velma, targetFrame, 10)

    print "go to point 3"
    (stpt_x, stpt_y, stpt_z, yaw) = relativePosition(T_B_Cabinet, 0.7, cabinet_radius + 0.3, 0.15)
    targetFrame = PyKDL.Frame(PyKDL.Rotation.RPY(0, 0, yaw+90.0/180*math.pi), PyKDL.Vector(stpt_x, stpt_y, stpt_z))
    moveInCartImpMode(velma, targetFrame, 10)

    rospy.sleep(2.0)
    print "release hand"
    #hideRightHand(velma)
    (stpt_x, stpt_y, stpt_z, yaw) = relativePosition(T_B_Cabinet, 0.30, cabinet_radius+0.4 , 0.15)
    targetFrame = PyKDL.Frame(PyKDL.Rotation.RPY(0, 0, yaw+80.0/180*math.pi), PyKDL.Vector(stpt_x, stpt_y, stpt_z))
    moveInCartImpMode(velma, targetFrame, 10)

    rospy.sleep(2.0)
    print "Back to default position"
    moveToPositionZero(velma)
    openLeftHand(velma)
    openRightHand(velma)

print "end"
    





    

    


    
