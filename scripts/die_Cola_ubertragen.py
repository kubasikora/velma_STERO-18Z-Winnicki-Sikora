#!/usr/bin/env python2



import roslib; roslib.load_manifest('velma_task_cs_ros_interface')
import rospy
import copy

from velma_common import *
from rcprg_planner import *
from rcprg_ros_utils import exitError

 # define a function for frequently used routine in this test
def planAndExecute(velma, q_dest):
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


def moveInCartMode(velma, T_B_dest):
    if not velma.moveCartImpRight([T_B_dest], [3.0], None, None, None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5):
        exitError(8)
    if velma.waitForEffectorRight() != 0:
        exitError(9)
    rospy.sleep(0.5)
    T_B_T_diff = PyKDL.diff(T_B_dest, velma.getTf("B", "Gr"), 1.0)
    if T_B_T_diff.vel.Norm() > 0.05 or T_B_T_diff.rot.Norm() > 0.05:
        exitError(10)

def prepareForGrip(velma, torso_angle):
    executable_q_map = copy.deepcopy(q_map_acquiring)
    executable_q_map['torso_0_joint'] = torso_angle
    planAndExecute(velma, executable_q_map)

def moveToInteractiveCursor(velma):
    print "Moving to interactive cursor..."
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

def hideBothHands(velma):
    dest_q = [90.0/180.0*math.pi, 90.0/180.0*math.pi, 90.0/180.0*math.pi, 180.0/180.0*math.pi]
    print "Hiding right fingers..."
    velma.moveHandRight(dest_q, [1,1,1,1], [2000,2000,2000,2000], 1000, hold=True)
    if velma.waitForHandRight() != 0:
        exitError(10)
    rospy.sleep(0.5)
    if not isHandConfigurationClose( velma.getHandRightCurrentConfiguration(), dest_q):
        exitError(11)

    print "Hiding left fingers..."
    velma.moveHandLeft(dest_q, [1,1,1,1], [2000,2000,2000,2000], 1000, hold=True)
    if velma.waitForHandRight() != 0:
        exitError(10)
    rospy.sleep(0.5)
    if not isHandConfigurationClose( velma.getHandRightCurrentConfiguration(), dest_q):
        exitError(11)

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

    moveToPositionZero(velma)
    hideBothHands(velma)

    print "Rotating robot..."
    # can position
    T_Wo_Can = velma.getTf("Wo", "target") 
    Can_x = T_Wo_Can.p[0]   
    Can_y = T_Wo_Can.p[1]
    Can_z = T_Wo_Can.p[2]

    torso_angle = math.atan2(Can_y, Can_x)
    prepareForGrip(velma, torso_angle)


    

    #T_B_Trd = PyKDL.Frame(T_Gr_orient.M, PyKDL.Vector(T_B_piwo_world.p[0]-math.cos(kat)*0.1,T_B_piwo_world.p[1]-math.sin(kat)*0.1,T_Gr_orient.p[2])) 



    