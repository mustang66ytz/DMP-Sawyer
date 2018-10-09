#!/usr/bin/env python
import basic_move
import roslib; 
roslib.load_manifest('dmp')
import rospy 
import numpy as np
from scipy import interpolate
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from dmp.srv import *
from dmp.msg import *
import math

from std_msgs.msg import (
    Float64,
    Header
)
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from sensor_msgs.msg import JointState
from intera_core_msgs.msg import (
    JointCommand,
    EndpointState
)
from intera_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest
)
# below are for the force control:
from intera_core_msgs.msg import InteractionControlCommand
from intera_motion_interface import (
    MotionTrajectory,
    MotionWaypoint, 
    MotionWaypointOptions,
    InteractionOptions
)
import intera_interface


#Learn a DMP from demonstration data
def makeLFDRequest(dims, traj, dt, K_gain, 
                   D_gain, num_bases):
    # initialize a dmp trajectory message
    demotraj = DMPTraj()
        
    for i in range(len(traj)):
        # initialize a dmp point
        pt = DMPPoint();
        # assign the points in the trajectory to the DMP point
        pt.positions = traj[i]
        # append the new point to the trajectory
        demotraj.points.append(pt)
        demotraj.times.append(dt*i)
            
    k_gains = [K_gain]*dims
    d_gains = [D_gain]*dims
        
    print "Starting LfD..."
    rospy.wait_for_service('learn_dmp_from_demo')
    try:
        lfd = rospy.ServiceProxy('learn_dmp_from_demo', LearnDMPFromDemo)
        resp = lfd(demotraj, k_gains, d_gains, num_bases)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
    print "LfD done"    
            
    return resp;


#Set a DMP as active for planning
def makeSetActiveRequest(dmp_list):
    try:
        sad = rospy.ServiceProxy('set_active_dmp', SetActiveDMP)
        sad(dmp_list)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


#Generate a plan from a DMP
def makePlanRequest(x_0, x_dot_0, t_0, goal, goal_thresh, 
                    seg_length, tau, dt, integrate_iter):
    print "Starting DMP planning..."
    rospy.wait_for_service('get_dmp_plan')
    try:
        gdp = rospy.ServiceProxy('get_dmp_plan', GetDMPPlan)
        resp = gdp(x_0, x_dot_0, t_0, goal, goal_thresh, 
                   seg_length, tau, dt, integrate_iter)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
    print "DMP planning done"   
            
    return resp;

#Test the dmp in the 7 dof joint space robot
def dmpJointSpace(arm, input_x_list, input_y_list, input_z_list, x_0, goal):
    
    inputJoint = []
    for i in range(len(input_x_list)):
        wayPoint = Pose()
        wayPoint.position.x = input_x_list[i]
        wayPoint.position.y = input_y_list[i]
        wayPoint.position.z = input_z_list[i]
        wayPoint.orientation.x = 1
        wayPoint.orientation.y = 0
        wayPoint.orientation.z = 0
        wayPoint.orientation.w = 0
        
        jointPos = arm.waypointToJoint(wayPoint)
        inputJoint.append(jointPos)
    inputJoints = []
    for i in inputJoint:
        temp = []
        temp.append(i['right_j0'])
        temp.append(i['right_j1'])
        temp.append(i['right_j2'])
        temp.append(i['right_j3'])
        temp.append(i['right_j4'])
        temp.append(i['right_j5'])
        temp.append(i['right_j6'])
        inputJoints.append(temp)
    # customize some hyper-parameters
    dims = 7 # 7 dof robotic arm
    dt = 0.5 # incremental time
    K = 100 # proportional gain
    D = 2.0 * np.sqrt(K) # derivative gain
    num_bases = 8 # number of gaussian primitives
    resp = makeLFDRequest(dims, inputJoints, dt, K, D, num_bases)
    makeSetActiveRequest(resp.dmp_list)

    #Now, generate a plan
    outputStart = Pose()
    outputStart.position.x = x_0[0]
    outputStart.position.y = x_0[1]
    outputStart.position.z = x_0[2]
    outputStart.orientation.x = x_0[3]
    outputStart.orientation.y = x_0[4]
    outputStart.orientation.z = x_0[5]
    outputStart.orientation.w = x_0[6]
    
    temp = arm.waypointToJoint(outputStart)
    joints_0 = []
    joints_0.append(temp['right_j0'])
    joints_0.append(temp['right_j1'])
    joints_0.append(temp['right_j2'])
    joints_0.append(temp['right_j3'])
    joints_0.append(temp['right_j4'])
    joints_0.append(temp['right_j5'])
    joints_0.append(temp['right_j6'])
    
    joints_dot_0 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]   
    t_0 = 0                
    
    outputEnd = Pose()
    outputEnd.position.x = goal[0]
    outputEnd.position.y = goal[1]
    outputEnd.position.z = goal[2]
    outputEnd.orientation.x = goal[3]
    outputEnd.orientation.y = goal[4]
    outputEnd.orientation.z = goal[5]
    outputEnd.orientation.w = goal[6]
    
    temp = arm.waypointToJoint(outputEnd)
    joints_end = []
    joints_end.append(temp['right_j0'])
    joints_end.append(temp['right_j1'])
    joints_end.append(temp['right_j2'])
    joints_end.append(temp['right_j3'])
    joints_end.append(temp['right_j4'])
    joints_end.append(temp['right_j5'])
    joints_end.append(temp['right_j6'])

    goal_thresh = [0.2,0.2, 0.2, 0.1, 0.1, 0.1, 0.1]
    seg_length = -1          #Plan until convergence to goal
    tau = 8 * resp.tau       #Desired plan should take twice as long as demo
    dt = 0.2
    integrate_iter = 5       #dt is rather large, so this is > 1  
    plan = makePlanRequest(joints_0, joints_dot_0, t_0, joints_end, goal_thresh, 
                        seg_length, tau, dt, integrate_iter)
    print plan
    moveTheRobot(arm, plan)

def moveTheRobot(arm, plan):
    jointPositions = plan.plan.points
    nameList = ['right_j0', 'right_j1', 'right_j2', 'right_j3', 'right_j4', 'right_j5', 'right_j6']
    
    jointPosList = []
    for temp in jointPositions:
        jointList = []
        for i in range(7):
            jointList.append(temp.positions[i])
        jointPos = dict(zip(nameList, jointList))
        jointPosList.append(jointPos)
    # move the arm follow the generated trajectory
    arm.basicTrajMove(jointPosList, 0.2, len(jointPosList), 10)
        #arm.basicPositionMove(jointPos, 0.2)

if __name__ == '__main__':
    try:
        rospy.init_node('dmp_move_robot')
        #Create a DMP from a 2-D trajectory
        dims = 3                
        dt = 1.0                
        K = 100                 
        D = 2.0 * np.sqrt(K)      
        num_bases = 8
        
        start_pt = [-0.03, 0.74, 0.36]
        mid_pt1 = [-0.005, 0.86, 0.36]
        mid_pt2 = [0.23, 0.87, 0.26]
        end_pt = [0.65, 0.58, 0.20]
        traj = [start_pt, mid_pt1, mid_pt2, end_pt] 
        resp = makeLFDRequest(dims, traj, dt, K, D, num_bases)
        #Set it as the active DMP
        makeSetActiveRequest(resp.dmp_list)
        #Now, generate a plan
        x_0 = [-0.0075, 0.53, 0.57]          #Plan starting at a different point than demo 
        x_dot_0 = [0.0, 0.0, 0.0]   
        t_0 = 0                
        goal = [0.54, 0.67, 0.44]         #Plan to a different goal than demo
        goal_thresh = [0.2,0.2, 0.2]
        seg_length = -1          #Plan until convergence to goal
        tau = 5 * resp.tau       #Desired plan should take twice as long as demo
        dt = 0.5
        integrate_iter = 5       #dt is rather large, so this is > 1  
        plan = makePlanRequest(x_0, x_dot_0, t_0, goal, goal_thresh, 
                            seg_length, tau, dt, integrate_iter)
        #print plan
        print plan
        # plot the input trajectory;
        input_x_list = []
        input_y_list = []
        input_z_list = []
        for point in traj:
            input_x_list.append(point[0])
            input_y_list.append(point[1])
            input_z_list.append(point[2])
        # plot the output trajectory:
        position_x_list = []
        position_y_list = []
        position_z_list = []
        for i in plan.plan.points:
            position_x_list.append(i.positions[0])
            position_y_list.append(i.positions[1])
            position_z_list.append(i.positions[2])
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        ax.scatter3D(input_x_list, input_y_list, input_z_list, c=input_z_list, cmap='Greens')
        ax.scatter3D(position_x_list, position_y_list, position_z_list, c=position_z_list, cmap='Blues')
        plt.show()

        # test the dmp on the robot arm
        arm = basic_move.LowLevelMotion()
        move_speed = 0.2
        # change the start position
        start = [-0.0075, 0.53, 0.57, 1, 0, 0, 0]   
        # change the goal position
        goal = [0.54, 0.67, 0.44, 1, 0, 0, 0] 
        dmpJointSpace(arm, input_x_list, input_y_list, input_z_list, start, goal)


    except rospy.ROSInterruptException:
        pass