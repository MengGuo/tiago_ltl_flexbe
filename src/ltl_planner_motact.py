#!/usr/bin/env python
import roslib
import numpy
import Queue
roslib.load_manifest('tiago_ltl_flexbe')
import rospy
import sys
import time




from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped

from math import pi as PI
from math import atan2, sin, cos, sqrt

from tf.transformations import euler_from_quaternion, quaternion_from_euler
from actionlib import SimpleActionClient, GoalStatus
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal

from robot_fts import robot_model
from ltl_tools.ts import MotionFts, ActionModel, MotActModel
from ltl_tools.planner import ltl_planner


def norm2(pose1, pose2):
    # 2nd norm distance
    return sqrt((pose1[0]-pose2[0])**2+(pose1[1]-pose2[1])**2)


def PoseCallback(posedata):
    # PoseWithCovarianceStamped data from amcl_pose
    global robot_pose # [time, [x,y,yaw]]
    header = posedata.header
    pose = posedata.pose
    if (not robot_pose[0]) or (header.stamp > robot_pose[0]):
        # more recent pose data received
        robot_pose[0] = header.stamp
        # TODO: maybe add covariance check here?
        # print('robot position update!')
        euler = euler_from_quaternion([pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w]) #roll, pitch, yaw
        robot_pose[1] = [pose.pose.position.x, pose.pose.position.y, euler[2]] # in radians
    return robot_pose


def SendGoal(GoalPublisher, goal, time_stamp):
    # goal: [x, y, yaw]
    GoalMsg = PoseStamped()
    #GoalMsg.header.seq = 0
    GoalMsg.header.stamp = time_stamp
    GoalMsg.header.frame_id = 'map'
    GoalMsg.pose.position.x = goal[0]
    GoalMsg.pose.position.y = goal[1]
    #GoalMsg.pose.position.z = 0.0
    quaternion = quaternion_from_euler(0, 0, goal[2])
    GoalMsg.pose.orientation.x = quaternion[0]
    GoalMsg.pose.orientation.y = quaternion[1]
    GoalMsg.pose.orientation.z = quaternion[2]
    GoalMsg.pose.orientation.w = quaternion[3]
    GoalPublisher.publish(GoalMsg)


def SendInitialPose(InitialPosePublisher, initial_pose, time_stamp):
    # goal: [x, y, yaw]
    InitialPoseMsg = PoseWithCovarianceStamped()
    #InitialPoseMsg.header.seq = 0
    InitialPoseMsg.header.stamp = time_stamp
    InitialPoseMsg.header.frame_id = 'map'
    InitialPoseMsg.pose.pose.position.x = initial_pose[0]
    InitialPoseMsg.pose.pose.position.y = initial_pose[1]
    #InitialPoseMsg.pose.position.z = 0.0
    quaternion = quaternion_from_euler(0, 0, initial_pose[2])
    InitialPoseMsg.pose.pose.orientation.x = quaternion[0]
    InitialPoseMsg.pose.pose.orientation.y = quaternion[1]
    InitialPoseMsg.pose.pose.orientation.z = quaternion[2]
    InitialPoseMsg.pose.pose.orientation.w = quaternion[3]
    InitialPosePublisher.publish(InitialPoseMsg)    


def planner(ts, init_pose, act, robot_task, robot_name='TIAGo'):
    global robot_pose
    robot_pose = [None, init_pose]
    rospy.init_node('ltl_planner_%s' %robot_name)
    print 'Robot %s: ltl_planner started!' %(robot_name)
    ###### publish to
    #----------
    #publish to
    #----------
    InitialPosePublisher = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size = 100)
    # for i in xrange(10):
    #     SendInitialPose(InitialPosePublisher, init_pose, rospy.Time.now())
    #     rospy.sleep(0.1)
    # print('Initial pose set to %s.' %str(init_pose))
    GoalPublisher = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size = 100)
    #----------
    #subscribe to
    #----------
    rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, PoseCallback)
    ####### robot information
    full_model = MotActModel(ts, act)
    planner = ltl_planner(full_model, robot_task, None)
    ####### initial plan synthesis
    planner.optimal(10)
    #######
    reach_xy_bound = 0.5 # m
    reach_yaw_bound = 0.5*PI # rad
    t0 = rospy.Time.now()
    while not rospy.is_shutdown():
        try:
            t = rospy.Time.now()-t0
            print '----------Time: %.2f----------' %t.to_sec()
            next_move = planner.next_move
            if isinstance(next_move, str):
                print 'Robot %s next move is action %s' %(str(robot_name), str(next_move))
                client = SimpleActionClient('/play_motion', PlayMotionAction)
                print 'Waiting for Action Server...'
                client.wait_for_server()
                goal = PlayMotionGoal()
                goal.motion_name = next_move
                goal.skip_planning = False
                goal.priority = 0  # Optional
                print 'Sending actionlib goal with motion: %s' %next_move
                client.send_goal(goal)
                print 'Waiting for result...'
                action_ok = client.wait_for_result(rospy.Duration(30.0))
                state = client.get_state()
                if state >= 1:
                    print 'Successful execution of action %s' %next_move
                    planner.find_next_move()
                else:
                    print 'Failed execution of action %s. Will retry' %next_move
            else:
                print 'Robot %s next move is motion to %s' %(str(robot_name), str(next_move))
                if ((norm2(robot_pose[1][0:2], next_move[0:2]) > reach_xy_bound) or (abs(robot_pose[1][2])-next_move[2]) > reach_yaw_bound):
                    SendGoal(GoalPublisher, next_move, t)
                    print('Goal %s sent to %s.' %(str(next_move),str(robot_name)))
                    rospy.sleep(10)
                else:
                    print('Goal %s reached by %s.' %(str(next_move),str(robot_name)))
                    planner.find_next_move()
        except rospy.ROSInterruptException:
            pass



if __name__ == '__main__':
    ########
    if len(sys.argv) == 2:
        robot_task = str(sys.argv[1])
        # to run: python ltl_planner.py '<> ((r2 && pick_from_floor) && <> (r3 && reach_max))'
    ###############
    try:
        [robot_motion, init_pose, robot_action] = robot_model
        planner(robot_motion, init_pose, robot_action, robot_task)
    except rospy.ROSInterruptException:
        pass
