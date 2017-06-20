#!/usr/bin/env python

""" 
infinite_tray.py: This script commands the UR10 robot to follow a Lemniscata trajectory with its end-effector.

The script makes use of the MoveIt ROS package for the computation of the desired cartesian path. The display 
of the robot and the desired trajectory markers are shown in Rviz. 
"""

__author__ = "Ismael Baira Ojeda"
__maintainer__ = "Ismael Baira Ojeda"
__email__ = "iboj@elektro.dtu.dk, i.bairao@gmail.com"

import copy
import geometry_msgs.msg
import math
import moveit_commander
import moveit_msgs.msg
import rospy
import sys

import timeit
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


def getWaypoints(t, group):
    '''Waypoints follow a Lemniscate of Bernoulli'''
    waypoints = []
    # Start with the current pose
    print "============ Current Pose:\n", group.get_current_pose('ee_link')
    print "============ "
    waypoints.append(group.get_current_pose().pose)

    # Continue with desired trajectory
    for i in xrange(t):        
        t = math.radians(i * 6)
        a = 0.5
        wpose = geometry_msgs.msg.Pose()
        wpose.orientation.x = 0
        wpose.orientation.y = 0
        wpose.orientation.z = 1
        wpose.orientation.w = 1
        wpose.position.x = ( a * math.sqrt(2) * math.cos(t) ) / ( math.pow(math.sin(t), 2) + 1 )
        wpose.position.y = 0.40
        wpose.position.z = ( a * math.sqrt(2) * math.cos(t) * math.sin(t) ) / ( math.pow(math.sin(t), 2) + 1 ) + 1.0

        waypoints.append(copy.deepcopy(wpose))

    return waypoints



def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_ur10', anonymous=True)
    cartesian = rospy.get_param('~cartesian', True)     # Get param from ~private namespace and 
                                                        # initialize it (True) if it doesn't exist
    markerPub = rospy.Publisher('robotMarker', Marker, queue_size=10)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group = moveit_commander.MoveGroupCommander("my_ur10_limited")

    # Allow replanning to increase the odds of a solution 
    group.allow_replanning(True) 

    # Set the reference frame 
    group.set_pose_reference_frame('base_link') 

    # Allow some leeway in position(meters) and orientation (radians) 
    group.set_goal_position_tolerance(0.01) 
    group.set_goal_orientation_tolerance(0.1) 

    print "============ Planning reference frame: %s" % group.get_planning_frame()
    print "============ End effector: %s" % group.get_end_effector_link()

    print "============ Robot Groups available: ", robot.get_group_names()

    print "============ Printing robot state"
    print robot.get_current_state()
    print "============" 


    print "Starting cartesian path planning: Lemniscata trajectory"
    waypoints = getWaypoints(60, group)

    # Set the internal state to the current state 
    group.set_start_state_to_current_state() 

    # Plot waypoints in Rviz as a points marker type
    triplePoints = []
    for i, waypt in enumerate(waypoints):
        p = Point() 
        p.x = waypt.position.x
        p.y = waypt.position.y
        p.z = waypt.position.z
        triplePoints.append(p)

    marker = Marker()
    marker.header.frame_id = "/world"
    marker.type = 8                         # 8 = Points
    marker.action = 0                       # 0 = Add
    marker.pose.orientation.w = 1

    marker.points = triplePoints
    marker.lifetime = rospy.Duration(0)     # 0 = infinite
    marker.scale.x = 0.015
    marker.scale.y = 0.015
    marker.scale.z = 0.015

    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.color.g = 1.0
    marker.color.b = 1.0

    # Publishing waypoints
    print "Displaying desired waypoints"
    markerPub.publish(marker)        

    # Plan the Cartesian path connecting the waypoints 
    start_time = timeit.default_timer()

    ## We want the cartesian path to be interpolated at a resolution of 1 cm
    ## which is why we will specify 0.01 as the eef_step in cartesian
    ## translation.  We will specify the jump threshold as 0.0, effectively
    ## disabling it.
    if cartesian: 
        fraction = 0.0 
        maxtries = 100 
        attempts = 0 

        while fraction < 1.0 and attempts < maxtries: 
            (plan, fraction) = group.compute_cartesian_path ( 
                                        waypoints,   # waypoint poses 
                                        0.01,        # eef_step 
                                        0.0,         # jump_threshold 
                                        True )       # avoid_collisions 

            attempts += 1 

            # Print out a progress message 
            if attempts % 10 == 0: 
                rospy.loginfo("Still trying after " + str(attempts) + " attempts...") 
        
        elapsed = timeit.default_timer() - start_time
        print "Planning cartesian path took ", elapsed, " seconds."

    else:
        rospy.loginfo("Cartesian path computation not done: Private param '~cartesian' needs to be set to 'True'") 
    print "============ "

    # If we have a complete plan, execute the trajectory 
    if fraction == 1.0: 
        rospy.loginfo("Path computed successfully. Moving the arm...") 
        group.execute(plan) 
        rospy.loginfo("Path execution complete.") 
    else: 
        rospy.loginfo("Path planning failed with only " + str(fraction) + " success after " + str(maxtries) + " attempts.")    

    print "============ Final pose:\n", group.get_current_pose('ee_link').pose 
    moveit_commander.roscpp_shutdown()
    moveit_commander.os._exit(0) 


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass








