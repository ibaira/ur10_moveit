#!/usr/bin/env python

import copy
import geometry_msgs.msg
import math
import moveit_commander
import moveit_msgs.msg
import rospy
import sys


def getWaypoints(t, group):
    waypoints = []
    # start with the current pose
    print "Current Pose: ", group.get_current_pose('ee_link')
    waypoints.append(group.get_current_pose().pose)

    # continue with desired trajectory
    for i in xrange(t):
        ''' Waypoints follow a Lemniscate of Bernoulli '''
        t = math.radians(i * 6)
        a = 0.5
        wpose = geometry_msgs.msg.Pose()
        wpose.orientation.x = 0
        wpose.orientation.y = 0
        wpose.orientation.z = 1
        wpose.orientation.w = 1
        wpose.position.x = ( a * math.sqrt(2) * math.cos(t) ) / ( math.pow(math.sin(t), 2) + 1 )
        wpose.position.y = 0.60
        wpose.position.z = ( a * math.sqrt(2) * math.cos(t) * math.sin(t) ) / ( math.pow(math.sin(t), 2) + 1 ) + 0.8

        waypoints.append(copy.deepcopy(wpose))

    return waypoints



def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_ur10', anonymous=True)
    cartesian = rospy.get_param('~cartesian', True) 

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
        
    if cartesian: 
        fraction = 0.0 
        maxtries = 100 
        attempts = 0 

    # Set the internal state to the current state 
    group.set_start_state_to_current_state() 

    # Plan the Cartesian path connecting the waypoints 
    ## We want the cartesian path to be interpolated at a resolution of 1 cm
    ## which is why we will specify 0.01 as the eef_step in cartesian
    ## translation.  We will specify the jump threshold as 0.0, effectively
    ## disabling it.
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

    # If we have a complete plan, execute the trajectory 
    if fraction == 1.0: 
        rospy.loginfo("Path computed successfully. Moving the arm...") 
        group.execute(plan) 
        rospy.loginfo("Path execution complete.") 
    else: 
        rospy.loginfo("Path planning failed with only " + str(fraction) + " success after " + str(maxtries) + " attempts.") 

    print "Final pose: ", group.get_current_pose('ee_link').pose 
    moveit_commander.roscpp_shutdown()
    moveit_commander.os._exit(0) 


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass








