#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker

_ACTIONS = [('N', [0,-1]),('E', [-1,0]),('S',[0,1]),('W',[1,0]),('NE',[-1,-1]),('NW',[1,-1]),('SE',[-1,1]),('SW',[1,1]), ('STAY',[0,0])]
_X = 0
_Y = 1

class stateNode():

def __init__(self):
    pub = rospy.Publisher('state', PointStamped, queue_size=10)
    markerPub = rospy.Publisher('robotMarker', Marker, queue_size=10)
    rospy.Subscriber("action", String, self.move_callback)

    rospy.init_node('stateNode', anonymous=True)

    rate = rospy.Rate(1)
    self.state = PointStamped()

    # initial starting location I might want to move to the param list
    self.h = rospy.get_param("height", 100)
    self.w = rospy.get_param("width", 100)
    self.state.point.x = self.h
    self.state.point.y = self.w
    self.state.point.z = 0

    self.robotMarker = Marker()
    self.robotMarker.header.frame_id = "/Cmap"
    self.robotMarker.header.stamp    = rospy.get_rostime()
    self.robotMarker.ns = "robot"
    self.robotMarker.id = 0
    self.robotMarker.type = 2 # sphere
    self.robotMarker.action = 0
    self.robotMarker.pose.position = self.state.point
    self.robotMarker.pose.orientation.x = 0
    self.robotMarker.pose.orientation.y = 0
    self.robotMarker.pose.orientation.z = 0
    self.robotMarker.pose.orientation.w = 1.0
    self.robotMarker.scale.x = 1.0
    self.robotMarker.scale.y = 1.0
    self.robotMarker.scale.z = 1.0

    self.robotMarker.color.r = 0.0
    self.robotMarker.color.g = 1.0
    self.robotMarker.color.b = 0.0
    self.robotMarker.color.a = 1.0

    self.robotMarker.lifetime = rospy.Duration(0)

    while not rospy.is_shutdown():
        pub.publish(self.state)
        markerPub.publish(self.robotMarker)
        print "sending marker", self.robotMarker
        rate.sleep()

def move_callback(self, action):
    for i in _ACTIONS:
        if String(i[0]) == action:

            self.state.point.x = self.state.point.x + i[1][_X]
            self.state.point.y = self.state.point.y + i[1][_Y]

            if self.state.point.x>self.h:
                self.state.point.x = self.h
            if self.state.point.y>self.w:
                self.state.point.y = self.w

            self.state.point.z = 0
            self.robotMarker.pose.position = self.state.point
            # print i
            print self.state.point.x, self.state.point.y

stateNode()