#!/usr/bin/env python

import rospy
import tf
import math
from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from math import pi
from minireach_world_state.srv import *
from std_srvs.srv import *

class PalletBroadcaster():

    def __init__(self):
        rospy.init_node('location_marker', anonymous=True)
        self.tf_listener = tf.TransformListener()

        self.pos_x = 0
        self.pos_y = 0
        self.pos_z = 0.4

        self.initMarker()
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            self.getPose()
            self.publishMarkers()
            rate.sleep()

    def getPose(self):
        try:
            self.tf_listener.waitForTransform('/map','/laser_link', rospy.Time(0), rospy.Duration(4.0))
            (trans, rot) = self.tf_listener.lookupTransform('/map', '/laser_link', rospy.Time(0))
            self.pos_x = trans[0]
            self.pos_y = trans[1]

            euler = euler_from_quaternion(rot)
            self.rot_z = euler[2]

            rospy.loginfo("Storage location in map x: %f, y: %f; rotoation z: %f", self.pos_x, self.pos_y, self.rot_z)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.loginfo("tf lookup for laser_link failed")


    def initMarker(self):
        marker_scale = 1.0 
        marker_lifetime = 0.5 # 0 if forever
        marker_ns = 'location'
        marker_id = 0
        marker_color = {'r': 0.0, 'g': 0.0, 'b': 1.0, 'a': 1.0}

        self.pos_z = marker_scale/2 + 0.1 

        self.marker_pub = rospy.Publisher('position_marker', Marker, queue_size=5)

        self.markers = Marker()
        self.markers.ns = marker_ns
        self.markers.id = marker_id
        self.markers.type = Marker.SPHERE
        self.markers.action = Marker.ADD
        self.markers.lifetime = rospy.Duration(marker_lifetime)

        self.markers.scale.x = marker_scale
        self.markers.scale.y = marker_scale
        self.markers.scale.z = marker_scale


        self.markers.color.r = marker_color['r']
        self.markers.color.g = marker_color['g']
        self.markers.color.b = marker_color['b']
        self.markers.color.a = marker_color['a']

        self.markers.header.frame_id = 'map'
        self.markers.header.stamp = rospy.Time.now()

    def publishMarkers(self):
        self.markers.pose.position.x = self.pos_x
        self.markers.pose.position.y = self.pos_y
        self.markers.pose.position.z = self.pos_z
        self.marker_pub.publish(self.markers)


if __name__ == '__main__':
    try:
        PalletBroadcaster()
    except rospy.ROSInterruptException:
        rospy.loginfo("Location marker finished")
