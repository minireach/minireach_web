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
from minireach_tasks.msg import AlvarMarkers

class TruckBroadcaster():

    def __init__(self):
        rospy.init_node('storage_location_broadcaster', anonymous=True)
        self.tf_listener = tf.TransformListener()

        self.pos_x = 0
        self.pos_y = 0
        self.pos_z = 0.1
        self.rot = None

        self.initMarker()
        self.point = Point(self.pos_x, self.pos_y, self.pos_z)
        rate = rospy.Rate(10.0)

        self.pallet_sub = rospy.Subscriber('/canvas_ar', AlvarMarkers, self.palletCallback)

        while not rospy.is_shutdown():
            self.updatePosition()
            #self.publishMarker()
            rate.sleep()

    def updatePosition(self):
        rospy.loginfo("Setting location")
        try:
            self.tf_listener.waitForTransform('/map','/base_footprint', rospy.Time(0), rospy.Duration(20.0))
            (trans, rot) = self.tf_listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
            self.pos_x = trans[0]
            self.pos_y = trans[1]

            euler = euler_from_quaternion(rot)
            self.rot = rot

            rospy.loginfo("Location in map x: %f, y: %f", self.pos_x, self.pos_y)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.loginfo("tf lookup for base_footprint failed")

    def palletCallback(self, pallets_message):
        for pallet in pallets_message.markers:
            self.PublishMarker(pallet.pose.pose)



    def initMarker(self):
        marker_scale = 0.3
        marker_lifetime = 0.5 # 0 if forever
        marker_ns = 'storage'
        marker_id = 0
        marker_color = {'r': 0.0, 'g': 0.0, 'b': 1.0, 'a': 1.0}

        self.marker_pub = rospy.Publisher('position_marker', Marker, queue_size=5)

        self.marker = Marker()
        self.marker.ns = marker_ns
        self.marker.id = marker_id
        self.marker.type = Marker.CUBE
        self.marker.action = Marker.ADD
        self.marker.lifetime = rospy.Duration(marker_lifetime)
        self.marker.scale.x = marker_scale
        self.marker.scale.y = marker_scale
        self.marker.color.r = marker_color['r']
        self.marker.color.g = marker_color['g']
        self.marker.color.b = marker_color['b']
        self.marker.color.a = marker_color['a']

        self.marker.header.frame_id = 'map'
        self.marker.header.stamp = rospy.Time.now()
        self.marker.points = list()


    def publishMarker(self, marker_pose):
        self.marker.pose = marker_pose
        self.marker_pub.publish(self.marker)


if __name__ == '__main__':
    try:
        TruckBroadcaster()
    except rospy.ROSInterruptException:
        rospy.loginfo("Pallet Broadcaster finished")
