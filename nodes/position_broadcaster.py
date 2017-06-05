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
		rospy.init_node('storage_location_broadcaster', anonymous=True)
		self.tf_listener = tf.TransformListener()
                rospy.Service('set_storage_location', Empty, self.setDeadReckoningGoal)
                rospy.Service('clear_storage_location', Empty, self.clearDeadReckoningGoal)

		self.found = False
		self.pos_x = 0
		self.pos_y = 0


		self.initMarkers()
		self.point = Point(self.pos_x, self.pos_y, 0)
		self.markers.points.append(self.point)
		rate = rospy.Rate(10.0)
		while not rospy.is_shutdown():
                        self.setDeadReckoningGoal()
                        self.publishMarkers()
                        self.broadcastPalletPosition()
			rate.sleep()

    def setDeadReckoningGoal(self):
        rospy.loginfo("Setting location")
        try:
            self.tf_listener.waitForTransform('/map','/base_footprint', rospy.Time(0), rospy.Duration(20.0))
            (trans, rot) = self.tf_listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
            self.pos_x = trans[0]
            self.pos_y = trans[1]

            euler = euler_from_quaternion(rot)
            self.rot_z = euler[2]

            self.found = True
            rospy.loginfo("Location in map x: %f, y: %f; rotoation z: %f", self.pos_x, self.pos_y, self.rot_z)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.loginfo("tf lookup for laser_link failed")
            self.found = False


    def clearDeadReckoningGoal(self, empty):
        self.found = False
        return EmptyResponse()


    def broadcastPalletPosition(self):
        br = tf.TransformBroadcaster()
        pallet_quaternion = quaternion_from_euler(0, 0, self.rot_z)
        br.sendTransform((self.pos_x, self.pos_y, 0),
                            pallet_quaternion,
                            rospy.Time.now(),
                            'storage_location',
                            'map')

    def initMarkers(self):
        marker_scale = 0.3
        marker_lifetime = 0.5 # 0 if forever
        marker_ns = 'storage'
        marker_id = 0
        marker_color = {'r': 0.0, 'g': 0.0, 'b': 1.0, 'a': 1.0}

        self.marker_pub = rospy.Publisher('position_marker', Marker, queue_size=5)

        self.markers = Marker()
        self.markers.ns = marker_ns
        self.markers.id = marker_id
        self.markers.type = Marker.CUBE_LIST
        self.markers.action = Marker.ADD
        self.markers.lifetime = rospy.Duration(marker_lifetime)
        self.markers.scale.x = marker_scale
        self.markers.scale.y = marker_scale
        self.markers.color.r = marker_color['r']
        self.markers.color.g = marker_color['g']
        self.markers.color.b = marker_color['b']
        self.markers.color.a = marker_color['a']

        self.markers.header.frame_id = 'map'
        self.markers.header.stamp = rospy.Time.now()
        self.markers.points = list()

    def publishMarkers(self):
        self.markers.points[0].x = self.pos_x
        self.markers.points[0].y = self.pos_y
        self.marker_pub.publish(self.markers)


if __name__ == '__main__':
    try:
        PalletBroadcaster()
    except rospy.ROSInterruptException:
        rospy.loginfo("Pallet Broadcaster finished")
