#!/usr/bin/env python

import rospy
import tf
import math
from visualization_msgs.msg import Marker, MarkerArray
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
        self.pallet_sub = rospy.Subscriber('/canvas_ar', AlvarMarkers, self.palletCallback)
        self.marker_pub = rospy.Publisher('position_marker', MarkerArray, queue_size=5)

        rospy.spin()

    def palletCallback(self, pallets_message):
        markers = []
        for pallet in pallets_message.markers:
            markers.append(pallet.pose.pose)
        rospy.loginfo("Publishing " + str(len(markers)) + "markers.")
        self.publishMarkers(markers)



    def initMarker(self, id=0):
        marker_scale = 0.3
        marker_lifetime = 0.5 # 0 if forever
        marker_ns = 'storage'
        marker_id = id
        marker_color = {'r': 0.0, 'g': 0.0, 'b': 1.0, 'a': 1.0}


        marker = Marker()
        marker.ns = marker_ns
        marker.id = marker_id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.lifetime = rospy.Duration(marker_lifetime)
        marker.scale.x = marker_scale
        marker.scale.y = marker_scale
        marker.color.r = marker_color['r']
        marker.color.g = marker_color['g']
        marker.color.b = marker_color['b']
        marker.color.a = marker_color['a']

        marker.header.frame_id = 'map'
        marker.header.stamp = rospy.Time.now()
        marker.points = list()

        return marker


    def publishMarkers(self, marker_poses):
        markers = MarkerArray()
        for id, marker_pose in enumerate(marker_poses):
            marker = self.initMarker(id=id)
            marker.pose = marker_pose
            markers.markers.append(marker)
        self.marker_pub.publish(markers)


if __name__ == '__main__':
    try:
        TruckBroadcaster()
    except rospy.ROSInterruptException:
        rospy.loginfo("Pallet Broadcaster finished")
