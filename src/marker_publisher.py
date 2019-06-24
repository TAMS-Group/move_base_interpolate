#! /usr/bin/env python
import rospy
import time
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

def publishMarkers(markerPublisher, bounds):
	markerCount = 0
	length = len(bounds)
	markerArray = MarkerArray()
	points = []
	# bounds are specified as x1,y1,x2,y2,...
	for i in range(0,length-1,2):
		marker = Marker()
		marker.header.frame_id = "map"
		marker.header.stamp = rospy.Time.now()
		marker.ns = "legal_area_corners"
		marker.id = markerCount
		marker.type = 3
		marker.action = Marker.ADD
		marker.pose.position.x = bounds[i]
		marker.pose.position.y = bounds[i+1]
		marker.pose.position.z = 0.5
		marker.pose.orientation.w = 1.0
		marker.scale.x = 0.1
		marker.scale.y = 0.1
		marker.scale.z = 1.0
		marker.color.a = 1.0
		marker.color.r = 1.0
		marker.color.b = 0.0
		marker.color.g = 0.0
		marker.frame_locked = True 
		markerCount += 1
		markerArray.markers.append(marker)
		points.append(Point(bounds[i],bounds[i+1],0))

	points.append(Point(bounds[0],bounds[1],0))
	marker = Marker()
	marker.header.frame_id = "map"
	marker.header.stamp = rospy.Time.now()
	marker.ns = "legal_area_border"
	marker.id = markerCount
	marker.type = 4
	marker.action = Marker.ADD
	marker.pose.orientation.w = 1.0
	marker.points = points
	marker.scale.x = 0.05
	marker.color.a = 1.0
	marker.color.r = 1.0
	marker.color.b = 0.0
	marker.color.g = 0.0
	marker.frame_locked = True

	markerArray.markers.append(marker)
	markerPublisher.publish(markerArray)


if __name__ == '__main__':
	rospy.init_node("move_base_markers")

	markerPublisher = rospy.Publisher('move_base/legal_area', MarkerArray, queue_size= 1, latch= True)

	bounds = rospy.get_param('move_base/legal_area')

	publishMarkers(markerPublisher, bounds)

	rospy.spin()
