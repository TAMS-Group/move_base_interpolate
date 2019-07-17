#! /usr/bin/env python
import rospy
import time
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from move_base_interpolate import scaleArea

def publishMarkers(markerPublisher, bounds):
        SOFT_BOUND_PARAMETER = 'move_base/bounds_tolerance_scale'
        soft_bound_scale = 0.95
        
        try:
                soft_bound_scale = rospy.get_param(SOFT_BOUND_PARAMETER)
        except KeyError: # Parameters were not defined.
		rospy.loginfo("move_base/soft_bound was not defined.")

        soft_bounds = scaleArea(bounds, soft_bound_scale)

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

        soft_bound_points = []
        # bounds are specified as x1,y1,x2,y2,...
	for i in range(0,len(soft_bounds)-1,2):
        	soft_bound_points.append(Point(soft_bounds[i],soft_bounds[i+1],0.01))
	soft_bound_points.append(Point(soft_bounds[0],soft_bounds[1],0))
        
        soft_bound_marker = Marker()
	soft_bound_marker.header.frame_id = "map"
	soft_bound_marker.header.stamp = rospy.Time.now()
	soft_bound_marker.ns = "legal_area_softborder"
	soft_bound_marker.id = markerCount
	soft_bound_marker.type = Marker.LINE_STRIP
	soft_bound_marker.action = Marker.ADD
	soft_bound_marker.pose.orientation.w = 1.0
	soft_bound_marker.points = soft_bound_points
	soft_bound_marker.scale.x = 0.05
	soft_bound_marker.color.a = 1.0
	soft_bound_marker.color.r = 0.2
	soft_bound_marker.color.b = 0.2
	soft_bound_marker.color.g = 1.0
	soft_bound_marker.frame_locked = True
        
	markerArray.markers.append(marker)
	markerArray.markers.append(soft_bound_marker)
        markerPublisher.publish(markerArray)

if __name__ == '__main__':
	rospy.init_node("move_base_markers")

	markerPublisher = rospy.Publisher('move_base/legal_area', MarkerArray, queue_size= 1, latch= True)

	bounds = rospy.get_param('move_base/legal_area')

	publishMarkers(markerPublisher, bounds)

	rospy.spin()
