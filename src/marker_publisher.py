#! /usr/bin/env python
import rospy
import time
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

def publishMarkers(markerPublisher, tableLinePublisher, Bounds, CustomerLine, BarLine):
	rate = rospy.Rate(20)
	while not rospy.is_shutdown():
		markerCount = 0
		length = len(Bounds)
		markerArray = MarkerArray()
		points = []
		for i in range(0,length-1,2):
			marker = Marker()
			marker.header.frame_id = "map"
			marker.header.stamp = rospy.Time.now()
			marker.id = markerCount
			marker.type = 3
			marker.action = Marker.ADD
			marker.pose.position.x = Bounds[i]
			marker.pose.position.y = Bounds[i+1]
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
			points.append(Point(Bounds[i],Bounds[i+1],0))

		points.append(Point(Bounds[0],Bounds[1],0))
		marker = Marker()
		marker.header.frame_id = "map"
		marker.header.stamp = rospy.Time.now()
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

		marker = Marker()
		marker.header.frame_id = "map"
		marker.header.stamp = rospy.Time.now()
		marker.id = 0
		marker.type = 4
		marker.action = Marker.ADD
		marker.pose.position.z = 0.15
		marker.pose.orientation.w = 1.0
		marker.points = [Point(CustomerLine[0],CustomerLine[1],0),Point(CustomerLine[2],CustomerLine[3],0)]
		marker.scale.x = 0.05
		marker.color.a = 1.0
		marker.color.r = 0.0
		marker.color.b = 1.0
		marker.color.g = 1.0
		marker.frame_locked = True
		tableLinePublisher.publish(marker)

		marker = Marker()
		marker.header.frame_id = "map"
		marker.header.stamp = rospy.Time.now()
		marker.id = 1
		marker.type = 4
		marker.action = Marker.ADD
		marker.pose.position.z = 0.15
		marker.pose.orientation.w = 1.0
		marker.points = [Point(BarLine[0],BarLine[1],0),Point(BarLine[2],BarLine[3],0)]
		marker.scale.x = 0.05
		marker.color.a = 1.0
		marker.color.r = 0.0
		marker.color.b = 1.0
		marker.color.g = 1.0
		marker.frame_locked = True
		tableLinePublisher.publish(marker)

		rate.sleep()


if __name__ == '__main__':
	rospy.init_node("move_base_markers")
	markerPublisher = rospy.Publisher('/pr2_markers/legal_area', MarkerArray, queue_size=32)

	tableLinePublisher = rospy.Publisher('/pr2_markers/customer_line', Marker, queue_size=32)
	time.sleep(1)
	Bounds = rospy.get_param('pr2_area')
	CustomerLine = rospy.get_param('customer_line')
	BarLine = rospy.get_param('bar_line')
	publishMarkers(markerPublisher,tableLinePublisher, Bounds, CustomerLine, BarLine)
	#rospy.spin()
