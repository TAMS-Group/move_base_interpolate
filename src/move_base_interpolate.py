#! /usr/bin/env python
import rospy
import math
import actionlib
import numpy as np
import move_base_msgs.msg
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Twist, Point
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import tf2_geometry_msgs
import tf2_ros
import sys

FLOAT_MAX = sys.float_info.max
FLOAT_MIN = sys.float_info.min

def clamp(x,min,max):
	if x < min:
		x = min
	if x > max:
		x = max
	return x

class Pose2D(object):
	def __init__(self,x,y,theta):
		self.x = x
		self.y = y
		self.theta = theta

	def updatePose(self,x,y,theta):
		self.x = x
		self.y = y
		self.theta = theta

def scaleArea(points,scale):
	xs = points[0::2]
	ys = points[1::2]
	centerx = sum(xs)/len(xs)
	centery = sum(ys)/len(ys)

	# scale*r = sqrt(scale^2*(x^2+y^2))
	scale_squared = scale*scale
	new_points = []
	for x,y in zip(xs,ys):
		relx = x - centerx
		rely = y - centery

		newx = (relx*scale_squared)+centerx
		newy = (rely*scale_squared)+centery

		new_points.append(newx)
		new_points.append(newy)

	return new_points

class MoveAction(object):
	def __init__(self, name, Bounds, bound_scale):
		self.x = None
		self.y = None
		self.theta = None

		self._action_name = name
		self._as = actionlib.SimpleActionServer("move_base", move_base_msgs.msg.MoveBaseAction, execute_cb=self.execute_cb, auto_start = False)

		self._move_base_wrapper_ac = actionlib.SimpleActionClient('move_base', move_base_msgs.msg.MoveBaseAction)

		rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.amcl_cb)

		self.publisher = rospy.Publisher('/base_controller/command', Twist, queue_size=10)
		self.goalmarkerPublisher = rospy.Publisher('move_base/goal_position', Marker, queue_size=32)
		self.areamarkerPublisher = rospy.Publisher('move_base/legal_area', MarkerArray, queue_size= 1, latch= True)

		self.SoftBound = Bounds
		self.HardBound = scaleArea(Bounds, bound_scale)

		self._tfBuffer = tf2_ros.Buffer()
		self._listener = tf2_ros.TransformListener(self._tfBuffer)

		self._feedback = move_base_msgs.msg.MoveBaseFeedback()
		self._result = move_base_msgs.msg.MoveBaseResult()

		self.publishAreaMarkers()

		self._as.start()
		rospy.Subscriber("move_base_simple/goal", PoseStamped, self.move_base_simple_cb)

		rospy.loginfo("move_base_interpolate startup successful")

	def move_base_simple_cb(self, data):
		print("resending move_base_simple/goal msg as an action")
		move_base_action_goal = move_base_msgs.msg.MoveBaseGoal(data)
		self._move_base_wrapper_ac.send_goal(move_base_action_goal)

	def amcl_cb(self, data):
		self.x = data.pose.pose.position.x
		self.y = data.pose.pose.position.y
		quaternion = data.pose.pose.orientation
		quaternion = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
		roll, pitch, yaw = euler_from_quaternion(quaternion)
		self.theta = yaw

	def publishAreaMarkers(self):
		markerCount = 0
		length = len(self.SoftBound)
		markerArray = MarkerArray()
		points = []

		# bounds are specified as x1,y1,x2,y2,...
		for i in range(0,length-1,2):
			marker = Marker()
			marker.header.frame_id = "map"
			marker.header.stamp = rospy.Time.now()
			marker.ns = "legal_area_corners"
			marker.id = markerCount
			marker.type = Marker.CYLINDER
			marker.action = Marker.ADD
			marker.scale.x = 0.1
			marker.scale.y = 0.1
			marker.scale.z = 0.3
			marker.pose.position.x = self.SoftBound[i]
			marker.pose.position.y = self.SoftBound[i+1]
			marker.pose.position.z = marker.scale.z/2
			marker.pose.orientation.w = 1.0
			marker.color.a = 1.0
			marker.color.r = 1.0
			marker.color.b = 0.0
			marker.color.g = 0.0
			marker.frame_locked = True
			markerCount += 1
			markerArray.markers.append(marker)
			points.append(Point(self.SoftBound[i],self.SoftBound[i+1],0))
		points.append(Point(self.SoftBound[0],self.SoftBound[1],0))

		marker = Marker()
		marker.header.frame_id = "map"
		marker.header.stamp = rospy.Time.now()
		marker.ns = "legal_area_border"
		marker.id = markerCount
		marker.type = Marker.LINE_STRIP
		marker.action = Marker.ADD
		marker.pose.orientation.w = 1.0
		marker.points = points
		marker.scale.x = 0.05
		marker.color.a = 1.0
		marker.color.r = 1.0
		marker.color.b = 0.0
		marker.color.g = 0.0
		marker.frame_locked = True

		hard_bound_points = []
		# bounds are specified as x1,y1,x2,y2,...
		for i in range(0,len(self.HardBound)-1,2):
			hard_bound_points.append(Point(self.HardBound[i],self.HardBound[i+1],0.01))
		hard_bound_points.append(Point(self.HardBound[0],self.HardBound[1],0))

		hard_bound_marker = Marker()
		hard_bound_marker.header.frame_id = "map"
		hard_bound_marker.header.stamp = rospy.Time.now()
		hard_bound_marker.ns = "legal_area_softborder"
		hard_bound_marker.id = markerCount
		hard_bound_marker.type = Marker.LINE_STRIP
		hard_bound_marker.action = Marker.ADD
		hard_bound_marker.pose.orientation.w = 1.0
		hard_bound_marker.points = hard_bound_points
		hard_bound_marker.scale.x = 0.05
		hard_bound_marker.color.a = 1.0
		hard_bound_marker.color.r = 0.2
		hard_bound_marker.color.b = 0.2
		hard_bound_marker.color.g = 1.0
		hard_bound_marker.frame_locked = True

		markerArray.markers.append(marker)
		markerArray.markers.append(hard_bound_marker)
		self.areamarkerPublisher.publish(markerArray)

	def execute_cb(self, goal):
		rate = rospy.Rate(20)
		success = True

		# Publish move_base goal marker
		quaternion = goal.target_pose.pose.orientation
		marker = Marker()
		marker.header.frame_id = "map"
		marker.header.stamp = rospy.Time.now()
		marker.id = 0
		marker.type = 0
		marker.action = Marker.ADD
		marker.pose.position.x = goal.target_pose.pose.position.x
		marker.pose.position.y = goal.target_pose.pose.position.y
		marker.pose.position.z = 0
		marker.pose.orientation = quaternion
		marker.scale.x = 1.0
		marker.scale.y = 0.1
		marker.scale.z = 0.1
		marker.color.a = 1.0
		marker.color.r = 0.0
		marker.color.b = 1.0
		marker.color.g = 0.0
		marker.frame_locked = True
		self.goalmarkerPublisher.publish(marker)

		pos = Pose2D(self.x,self.y,self.theta)

		# Check if PR2 is inside the bounds
		if not self.check_bounds(pos, self.HardBound):
			rospy.logwarn('Trixi is not in the legal area. Please use the joystick to navigate into the legal area.')
			#self._result.success = False
			#self._result.error_message = "Trixi is not in the legal area. Please use the joystick to navigate into the legal area."
			self._as.set_aborted(self._result)
			return

		# Check if the target is inside the bounds
		#self._transformer.waitForTransform('map', goal.target_pose.header.frame_id, rospy.Time(), rospy.Duration(2))
		goalMap = self._tfBuffer.transform(goal.target_pose,"map")
		ignored1,ignored2,yaw = euler_from_quaternion([goalMap.pose.orientation.x,goalMap.pose.orientation.y,goalMap.pose.orientation.z,goalMap.pose.orientation.w])
		goal2D = Pose2D(goalMap.pose.position.x, goalMap.pose.position.y, yaw)
		if not self.check_bounds(goal2D, self.SoftBound):
			rospy.logwarn('Requested point is outside the legal soft bound.')
			#self._result.success = False
			#self._result.error_message = "Requested point is outside the legal bounds."
			self._as.set_aborted(self._result)
			return

		integral_angle = 0
		integral_translation_x = 0
		integral_translation_y = 0
		while not rospy.is_shutdown():
			# Check if PR2 is inside the bounds
			pos.updatePose(self.x,self.y,self.theta)
			if not self.check_bounds(pos, self.HardBound):
				rospy.logwarn('Trixi is not in the legal area. Please use the joystick to navigate into the legal area.')
				#self._result.success = False
				#self._result.error_message = "Trixi is not in the legal area. Please use the joystick to navigate into the legal area."
				self._as.set_aborted(self._result)
				return

			if self._as.is_preempt_requested():
				rospy.loginfo('%s: Preempted' % self._action_name)
				self._as.set_preempted()
				success = False
				break
			try:
				goal.target_pose.header.stamp = rospy.Time() #rospy.get_rostime()
				goalBaseLink = self._tfBuffer.transform(goal.target_pose, 'base_footprint')
				dir = goalBaseLink.pose.position
				ignored1,ignored2,angle_diff = euler_from_quaternion([goalBaseLink.pose.orientation.x,goalBaseLink.pose.orientation.y,goalBaseLink.pose.orientation.z,goalBaseLink.pose.orientation.w])
				# TODO: fine tune on PR2
				# TODO: PI-controller
				translation_error = math.sqrt(dir.x**2 + dir.y**2)
				integral_angle += angle_diff
				integral_translation_x += dir.x
				integral_translation_y += dir.y
				if not(-0.087 < angle_diff < 0.087):  # ca. 5 degrees of tollerance
					message = Twist()
					max_ang_vel = math.pi/6
					# TODO fine tune on PR2
					message.angular.z = clamp(angle_diff*0.8,-max_ang_vel,max_ang_vel)
					self.publisher.publish(message)
				elif not (translation_error < 0.1):
					message = Twist()
					max_lin_vel = 0.15
					# TODO fine tune on PR2
					message.linear.x = clamp(dir.x,-max_lin_vel,max_lin_vel)
					message.linear.y = clamp(dir.y,-max_lin_vel,max_lin_vel)
					self.publisher.publish(message)
				else:
					break

				# TODO: fill out move_base feedback
				# should you use the amcl position or just ask tf for (0,0,0) base_footprint transformed to map
				self._as.publish_feedback(self._feedback)

			# TODO: doublecheck the documentation if these exceptions are still relevant in tf2_ros for the transform method
			except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
				print("failed to get transform")
				print("[" + str(type(e)) + "]" + str(e))
				rate.sleep()
				continue
			rate.sleep()

		if success:
			#self._result.success = True
			#self._result.error_message = "We successfully reached the specified goal."
			rospy.loginfo('Succeeded')
			self._as.set_succeeded(self._result)

	# follows https://rosettacode.org/wiki/Ray-casting_algorithm#Python
	# Checks if the goal is in the convex polygon with the corners in Bounds
	def check_bounds(self, goal, Bound):
		length = len(Bound)
		count = 0
		# iterating over the edges
		for i in range(0,length-1,2):

			ax = Bound[i]
			ay = Bound[i+1]
			bx = Bound[(i+2)%length]
			by = Bound[(i+3)%length]

			# makes sure b is the point with the greater y value
			if ay > by:
				tempx = ax
				tempy = ay
				ax = bx
				ay = by
				bx = tempx
				by = tempy
			if goal.y == ay or goal.y == by:
				goal.y += 0.0001
			# goal possabily not a point in the polygon
			# because it is greater than by or less than ay
			# or it is greater than ax and bx
			if (goal.y > by or goal.y < ay) or (goal.x > max(ax, bx)):
				continue
			# if the goal.x is less than ax and bx, increase count by 1
			# if it happens twice the goal is outside, because it is less than two edges
			if goal.x < min(ax, bx):
				count += 1
			# compare the slopes of the edge of a and b with the edge of a and the goal
			# only happens
			else:
				slope1 = 0.0
				slope2 = 0.0
				if abs(ax - bx) > FLOAT_MIN:
					slope1 = (by - ay) / float(bx - ax)
				else:
					slope1 = FLOAT_MAX
				if abs(ax - goal.x) > FLOAT_MIN:
					slope2 = (goal.y - ay) / float(goal.x - ax)
				else:
					slope2 = FLOAT_MAX
				# if the slope of the goal and a is greater than a and b, the goal is a possible 
				# point in the polygon
				if slope2 >= slope1:
					count += 1
		return count%2==1

if __name__ == '__main__':
	rospy.init_node("move_base")
	AREA_PARAMETER = 'move_base/legal_area'
	BOUND_TOLERANCE_PARAMETER = 'move_base/bounds_tolerance_scale'
	bounds_tolerance_scale = 1.05

	try:
		bounds_tolerance_scale = rospy.get_param(BOUND_TOLERANCE_PARAMETER)
	except KeyError: # Parameters were not defined.
		rospy.loginfo(BOUND_TOLERANCE_PARAMETER+" was not defined. Assuming "+str(bounds_tolerance_scale))
	if bounds_tolerance_scale < 1.0:
		rospy.logwarn(BOUND_TOLERANCE_PARAMETER+" should be >= 1.0 to ensure hard bounds outside the defined polygon. Assume 1.0")
		bounds_tolerance_scale = 1.0
	try:
		bounds = rospy.get_param(AREA_PARAMETER)

		server = MoveAction(rospy.get_name(), bounds, bounds_tolerance_scale)
		rospy.spin()
	except KeyError: # Parameters were not defined.
		rospy.logerr("No legal area was defined. Please define a convex, obstacle-free area in the " + AREA_PARAMETER + " parameter.")
