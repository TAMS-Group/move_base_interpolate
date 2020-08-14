#!/usr/bin/env python
# Author v4hn@20200814

# Assuming a convex legal area (required anyway)
# sample a weighting between the waypoints
# and repeatedly target the weighted average

# Not uniformally distributed, but good enough for testing

import numpy as np
import rospy
import random
import tf.transformations

from geometry_msgs.msg import PoseStamped

rospy.init_node('random_motion')

publisher= rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size= 1)

rate = rospy.Rate(0.1)

area = rospy.get_param('move_base/legal_area')
area = np.reshape(area, (-1,2)).T

print ("Legal Area:")
print (area)

while not rospy.is_shutdown():
  target = PoseStamped()
  target.header.frame_id= "map"

  # discrete distribution
  w = np.random.random(area.shape[1])
  w = w/w.sum()

  # weighted target in convex legal area
  t = (area * w).sum(1)/ (len(area)/2)

  target.pose.position.x= t[0]
  target.pose.position.y= t[1]

  q = tf.transformations.quaternion_from_euler(0,0,np.pi*2*(random.random()-0.5))
  target.pose.orientation.x = q[0]
  target.pose.orientation.y = q[1]
  target.pose.orientation.z = q[2]
  target.pose.orientation.w = q[3]
  publisher.publish(target)
  rate.sleep()
