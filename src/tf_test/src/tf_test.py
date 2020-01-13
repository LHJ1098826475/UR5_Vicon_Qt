#!/usr/bin/env python
import roslib
import rospy
import tf

if __name__=='__main__':
	rospy.init_node("tf_test")

	listener = tf.TransformListener()

	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		
		try:
			(trans, rot) = listener.lookupTransform('/base', '/tool0', rospy.Time(0))
		except(tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			continue

		print("x: ", trans[0], "y: ", trans[1], "z: ", trans[2])
		print("x: ", rot[0], "y: ", rot[1], "z: ", rot[2], "w: ", rot[3])

		rate.sleep()
