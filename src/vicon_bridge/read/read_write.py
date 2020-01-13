#!/usr/bin/env python
import roslib
import rospy
import tf


if __name__=='__main__':
	rospy.init_node("tf_test(modify)")

	listener = tf.TransformListener()

	rate = rospy.Rate(10)

	#f =open("data.txt","w")
	
	for i in range(100):
                
                while not rospy.is_shutdown():
                        
                        try:
                                (trans, rot) = listener.lookupTransform('/marker_0', '/marker_5', rospy.Time(0))
                        except(tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                                continue

                             
                        data1[i:] = trans
                        data2[i:] = rot

                        print("x: ", trans[0], "y: ", trans[1], "z: ", trans[2])
                        print("X: ", rot[0], "Y: ", rot[1], "Z: ", rot[2], "W: ", rot[3])
                        
                        time.sleep(0.5)
                                
         f1 = open(r"/home/xtc/Desktop/data1.txt","w")
         f2 = open(r"/home/xtc/Desktop/data2.txt","w")
         
         for i in data1:
                 
                 f1.write(repr(i)+'\n')

        f1.close()
                                  

        for j in data2:

                f2.write(repr(j)+'\n')

        f2.close()
                        
                                        
	print("x: ", trans[0], "y: ", trans[1], "z: ", trans[2])
	print("x: ", rot[0], "y: ", rot[1], "z: ", rot[2], "w: ", rot[3])

	rate.sleep()
