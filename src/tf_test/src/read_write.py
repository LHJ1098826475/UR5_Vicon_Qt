#!/usr/bin/env python
import roslib
import rospy
import tf
import time
data1 = []
data2 = []
h = 0


if __name__=='__main__':
	rospy.init_node("read_write")

	listener = tf.TransformListener()

	rate = rospy.Rate(1)

	
	
                
        while not rospy.is_shutdown():
                         
                try:
                        
                        (trans, rot) = listener.lookupTransform('/vicon_root', '/vicon/Bar/Bar', rospy.Time(0))
                        
                except(tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                        
                        continue

                             
                data1[h:] = trans
                data2[h:] = rot

                #print("x: ", trans[0], "y: ", trans[1], "z: ", trans[2])
                #print("X: ", rot[0], "Y: ", rot[1], "Z: ", rot[2], "W: ", rot[3])
                        
                
                
                h += 1
                rate.sleep()

                if h >= 300:
                        break
        
                                
        f1 = open(r"\home\xtc\Desktop\data1.txt","w")
        f2 = open(r"\home\xtc\Desktop\data2.txt","w")
         
        for i in data1:

                
                f1.write(repr(i)+' ')
                print(i)
                  

        f1.close()
                                  

        for j in data2:

                f2.write(repr(j)+' ')
                
                    

        f2.close()
                        
                                        
	#print("x: ", trans[0], "y: ", trans[1], "z: ", trans[2])
	#print("x: ", rot[0], "y: ", rot[1], "z: ", rot[2], "w: ", rot[3])

	
