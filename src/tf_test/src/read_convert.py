#!/user/bin/env python
import numpy as np
import roslib
import rospy
import tf
import time

f1 = open(r"C:\Users\21198_000\Destop\data1.txt")
f2 = open(r"C:\Users\21198_000\Destop\data2.txt")

L1 = f1.readlines()
L2 = f2.readlines()

lit1 = []
lit2 = []
sum = 0


for fields in L1:
    sum = sum + 1
    fields = fields.strip("[]")
    fields = fields.split()
    lit1.append(fields)

for lines in L2:
    lines = lines.strip("[]")
    lines = lines.split()
    lit2.append(lines)

f1.close()
f2.close()

rospy.init_node("read_convert")

listener = tf.TransformListener()


try:
    
    (trans, rot) = listener.lookupTransform('/vicon_world','/middle',rospy.Time(0))
    (trans1, rot1) = listener.lookupTransform('/vicon_world','/workpiece',rospy.Time(0))
    
    
except(tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
    
    continue

trans[1] = trans[1] + 65

Frame_workpiece_vicon = []
Frame_workpiece_vicon[0][0] = 1 - 2*rot1[1]**2 - 2*rot1[2]**2
Frame_workpiece_vicon[0][1] = 2*rot1[0]*rot1[1] - 2*rot1[3]*rot1[2]
Frame_workpiece_vicon[0][2] = 2*rot1[0]*rot1[2] + 2*rot1[3]*rot1[1]
Frame_workpiece_vicon[1][0] = 2*rot1[0]*rot1[1] + 2*rot1[3]*rot1[2]
Frame_workpiece_vicon[1][1] = 1 - 2*rot1[0]**2 - 2*rot1[2]**2
Frame_workpiece_vicon[1][2] = 2*rot1[1]*rot1[2] - 2*rot1[3]*rot1[0]
Frame_workpiece_vicon[2][0] = 2*rot1[0]*rot1[2] - 2*rot1[3]*rot1[1]
Frame_workpiece_vicon[2][1] = 2*rot1[1]*rot1[2] + 2*rot1[3]*rot1[0]
Frame_workpiece_vicon[2][2] = 1 - 2*rot1[0]**2 - 2*rot1[1]**2




Frame_base_vicon = []
Frame_base_vicon[0][0] = 1 - 2*rot[1]**2 - 2*rot[2]**2
Frame_base_vicon[0][1] = 2*rot[0]*rot[1] - 2*rot[3]*rot[2]
Frame_base_vicon[0][2] = 2*rot[0]*rot[2] + 2*rot[3]*rot[1]
Frame_base_vicon[1][0] = 2*rot[0]*rot[1] + 2*rot[3]*rot[2]
Frame_base_vicon[1][1] = 1 - 2*rot[0]**2 - 2*rot[2]**2
Frame_base_vicon[1][2] = 2*rot[1]*rot[2] - 2*rot[3]*rot[0]
Frame_base_vicon[2][0] = 2*rot[0]*rot[2] - 2*rot[3]*rot[1]
Frame_base_vicon[2][1] = 2*rot[1]*rot[2] + 2*rot[3]*rot[0]
Frame_base_vicon[2][2] = 1 - 2*rot[0]**2 - 2*rot[1]**2


Frame_base_vicon = np.array(Frame_base_vicon)
a = np.array([0 for _ in range(3)])
Frame_base_vicon = np.vstack((Frame_base_vicon, a))
b = np.array([[trans[0]],
              [trans[1]],
              [trans[2]],
              [1]])

Frame_base_vicon = np.hstack((Frame_base_vicon, b))
Frame_base_vicon.dtype = 'float64'



Frame_workpiece_vicon = np.array(Frame_workpiece_vicon)
Frame_workpiece_vicon = np.vstack((Frame_workpiece_vicon, a))
b1 = np.array([[trans1[0]],
               [trans1[1],
               [trans[2],
               [1]])

Frame_workpiece_vicon = np.hstack((Frame_workpiece_vicon, b1))
Frame_workpiece_vicon.dtype = 'float64'

               

Frame_base_vicon_inverse = np.linalg.inv(Frame_base_vicon)

Transfrom = np.dot(Frame_base_vicon_inverse, Frame_workpiece_vicon)


#result = np.dot(Frame_base_vicon, )
#Frame_tool_workpiece = np.array(Frame_tool_workpiece)
#Frame_tool_workpiece = np.vstack((Frame_tool_workpiece, a))


for i in range(100):
    for j in range(3):
        c = np.array([[lit1[i][j]],
                     [lit1[i][j]],
                     [lit1[i][j]],
                     [1]])

        result = np.dot(Transform, c)
        #Frame_tool_workpiece.dtype = 'float64'
        #median2 = np.dot(median1, Frame_tool_workpiece)
        #result = np.dot(median2,c)
        print(result)
        
        
        





    




    
