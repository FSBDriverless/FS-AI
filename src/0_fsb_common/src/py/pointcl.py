import rospy
import ros_numpy
import numpy as np
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from std_msgs.msg import Header

#LIDAR: -0.15 0.0 0.79


def callback(pc2_msg):
    
    height = pc2_msg.height
    width = pc2_msg.width
    
    realh = height/2

    point_step = pc2_msg.point_step
    row_step = pc2_msg.row_step

    step = width/6
    jump = step*point_step
    
    realw = width
    #realw = width-2*step
    realRowStep = row_step-2*jump

    fields = pc2_msg.fields
    data = pc2_msg.data
    
    index = row_step*realh

    print(index)
    print(len(data))

    realData = data[index:] 
    newData = []

    #for h in range(0, realh-1):
        #newData.append(realData[(point_step*h+jump):(point_step*(h+1)-jump)])
        #print(h)

    pc2_msg.height = realh
    pc2_msg.width = realw
    pc2_msg.data = realData
    #pc2_msg.data = newData
    #pc2_msg.row_step = realRowStep

    pub.publish(pc2_msg)
    
    
    
    xyz_array = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(pc2_msg)
    arr = np.array(xyz_array)
    print(arr.shape)
    print(len(xyz_array))
    print(len(xyz_array[0]))
    #print(len(xyz_array[0][0]))


    print('-----------------------')


rospy.init_node('listener', anonymous=True)
rospy.Subscriber("/zed/point_cloud/cloud_registered", PointCloud2, callback)
pub = rospy.Publisher("/filtrado", PointCloud2, queue_size=1)
rospy.spin()
