import rospy
import ros_numpy
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from std_msgs.msg import Header

#LIDAR: -0.15 0.0 0.79


def callback(pc2_msg):
    
    height = pc2_msg.height
    width = pc2_msg.width

    fields = pc2_msg.fields
    data = pc2_msg.data

    point_step = pc2_msg.point_step
    row_step = pc2_msg.row_step

    print(len(pc2_msg.data))

rospy.init_node('listener', anonymous=True)
rospy.Subscriber("/filtrado", PointCloud2, callback)
rospy.spin()
