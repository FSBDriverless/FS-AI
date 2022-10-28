from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from eufs_msgs.msg import ConeArray
import rospy


def callback(msg):
    frame = msg.header.frame_id
    stamp = msg.header.stamp
    markerArray = MarkerArray()
    i = 0
    for cone in msg.orange_cones:
        marker = Marker()
        marker.header.frame_id = frame
        marker.header.stamp = stamp
        marker.type = Marker.MESH_RESOURCE
        marker.ns = ''
        marker.id = i
        marker.action = 0
        cone.z = 0.0
        marker.pose.position = cone
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1
        marker.scale.x = 1.5
        marker.scale.y = 1.5
        marker.scale.z = 1.5
        marker.color.r = 1.0
        marker.color.g = 0.270999997854
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.lifetime.secs = 0
        marker.lifetime.nsecs = 200000000
        marker.frame_locked = False
        marker.mesh_resource = "package://eufs_description/meshes/cone_big.dae"
        marker.mesh_use_embedded_materials = False
        print(marker)
        markerArray.markers.append(marker)
        i = i + 1
    pub.publish(markerArray)


rospy.init_node('cone_visual')
pub = rospy.Publisher("/visualCone", MarkerArray, queue_size=10)
subLS = rospy.Subscriber('/conoPercepcion/local', ConeArray, callback)
rospy.spin()