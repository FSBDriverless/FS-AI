import tf2_geometry_msgs
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PointStamped, Pose
from eufs_msgs.msg import ConeArray, PointArrayStamped
import tf2_ros
import rospy



conosTotales = []
markerArray = MarkerArray()
i = 0

def callback(msg):
    global conosTotales
    global markerArray
    global i

    from_frame = msg.header.frame_id
    to_frame = "map"
    stamp = rospy.Time(0)
    transform = getTransform(stamp, from_frame, to_frame)

    for cone in msg.orange_cones:
        dist = cone.z
        cone.z = 0.0
        coneTransformed = transform_point(transform, cone).point
        coneTransformed.z = dist
        flag = conoNuevo(coneTransformed)

        if not flag: # el cono no esta metido
            conosTotales.append(coneTransformed)
            marker = Marker()
            marker.header.frame_id = to_frame
            marker.header.stamp = stamp
            marker.type = Marker.MESH_RESOURCE
            marker.ns = str(dist)
            marker.id = i
            marker.action = 0
            marker.pose.position = coneTransformed
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1
            marker.scale.x = 1.5
            marker.scale.y = 1.5
            marker.scale.z = 1.5
            marker.color.r = 1.0
            marker.color.g = 0.470999997854
            marker.color.b = 0.0
            marker.color.a = 1.0
            marker.lifetime.secs = 0
            marker.lifetime.nsecs = 0
            marker.frame_locked = False
            marker.mesh_resource = "package://eufs_description/meshes/cone_big.dae"
            marker.mesh_use_embedded_materials = False
            # print(marker)
            markerArray.markers.append(marker)
            i = i + 1
    pub.publish(markerArray)

def conoNuevo (cono):
    global conosTotales

    i = 0
    flag = False
    while (i < len(conosTotales) and not flag):
        act = conosTotales[i]
        if (pow(act.x-cono.x,2)>=0.2 or pow(act.y-cono.y,2)>=0.2): #El cono no esta metido
            i = i + 1
        else: # El cono esta metido ya
            flag = True
    return flag

def getTransform(stamp, from_frame, to_frame):
    tf_buffer = tf2_ros.Buffer(rospy.Duration(100.0))  # tf buffer length
    tf2_ros.TransformListener(tf_buffer)
    transform = tf_buffer.lookup_transform(to_frame,
                                           # source frame:
                                           from_frame,
                                           # get the tf at the time the pose was valid
                                           stamp,
                                           # wait for at most 1 second for transform, otherwise throw
                                           rospy.Duration(1.0))
    return transform

def transform_point(transform, input_point):
    point = PointStamped()
    point.point = input_point
    point.header = transform.header

    return tf2_geometry_msgs.do_transform_point(point, transform)

    # # **Assuming /tf2 topic is being broadcasted
    # tf_buffer = tf2_ros.Buffer()
    # listener = tf2_ros.TransformListener(tf_buffer)
    #
    # pose_stamped = tf2_geometry_msgs.PoseStamped()
    # pose_stamped.pose = input_pose
    # pose_stamped.header.frame_id = from_frame
    # pose_stamped.header.stamp = rospy.Time.now()
    #
    # try:
    #     # ** It is important to wait for the listener to start listening. Hence the rospy.Duration(1)
    #     output_pose_stamped = tf_buffer.transform(pose_stamped, to_frame, rospy.Duration(1))
    #     return output_pose_stamped.pose
    #
    # except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
    #     raise

rospy.init_node('conos_global')
rospy.Subscriber('/conoPercepcion', ConeArray, callback)
pub = rospy.Publisher("/visualConeGlobal", MarkerArray, queue_size=10)
rospy.spin()