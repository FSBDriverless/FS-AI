import rospy
import math
import std_msgs.msg
import tf2_ros
import tf2_geometry_msgs
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from eufs_msgs.msg import ConeArray

def callbackLS(msg):
    rangeMin = msg.range_min
    rangeMax = msg.range_max
    angleIncrement = msg.angle_increment
    angleMin = msg.angle_min
    angleMax = msg.angle_max

    angleTotal = angleMax-angleMin
    laserTotal = angleTotal/angleIncrement

    ranges = msg.ranges

    i = 0
    a = 1

    distances = []

    for act in ranges:
        if act != float('inf'):

            angle = i * angleIncrement
            x = math.sin(angle) * act
            y = (math.cos(angle) * act)*(-1)
            distances.append((x, y, act))
            a = a + 1
        
        i = i + 1

    print("--------")

    flag = False
    cones = []
    aux = []

    arrayConos = ConeArray()
    arrayConos.header.frame_id = "base_footprint"
    arrayConos.blue_cones = []
    arrayConos.yellow_cones = []
    arrayConos.orange_cones = []
    arrayConos.big_orange_cones = []
    arrayConos.unknown_color_cones = []

    if len(distances) == 1:
        cones.append(distances[0])

    elif len(distances) > 1:
        prev = distances[0]
        for i in range(1, len(distances)):
            print(prev)
            act = distances[i]
                
            x1 = act[0]
            y1 = act[1]
            # rango1 = act[2]

            x0 = prev[0]
            y0 = prev[1]
            # rango0 = prev[2]

            if (pow(x0-x1, 2) <= 0.0225 and pow(y0-y1,2) <= 0.0225 ): #Es el mismo cono, este y el anterior: distancia 0,2

                flag = True
                aux.append(prev)

                if len(distances)-1 == i:
                    aux.append(act)
                    xList, yList, distList = zip(*aux)
                    xMean = sum(xList) / len(xList)
                    yMean = sum(yList) / len(yList)
                    distMean = sum(distList) / len(distList)

                    newCone = Point()
                    newCone.x = xMean
                    newCone.y = yMean
                    newCone.z = distMean
                    arrayConos.orange_cones.append(newCone)

            elif (flag): #Los anteriores eran el mismo cono, pero este no
                flag = False
                aux.append(prev)

                xList, yList, distList = zip(*aux)
                xMean = sum(xList)/len(xList)
                yMean = sum(yList)/len(yList)
                distMean = sum(distList)/len(distList)

                newCone = Point()
                newCone.x = xMean
                newCone.y = yMean
                newCone.z = distMean
                arrayConos.orange_cones.append(newCone)

                aux = []

                if len(distances)-1 == i:
                    cones.append(act)
                    newCone = Point()
                    newCone.x = act[0]
                    newCone.y = act[1]
                    newCone.z = act[2]
                    arrayConos.orange_cones.append(newCone)

            else:
                flag = False
                cones.append(prev)
                newCone = Point()
                newCone.x = prev[0]
                newCone.y = prev[1]
                newCone.z = prev[2]
                arrayConos.orange_cones.append(newCone)
                if len(distances)-1 == i:
                    cones.append(act)
                    newCone = Point()
                    newCone.x = act[0]
                    newCone.y = act[1]
                    newCone.z = act[2]
                    arrayConos.orange_cones.append(newCone)

            prev = act

    arrayConos.header.stamp = rospy.Time.now() # Probar
    conosPub.publish(arrayConos)
    # print(arrayConos)

    
    print('----------------------------------')

rospy.init_node('scan_values')
subLS = rospy.Subscriber('/scan', LaserScan, callbackLS)
conosPub = rospy.Publisher("/conoPercepcion/local", ConeArray, queue_size=10)
#subO = rospy.Subscriber('/ground_truth/odom', Odometry, callbackO)
rospy.spin()
