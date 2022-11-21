# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from nav_msgs.msg import Path
from eufs_msgs.msg import ConeArrayWithCovariance
import matplotlib.pyplot as plt
from matplotlib.offsetbox import OffsetImage, AnnotationBbox
import numpy as np
from scipy.spatial import Delaunay
from scipy.interpolate import splprep
from scipy.interpolate import splev
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class TrackMiddlePath(Node):
    def __init__(self, pathMarkerPublisher):
        super().__init__('track_middle_path')
        self.pathMarkerPublisher = pathMarkerPublisher
        self.subscription = self.create_subscription(
            ConeArrayWithCovariance,
            '/ground_truth/cones',
            self.callback_triangulacion,
            10)
        self.subscription  # prevent unused variable warning

        # to run GUI event loop
        # plt.ion()
        #
        # self.figure = plt.figure(figsize=(10, 10))
        # self.ax = self.figure.add_subplot(111)
        #
        # self.imgCoche = AnnotationBbox(self.getImage('img/ads.png', zoom=0.4), (0, 0), frameon=False)
        # self.ax.add_artist(self.imgCoche)
        #
        # self.conosPlt, = self.ax.plot([], [], '.', label='Conos', color='orange')
        # self.puntosMediosPlt, = self.ax.plot([], [], '.', label='Puntos Medios', color='blue')
        # self.interpolacionPlt, = self.ax.plot([], [], label='Interpolación', color='red')
        # self.triangulacionPlt = self.ax.triplot([0,0.1,0.1], [0,0.1,-0.1], [[0,1,2]], label='Triangulación', color='grey')
        #
        # plt.xlabel("X(m)")
        # plt.ylabel("Y(m)")
        # plt.title('Triangulación de delaunay \nsin bordes exteriores y \ncon puntos medios',
        #           fontsize=18,
        #           color="black")
        #
        # plt.legend()
    def callback_triangulacion(self, msg):
        blue_cones = msg.blue_cones
        yellow_cones = msg.yellow_cones

        triangulacion = Triangulacion(blue_cones, yellow_cones)
        waypoints, path = triangulacion.triangulacion()
        if (path != None):
            self.pathMarkerPublisher.publish_marker(path[0], path[1])

class Triangulacion():

    def __init__(self, conos_interiores, conos_exteriores):
        self.internalNp = None
        self.interpolacion = None
        self.conos_interiores = np.array([[o.point.x, o.point.y] for o in conos_interiores])
        self.conos_exteriores = np.array([[o.point.x, o.point.y] for o in conos_exteriores])
        # ESTO ES PROVISIONAL, Precondición: La lista de conos ya viene ordenada
        self.conos_exteriores = self.ordenar_respecto([0, 0], self.conos_exteriores)
        self.conos_interiores = self.ordenar_respecto([0, 0], self.conos_interiores)

    def ordenar_respecto(self, coche, lista):
        return np.array(sorted(lista, key=lambda p: (p[0] - coche[0]) ** 2 + (p[1] - coche[1]) ** 2))

    def getEdges(self, triangle, edges, isEven):
        # t0 t1
        if (isEven[0] + isEven[1] == 1):
            edges[triangle[0], triangle[1]] = 1
            edges[triangle[1], triangle[0]] = 1
        # t0 t2
        if (isEven[0] + isEven[2] == 1):
            edges[triangle[0], triangle[2]] = 1
            edges[triangle[2], triangle[0]] = 1
        # t1 t2
        if (isEven[1] + isEven[2] == 1):
            edges[triangle[1], triangle[2]] = 1
            edges[triangle[2], triangle[1]] = 1
        return edges

    def triangulacion(self):

        nci = len(self.conos_interiores)  # Numeros de conos interiores en la lista de conos
        col = 2
        nce = len(self.conos_exteriores)  # Numeros de conos exteriores en la lista de conos
        if (nci > 0 and nce > 0):

            maxConosLado = max(nci, nce)
            P = np.zeros((2 * maxConosLado, col))

            for i in range(0, maxConosLado):
                if (i >= nci):  # Solo quedan conos exteriores
                    P[i * 2] = self.conos_interiores[nci - 1]
                    P[i * 2 + 1] = self.conos_exteriores[i]
                elif (i >= nce):  # Solo quedan conos interiores
                    P[i * 2] = self.conos_interiores[i]
                    P[i * 2 + 1] = self.conos_exteriores[nce - 1]
                else:  # Quedan de los 2
                    P[i * 2] = self.conos_interiores[i]
                    P[i * 2 + 1] = self.conos_exteriores[i]

            internal = []

            xMaxI, yMaxI = self.conos_interiores.max(axis=0)
            xMinI, yMinI = self.conos_interiores.min(axis=0)

            xMaxE, yMaxE = self.conos_exteriores.max(axis=0)
            xMinE, yMinE = self.conos_exteriores.min(axis=0)

            xMax, yMax = max(xMaxI, xMaxE), max(yMaxI, yMaxE)
            xMin, yMin = max(xMinI, xMinE), max(yMinI, yMinE)

            # self.ax.set_xlim([-2.0, xMax + 1.0])
            # self.ax.set_ylim([yMin - 2.0, yMax + 2.0])

            # Posición del eje de dirección (No esta bien)
            dirX = 0.8
            dirY = 0.0

            internal.append(np.array([dirX, dirY]))  # Para despues hacer la trayectoria desde el morro del coche

            # edgesMatrix = np.zeros([P.shape[0], P.shape[0]])
            edgesMatrix = np.zeros([maxConosLado * 2, maxConosLado * 2])

            # Crear triangulacion con constantes
            TR = Delaunay(P)

            s = TR.simplices
            i = 0
            while (i < s.shape[0]):

                x = s[i]
                isEven = x % 2
                if ((isEven[0] == 0 and isEven[1] == 0 and isEven[2] == 0) or (
                        isEven[0] == 1 and isEven[1] == 1 and isEven[2] == 1)):
                    s = np.delete(s, i, 0)
                else:
                    edgesMatrix = self.getEdges(x, edgesMatrix, isEven)
                    i = i + 1

            for fila in range(0, P.shape[0]):
                for columna in range(0, fila):
                    if (edgesMatrix[fila][columna] == 1):  # Es uno interno
                        p1 = P[fila]
                        p2 = P[columna]

                        internal.append(np.array([(p1[0] + p2[0]) / 2, (p1[1] + p2[1]) / 2]))

            self.internalNp = np.array(internal)

            tck, u = splprep([self.internalNp[:, 0], self.internalNp[:, 1]], k=3, s=32)
            u = np.linspace(0, 1, num=50, endpoint=True)
            self.interpolacion = splev(u, tck)

            return self.internalNp, self.interpolacion
        else:
            return None, None

class PathMarkerPublisher(Node):

    def __init__(self, topic):
        self.topic = topic
        super().__init__('path_marker_publisher')
        self.publisher_ = self.create_publisher(Path, topic, 10)

    def publish_marker(self, pathX, pathY):
        msg = Path()
        msg.header.frame_id = "base_footprint"
        #msg.header.stamp = Node.get_clock().now()

        for i in range(0, len(pathX)):
            actX = pathX[i]
            actY = pathY[i]
            pose = PoseStamped()
            pose.pose.position.x = actX
            pose.pose.position.y = actY
            pose.pose.position.z = 0.0
            msg.poses.append(pose)

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    topicPathMarker = '/planificacion/path'
    pathMarkerPublisher = PathMarkerPublisher(topicPathMarker)
    # rclpy.spin(pathMarkerPublisher)
    trackMiddelPath = TrackMiddlePath(pathMarkerPublisher)

    rclpy.spin(trackMiddelPath)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)

    # pathMarkerPublisher.destroy_node()
    # trackMiddelPath.destroy_node()

    rclpy.shutdown()
if __name__ == '__main__':
    main()