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
from matplotlib.offsetbox import OffsetImage, AnnotationBbox
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import String
from eufs_msgs.msg import ConeArrayWithCovariance
from eufs_msgs.msg import ConeWithCovariance
import numpy as np
import math
from scipy.spatial import Delaunay
from scipy.interpolate import lagrange
from scipy.interpolate import InterpolatedUnivariateSpline
from scipy.interpolate import splprep
from scipy.interpolate import splrep
from scipy.interpolate import splev
from scipy.spatial.distance import squareform
from scipy.spatial.distance import pdist
import matplotlib.pyplot as plt
import triangle as tr
import time

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Path, '/path', 10)

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
        print('Publishing')

class MinimalSubscriber(Node):
    def ordenar_respecto(self, coche, lista):
        x = coche[0]
        y = coche[1]
        return sorted(lista, key=lambda p: (p[0] - x) ** 2 + (p[1] - y) ** 2)

    def getImage(self, path, zoom=1):
        return OffsetImage(plt.imread(path), zoom=zoom)

    def getEdges(self, triangle, edges, isEven):
        # t0 t1
        if (isEven[0] + isEven[1] == 1):
            # edges.append([triangle[0], triangle[1]])
            edges[triangle[0], triangle[1]] = 1
            edges[triangle[1], triangle[0]] = 1

        # t0 t2
        if (isEven[0] + isEven[2] == 1):
            # edges.append([triangle[0], triangle[2]])
            edges[triangle[0], triangle[2]] = 1
            edges[triangle[2], triangle[0]] = 1

        # t1 t2
        if (isEven[1] + isEven[2] == 1):
            # edges.append([triangle[1], triangle[2]])
            edges[triangle[1], triangle[2]] = 1
            edges[triangle[2], triangle[1]] = 1

        return edges

    def triangulacion(self, conos_interiores, conos_exteriores):
        print('triangulacion')
        points_interiores = np.array([[o.point.x, o.point.y] for o in conos_interiores])
        points_exteriores = np.array([[o.point.x, o.point.y] for o in conos_exteriores])

        # De precondición tendremos que la lista tiene que venir ordenada
        points_exteriores = self.ordenar_respecto([0, 0], points_exteriores)
        points_interiores = self.ordenar_respecto([0, 0], points_interiores)

        m = len(points_interiores)  # Numeros de conos pares en la lista P
        nc = len(points_interiores[0])  # Numeros de conos impares en la lista P
        mo = len(points_exteriores)
        interv = min(m, mo)

        P = np.zeros((interv * 2, nc))

        for i in range(0, interv * 2):
            if (i % 2 == 0):  # conos interiores
                P[i] = points_interiores[math.floor(i / 2)]
            else:
                P[i] = points_exteriores[math.floor(i / 2)]

        internal = []

        xmax, ymax = P.max(axis=0)
        xmin, ymin = P.min(axis=0)

        self.ax.set_xlim([-2.0, xmax + 1.0])
        self.ax.set_ylim([ymin - 2.0, ymax + 2.0])

        # Posición del eje de dirección (No esta bien)
        dirX = 2.0
        dirY = 0.0
        internal.append(np.array([dirX, dirY]))  # Para despues hacer la trayectoria desde el morro del coche

        edgesMatrix = np.zeros([P.shape[0], P.shape[0]])

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

        interna = np.array(internal)
        tck, u = splprep([interna[:, 0], interna[:, 1]], k=3, s=32)

        u = np.linspace(0, 1, num=50, endpoint=True)
        out = splev(u, tck)

        # self.interpolacionPlt.set_xdata(np.append(self.interpolacionPlt.get_xdata(), out[0]))
        # self.interpolacionPlt.set_ydata(np.append(self.interpolacionPlt.get_ydata(), out[1]))
        #
        # self.conosPlt.set_xdata(np.append(self.conosPlt.get_xdata(), P[:, 0]))
        # self.conosPlt.set_ydata(np.append(self.conosPlt.get_ydata(), P[:, 1]))
        #
        # self.puntosMediosPlt.set_xdata(np.append(self.puntosMediosPlt.get_xdata(), interna[:, 0]))
        # self.puntosMediosPlt.set_ydata(np.append(self.puntosMediosPlt.get_ydata(), interna[:, 1]))

        self.conosPlt.set_xdata(P[:, 0])
        self.conosPlt.set_ydata(P[:, 1])

        self.puntosMediosPlt.set_xdata(interna[:, 0])
        self.puntosMediosPlt.set_ydata(interna[:, 1])

        self.interpolacionPlt.set_xdata(out[0])
        self.interpolacionPlt.set_ydata(out[1])

        self.triangulacionPlt.pop(0).remove()
        self.triangulacionPlt = self.ax.triplot(P[:,0], P[:,1], s, color='grey')

        # drawing updated values
        self.figure.canvas.draw()

        # This will run the GUI event
        # loop until all UI events
        # currently waiting have been processed
        self.figure.canvas.flush_events()
        self.minimal_publisher.publish_marker(out[0], out[1])

    #def __init__(self, minimal_publisher):
    def __init__(self):
        super().__init__('minimal_subscriber')

        self.minimal_publisher = MinimalPublisher()
        #rclpy.spin(self.minimal_publisher)

        self.subscription = self.create_subscription(
            ConeArrayWithCovariance,
            '/ground_truth/cones',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        # to run GUI event loop
        plt.ion()

        self.figure = plt.figure(figsize=(10, 10))
        self.ax = self.figure.add_subplot(111)

        self.imgCoche = AnnotationBbox(self.getImage('img/ads.png', zoom=0.4), (0, 0), frameon=False)
        self.ax.add_artist(self.imgCoche)

        self.conosPlt, = self.ax.plot([], [], '.', label='Conos', color='orange')
        self.puntosMediosPlt, = self.ax.plot([], [], '.', label='Puntos Medios', color='blue')
        self.interpolacionPlt, = self.ax.plot([], [], label='Interpolación', color='red')
        self.triangulacionPlt = self.ax.triplot([0,0.1,0.1], [0,0.1,-0.1], [[0,1,2]], label='Triangulación', color='grey')

        plt.xlabel("X(m)")
        plt.ylabel("Y(m)")
        plt.title('Triangulación de delaunay \nsin bordes exteriores y \ncon puntos medios',
                  fontsize=18,
                  color="black")

        plt.legend()

    def listener_callback(self, msg):
        blue_cones = msg.blue_cones
        yellow_cones = msg.yellow_cones

        self.triangulacion(blue_cones, yellow_cones)


def main(args=None):
    rclpy.init(args=args)
    print(1)
    #minimal_publisher = MinimalPublisher()
    #rclpy.spin(minimal_publisher)
    print(2)
    #minimal_subscriber = MinimalSubscriber(minimal_publisher)
    minimal_subscriber = MinimalSubscriber()
    print(3)
    rclpy.spin(minimal_subscriber)
    print(4)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)

    #minimal_publisher.destroy_node()
    #minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
