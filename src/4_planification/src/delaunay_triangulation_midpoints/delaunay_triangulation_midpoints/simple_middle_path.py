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
from scipy.interpolate import splprep
from scipy.interpolate import splev
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class TrackMiddlePath(Node):
    def getImage(self, path, zoom=1):
        return OffsetImage(plt.imread(path), zoom=zoom)
    def __init__(self, pathMarkerPublisher, plot):
        super().__init__('track_middle_path')
        self.plot = plot
        self.pathMarkerPublisher = pathMarkerPublisher
        self.subscription = self.create_subscription(
            ConeArrayWithCovariance,
            '/ground_truth/cones',
            self.callback_planificacion,
            10)
        self.subscription  # prevent unused variable warning

        if self.plot:
            # to run GUI event loop
            plt.ion()

            self.figure = plt.figure(figsize=(10, 10))
            self.ax = self.figure.add_subplot(111)

            self.imgCoche = AnnotationBbox(self.getImage('img/ads.png', zoom=0.25), (0, 0), frameon=False)
            self.ax.add_artist(self.imgCoche)

            self.conosExtPlt, = self.ax.plot([], [], '.', markersize=15,label='Conos exteriores', color='blue')
            self.conosExtLinePlt, = self.ax.plot([0,1], [1,1], color='blue')
            self.conosIntPlt, = self.ax.plot([], [], '.', markersize=15, label='Conos interiores', color='#f6bd60')
            self.conosIntLinePlt, = self.ax.plot([0,1], [0,0], color='#f6bd60')
            self.puntosMediosPlt, = self.ax.plot([], [], '.', label='Puntos Medios', markersize=20,color='#a7c957')
            self.interpolacionPlt, = self.ax.plot([], [], label='Interpolación', color='red')
            self.conexionPlt = self.ax.plot([], [], color='grey')
            self.lines = []
            plt.xlabel("X(m)")
            plt.ylabel("Y(m)")
            plt.title('Planificación del centro de carril\nLINEAL',
                      fontsize=18,
                      color="black")

            plt.legend()
    def callback_planificacion(self, msg):
        blue_cones = msg.blue_cones
        yellow_cones = msg.yellow_cones

        simple_path = SimpleMiddlePath(blue_cones, yellow_cones)
        waypoints, path, P, s, x, y = simple_path.planificacion()

        if (path != None):
            self.pathMarkerPublisher.publish_marker(path[0], path[1])
            if self.plot: # Solo plotea si se lo hemos especificado por via parametros de ROS
                self.ax.set_xlim([-2.0, x[0] + 1.0])
                self.ax.set_ylim([y[1] - 2.0, y[0] + 2.0])

                rangoInt = range(0, P.shape[0], 2)
                self.conosIntPlt.set_xdata(P[rangoInt, 0])
                self.conosIntPlt.set_ydata(P[rangoInt, 1])

                self.conosIntLinePlt.set_xdata(P[rangoInt, 0])
                self.conosIntLinePlt.set_ydata(P[rangoInt, 1])

                rangoExt = range(1,P.shape[0],2)
                self.conosExtPlt.set_xdata(P[rangoExt, 0])
                self.conosExtPlt.set_ydata(P[rangoExt, 1])


                self.conosExtLinePlt.set_xdata(P[rangoExt, 0])
                self.conosExtLinePlt.set_ydata(P[rangoExt, 1])

                self.puntosMediosPlt.set_xdata(waypoints[:, 0])
                self.puntosMediosPlt.set_ydata(waypoints[:, 1])

                self.interpolacionPlt.set_xdata(path[0])
                self.interpolacionPlt.set_ydata(path[1])
                [line.pop(0).remove() for line in self.lines]
                self.lines=[]
                for i in range(0, P.shape[0], 2):
                    self.lines.append(self.ax.plot([P[i, 0], P[i + 1, 0]], [P[i, 1], P[i + 1, 1]], color='grey'))

                # drawing updated values
                self.figure.canvas.draw()

                # This will run the GUI event
                # loop until all UI events
                # currently waiting have been processed
                self.figure.canvas.flush_events()

class SimpleMiddlePath():
    conos_exteriores = None
    conos_interiores = None

    def __init__(self, conos_exteriores, conos_interiores):
        self.internalNp = None
        self.interpolacion = None
        # self.conos_exteriores = conos_exteriores
        # self.conos_interiores = conos_interiores
        self.conos_interiores = np.array([[o.point.x, o.point.y] for o in conos_interiores])
        self.conos_exteriores = np.array([[o.point.x, o.point.y] for o in conos_exteriores])

        # TODO, ESTO ES PROVISIONAL, Precondición: La lista de conos ya viene ordenada
        self.conos_exteriores = self.ordenar_respecto([0, 0], self.conos_exteriores)
        self.conos_interiores = self.ordenar_respecto([0, 0], self.conos_interiores)
        # self.conos_exteriores = np.array(self.conos_exteriores)
        # self.conos_interiores = np.array(self.conos_interiores)

    def ordenar_respecto(self, coche, lista):
        return np.array(sorted(lista, key=lambda p: (p[0] - coche[0]) ** 2 + (p[1] - coche[1]) ** 2))

    def planificacion(self):
        s = []
        nci = len(self.conos_interiores)  # Numeros de conos interiores en la lista de conos
        col = 2
        nce = len(self.conos_exteriores)  # Numeros de conos exteriores en la lista de conos
        if (nci > 0 and nce > 0):
            if (nci>nce): #Maximo 1 mas vamos a usar
                nci = nce+1
            elif(nce>nci): #Maximo 1 mas vamos a usar
                nce = nci+1
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
            xMin, yMin = min(xMinI, xMinE), min(yMinI, yMinE)

            # Posición del eje de dirección (No esta bien)
            dirX = 0.0
            dirY = 0.0

            internal.append(np.array([dirX, dirY]))  # Para despues hacer la trayectoria desde el morro del coche

            cant_conos = max(nci, nce)
            for i in range(0, cant_conos):
                interior = P[2 * i]
                exterior = P[2 * i + 1]
                internal.append(np.array([(interior[0] + exterior[0]) / 2, (interior[1] + exterior[1]) / 2]))
                s.append([2 * i, 2 * i + 1])

            self.internalNp = np.array(internal)

            tck, u = splprep([self.internalNp[:, 0], self.internalNp[:, 1]], k=3, s=32)
            u = np.linspace(0, 1, num=100, endpoint=True)
            self.interpolacion = splev(u, tck)

            return self.internalNp, self.interpolacion, P, s, (xMax, xMin), (yMax, yMin)
        else:
            return None, None, None, None, None, None


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
        print('Publishing path at ')
class Params(Node):
    def __init__(self):
        super().__init__('params_rclpy')
        self.declare_parameter('plot', False)
    def getPlot(self):
        return self.get_parameter('plot').value

def main(args=None):
    rclpy.init(args=args)
    topicPathMarker = '/planificacion/path'
    pathMarkerPublisher = PathMarkerPublisher(topicPathMarker)

    params = Params()
    plot = bool(params.getPlot())

    trackMiddelPath = TrackMiddlePath(pathMarkerPublisher, plot)
    rclpy.spin(trackMiddelPath)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)

    # pathMarkerPublisher.destroy_node()
    # trackMiddelPath.destroy_node()
    # params.destroy_node()

    rclpy.shutdown()
if __name__ == '__main__':
    main()