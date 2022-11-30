# PLANIFICACIÓN BRANCH

En esta rama encontraremos los desarrollos relacionados con el modulo de planificación, el cual se encargara de hacer la planificación de trayectoria dependiendo de la misión actual.

## Estructura del workspace:
```
~/FS-AI
  |__ img
  |   |...
  |
  |__ src
      |__ 0_fsb_common
      |   |...
      |
      |__ 1_perception
      |   |...
      |
      |__ 2_estimation
      |   |...
      |
      |__ 3_slam
      |   |...     
      |
      |__ 4_planification
      |   |__ src
      |       |__ delaunay_triangulation_midpoints
      |
      |__ 5_control
          |...
```

## Source del workspace
Suponiendo que el el proyecto está clonado en ~/FS-AI y tenemos una terminal abierta en esa dirección:
* Para construir el modulo solo con el paquete de delaunay_triangulation_midpoints
```
colcon build --packages-select delaunay_triangulation_midpoints
source ./install/setup.bash
```
* Para ejecutar el paquete que hace una planificación lineal:
```
ros2 run delaunay_triangulation_midpoints simple_middle_path  # Sin plot
ros2 run delaunay_triangulation_midpoints simple_middle_path --ros-args -p plot:=True
 # Con plot  
ros2 run delaunay_triangulation_midpoints simple_middle_path --ros-args -p plot:=False
 # Sin plot  
```
* Para ejecutar el paquete que hace una planificación mediante la triangulación de Delaunay:
```
ros2 run delaunay_triangulation_midpoints delaunay_middle_path  # Sin plot
ros2 run delaunay_triangulation_midpoints delaunay_middle_path --ros-args -p plot:=True
 # Con plot  
ros2 run delaunay_triangulation_midpoints delaunay_middle_path --ros-args -p plot:=False
 # Sin plot  
```