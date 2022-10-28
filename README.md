# FSB Workspace
Este es el workspace de desarrollo driverless de Formula Student Bizkaia.


## Estructura del workspace:
```
~/fsbdriverless
  |__ build
  |__ devel
  |__ src
      |__ 0_fsb_common
      |   |__ fsb_common_meta
      |   |   |__ missions
      |   |
      |   |__ fsb_common_msg
      |   |__ src
      |   |   |__ pointcludo_to_laserscan
      |   |   |__ py
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
      |__ 3_slam
      |   |...
      |
      |__ 4_planification
      |   |...
      |
      |__ 5_control
          |...
```

## Source del workspace
Suponiendo que el el proyecto está clonado en $HOME:  
```
echo "source ~/FSBDriverless/workspace/devel/setup.bash">> ~/.bashrc
```
