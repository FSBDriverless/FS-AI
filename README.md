# FSB Workspace
Este es el workspace de desarrollo driverless de Formula Student Bizkaia.


## Estructura del workspace:
```
~/fsbdriverless
  |__ build
  |__ devel
  |__ src
      |__ fsb_common
      |   |__ include
      |   |__ src
      |   |   |__ pointcludo_to_laserscan
      |   |   |__ py
      |
      |__ perception
      |   |...
      |
      |__ estimation
      |   |__ include
      |   |__ src
      |
      |__ control
          |...
```

## Source del workspace
Suponiendo que el el proyecto está clonado en $HOME:  
```
echo "source ~/FSBDriverless/workspace/devel/setup.bash">> ~/.bashrc
```
