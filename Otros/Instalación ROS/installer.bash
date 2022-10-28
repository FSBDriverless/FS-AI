#! /bin/bash

# Script de instalación automática de ROS Melodic Ubuntu
# Información recogida en: http://wiki.ros.org/melodic/Installation/Ubuntu

NO_EXECUTION="FALSE"
ROS_DISTRO="galactic"
SHELL_RC_INSTALL=".bashrc"
VERSION_INSTALL="Desktop-Full"

############################################################
#                       HELP                               #
############################################################
function HELP(){
    # Display help
    echo "Script de instalación automática de ROS Melodic Ubuntu"
    echo
    echo -e "\t-h\t Imprime la ayuda del script."
    echo -e "\t-d\t Especifica cuál es la distribución de ROS a instalar (ROS galactic por defecto)"
    echo -e "\t\t Distribuciones ROS Ubuntu: ["galactic", "melodic", "noetic"]"
    echo -e "\t-s\t Especifica cuál es el archivo de configuración del shell (.bashrc por defecto)."
    echo -e "\t\t Si se establece vacío ("") no se hará un source automático de ROS en el shell"
    echo -e "\t-v\t Especifica la versión de ROS instalar (Galactic Desktop-Full por defecto)"
    echo -e "\t\t Versiones disponibles Ubuntu: "
    echo -e "\t\t\t GALACTIC ["Desktop-Full", "Desktop","ROS-Base"]"
    echo -e "\t\t\t MELODIC ["Desktop-Full", "Desktop","ROS-Base"]"
}

############################################################
#                       GALACTIC INSTALLATION              #
############################################################
function installGALACTICUBUNTU(){
    echo "Installing Ros Galactic Ubuntu $VERSION_INSTALL"
    echo "Taking in consideration the system is in UTF-8 format (\$locale)"

    sudo apt install software-properties-common
    sudo add-apt-repository universe
    sudo apt update && sudo apt install curl gnupg lsb-release
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

    case $VERSION_INSTALL in
        "Desktop-Full")
            sudo apt install ros-galactic-desktop 
            ;;
        "Desktop")
            sudo apt install ros-galactic-desktop 
            ;;
        "ROS-Base")
            sudo apt install ros-galactic-ros-base
            ;;
        *)
            sudo apt install ros-galactic-desktop # Opción por defecto
            ;;
    esac

    # Enviroment setup
    # To set an especific shell config file set venv SHELL_RC_INSTALL 

    path="/home/"$(id -un)"/"$SHELL_RC_INSTALL
    aux=$(cat $path | grep "source /opt/ros/$ROS_DISTRO/setup.bash")

    if [ -z "$aux" ]; then
        echo "Adding the enviroment setup..."
        $(echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> $path)
    else
        echo "Enviroment ready"
    fi

    source $path 
    
    # Dependencies for building packages
    echo "Installing dependencies for building packages..."
    sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
    sudo apt install python3-roslaunch

    # Initialize rosdep
    echo "Initializing rosdep..."
    sudo apt install python3-rosdep
    sudo rosdep init
    rosdep update
}

############################################################
#                       MELODIC INSTALLATION               #
############################################################
function installMELODICUBUNTU(){
    echo "Installing Ros Melodic Ubuntu $VERSION_INSTALL"
    case $VERSION_INSTALL in
        "Desktop-Full")
            sudo apt install ros-melodic-desktop-full
            ;;
        "Desktop")
            sudo apt install ros-melodic-desktop
            ;;
        "ROS-Base")
            sudo apt install ros-melodic-ros-base
            ;;
        *)
            sudo apt install ros-melodic-desktop-full # Opción por defecto
            ;;
    esac

    # Enviroment setup
    # To set an especific shell config file set venv SHELL_RC_INSTALL 

    path="/home/"$(id -un)"/"$SHELL_RC_INSTALL
    aux=$(cat $path | grep "source /opt/ros/$ROS_DISTRO/setup.bash")

    if [ -z "$aux" ]; then
        echo "Adding the enviroment setup..."
        $(echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> $path)
    else
        echo "Enviroment ready"
    fi

    source $path 
    
    # Dependencies for building packages
    echo "Installing dependencies for building packages..."
    sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential

    # Initialize rosdep
    echo "Initializing rosdep..."
    sudo apt install python-rosdep
    sudo rosdep init
    rosdep update
}

############################################################
#                       NOETIC INSTALLATION                #
############################################################
function installNOETICUBUNTU(){
    echo "Installing Ros Noetic Ubuntu $VERSION_INSTALL"
    case $VERSION_INSTALL in
        "Desktop-Full")
            sudo apt install ros-noetic-desktop-full
            ;;
        "Desktop")
            sudo apt install ros-noetic-desktop
            ;;
        "ROS-Base")
            sudo apt install ros-noetic-ros-base
            ;;
        *)
            sudo apt install ros-noetic-desktop-full # Opción por defecto
            ;;
    esac

    # Enviroment setup
    # To set an especific shell config file set venv SHELL_RC_INSTALL 

    path="/home/"$(id -un)"/"$SHELL_RC_INSTALL
    aux=$(cat $path | grep "source /opt/ros/$ROS_DISTRO/setup.bash")

    if [ -z "$aux" ]; then
        echo "Adding the enviroment setup..."
        $(echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> $path)
    else
        echo "Enviroment ready"
    fi

    source $path 
    
    # Dependencies for building packages
    echo "Installing dependencies for building packages..."
    sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential

    # Initialize rosdep
    echo "Initializing rosdep..."
    sudo apt install python3-rosdep
    sudo rosdep init
    rosdep update
}
############################################################
#                       UBUNTU INSTALLATION                #
############################################################
function installROSUBUNTU(){
    echo "Setting up your computer to accept software from packages.ros.org"
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

    # In case it is not installed
    echo "Checkign installation of curl"
    sudo apt install curl

    echo "Settin up your keys..."
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

    echo "Updating apt packages..."
    sudo apt update
    sudo apt upgrade -y

    # Installation
    # There are different default installations

    case $ROS_DISTRO in
        "galactic")installGALACTICUBUNTU;;
        "melodic")installMELODICUBUNTU;;
        "noetic")installNOETICUBUNTU;;
        *)installMELODICUBUNTU;;
    esac


}

############################################################
#                       MAIN                               #
############################################################
function main(){
   # Actualmente sólo está soportada la versión Melodic Ubuntu
   # Establece las opciones por defecto si no se han especificado otras
    echo "Iniciando script..."
    echo -e "Se va a porceder con la instalación de ROS $ROS_DISTRO Ubuntu [$VERSION_INSTALL]"
    echo -e "Se ha establecido el archivo de configuración del shell en: ~/$SHELL_RC_INSTALL"

    read -p "¿Desea continuar? [S/n]" opcion
    case $opcion in
        S);;
        s);;
        *)exit 0;;
    esac

    installROSUBUNTU
}


# Process the input options
while getopts "hd:s:v:" option; do
   case $option in
      h)
          HELP #Display help
          NO_EXECUTION="TRUE"
          ;; 
      d)ROS_DISTRO=$OPTARG;;
      s)SHELL_RC_INSTALL=$OPTARG;;
      v)VERSION_INSTALL=$OPTARG;;
      \?)echo "ERROR: opción inválida. Prueba con -h para ver la ayuda.";;

   esac
done

# Si no se ha preguntado por la ayuda ejecutamos el script
if [ "$NO_EXECUTION" = "FALSE" ]; then main; fi

exit 0
