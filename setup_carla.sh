#!/usr/bin/env zsh
# --- setup_carla.sh -------------------------------------------------
# Script de entorno para CARLA + BehaviorMetrics + ROS 2

export ROS_VERSION=2
export CARLA_ROOT="/home/canveo/carla-simulator"

export OBJECT_PATH="/home/canveo/Projects/BehaviorMetrics/behavior_metrics/configs/CARLA/CARLA_launch_files/CARLA_object_files/parked_car_objects_mDeepest.json"

# Añadimos los módulos Python de CARLA
export PYTHONPATH="$PYTHONPATH:$CARLA_ROOT/PythonAPI/carla"
export PYTHONPATH="$PYTHONPATH:$CARLA_ROOT/PythonAPI/carla/dist/carla-0.9.15-py3.10-linux-x86_64.egg"

# Activamos el entorno virtual
source ~/virtualenvs/bm-venv/bin/activate

# Cargamos la *overlay* de ROS 2 (en zsh; en bash sería `setup.bash`)
source ~/carla_ws/install/setup.zsh

# Cambio de directorio de proyectos
cd ~/Projects/test_ros2/behavior_metrics
