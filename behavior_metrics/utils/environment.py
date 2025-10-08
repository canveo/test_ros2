#!/usr/bin/env python

"""This module contains the environment handler.
This module is in charge of loading and stopping gazebo and ros processes such as gazebo and ros launch files.
This program is free software: you can redistribute it and/or modify it under
the terms of the GNU General Public License as published by the Free Software
Foundation, either version 3 of the License, or (at your option) any later
version.
This program is distributed in the hope that it will be useful, but WITHOUT
ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
You should have received a copy of the GNU General Public License along with
this program. If not, see <http://www.gnu.org/licenses/>.
"""

import subprocess
import sys
import time
import os
import random

from utils.logger import logger
from utils.constants import ROOT_PATH, CARLA_TOWNS_SPAWN_POINTS

import xml.etree.ElementTree as ET

# TODO: remove absolute paths

__author__ = 'fqez'
__contributors__ = []
__license__ = 'GPLv3'


ros_version = os.environ.get('ROS_VERSION', '2')

def launch_env(launch_file, random_spawn_point=False, carla_simulator=False, config_spawn_point=None, config_town=None):
    """Launch the environmet specified by the launch_file given in command line at launch time.
    Arguments:
        launch_file {str} -- path of the launch file to be executed
    """
    # close previous instances of ROS and simulators if hanged.
    close_ros_and_simulators()
    
    try:
        spawn_point = None
        town = None
        tree = None
        root = None
        launch_file_path = None
        
        # detected ROS version
        ros_version = os.environ.get('ROS_VERSION', '2')   
        
        if carla_simulator:
            logger.debug(f"launch_file: {launch_file} ({type(launch_file)})")
            
            # case ROS 1 with .launch XML file
            if launch_file.endswith('.launch'):
                xml_path = os.path.join(ROOT_PATH, launch_file)
                tree = ET.parse(xml_path)
                root = tree.getroot()               

            # case ROS 2 with .launch.py (use towns file -.launch XML)
            elif launch_file.endswith('.launch.py') and ros_version == '2':
                xml_path = os.path.join(ROOT_PATH, launch_file.replace('.launch.py', '.launch'))
                if not os.path.exists(xml_path):
                    logger.warning(f"No XML 'twins' file found for {launch_file}, will not be readable town/spawn_point.")
                else:
                    tree = ET.parse(xml_path)
                    root = tree.getroot()
            else:
                logger.warning("launch_file is not supported for carla_simulator.")
                xml_path = None

            if tree is not None and root is not None:
                town = root.find(".//*[@name='town']")
                spawn_point = root.find(".//*[@name='spawn_point']")

                if town is None or spawn_point is None:
                    raise ValueError("Don't find town or spawn_point in the launch file XML")
                
                if random_spawn_point:
                    spawn_point.attrib['default'] = config_spawn_point or random.choice(CARLA_TOWNS_SPAWN_POINTS[town.attrib['default']])
                elif config_spawn_point and config_town:
                    spawn_point.attrib['default'] = config_spawn_point
                    town.attrib['default'] = config_town
                    
                # write temporary launch file with the selected town and spawn point
                tmp_launch = os.path.join(ROOT_PATH, 'tmp_circuit.launch')
                tree.write(tmp_launch)
                if ros_version == '1':
                    launch_file = 'tmp_circuit.launch'
                else:
                    # ROS 2 needs the .launch.py file, so copy it too
                    launch_file_path = os.path.join(ROOT_PATH, launch_file)
            else:
                # fallback to original launch file
                launch_file_path = os.path.join(ROOT_PATH, launch_file)

            # launch carla simulator
            with open("/tmp/.carlalaunch_stdout.log", "w") as out, open("/tmp/.carlalaunch_stderr.log", "w") as err:
                if ros_version == '2':
                    # tree = ET.parse(ROOT_PATH + '/' + launch_file.replace('.launch.py', '.launch'))
                    # root = tree.getroot()
                    quality = root.find(".//*[@name=\"quality\"]") if root is not None else None
                    # logger.info(f"SimulatorEnv: launching CARLA server with quality {quality.attrib['default'] if quality is not None else 'default'}")
                    if quality is not None and quality.attrib['default'] == 'Low':
                        subprocess.Popen([os.environ["CARLA_ROOT"] + "CarlaUE4.sh", "-RenderOffScreen", "-quality-level=Low"], stdout=out, stderr=err) 
                    else:
                        subprocess.Popen([os.environ["CARLA_ROOT"] + "CarlaUE4.sh", "-RenderOffScreen"], stdout=out, stderr=err)
                else:
                    # In ros1, quality value parser is passed as XML argument
                    # tree = ET.parse(ROOT_PATH + '/' + launch_file)
                    # root = tree.getroot()
                    quality = root.find(".//*[@name=\"quality\"]") if root is not None else None
                    if quality is not None:
                        subprocess.Popen([os.environ["CARLA_ROOT"] + "CarlaUE4.sh", "-RenderOffScreen"], stdout=out, stderr=err)
                    elif quality.attrib['default'] == 'Low':
                        subprocess.Popen([os.environ["CARLA_ROOT"] + "CarlaUE4.sh", "-RenderOffScreen", "-quality-level=Low"], stdout=out, stderr=err)
                    else:
                        subprocess.Popen([os.environ["CARLA_ROOT"] + "CarlaUE4.sh", "-RenderOffScreen"], stdout=out, stderr=err)
                    #subprocess.Popen(["/home/jderobot/Documents/Projects/carla_simulator_0_9_13/CarlaUE4.sh", "-RenderOffScreen", "-quality-level=Low"], stdout=out, stderr=err)
            time.sleep(5)
            
            # ROS (1 or 2) launch file
            with open("/tmp/.roslaunch_stdout.log", "w") as out, open("/tmp/.roslaunch_stderr.log", "w") as err:
                if ros_version == '2':
                    # launch_path = os.path.join(ROOT_PATH, launch_file)
                    # ros_cmd = ["ros2", "launch", ROOT_PATH, launch_file]
                    # package =  "behavior_metrics"  # file like package/launch/file.launch.py
                    launch_path = os.path.abspath(launch_file)        
                    ros_cmd = ['ros2', 'launch', launch_path]
                    
                else:
                    ros_cmd = ["roslaunch", launch_file_path]
                child = subprocess.Popen(ros_cmd, stdout=out, stderr=err)

        else:
            # launch ROS without carla simulator
            with open("/tmp/.roslaunch_stdout.log", "w") as out, open("/tmp/.roslaunch_stderr.log", "w") as err:
                # if os.environ.get('ROS_VERSION', '1') == '2':
                if ros_version == '2':
                    launch_path = os.path.join(ROOT_PATH, launch_file)
                    ros_cmd = ['ros2', 'launch', launch_path]
                    # ros_cmd = ["ros2", "launch", launch_file]
                else:
                    ros_cmd = ["roslaunch", launch_file]
                child = subprocess.Popen(ros_cmd,  shell=True, stdout=out, stderr=err, preexec_fn=os.setsid)
    except OSError as oe:
        logger.error("SimulatorEnv: exception raised launching simulator server. {}".format(oe))
        close_ros_and_simulators()
        sys.exit(-1)

    # give simulator some time to initialize
    time.sleep(10) 


def close_ros_and_simulators(close_ros_resources=True):
    """Kill all the simulators and ROS processes."""
    try:
        ps_output = subprocess.check_output(["ps", "-Af"]).decode('utf-8').strip("\n")
    except subprocess.CalledProcessError as ce:
        logger.error("SimulatorEnv: exception raised executing ps command {}".format(ce))
        sys.exit(-1)

    if ps_output.count('gzclient') > 0:
        try:
            subprocess.check_call(["killall", "-9", "gzclient"])
            logger.debug("SimulatorEnv: gzclient killed.")
        except subprocess.CalledProcessError as ce:
            logger.error("SimulatorEnv: exception raised executing killall command for gzclient {}".format(ce))

    if ps_output.count('gzserver') > 0:
        try:
            subprocess.check_call(["killall", "-9", "gzserver"])
            logger.debug("SimulatorEnv: gzserver killed.")
        except subprocess.CalledProcessError as ce:
            logger.error("SimulatorEnv: exception raised executing killall command for gzserver {}".format(ce))

    if ps_output.count('CarlaUE4.sh') > 0:
        try:
            subprocess.check_call(["killall", "-9", "CarlaUE4.sh"])
            logger.debug("SimulatorEnv: CARLA server killed.")
        except subprocess.CalledProcessError as ce:
            logger.error("SimulatorEnv: exception raised executing killall command for CARLA server {}".format(ce))

    if ps_output.count('CarlaUE4-Linux-Shipping') > 0:
        try:
            subprocess.check_call(["killall", "-9", "CarlaUE4-Linux-Shipping"])
            logger.debug("SimulatorEnv: CarlaUE4-Linux-Shipping killed.")
        except subprocess.CalledProcessError as ce:
            logger.error("SimulatorEnv: exception raised executing killall command for CarlaUE4-Linux-Shipping {}".format(ce))

    if ps_output.count('rosout') > 0 and close_ros_resources:
        try:
            import rosnode
            for node in rosnode.get_node_names():
                if node != '/carla_ros_bridge':
                    subprocess.check_call(["rosnode", "kill", node])

            logger.debug("SimulatorEnv:rosout killed.")
        except subprocess.CalledProcessError as ce:
            logger.error("SimulatorEnv: exception raised executing killall command for rosout {}".format(ce))

    if ps_output.count('bridge.py') > 0:
        try:
            os.system("ps -ef | grep 'bridge.py' | awk '{print $2}' | xargs kill -9")
            logger.debug("SimulatorEnv:bridge.py killed.")
        except subprocess.CalledProcessError as ce:
            logger.error("SimulatorEnv: exception raised executing killall command for bridge.py {}".format(ce))
        except FileNotFoundError as ce:
            logger.error("SimulatorEnv: exception raised executing killall command for bridge.py {}".format(ce))

    if ps_output.count('rosmaster') > 0 and close_ros_resources:
        try:
            subprocess.check_call(["killall", "-9", "rosmaster"])
            logger.debug("SimulatorEnv: rosmaster killed.")
        except subprocess.CalledProcessError as ce:
            logger.error("SimulatorEnv: exception raised executing killall command for rosmaster {}".format(ce))

    if ps_output.count('roscore') > 0 and close_ros_resources:
        try:
            subprocess.check_call(["killall", "-9", "roscore"])
            logger.debug("SimulatorEnv: roscore killed.")
        except subprocess.CalledProcessError as ce:
            logger.error("SimulatorEnv: exception raised executing killall command for roscore {}".format(ce))

    if ps_output.count('px4') > 0:
        try:
            subprocess.check_call(["killall", "-9", "px4"])
            logger.debug("SimulatorEnv: px4 killed.")
        except subprocess.CalledProcessError as ce:
            logger.error("SimulatorEnv: exception raised executing killall command for px4 {}".format(ce))

    if ps_output.count('roslaunch') > 0 and close_ros_resources:
        try:
            subprocess.check_call(["killall", "-9", "roslaunch"])
            logger.debug("SimulatorEnv: roslaunch killed.")
        except subprocess.CalledProcessError as ce:
            logger.error("SimulatorEnv: exception raised executing killall command for roslaunch {}".format(ce))
    
    if ps_output.count('rosout') > 0 and close_ros_resources:
        try:
            subprocess.check_call(["killall", "-9", "rosout"])
            logger.debug("SimulatorEnv:rosout killed.")
        except subprocess.CalledProcessError as ce:
            logger.error("SimulatorEnv: exception raised executing killall command for rosout {}".format(ce))
    
    if ps_output.count('carla_manual_control') > 0:
        try:
            subprocess.check_call(["killall", "-9", "carla_manual_control"])
            logger.debug("SimulatorEnv: carla_manual_control killed.")
        except subprocess.CalledProcessError as ce:
            logger.error("SimulatorEnv: exception raised executing killall command for carla_manual_control {}".format(ce))


def is_gzclient_open():
    """Determine if there is an instance of Gazebo GUI running
    Returns:
        bool -- True if there is an instance running, False otherwise
    """

    try:
        ps_output = subprocess.check_output(["ps", "-Af"], encoding='utf8').strip("\n")
    except subprocess.CalledProcessError as ce:
        logger.error("SimulatorEnv: exception raised executing ps command {}".format(ce))
        sys.exit(-1)

    return ps_output.count('gzclient') > 0


def close_gzclient():
    """Close the Gazebo GUI if opened."""

    if is_gzclient_open():
        try:
            subprocess.check_call(["killall", "-9", "gzclient"])
            logger.debug("SimulatorEnv: gzclient killed.")
        except subprocess.CalledProcessError as ce:
            logger.error("SimulatorEnv: exception raised executing killall command for gzclient {}".format(ce))


def open_gzclient():
    """Open the Gazebo GUI if not running"""

    if not is_gzclient_open():
        try:
            with open("/tmp/.roslaunch_stdout.log", "w") as out, open("/tmp/.roslaunch_stderr.log", "w") as err:
                subprocess.Popen(["gzclient"], stdout=out, stderr=err)
            logger.debug("SimulatorEnv: gzclient started.")
        except subprocess.CalledProcessError as ce:
            logger.error("SimulatorEnv: exception raised executing gzclient {}".format(ce))