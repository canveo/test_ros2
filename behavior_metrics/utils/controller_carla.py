#!/usr/bin/env python

"""This module contains the controller of the application.

This application is based on a type of software architecture called Model View Controller. This is the controlling part
of this architecture (controller), which communicates the logical part (model) with the user interface (view).

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

import shlex
import subprocess
import threading
import cv2
# import rospy
import os
import time
# import rosbag
import json
import math
from utils.logger import logger

import pickle

ros_version = os.environ.get('ROS_VERSION',"2")

if ros_version == "2":
    import rclpy
    from rclpy.node import Node
else:
    import rospy
    import rosbag
    
try:
    import carla
except ModuleNotFoundError as ex:
    logger.error('CARLA is not supported')
    
from std_srvs.srv import Empty
from sensor_msgs.msg import Image as RosImage
from cv_bridge import CvBridge
from datetime import datetime
from std_msgs.msg import String
from utils import metrics_carla
from utils.constants import CARLA_INFRACTION_PENALTIES
try:
    from carla_msgs.msg import CarlaLaneInvasionEvent
    from carla_msgs.msg import CarlaCollisionEvent
except ModuleNotFoundError as ex:
    logger.error('CARLA is not supported')
from PIL import Image as PILImage

__author__ = 'sergiopaniego'
__contributors__ = []
__license__ = 'GPLv3'

METRICS_BACKEND = os.environ.get("METRICS_BACKEND") # "carla_api"


# debug. function to convert numpy types to native python types
def convert_np_to_native(obj):
        """Convert numpy types to native Python types."""
        if isinstance(obj, dict):
            return {k: convert_np_to_native(v) for k, v in obj.items()}
        elif isinstance(obj, list):
            return [convert_np_to_native(v) for v in obj]
        elif isinstance(obj, tuple):
            return tuple(convert_np_to_native(v) for v in obj)
        elif hasattr(obj, "item") and callable(obj.item):  # numpy scalar
            return obj.item()
        else:
            return obj

class ControllerCarla:
    """This class defines the controller of the architecture, responsible of the communication between the logic (model)
    and the user interface (view).

    Attributes:
        data {dict} -- Data to be sent to the view. The key is a frame_if of the view and the value is the data to be
        displayed. Depending on the type of data the frame handles (images, laser, etc)
        pose3D_data -- Pose data to be sent to the view
        recording {bool} -- Flag to determine if a rosbag is being recorded
    """
    
    def __init__(self, node: Node):
        """ Constructor of the class. """
        # pass        
        self.node = node
        self.__data_loc = threading.Lock()
        self.__pose_loc = threading.Lock()
        self.data = {}
        self.pose3D_data = None
        self.recording = False
        self.cvbridge = CvBridge()
        
        self.rosbag_proc = None
        self.proc = None

        client = carla.Client('localhost', 2000)
        client.set_timeout(100.0) # seconds
        try:
            self.world = client.get_world()
        except RuntimeError as e:
            logger.warning("CARLA RuntimeError: {}".format(e))
        except carla.TimeoutException as e:
            logger.warning("CARLA timeout al cerrar {}".format(e))
            self.world = None
            
        time.sleep(30) # takes a few second for the correct map to finish loading  ->debug original en 10
        self.carla_map = self.world.get_map()
        while len(self.world.get_actors().filter('vehicle.*')) == 0:
            logger.info("Waiting for vehicles!")
            time.sleep(1)
        ego_vehicle_role_name = "ego_vehicle"
        self.ego_vehicle = None
        while self.ego_vehicle is None:
            for vehicle in self.world.get_actors().filter('vehicle.*'):
                if vehicle.attributes.get('role_name') == ego_vehicle_role_name:
                    self.ego_vehicle = vehicle
                    break
            if self.ego_vehicle is None:
                logger.info("Waiting for vehicle with role_name 'ego_vehicle'")
                time.sleep(1)  # sleep for 1 second before checking again
        # TODO: agregar solo waypoints de la ruta deseada
        self.map_waypoints = self.carla_map.generate_waypoints(0.5)
        self.weather = self.world.get_weather()
        
        # campos para loggin con python api
        self.metrics_live_enabled = (METRICS_BACKEND == "carla_api")
        self._poses_xyz = []         # (x,y,z,t)
        self._speeds_mps = []        # v (m/s) o (v,t)
        self._controls = []          # (thr, steer, brake, t)
        self._collisions = []        # (x,y,t, other_actor_id)
        self._lane_invasions = []    # (x,y,t)

        self._collision_sensor = None
        self._lane_sensor = None
        self._tick_conn = None
        self._sim_start_time = None

               
    # GUI update
    def update_frame(self, frame_id, data):
        """Update the data to be retrieved by the view.

        This function is called by the logic to update the data obtained by the robot to a specific frame in GUI.

        Arguments:
            frame_id {str} -- Identifier of the frame that will show the data
            data {dict} -- Data to be shown
        """
        try:
            with self.__data_loc:
                self.data[frame_id] = data
        except Exception as e:
            logger.info("Error updating frame {}: {}".format(frame_id, e))

    def get_data(self, frame_id):
        """Function to collect data retrieved by the robot for an specific frame of the GUI

        This function is called by the view to get the last updated data to be shown in the GUI.

        Arguments:
            frame_id {str} -- Identifier of the frame.

        Returns:
            data -- Depending on the caller frame could be image data, laser data, etc.
        """
        try:
            with self.__data_loc:
                data = self.data.get(frame_id, None)
        except Exception:
            pass

        return data

    def update_pose3d(self, data):
        """Update the pose3D data retrieved from the robot

        Arguments:
            data {pose3d} -- 3D position of the robot in the environment
        """
        try:
            with self.__pose_loc:
                self.pose3D_data = data
        except Exception:
            pass

    def get_pose3D(self):
        """Function to collect the pose3D data updated in `update_pose3d` function.

        This method is called from the view to collect the pose data and display it in GUI.

        Returns:
            pose3d -- 3D position of the robot in the environment
        """
        return self.pose3D_data

    # Simulation and dataset
    def reset_carla_simulation(self):
        logger.info("Restarting simulation")

    def pause_carla_simulation(self):
        logger.info("Pausing simulation")
        self.pilot.stop_event.set()

    def unpause_carla_simulation(self):
        logger.info("Resuming simulation")
        self.pilot.stop_event.clear()

    def record_rosbag(self, topics, dataset_name):
        """Start the recording process of the dataset using rosbags

        Arguments:
            topics {list} -- List of topics to be recorde
            dataset_name {str} -- Path of the resulting bag file
        """
        if self.recording:
            logger.info("Rosbag already recording")
            self.stop_record()
            return
        
        self.recording = True
        
        if ros_version == "2":
            command = "ros2 bag record -o " + dataset_name + "/behav_bag" + " " + " ".join(topics)
        else:
            command = "rosbag record -O " + dataset_name + " " + " ".join(topics) + " __name:=behav_bag"
            
        logger.info("Recording bag at: {}".format(dataset_name))
        cmd_split = shlex.split(command)
        with open("./logs/.roslaunch_stdout.log", "w") as out, open("./logs/.roslaunch_stderr.log", "w") as err:
            self.rosbag_proc = subprocess.Popen(cmd_split, stdout=out, stderr=err)

    def stop_record(self):
        """Stop the rosbag recording process."""
        if not self.recording or not self.rosbag_proc:
            logger.info("No bag recording")
            return
        
        if ros_version == "2":
            self.rosbag_proc.terminate()
            self.rosbag_proc.wait()
            logger.info("Stopped bag recording")
        else:
            command = "rosnode kill /behav_bag"
            command_split = shlex.split(command)
            with open("./logs/.roslaunch_stdout.log", "w") as out, open("./logs/.roslaunch_stderr.log", "w") as err:
                subprocess.Popen(command_split, stdout=out, stderr=err)
        self.recording = False  
        self.rosbag_proc = None

    def reload_brain(self, brain, model=None):
        """Helper function to reload the current brain from the GUI.

        Arguments:
            brain {srt} -- Brain to be reloadaed.
        """
        logger.info("Reloading brain... {}".format(brain))

        self.pause_pilot()
        self.pilot.reload_brain(brain, model)

    # Helper functions (connection with logic)

    def set_pilot(self, pilot):
        self.pilot = pilot

    def stop_pilot(self):
        self.pilot.kill_event.set()

    def pause_pilot(self):
        self.pilot.stop_event.set()

    def resume_pilot(self):
        self.start_time = datetime.now()
        self.pilot.start_time = datetime.now()
        self.pilot.stop_event.clear()

    def initialize_robot(self):
        self.pause_pilot()
        self.pilot.initialize_robot()


    def record_metrics(self, metrics_record_dir_path, world_counter=None, brain_counter=None, repetition_counter=None):
        logger.info("Recording metrics bag: {}".format(metrics_record_dir_path))
        
        self.pilot.brain_iterations_real_time = []
        self.time_str = time.strftime("%Y%m%d-%H%M%S") 
        if world_counter is not None:
            current_world_head, current_world_tail = os.path.split(self.pilot.configuration.current_world[world_counter])
        else:
            current_world_head, current_world_tail = os.path.split(self.pilot.configuration.current_world)
        if brain_counter is not None:
            current_brain_head, current_brain_tail = os.path.split(self.pilot.configuration.brain_path[brain_counter])
        else:
            current_brain_head, current_brain_tail = os.path.split(self.pilot.configuration.brain_path)
        self.experiment_metrics = {
            'timestamp': self.time_str,
            'experiment_configuration': self.pilot.configuration.__dict__,
            'world_launch_file': current_world_tail,
            'brain_file': current_brain_tail,
            'robot_type': self.pilot.configuration.robot_type,
            'carla_map': self.carla_map.name,
            'ego_vehicle': self.ego_vehicle.type_id,
            'vehicles_number': len(self.world.get_actors().filter('vehicle.*')),
            'async_mode': self.pilot.configuration.async_mode,
            'weather': {
                'cloudiness': self.weather.cloudiness,
                'precipitation': self.weather.precipitation,
                'precipitation_deposits': self.weather.precipitation_deposits,
                'wind_intensity': self.weather.wind_intensity,
                'sun_azimuth_angle': self.weather.sun_azimuth_angle,
                'sun_altitude_angle': self.weather.sun_altitude_angle,
                'fog_density': self.weather.fog_density,
                'fog_distance': self.weather.fog_distance,
                'fog_falloff': self.weather.fog_falloff,
                'wetness': self.weather.wetness,
                'scattering_intensity': self.weather.scattering_intensity,
                'mie_scattering_scale': self.weather.mie_scattering_scale,
                'rayleigh_scattering_scale': self.weather.rayleigh_scattering_scale,
                },
        }
        if hasattr(self.pilot.configuration, 'experiment_model'):
            if brain_counter is not None:
                self.experiment_metrics['experiment_model'] = self.pilot.configuration.experiment_model[brain_counter]
            else:
                self.experiment_metrics['experiment_model'] = self.pilot.configuration.experiment_model

        if hasattr(self.pilot.configuration, 'experiment_name'):
            self.experiment_metrics['experiment_name'] = self.pilot.configuration.experiment_name

        if hasattr(self.pilot.configuration, 'experiment_name'):
            self.experiment_metrics['experiment_name'] = self.pilot.configuration.experiment_name
            self.experiment_metrics['experiment_description'] = self.pilot.configuration.experiment_description
            self.experiment_metrics['experiment_timeout'] = self.pilot.configuration.experiment_timeouts[world_counter]
            self.experiment_metrics['experiment_repetition'] = repetition_counter
        

        self.metrics_record_dir_path = metrics_record_dir_path
        os.mkdir(self.metrics_record_dir_path + self.time_str)
        self.experiment_metrics_bag_filename = os.path.join(self.metrics_record_dir_path, self.time_str, self.time_str)
        
        topics = [
            '/carla/npc_vehicle_1/odometry',
            '/carla/ego_vehicle/odometry',
            '/carla/ego_vehicle/collision',
            '/carla/ego_vehicle/lane_invasion',
            '/carla/ego_vehicle/speedometer',
            '/carla/ego_vehicle/vehicle_status',
            '/clock',
            '/carla/ego_vehicle/rgb_front/image',  #first image
            ]

        if ros_version == "2":
            command = "ros2 bag record -o " + self.experiment_metrics_bag_filename + " " + " ".join(topics)
        else:
            command = "rosbag record -O " + self.experiment_metrics_bag_filename + " " + " ".join(topics) + " __name:=behav_metrics_bag"
        
        command = shlex.split(command)
        with open("./logs/.roslaunch_stdout.log", "w") as out, open("./logs/.roslaunch_stderr.log", "w") as err:
            logger.info(f"Starting metrics bag recording with command: {' '.join(command)}")
            # time.sleep(12)  # debug
            self.proc = subprocess.Popen(command, stdout=out, stderr=err)
            
        # with open(self.experiment_metrics_bag_filename + '_metadata.json', 'w') as f:
        #     json.dump(self.experiment_metrics, f)
        logger.info("Started metrics bag recording")

    def stop_recording_metrics(self, termination_code=None, route_length=None):
        logger.info("Stopping metrics bag recording")
        end_time = time.time()

        if ros_version == "2":
            # command = "ros2 node kill /behav_metrics_bag"
            if self.proc:
                self.proc.terminate()
                self.proc.wait()
                logger.info("Stopped bag recording")
        else:
            command = "rosnode kill /behav_metrics_bag"
            command = shlex.split(command)
            with open("./logs/.roslaunch_stdout.log", "w") as out, open("./logs/.roslaunch_stderr.log", "w") as err:
                subprocess.Popen(command, stdout=out, stderr=err)

        timeout_counter = 20
        bag_active_file = self.experiment_metrics_bag_filename + '.active'
        while os.path.isfile(bag_active_file) and timeout_counter > 0:
            time.sleep(1)
            timeout_counter -= 1

        if timeout_counter <= 0:
            logger.warning(f"Timeout: {bag_active_file} not removed in time.")
            
        experiment_metrics_filename = self.metrics_record_dir_path + self.time_str + '/' + self.time_str
        try:
            self.experiment_metrics = metrics_carla.get_metrics(
                self.experiment_metrics,
                self.experiment_metrics_bag_filename,
                self.map_waypoints,
                experiment_metrics_filename,
                self.pilot.configuration
            )
            
        except Exception as e:
            logger.error(f"Error while processing metrics: {e}")
            self.experiment_metrics = { }

        self.experiment_metrics['experiment_total_real_time'] = end_time - self.pilot.pilot_start_time

        experiment_json_path = os.path.join(self.metrics_record_dir_path, self.time_str, self.time_str + '.json')
        os.makedirs(os.path.dirname(experiment_json_path), exist_ok=True)
        
        
        with open(experiment_json_path, 'w') as f:
            # json.dump(self.experiment_metrics, f)
            json.dump(convert_np_to_native(self.experiment_metrics), f)

        logger.info(f"Metrics stored in JSON file: {experiment_json_path}")
        logger.info("Stopped metrics bag recording")
   
    def save_metrics(self, first_images, last_images):        
        with open(self.metrics_record_dir_path + self.time_str + '/' + self.time_str + '.json', 'w') as f:
            json.dump(self.experiment_metrics, f)
        logger.info("Metrics stored in JSON file")

        for counter, image in enumerate(first_images):
            im = PILImage.fromarray(image)
            im.save(self.metrics_record_dir_path + self.time_str + '/' + self.time_str + "_first_image_" + str(counter) + ".jpeg")

        for counter, image in enumerate(last_images):
            im = PILImage.fromarray(image)
            im.save(self.metrics_record_dir_path + self.time_str + '/' + self.time_str + "_last_image_" + str(counter) + ".jpeg")
            
    