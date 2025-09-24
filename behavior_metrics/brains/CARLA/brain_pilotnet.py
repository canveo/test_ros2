import numpy as np
import torch
import torch.nn as nn
import time
import cv2
import carla

from traceback import print_exc
from utils.logger import logger
from os import path
from utils.constants import PRETRAINED_MODELS_DIR, ROOT_PATH
from brains.CARLA.utils.high_level_command import HighLevelCommandLoader
from brains.CARLA.utils.pilotnet import PilotNet
# from brains.CARLA.utils.pilotnet_two_output import PilotNetTwoOutput


PRETRAINED_MODELS = ROOT_PATH + "/" + PRETRAINED_MODELS_DIR + "CARLA/"

class Brain:
    def __init__(self, sensors, actuators, model=None, handler=None, config=None):
        self.motors = actuators.get_motor("motors_0")
        self.camera_rgb = sensors.get_camera("camera_0")
        self.camera_seg = sensors.get_camera("camera_2")
        self.handler = handler
        self.inference_times = []
        self.gpu_inference = config.get("GPU", True)
        self.device = torch.device("cuda" if torch.cuda.is_available() and self.gpu_inference else "cpu")

        self.vehicle = self._wait_for_ego_vehicle()
        self.map = self.vehicle.get_world().get_map()
        self.vehicle.get_world().set_weather(carla.WeatherParameters.ClearNoon)

        if model:
            model_path = PRETRAINED_MODELS + model
            self.monolithic_model = PilotNet(image_shape=(3, 66, 200), num_labels=2, dropout_rate=0.3)
            self.monolithic_model.load_state_dict(torch.load(model_path, map_location=self.device))
            self.monolithic_model.to(self.device)
            self.monolithic_model.eval()
            print(f"Loaded PilotNet model from: {model_path}")

        route = config.get("Route", None)
        if route:
            print("Route loaded:", route)

        self.hlc_loader = HighLevelCommandLoader(self.vehicle, self.map, route=route)
        self.prev_hlc = 0
        self.prev_yaw = None
        self.delta_yaw = 0
        self.target_point = None
        self.termination_code = 0

    def _wait_for_ego_vehicle(self):
        client = carla.Client("localhost", 2000)
        client.set_timeout(100.0)
        world = client.get_world()
        vehicle = None
        while vehicle is None:
            for actor in world.get_actors().filter("vehicle.*"):
                if actor.attributes.get("role_name") == "ego_vehicle":
                    vehicle = actor
                    break
            if vehicle is None:
                print("Waiting for vehicle with role_name 'ego_vehicle'")
                time.sleep(1)
        return vehicle

    def update_frame(self, frame_id, data):
        if data.shape[0] != data.shape[1]:
            h, w = data.shape[:2]
            top, bottom, left, right = 0, 0, 0, 0
            if h > w:
                pad = (h - w) // 2
                left, right = pad, pad
            else:
                pad = (w - h) // 2
                top, bottom = pad, pad
            data = np.pad(data, ((top, bottom), (left, right), (0, 0)), mode="constant", constant_values=0)
        self.handler.update_frame(frame_id, data)

    def preprocess_segmentation(self, image_seg):
        calzada_color = [128, 64, 128]
        mask = cv2.inRange(image_seg, np.array(calzada_color), np.array(calzada_color))
        masked = np.zeros_like(image_seg)
        masked[mask > 0] = [255, 255, 255]
        # cropped = masked[200:, :]
        resized = cv2.resize(masked[200:-1, :], (200, 66))
        gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)
        rgb_like = cv2.merge([gray, gray, gray])
        tensor = torch.tensor(rgb_like, dtype=torch.float32).permute(2, 0, 1)
        return tensor.unsqueeze(0).to(self.device)

    def predict_controls(self, model, image_seg):
        input_tensor = self.preprocess_segmentation(image_seg)
        with torch.no_grad():
            prediction = model(input_tensor)
            steer, throttle = prediction[0].tolist()
            # steer = output[0, 0]
            # throttle = output[0, 1] 
            print(f"Steer: {steer}, Throttle: {throttle}")           
        
        return steer, throttle

    def execute(self):
        rgb_image = self.camera_rgb.getImage().data
        seg_image = self.camera_seg.getImage().data

        self.update_frame("frame_0", rgb_image)
        self.update_frame("frame_1", seg_image)

        start_time = time.time()

        try:
            steer, throttle = self.predict_controls(self.monolithic_model, seg_image)
            self.motors.sendThrottle(float(throttle))
            self.motors.sendSteer(float(steer))
            self.inference_times.append(time.time() - start_time)
        except Exception as ex:
            logger.info("Error inside brain: Exception!")
            logger.warning(type(ex).__name__)
            print_exc()
            raise
