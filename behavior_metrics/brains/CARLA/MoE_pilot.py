
from brains.CARLA.utils.test_utils import (
    traffic_light_to_int,
    model_control,
    calculate_delta_yaw,
)
from utils.constants import PRETRAINED_MODELS_DIR, ROOT_PATH
from brains.CARLA.utils.high_level_command import HighLevelCommandLoader
from os import path
from utils.logger import logger
from traceback import print_exc

import numpy as np

import torch
import time
import math
import carla

from torchvision.models import resnet18, ResNet18_Weights, efficientnet_v2_s, EfficientNet_V2_S_Weights
import torch.nn as nn

from torchvision import transforms
import cv2
import onnxruntime as ort
from PIL import Image as PILImage

import concurrent.futures

import logging
# logging.basicConfig(level=logging.INFO)


PRETRAINED_MODELS = ROOT_PATH + "/" + PRETRAINED_MODELS_DIR + "CARLA/"

selector_path = "/home/canveo/Projects/BehaviorMetrics/behavior_metrics/models/CARLA/curvature_selector.onnx"
straight_path = "/home/canveo/Projects/BehaviorMetrics/behavior_metrics/models/CARLA/efficientnet_recta.onnx"
curve_path = "/home/canveo/Projects/BehaviorMetrics/behavior_metrics/models/CARLA/efficientnet_curva.onnx"

# ----- PREPROCESAMIENTO -----
selector_transform = transforms.Compose(
    [
        transforms.Resize((224, 224)),
        transforms.ToTensor(),
        transforms.Normalize(mean=[0.485, 0.456, 0.406],
                             std=[0.229, 0.224, 0.225]),
    ]
)
# TIME_CYCLE = 0.1  # seconds, 10 Hz

class Brain:

    def __init__(self, sensors, actuators, model=None, handler=None, config=None):
        self.motors = actuators.get_motor("motors_0")
        self.camera_rgb = sensors.get_camera("camera_0")  # rgb front view camera
        self.camera_seg = sensors.get_camera("camera_2")  # segmentation camera
        self.handler = handler
        self.inference_times = []
        self.gpu_inference = config["GPU"]
        self.device = torch.device(
            "cuda" if (torch.cuda.is_available() and self.gpu_inference) else "cpu"
        )
        self.red_light_counter = 0
        self.running_light = False

        client = carla.Client("localhost", 2000)
        client.set_timeout(100.0)
        self.world = client.get_world()
        self.map = self.world.get_map()
        
        # contar ticks
        # self.delta = self.world.get_settings().fixed_delta_seconds or 0.05
        # self.prev_frame = None
        
        weather = carla.WeatherParameters.ClearNoon
        self.world.set_weather(weather)

        self.vehicle = None
        while self.vehicle is None:
            for vehicle in self.world.get_actors().filter("vehicle.*"):
                if vehicle.attributes.get("role_name") == "ego_vehicle":
                    self.vehicle = vehicle
                    break
            if self.vehicle is None:
                print("Waiting for vehicle with role_name 'ego_vehicle'")
                time.sleep(1)  # sleep for 1 second before checking again

        if model:
            self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
            
            # monolithic_model_path = PRETRAINED_MODELS + model
            # print("Loading model from: ", monolithic_model_path)
                       
            # onnx model
            # providers = [('CUDAExecutionProvider', {})] if ort.get_available_providers().__contains__('CUDAExecutionProvider') else ['CPUExecutionProvider']
            # self.ort_session = ort.InferenceSession(str(monolithic_model_path), providers=providers)
            
            # Seleccion de GPU
            providers = [('CUDAExecutionProvider', {})] if ort.get_available_providers().__contains__('CUDAExecutionProvider') else ['CPUExecutionProvider']

            so = ort.SessionOptions()
            so.log_severity_level = 3

            self.ort_selector = ort.InferenceSession(str(selector_path), providers=providers, sess_options=so)
            self.sel_in = self.ort_selector.get_inputs()[0].name
            self.sel_out = self.ort_selector.get_outputs()[0].name

            self.ort_straight = ort.InferenceSession(str(straight_path), providers=providers, sess_options=so)
            self.st_in = self.ort_straight.get_inputs()[0].name
            self.st_out = self.ort_straight.get_outputs()[0].name

            self.ort_curve = ort.InferenceSession(str(curve_path), providers=providers, sess_options=so)
            self.cv_in = self.ort_curve.get_inputs()[0].name
            self.cv_out = self.ort_curve.get_outputs()[0].name
            
            print(f"onnx ort session providers: {self.ort_selector.get_providers()}")          

            # # nombre de la(s) entrada(s) y salida(s)
            # self._in_name  = self.ort_session.get_inputs()[0].name
            # self._out_name = self.ort_session.get_outputs()[0].name
            
        if "Route" in config:
            route = config["Route"]
            print("route: ", route)
        else:
            route = None

        self.hlc_loader = HighLevelCommandLoader(self.vehicle, self.map, route=route)
        self.prev_hlc = 0
        self.prev_yaw = None
        self.delta_yaw = 0

        self.target_point = None
        self.termination_code = (
            0  # 0: not terminated; 1: arrived at target; 2: wrong turn
        )
        self._last_tick = None # para calcular el tiempo entre ticks
        
        ## debugging, tick time calculation
        # Al cargar el modelo ONNX (por ejemplo en __init__ o setup)
        dummy_input_selector = np.random.rand(1, 3, 224, 224).astype(np.float32)
        dummy_input_expert = np.random.rand(1, 3, 66, 200).astype(np.float32)

        for _ in range(10):
            _ = self.ort_selector.run([self.sel_out], {self.sel_in: dummy_input_selector})
            _ = self.ort_straight.run([self.st_out], {self.st_in: dummy_input_expert})
            _ = self.ort_curve.run([self.cv_out], {self.cv_in: dummy_input_expert})
            
        # Umbral selector
        # self.selector_threshold = 0.5  # "recta" if predicted==1 else "curva"

    def update_frame(self, frame_id, data):
        """Update the information to be shown in one of the GUI's frames.

        Arguments:
            frame_id {str} -- Id of the frame that will represent the data
            data {*} -- Data to be shown in the frame. Depending on the type of frame (rgbimage, laser, pose3d, etc)
        """
        if data.shape[0] != data.shape[1]:
            if data.shape[0] > data.shape[1]:
                difference = data.shape[0] - data.shape[1]
                extra_left, extra_right = int(difference / 2), int(difference / 2)
                extra_top, extra_bottom = 0, 0
            else:
                difference = data.shape[1] - data.shape[0]
                extra_left, extra_right = 0, 0
                extra_top, extra_bottom = int(difference / 2), int(difference / 2)

            data = np.pad(
                data,
                ((extra_top, extra_bottom), (extra_left, extra_right), (0, 0)),
                mode="constant",
                constant_values=0,
            )

        self.handler.update_frame(frame_id, data)

    def _preprocess_rgb_for_selector(self, img_bgr):
        img_pil = PILImage.fromarray(cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB))
        tensor = selector_transform(img_pil)
        # input_np = tensor.unsqueeze(0).numpy().astype(np.float32)
        input_np = tensor.numpy()[np.newaxis, :].astype(np.float32)
        return input_np
        
    def _preprocess_seg_to_input(self, image_seg):
        calzada_color = [128, 64, 128]
        mask = cv2.inRange(image_seg, np.array(calzada_color), np.array(calzada_color))
        
        masked_image = np.zeros_like(image_seg)
        masked_image[mask > 0] = [255, 255, 255]
        
        h = masked_image.shape[0]
        y0 = min(200, max(0, h - 2))           # evita salirte si h < 200
        crop = masked_image[y0:-1, :]
        if crop is None or crop.size == 0:     # fallback si el recorte quedó vacío
            crop = masked_image
        resized = cv2.resize(crop, (200, 66))  # OpenCV: (width, height)
        
        # resized = cv2.resize(masked_image[200:-1, :], (200, 66))
        gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)
        rgb_like = cv2.merge([gray, gray, gray])
        
        # input_tensor = torch.tensor(rgb_like, dtype=torch.float32).permute(2, 0, 1)
        # input_np = input_tensor.unsqueeze(0).cpu().numpy().astype(np.float32)
        input_tensor = np.transpose(rgb_like, (2, 0, 1))[np.newaxis, :] 
        input_tensor = input_tensor.astype(np.float32)
        # input_np = input_tensor.unsqueeze(0).cpu().numpy()
        
        return input_tensor


    def predict_controls(self, image_seg):
        # Obtener imagen RGB y preprocesar para el selector
        rgb_img = self.camera_rgb.getImage().data
        input_selector = self._preprocess_rgb_for_selector(rgb_img)

        # Ejecutar modelo selector (0 = recta, 1 = curva)
        selector_output = self.ort_selector.run([self.sel_out], {self.sel_in: input_selector})[0]
        is_curve = int(np.argmax(selector_output, axis=1)[0]) #== 1

        # Preprocesar imagen segmentada para el experto
        input_expert = self._preprocess_seg_to_input(image_seg)

        # Ejecutar modelo experto según el selector
        if is_curve:
            pred = self.ort_curve.run([self.cv_out], {self.cv_in: input_expert})[0]
            # logger.info("Modelo experto: Curva")
        else:
            pred = self.ort_straight.run([self.st_out], {self.st_in: input_expert})[0]
            # logger.info("Modelo experto: Recta")

        # Convertir a valores escalares
        steer, throttle = pred[0].astype(np.float32)
        return float(steer), float(throttle)

    def execute(self):
        """Main loop of the brain. This will be called iteratively each TIME_CYCLE (see pilot.py)"""      
        rgb_image = self.camera_rgb.getImage().data  # rgb_image shape:  (768, 1024, 3)     
        seg_image = self.camera_seg.getImage().data   # seg_image shape:  (80, 400, 3)   
             
        self.update_frame("frame_0", rgb_image) # → @
        self.update_frame("frame_1", seg_image) 
         
        steer, throttle = self.predict_controls(seg_image) # se saco self.monolithic_model como argumento       
        # logger.info(f"steer {steer}, throttle {throttle}")
        
        vehicle_location = self.vehicle.get_transform().location
        
        if self.target_point is not None:
            distance_to_target = np.sqrt(
                (self.target_point[0] - vehicle_location.x) ** 2 +
                (self.target_point[1] - (-vehicle_location.y)) ** 2)

            print(f'Euclidean distance to target: {distance_to_target}')
            if distance_to_target < 3.0:   # aumentado para pistas rapidas antes 1.5
                self.termination_code = 1
                arrived = True
                print(f"======== Arrived at target point {distance_to_target} m away============.")
        
        self.motors.sendThrottle(throttle)
        self.motors.sendSteer(steer)
        self.motors.sendBrake(0.0)
