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

import logging
# logging.basicConfig(level=logging.INFO)


PRETRAINED_MODELS = ROOT_PATH + "/" + PRETRAINED_MODELS_DIR + "CARLA/"

# ----- PREPROCESAMIENTO -----
preprocess = transforms.Compose(
    [
        transforms.Resize((224, 224)),
        transforms.ToTensor(),
        transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
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
            
            monolithic_model_path = PRETRAINED_MODELS + model
            print("Loading model from: ", monolithic_model_path)
            
            # self.monolithic_model = ModifiedDeepestLSTM(image_shape=(66, 200, 3), num_labels=2)
            # self.monolithic_model = resnet18(weights=ResNet18_Weights.DEFAULT)
            # self.monolithic_model.fc = nn.Linear(self.monolithic_model.fc.in_features, 2)
            # self.monolithic_model.load_state_dict(torch.load(monolithic_model_path, map_location=self.device))
            # self.monolithic_model.to(self.device)
            # self.monolithic_model.eval()
            
            # self.monolithic_model = efficientnet_v2_s(weights=None)
            # self.monolithic_model.classifier[-1] = nn.Linear(self.monolithic_model.classifier[-1].in_features, 2)
            # self.monolithic_model.load_state_dict(torch.load(monolithic_model_path, map_location=self.device))
            # self.monolithic_model.to(self.device)            
            # self.monolithic_model.eval()
            
            # onnx model
            providers = [('CUDAExecutionProvider', {})] if ort.get_available_providers().__contains__('CUDAExecutionProvider') else ['CPUExecutionProvider']
            self.ort_session = ort.InferenceSession(str(monolithic_model_path), providers=providers)
            
            print(f"onnx ort session providers: {self.ort_session.get_providers()}")

            # nombre de la(s) entrada(s) y salida(s)
            self._in_name  = self.ort_session.get_inputs()[0].name
            self._out_name = self.ort_session.get_outputs()[0].name
            
        if "Route" in config:
            route = config["Route"]
            print("route: ", route)
        else:
            route = None

        self.hlc_loader = HighLevelCommandLoader(self.vehicle, self.map, route=route)
        self.prev_hlc = 0
        self.prev_yaw = None
        self.delta_yaw = 0

        self.target_point = [160.0,-105.3,0.42,0.00,0.00,180.00]
        self.termination_code = 0  # 0: not terminated; 1: arrived at target; 2: wrong turn
    
        self._last_tick = None # para calcular el tiempo entre ticks
        
        ## debugging, tick time calculation
        # Al cargar el modelo ONNX (por ejemplo en __init__ o setup)
        dummy_input = np.random.rand(1, 3, 66, 200).astype(np.float32)   # Tamaño esperado

        for _ in range(10):
            _ = self.ort_session.run([self._out_name], {self._in_name: dummy_input})


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


    def predict_controls(self, image_seg):
        """
        Realiza el preprocesamiento coherente con el entrenamiento:
        - Recorte fijo desde la parte superior (top_ratio=0.4)
        - Redimensiona a (200, 66)
        - Normaliza con mean=std=0.5 → rango [-1,1]
        - Realiza inferencia ONNX (steer, throttle)
        """

        # 1️⃣ Extraer calzada (color Cityscapes)
        road_color = np.array([128, 64, 128], dtype=np.uint8)
        mask = cv2.inRange(image_seg, road_color, road_color)

        masked = np.zeros_like(image_seg)
        masked[mask > 0] = (255, 255, 255)

        # 2️⃣ Recorte fijo (top_ratio=0.4)
        h, w = masked.shape[:2]
        top = int(h * 0.4)
        cropped = masked[top:, :]

        # 3️⃣ Redimensionar y convertir a RGB-like
        resized = cv2.resize(cropped, (200, 66), interpolation=cv2.INTER_AREA)
        gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)
        rgb_like = cv2.merge([gray, gray, gray])

        # 4️⃣ Normalización coherente con entrenamiento
        img = rgb_like.astype(np.float32) / 255.0
        img = (img - 0.5) / 0.5                      # → [-1,1]
        img = np.transpose(img, (2, 0, 1))           # → (3,66,200)
        input_np = np.expand_dims(img, axis=0)       # → (1,3,66,200)

        # 5️⃣ Inferencia ONNX
        start_infer = time.perf_counter()
        pred = self.ort_session.run([self._out_name], {self._in_name: input_np})[0]
        infer_time = (time.perf_counter() - start_infer) * 1000

        steer, throttle = pred[0].astype(np.float32)

        # 6️⃣ Limitadores por seguridad
        steer = float(np.clip(steer, -1.0, 1.0))
        throttle = float(np.clip(throttle, 0.0, 1.0))

        # Opcional: imprimir tiempos
        # print(f"Inferencia: {infer_time:.2f} ms | steer={steer:.3f}, throttle={throttle:.3f}")
        return steer, throttle


    def execute(self):
        """Main loop of the brain. This will be called iteratively each TIME_CYCLE (see pilot.py)"""      
        rgb_image = self.camera_rgb.getImage().data  # rgb_image shape:  (768, 1024, 3)     
        seg_image = self.camera_seg.getImage().data   # seg_image shape:  (80, 400, 3)   
             
        self.update_frame("frame_0", rgb_image)
        self.update_frame("frame_1", seg_image)
    
        # if hasattr(self, 'pilot'):
        #     current = self.pilot.tick_counter
        #     if self._last_tick is None:
        #         delta = 0
        #     else:
        #         delta = current - self._last_tick
        #     logger.info(f"{delta} ticks -> inferencia")
        #     self._last_tick = current
        
        # print("Inferencia ejecutandose") # (600, 800, 3)

        steer, throttle = self.predict_controls(seg_image) # se saco self.monolithic_model como argumento       
        # print(f"Predicted - Steer: {steer:.3f}, Throttle: {throttle:.3f}")
        
        vehicle_location = self.vehicle.get_transform().location
        # calculate distance to target point
        # print(f'vehicle location: ({vehicle_location.x}, {vehicle_location.y})')
        # print(f'target point: ({self.target_point[0]}, {self.target_point[1]})')
        
        # arrived = False # flag to indicate if the vehicle has arrived at the target point
        # print(f"[DEBUG] vehicle: ({vehicle_location.x:.2f}, {vehicle_location.y:.2f}) "
        #         f"target: ({self.target_point[0]:.2f}, {self.target_point[1]:.2f})")
        
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
        