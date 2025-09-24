from brains.CARLA.utils.pilotnet_onehot import PilotNetOneHot
# from brains.CARLA.utils.modifiedDeepestLSTM import ModifiedDeepestLSTM
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

PRETRAINED_MODELS = ROOT_PATH + "/" + PRETRAINED_MODELS_DIR + "CARLA/"

# # ----- PREPROCESAMIENTO -----
# preprocess = transforms.Compose(
#     [
#         transforms.Resize((224, 224)),
#         transforms.ToTensor(),
#         transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
#     ]
# )


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
        world = client.get_world()
        self.map = world.get_map()

        weather = carla.WeatherParameters.ClearNoon
        world.set_weather(weather)

        self.vehicle = None
        while self.vehicle is None:
            for vehicle in world.get_actors().filter("vehicle.*"):
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
            self.monolithic_model = resnet18(weights=None)
            self.monolithic_model.fc = nn.Linear(self.monolithic_model.fc.in_features, 2)
            self.monolithic_model.load_state_dict(torch.load(monolithic_model_path, map_location=self.device))
            self.monolithic_model.to(self.device)
            self.monolithic_model.eval()
            
            # self.monolithic_model = efficientnet_v2_s(weights=None)
            # self.monolithic_model.classifier[-1] = nn.Linear(self.monolithic_model.classifier[-1].in_features, 2)
            
            # self.monolithic_model.load_state_dict(torch.load(monolithic_model_path, map_location=self.device))
            # self.monolithic_model.to(self.device)
            # self.monolithic_model.eval()

            
            
            
            # self.monolithic_model.load_state_dict(torch.load(monolithic_model_path, map_location=self.device))
            # self.monolithic_model.to(self.device)
            # self.monolithic_model.eval()
            # if not path.exists(PRETRAINED_MODELS + model):
            #     print("File " + model + " cannot be found in " + PRETRAINED_MODELS)

            # if config["UseOptimized"]:
            #     self.net = torch.jit.load(PRETRAINED_MODELS + model).to(
            #         self.device
            #     )  # TorchScript model verification
            # else:
            #     # self.net = PilotNetOneHot((288, 200, 6), 3, 4, 4).to(self.device)
            #     self.net = ModifiedDeepestLSTM((66, 200, 3), 2).to(
            #         self.device
            #     )  # image size 200x66x3, 2 num labels
            #     self.net.load_state_dict(
            #         torch.load(PRETRAINED_MODELS + model, map_location=self.device)
            #     )
            #     self.net.eval()
            #     print("Model modified loaded: ", model)

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

    def process_image_rgb(self, image_seg):
        """Process the image to extract the road and convert it to grayscale.

        Arguments:
            image_seg {numpy.ndarray} -- Segmentation image
        Returns:
            numpy.ndarray -- Processed image (grayscale)
        """
        calzada_color = [128, 64, 128]
        mask = cv2.inRange(image_seg, np.array(calzada_color), np.array(calzada_color))

        image_seg_masked = np.zeros_like(image_seg)
        image_seg_masked[mask > 0] = [255, 255, 255]

        image_seg_rgb = cv2.resize(image_seg_masked[200:-1, :], (200, 66))
        image_seg_rgb = cv2.cvtColor(image_seg_rgb, cv2.COLOR_BGR2GRAY)
        image_seg_rgb = cv2.merge([image_seg_rgb, image_seg_rgb, image_seg_rgb])
        return image_seg_rgb

    # def predict_controls(self, model, image_seg):
    #     """Predict the steering and throttle values using the model.

    #     Arguments:
    #         model {torch.nn.Module} -- The trained model
    #         image_seg {numpy.ndarray} -- Segmentation image
    #     Returns:
    #         tuple -- Steering and throttle values
    #     """
    #     # image_seg_processed = self.process_image_rgb(image_seg)  # (66, 200, 3)
    #     # # Convertir a tensor y agregar dimensión de batch
    #     # input_tensor = np.expand_dims(image_seg_processed, axis=0)  # (1, 66, 200, 3)
    #     # input_tensor = torch.from_numpy(input_tensor).float()
    #     # # Reordenar dimensiones: (batch, canales, alto, ancho)
    #     # input_tensor = input_tensor.permute(0, 3, 1, 2)
    #     # input_tensor = input_tensor.to(self.device)       #  (torch.device("cpu"))
    #     # with torch.no_grad():
    #     #     prediction = model(input_tensor)
    #     # # Se asume que el modelo devuelve una tupla (steer, throttle)
    #     # steer = prediction[0].item()
    #     # throttle = prediction[1].item()
    #     # return steer, throttle
    #     image_seg_processed = self.process_image_rgb(image_seg)  # (66, 200, 3) → tu modelo espera (240, 640, 3)

    #     # Aquí ajustamos al formato de entrada entrenado (240x640)
    #     # resized = cv2.resize(image_seg_processed, (640, 240))  # resize from (66,200) to (240,640)  
    #     resized = cv2.resize(image_seg_processed, (224, 224))  # para EfficientNet_V2_S

    #     input_tensor = torch.tensor(resized, dtype=torch.float32).permute(2, 0, 1) / 255.0
    #     input_tensor = input_tensor.unsqueeze(0).to(self.device)  # Añadir batch dim

    #     with torch.no_grad():
    #         prediction = model(input_tensor)

    #     steer, throttle = prediction[0].tolist()
    #     return steer, throttle
    
    def predict_controls(self, model, image_seg):
        # Asegurarse de que sea una imagen BGR como la cargada por cv2.imread
        calzada_color = [128, 64, 128]
        mask = cv2.inRange(image_seg, np.array(calzada_color), np.array(calzada_color))
        
        masked_image = np.zeros_like(image_seg)
        masked_image[mask > 0] = [255, 255, 255]
        
        cropped = masked_image[200:-1, :]
        resized = cv2.resize(cropped, (640, 240))
        gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)
        rgb_like = cv2.merge([gray, gray, gray])

        input_tensor = torch.tensor(rgb_like, dtype=torch.float32).permute(2, 0, 1) / 255.0
        input_tensor = input_tensor.unsqueeze(0).to(self.device)

        with torch.no_grad():
            prediction = model(input_tensor)

        steer, throttle = prediction[0].tolist()
        return steer, throttle
    

    def execute(self):
        """Main loop of the brain. This will be called iteratively each TIME_CYCLE (see pilot.py)"""

        rgb_image = self.camera_rgb.getImage().data  # rgb_image shape:  (768, 1024, 3)
        # rgb_image = cv2.resize(rgb_image, (200, 66))
        seg_image = self.camera_seg.getImage().data   # seg_image shape:  (80, 400, 3)
        # seg_image = cv2.resize(seg_image, (200, 66))
                
        # print("rgb_image shape: ", rgb_image.shape)
        # print("seg_image shape: ", seg_image.shape)

        self.update_frame("frame_0", rgb_image)
        self.update_frame("frame_1", seg_image)
        

        start_time = time.time()

        try:
            steer, throttle = self.predict_controls(self.monolithic_model, seg_image)
            denormalized_steer = np.interp(steer, (0, 1), (-1, 1))

            # print(f"steer {denormalized_steer:.2f}, throttle {throttle:.2f}")
            
            self.motors.sendThrottle(throttle)
            self.motors.sendSteer(steer)
            # self.motors.sendSteer(denormalized_steer)       

            self.inference_times.append(time.time() - start_time)
            
            # print(self.inference_times[-1])
            
        except Exception as ex:
            logger.info("Error inside brain: Exception!")
            logger.warning(type(ex).__name__)
            print_exc()
            raise Exception(ex)
