# robot/interfaces/carla_api/sensors.py
from __future__ import annotations
import queue
import carla
import math

def _make_transform(tf):
    loc = carla.Location(x=tf.get("x",0), y=tf.get("y",0), z=tf.get("z",0))
    rot = carla.Rotation(roll=tf.get("roll",0), pitch=tf.get("pitch",0), yaw=tf.get("yaw",0))
    return carla.Transform(loc, rot)

class _BaseActorSensor:
    def __init__(self): self._actor = None
    def stop(self):
        try:
            if self._actor is not None:
                self._actor.stop() if hasattr(self._actor, "stop") else None
                self._actor.destroy()
                self._actor = None
        except RuntimeError:
            pass

class CarlaCamera(_BaseActorSensor):
    def __init__(self, world, bp_lib, parent, tf, fps=20, params=None):
        super().__init__()
        params = params or {}
        bp = bp_lib.find("sensor.camera.rgb")
        bp.set_attribute("image_size_x", str(params.get("width", 800)))
        bp.set_attribute("image_size_y", str(params.get("height", 600)))
        bp.set_attribute("fov", str(params.get("fov", 90)))
        if fps: bp.set_attribute("sensor_tick", str(1.0/float(fps)))
        self.queue = queue.Queue()
        self._actor = world.spawn_actor(bp, _make_transform(tf), attach_to=parent)
        self._actor.listen(lambda data: self.queue.put((data.frame, data.timestamp, data)))

class CarlaLidar(_BaseActorSensor):
    def __init__(self, world, bp_lib, parent, tf, fps=20, params=None):
        super().__init__()
        params = params or {}
        bp = bp_lib.find("sensor.lidar.ray_cast")
        for k, v in {
            "range": params.get("range", 50.0),
            "rotation_frequency": params.get("rotation_frequency", float(fps)),
            "points_per_second": params.get("points_per_second", 100000)
        }.items():
            bp.set_attribute(k, str(v))
        self.queue = queue.Queue()
        self._actor = world.spawn_actor(bp, _make_transform(tf), attach_to=parent)
        self._actor.listen(lambda data: self.queue.put((data.frame, data.timestamp, data)))

class CarlaPose3D:
    """Pose3D sin actor: lee transform/velocidad del vehículo."""
    def __init__(self, world, vehicle):
        self.world = world
        self.vehicle = vehicle
        
    def stop(self): 
        pass
    
    def get_pose(self):
        tf = self.vehicle.get_transform()
        vel = self.vehicle.get_velocity()
        spd = math.sqrt(vel.x**2 + vel.y**2 + vel.z**2)
        return {
            "x": tf.location.x, "y": tf.location.y, "z": tf.location.z,
            "roll": tf.rotation.roll, "pitch": tf.rotation.pitch, "yaw": tf.rotation.yaw,
            "speed": spd
        }

class CarlaSpeedometer:
    def __init__(self, vehicle): 
        self.vehicle = vehicle
        
    def stop(self):
        pass
    
    def get_speed(self):
        v = self.vehicle.get_velocity()
        return math.sqrt(v.x**2 + v.y**2 + v.z**2)

class CarlaBirdEyeView(_BaseActorSensor):
    """Stub: si ya tienes tu BirdEyeView ROS, aquí puedes renderizar con CARLA."""
    def __init__(self, world, vehicle, params=None):
        super().__init__()
        # Implementación mínima opcional (puede ser una cámara cenital temporal)
        bp = world.get_blueprint_library().find("sensor.camera.semantic_segmentation")
        tf = {"x":0, "y":0, "z":params.get("z", 20), "roll":-90, "pitch":0, "yaw":0}
        self.queue = queue.Queue()
        self._actor = world.spawn_actor(bp, _make_transform(tf), attach_to=vehicle)
        self._actor.listen(lambda data: self.queue.put((data.frame, data.timestamp, data)))
