import os, math
import numpy as np
import carla

# CARLA CLIENT CONNECTIONS
_client = _world = _ego = None
def get_carla():
    global _client, _world, _ego
    if _client is None:
        host = os.getenv("CARLA_HOST","127.0.0.1")
        port = int(os.getenv("CARLA_PORT","2000"))
        _client = carla.Client(host, port); _client.set_timeout(10)
        _world = _client.get_world()
        _ego = _find_ego(_world, os.getenv("EGO_ROLE_NAME","ego_vehicle"))
    return _client, _world, _ego

def _find_ego(world, role_name):
    for a in world.get_actors().filter("vehicle.*"):
        if a.attributes.get("role_name") == role_name:
            return a
    raise RuntimeError(f"Ego vehicle with role_name={role_name} not found")

def _str_pos_to_transform(pos_str, order="x y z yaw pitch roll"):
    vals = list(map(float, pos_str.split()))
    x,y,z,yaw,pitch,roll = vals
    return carla.Transform(
        carla.Location(x=x,y=y,z=z),
        carla.Rotation(yaw=yaw,pitch=pitch,roll=roll)
    )

class CarlaApiCamera:
    def __init__(self, cfg):
        _, world, ego = get_carla()
        bp = world.get_blueprint_library().find(f"sensor.camera.{cfg['Type']}")
        if bp.has_attribute('image_size_x'):
            # Use WIDTH/HEIGHT globals or from YAML if provided
            w = str(cfg.get('Width', os.getenv("CAM_X","1280")))
            h = str(cfg.get('Height', os.getenv("CAM_Y","720")))
            bp.set_attribute('image_size_x', w)
            bp.set_attribute('image_size_y', h)
        if bp.has_attribute('gamma'):
            bp.set_attribute('gamma', str(cfg.get('Gamma', 2.2)))
        if 'FOV' in cfg:
            bp.set_attribute('fov', str(cfg['FOV']))

        transform = _str_pos_to_transform(cfg['Position'])
        self._actor = world.spawn_actor(bp, transform, attach_to=ego)
        self._frame = None
        self._actor.listen(self._on_image)

    def _on_image(self, image: carla.Image):
        # saves the last frame received
        self._frame = image  

    def getImage(self):
        return self._frame

    def stop(self):
        self.destroy()

    def destroy(self):
        if self._actor:
            try: self._actor.stop()
            except Exception: pass
            self._actor.destroy()
            self._actor = None


class CarlaApiLidar:
    def __init__(self, cfg):
        _, world, ego = get_carla()
        bp = world.get_blueprint_library().find("sensor.lidar.ray_cast")
        
        bp.set_attribute('range', str(cfg.get('Range', 50)))
        bp.set_attribute('rotation_frequency', str(cfg.get('RotationHz', 10)))
        bp.set_attribute('channels', str(cfg.get('Channels', 32)))
        bp.set_attribute('points_per_second', str(cfg.get('PointsPerSecond', 100000)))
        transform = _str_pos_to_transform(cfg['Position'])
        self._actor = world.spawn_actor(bp, transform, attach_to=ego)
        self._scan = None
        self._actor.listen(self._on_lidar)

    def _on_lidar(self, data: carla.LidarMeasurement):
        self._scan = data

    def getScan(self):
        return self._scan

    def stop(self): 
        self.destroy()
        
    def destroy(self):
        if self._actor:
            try: self._actor.stop()
            except Exception: 
                pass
            self._actor.destroy()
            self._actor = None


class CarlaApiPose3D:

    def __init__(self, cfg):
        _, world, ego = get_carla()
        self._world = world
        self._ego = ego

    def getPose(self):
        t: carla.Transform = self._ego.get_transform()
        v = self._ego.get_velocity()
        pose = {
            "x": t.location.x, "y": t.location.y, "z": t.location.z,
            "roll": t.rotation.roll, "pitch": t.rotation.pitch, "yaw": t.rotation.yaw,
            "vx": v.x, "vy": v.y, "vz": v.z
        }
        return pose

    def stop(self):
        pass
    def destroy(self):
        pass


class CarlaApiSpeedometer:
    
    def __init__(self, cfg):
        _, world, ego = get_carla()
        self._ego = ego

    def getSpeed(self):
        v = self._ego.get_velocity()
        speed_m_s = math.sqrt(v.x**2 + v.y**2 + v.z**2)
        return 3.6 * speed_m_s

    def stop(self):
        pass
    def destroy(self):
        pass
