import carla

class CarlaApiMotors:
    def __init__(self, vmax, wmax, client_host='localhost', client_port=2000, ego_role='ego_vehicle'):
        self.vmax = float(vmax)
        self.wmax = float(wmax)
        self._client = carla.Client(client_host, client_port)
        self._client.set_timeout(5.0)
        self._world = self._client.get_world()
        self._vehicle = self._find_ego(ego_role)

    def _find_ego(self, role):
        for v in self._world.get_actors().filter('vehicle.*'):
            if v.attributes.get('role_name') == role:
                return v
        raise RuntimeError(f"No se encontró vehículo con role_name='{role}'")

    # API pública compatible con Publisher* (nombres típicos)
    def sendThrottle(self, val):
        ctrl = self._vehicle.get_control()
        ctrl.throttle = float(max(0.0, min(1.0, val)))
        self._vehicle.apply_control(ctrl)

    def sendBrake(self, val):
        ctrl = self._vehicle.get_control()
        ctrl.brake = float(max(0.0, min(1.0, val)))
        self._vehicle.apply_control(ctrl)

    def sendSteer(self, val):
        # val en [-1,1]
        ctrl = self._vehicle.get_control()
        ctrl.steer = float(max(-1.0, min(1.0, val)))
        self._vehicle.apply_control(ctrl)

    def sendReverse(self, reverse_on: bool):
        ctrl = self._vehicle.get_control()
        ctrl.reverse = bool(reverse_on)
        self._vehicle.apply_control(ctrl)

    def stop(self):
        try:
            ctrl = self._vehicle.get_control()
            ctrl.throttle = 0.0
            ctrl.brake = 1.0
            self._vehicle.apply_control(ctrl)
        except Exception:
            pass

    def destroy(self):
        self.stop()
