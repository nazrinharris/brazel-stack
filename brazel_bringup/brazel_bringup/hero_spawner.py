#!/usr/bin/env python3

import os
import yaml

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

import carla

class HeroSpawner(Node):
    """ROS 2 node that spawns the hero vehicle + sensor stack in CARLA,
    enables native ROS 2 publishing via enable_for_ros(), and runs the sim tick loop."""

    def __init__(self):
        super().__init__("hero_spawner")

        # Load entire config from YAML — one mechanism, one source of truth
        config_path = os.path.join(
            get_package_share_directory("brazel_bringup"),
            "config",
            "stack.yaml",
        )
        with open(config_path, "r") as f:
            self._config = yaml.safe_load(f)

        self._host = self._config.get("host", "localhost")
        self._port = self._config.get("port", 2000)
        self._vehicle_type = self._config.get("vehicle_type", "vehicle.audi.etron")
        self._vehicle_id = self._config.get("vehicle_id", "hero")
        self._tick_rate = self._config.get("tick_rate", 0.05)
        self._sensors_config = self._config.get("sensors", {})
        self._spawn_cfg = self._config.get("spawn", {})

        self._vehicle = None
        self._sensors = []
        self._original_settings = None
        self._client = None
        self._world = None

        try:
            self._connect()
            self._spawn_vehicle()
            self._spawn_sensors()
            self._start_sync_mode()

            self._tick_timer = self.create_timer(self._tick_rate, self._tick)
            self.get_logger().info("Hero spawned. Ticking...")

        except Exception as e:
            self.get_logger().fatal(f"Initialization failed: {e}")
            raise

    def _connect(self):
        self.get_logger().info(f"Connecting to CARLA at {self._host}:{self._port}")
        self._client = carla.Client(self._host, self._port)
        self._client.set_timeout(10.0)
        self._world = self._client.get_world()

    def _spawn_vehicle(self):
        bp_library = self._world.get_blueprint_library()
        bp = bp_library.filter(self._vehicle_type)[0]
        bp.set_attribute("role_name", self._vehicle_id)
        bp.set_attribute("ros_name", self._vehicle_id)

        spawn_points = self._world.get_map().get_spawn_points()
        start_index = self._spawn_cfg.get("index", 0)
        fallback = self._spawn_cfg.get("fallback", True)

        indices = range(start_index, len(spawn_points)) if fallback else [start_index]

        for i in indices:
            vehicle = self._world.try_spawn_actor(bp, spawn_points[i])
            if vehicle is not None:
                self._vehicle = vehicle
                self.get_logger().info(
                    f"Vehicle spawned at point {i}: {self._vehicle_type} (id={vehicle.id})"
                )
                return

        raise RuntimeError(f"Spawn failed at point {start_index} (fallback exhausted)")

    def _spawn_sensors(self):
        if not self._sensors_config:
            self.get_logger().warn("No sensors configured")
            return

        bp_library = self._world.get_blueprint_library()

        for name, sensor_cfg in self._sensors_config.items():
            bp = bp_library.filter(sensor_cfg["type"])[0]
            bp.set_attribute("ros_name", sensor_cfg["id"])
            bp.set_attribute("role_name", sensor_cfg["id"])

            for key, value in sensor_cfg.get("attributes", {}).items():
                bp.set_attribute(str(key), str(value))

            sp = sensor_cfg["spawn_point"]
            transform = carla.Transform(
                location=carla.Location(x=sp["x"], y=-sp["y"], z=sp["z"]),
                rotation=carla.Rotation(
                    roll=sp["roll"], pitch=-sp["pitch"], yaw=-sp["yaw"]
                ),
            )

            sensor = self._world.spawn_actor(bp, transform, attach_to=self._vehicle)
            sensor.enable_for_ros()
            self._sensors.append(sensor)
            self.get_logger().info(
                f"Sensor spawned + ROS enabled: {sensor_cfg['id']} ({sensor.type_id})"
            )

    def _start_sync_mode(self):
        self._original_settings = self._world.get_settings()
        settings = self._world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = self._tick_rate
        self._world.apply_settings(settings)

        self._vehicle.set_autopilot(True)

    def _tick(self):
        try:
            self._world.tick()
        except Exception as e:
            self.get_logger().error(f"Tick failed: {e}")

    def destroy_node(self):
        # TODO: Harden it, if sim crashes before this node, weird and ugly tracebacks exist. Just a precaution here.

        self.get_logger().info("Shutting down...")

        if hasattr(self, "_tick_timer"):
            self.destroy_timer(self._tick_timer)

        for sensor in self._sensors:
            sensor.destroy()
        if self._vehicle:
            self._vehicle.destroy()

        if self._original_settings and self._world:
            self._world.apply_settings(self._original_settings)

        self.get_logger().info("Safely destroyed vehicles and sensors")

        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = HeroSpawner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()