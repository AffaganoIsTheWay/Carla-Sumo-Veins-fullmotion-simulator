"""

 ██████╗ █████╗ ██████╗ ██╗      █████╗
██╔════╝██╔══██╗██╔══██╗██║     ██╔══██╗
██║     ███████║██████╔╝██║     ███████║
██║     ██╔══██║██╔══██╗██║     ██╔══██║
╚██████╗██║  ██║██║  ██║███████╗██║  ██║
 ╚═════╝╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝╚═╝  ╚═╝

███╗   ███╗ █████╗ ███████╗████████╗███████╗██████╗
████╗ ████║██╔══██╗██╔════╝╚══██╔══╝██╔════╝██╔══██╗
██╔████╔██║███████║███████╗   ██║   █████╗  ██████╔╝
██║╚██╔╝██║██╔══██║╚════██║   ██║   ██╔══╝  ██╔══██╗
██║ ╚═╝ ██║██║  ██║███████║   ██║   ███████╗██║  ██║
╚═╝     ╚═╝╚═╝  ╚═╝╚══════╝   ╚═╝   ╚══════╝╚═╝  ╚═╝

Class to handle a Carla master instance.

"""

# 1# IMPORTS

# Standard imports
import sys
import os
import tomli
from queue import Queue, Empty
import logging
import pickle
from time import sleep
import datetime

# Third-party imports
import carla
import zmq
import numpy as np
import pygame as pg
import flatbuffers as fb

# Local imports
from carla_control.classes.camera_transforms import CameraTransform

# Import the interfaces
INTERFACES_PATH = os.path.join(
    os.path.dirname(__file__),
    *[".."] * 2,
    "third_party",
    "DrivingSimulatorInterfaces",
    "interfaces",
)
sys.path.append(INTERFACES_PATH)

from DrivingSimulator.Cockpit import ICockpit, GearShift, SpecialButtonLeft
from DrivingSimulator.EnvironmentInteractions import IEnvironmentInteractions
from DrivingSimulator.EgoVehicle import IEgoVehicle
from DrivingSimulator.EgoVehicle.Chassis import IChassis
from DrivingSimulator.EgoVehicle.ECU import IECU
from DrivingSimulator.EgoVehicle.Powertrain import IPowertrain, CombustionEngine
from DrivingSimulator.EgoVehicle.Sensors import ISensors
from DrivingSimulator.EgoVehicle.Sensors import IMU as EGO_IMU
from DrivingSimulator.EgoVehicle.Sensors import Vec3 as EGO_Vec3
from DrivingSimulator.EgoVehicle.Sensors import (
    TransformationMatrix as EGO_TransformationMatrix,
)
from DrivingSimulator.EgoVehicle.SteeringSystem import ISteeringSystem
from DrivingSimulator.Sensors import (
    GNSS,
    IMU,
    Camera,
    CameraType,
    TransformationMatrix,
    Vec3,
)


SUMO_ADDR_EGO="tcp://10.196.16.101:5580"
EME_BRAKE_ADDR="tcp://10.196.16.101:5590"

class CarlaMaster:
    """
    CarlaMaster carla master instance.
    """

    def __init__(self, settings, ego_config, slave_num):
        """
        __init__ constructor.

        :param settings: The settings TOML
        :type settings: dict
        :param ego_config: The ego vehicle config JSON
        :type ego_config: dict
        :param slave_num: The number of slaves
        :type slave_num: int
        """

        # Save the front-end and back-end TCP addresses
        self.frontend_tcp = settings["broker"]["frontend_tcp"]
        self.backend_tcp = settings["broker"]["backend_tcp"]

        # Save a flag to know if the external vehicle model is enabled
        self.external_vehicle_model = False

        # Prepare a egovehicle flatbuffer object
        self.ego_vehicle_fb = IEgoVehicle.IEgoVehicleT()

        # Map to properly handle button presses
        self.btn_presses = {
            "special_button_left": SpecialButtonLeft.SpecialButtonLeft.RELEASED
        }

        # Initialise ZMQ
        self.context = zmq.Context()
        self.frontend = self.context.socket(zmq.PUB)
        self.backend = self.context.socket(zmq.SUB)

        self.frontend.setsockopt(zmq.LINGER, 0)

        self.frontend.connect(self.frontend_tcp)
        self.backend.connect(self.backend_tcp)

        # Load the topics
        with open(os.path.join(INTERFACES_PATH, "topics.toml"), "rb") as file:
            self.topics = tomli.load(file)

        # Subscribe to ICockpit and IEnvironmentInteractions
        self.backend.subscribe(self.topics["DrivingSimulator"]["Cockpit"]["ICockpit"])
        self.backend.subscribe(
            self.topics["DrivingSimulator"]["EnvironmentInteractions"][
                "IEnvironmentInteractions"
            ]
        )

        # Initialize internal ZMQ connection with the slaves
        self.master_frontend = self.context.socket(zmq.PUB)
        self.master_frontend.setsockopt(zmq.LINGER, 0)
        self.master_frontend.bind(settings["master"]["frontend_tcp"])
        self.master_backend = self.context.socket(zmq.SUB)
        self.master_backend.bind(settings["master"]["backend_tcp"])
        self.master_backend.subscribe("")
        self.slaves_number = slave_num

        # Initialise internal ZMQ connection with the traffic generator
        self.traffic_frontend = self.context.socket(zmq.PUB)
        self.traffic_frontend.setsockopt(zmq.LINGER, 0)
        self.traffic_frontend.connect(settings["simulation"]["traffic"]["frontend_tcp"])

        self.traffic_backend = self.context.socket(zmq.SUB)
        self.traffic_backend.setsockopt(zmq.CONFLATE, 1)
        self.traffic_backend.connect(settings["simulation"]["traffic"]["backend_tcp"])
        self.traffic_backend.subscribe("")

        self.ego_sumo = self.context.socket(zmq.PUB)
        self.ego_sumo.bind(SUMO_ADDR_EGO)

        self.eme_brake_zmq = self.context.socket(zmq.SUB)
        self.eme_brake_zmq.bind(EME_BRAKE_ADDR)
        self.eme_brake_zmq.subscribe("")
        self.eme_brake = b'0'

        # Traffic seed
        self.traffic_seed = settings["simulation"]["traffic"]["tm_seed"]

        # Initialise carla
        self.client = carla.Client(
            settings["master"]["carla_host"], settings["master"]["carla_port"]
        )
        self.client.set_timeout(settings["master"]["carla_timeout"])

        # Get the world
        self.world = self.client.load_world(
            settings["simulation"]["environment"]["town"]
        )

        # Reset the traffic lights
        self.world.reset_all_traffic_lights()

        # Set the FPS
        self.fps = settings["simulation"]["fps"]
        self.timestep = 1.0 / self.fps
        self.half_timestep_squared = 0.5 * self.timestep**2

        # Enable synchronous mode
        carla_settings = self.world.get_settings()
        carla_settings.synchronous_mode = True
        carla_settings.fixed_delta_seconds = 1.0 / self.fps
        self.world.apply_settings(carla_settings)

        weather = self.world.get_weather()
        weather.cloudiness = settings["simulation"]["weather"]["cloudiness"]
        weather.precipitation = settings["simulation"]["weather"]["precipitation"]
        weather.precipitation_deposits = settings["simulation"]["weather"][
            "precipitation_deposits"
        ]
        weather.sun_altitude_angle = settings["simulation"]["weather"][
            "sun_altitude_angle"
        ]
        self.world.set_weather(weather)

        # Get spawn points
        self.spawn_points = self.world.get_map().get_spawn_points()
        if not self.spawn_points:
            self.spawn_points = [
                carla.Transform(location=carla.Location(x=0, y=0, z=258))
            ]

        # Spawn the ego vehicle parsing the config file
        self.ego_vehicle = None
        self.ego_vehicle_transform = None
        self.ego_vehicle_predicted_transform = None
        self.sensors = {}
        self.sensor_queue = Queue()
        self.parse_ego_config(ego_config)

        # Get the spectator
        self.spectator = self.world.get_spectator()
        self.spectator_origin = self.spectator.get_transform()
        self.spectator_transform = self.spectator_origin
        self.spectator_relative_transform = CameraTransform().get_camera_transform(
            settings["master"]["screen"]
        )

        # Create the cockpit
        self.cockpit = None
        self.reverse = False

        # Create the environment interactions
        self.environment_interactions = None

        # Get the traffic & traffic transforms
        self.traffic = {}
        self.traffic_transforms = [vehicle.get_transform() for vehicle in list(self.traffic.values())]

    def transform_to_dict(self, transform):
        """
        transform_to_dict transform a carla transform to a dictionary.

        :param transform: The carla transform
        :type transform: carla.Transform
        :return: The dictionary
        :rtype: dict
        """

        return {
            "x": transform.location.x,
            "y": transform.location.y,
            "z": transform.location.z,
            "pitch": transform.rotation.pitch,
            "yaw": transform.rotation.yaw,
            "roll": transform.rotation.roll,
        }

    def fill_message(self):
        """
        init_message initialise the message to send to the slaves.

        :return: The message
        :rtype: dict
        """

        return {
            "ego_vehicle": {
                "transform": self.transform_to_dict(self.ego_vehicle_transform),
                "predicted_transform": self.transform_to_dict(
                    self.ego_vehicle_predicted_transform
                ),
            },
            "traffic": [
                self.transform_to_dict(vehicle.get_transform())
                for vehicle in list(self.traffic.values())
            ],
        }

    def get_actor_blueprints(self, filter_bp, generation):
        """
        get_actor_blueprints get the actor blueprints list filtered out.

        Args:
            filter_bp (_type_): _description_
            generation (_type_): _description_

        Returns:
            _type_: _description_
        """

        bps = self.world.get_blueprint_library().filter(filter_bp)

        if generation.lower() == "all":
            return bps

        # If the filter returns only one bp, we assume that this one needed
        # and therefore, we ignore the generation
        if len(bps) == 1:
            return bps

        try:
            int_generation = int(generation)
            # Check if generation is in available generations
            if int_generation in [1, 2]:
                bps = [
                    x
                    for x in bps
                    if int(x.get_attribute("generation")) == int_generation
                ]
                return bps
            else:
                print(
                    "   Warning! Actor Generation is not valid. No actor will be spawned."
                )
                return []
        except:
            print(
                "   Warning! Actor Generation is not valid. No actor will be spawned."
            )
            return []

    # Spawn fucntion that take a list of dict of vehicle position and creatre them
    def spawn_traffic(self, vehicles):
        blueprints = self.world.get_blueprint_library().filter("vehicle.*")

        for v in vehicles:
            sid = v["id"]
            if sid in self.traffic:
                continue

            bp = np.random.choice(blueprints)
            bp.set_attribute("role_name", "external")

            transform = carla.Transform(
                carla.Location(v["x"], v["y"], v["z"]+250), #safe position to spawn
                carla.Rotation(
                    pitch=v["pitch"],
                    yaw=v["yaw"],
                    roll=v["roll"]
                )
            )

            actor = self.world.spawn_actor(bp, transform)

            # Disable physics
            actor.set_simulate_physics(False)

            # Save vehicle ids in order to control them dynamically
            self.traffic[sid] = actor

            print("Traffic spawned")

    # Dynamic despawn function that check the difference by actor ids
    def despawn_traffic(self, vehicles):
        alive = {v["id"] for v in vehicles}

        for sid in list(self.traffic.keys()):
            if sid not in alive:
                self.traffic[sid].destroy()
                del self.traffic[sid]

        print("Traffic despawned")

    # Change position of actors based on the new dict
    def update_traffic(self, vehicles):
        batch = []

        for v in vehicles:
            actor = self.traffic.get(v["id"])
            if not actor:
                self.close()

            transform = carla.Transform(
                carla.Location(v["x"], v["y"], v["z"]),
                carla.Rotation(
                    pitch=v["pitch"],
                    yaw=v["yaw"],
                    roll=v["roll"]
                )
            )

            batch.append(
                carla.command.ApplyTransform(actor.id, transform)
            )

        if batch:
            self.client.apply_batch_sync(batch, False)

        print("Traffic updated")


    def destroy_traffic(self):
        """
        destroy_traffic destroy the traffic actors.

        :param traffic_actors: The traffic actors
        :type traffic_actors: list
        """
        for vehicle in list(self.traffic.values()):
            # print("Destroying vehicle:" + str(vehicle.id))
            vehicle.destroy()

        self.traffic = {}
        self.traffic_transforms = [vehicle.get_transform() for vehicle in list(self.traffic.values())]

    def close(self):
        """
        close clean-up
        """
        print("Closing ...")

        # Destroy ego_vehicle
        self.ego_vehicle.destroy()

        # Stop and destroy sensors
        for sensor in self.sensors.values():
            sensor.stop()
            sensor.destroy()

        # Destroy traffic
        self.destroy_traffic()

        # Reset the spectator
        self.spectator.set_transform(self.spectator_origin)

        # Update the world
        self.world.tick()

        # Disable synchronous mode
        carla_settings = self.world.get_settings()
        carla_settings.synchronous_mode = False
        self.world.apply_settings(carla_settings)

        # Close ZMQ
        self.context.destroy()

    def parse_ego_config(self, ego_config):
        """
        parse_ego_config parse the ego vehicle config.

        :param ego_config: The ego vehicle config
        :type ego_config: dict
        """

        # Get the blueprint library
        blueprint_library = self.world.get_blueprint_library()

        # Get the blueprint
        bp = blueprint_library.filter(ego_config["vehicle"]["blueprint"])[0]
        bp.set_attribute("role_name", "hero")

        # Spawn the vehicle
        if "transform" not in ego_config["vehicle"]:
            spawn_point = self.spawn_points[0]
        else:
            spawn_point = carla.Transform(
                carla.Location(
                    x=ego_config["vehicle"]["transform"]["x"],
                    y=ego_config["vehicle"]["transform"]["y"],
                    z=ego_config["vehicle"]["transform"]["z"],
                ),
                carla.Rotation(
                    pitch=ego_config["vehicle"]["transform"]["pitch"],
                    yaw=ego_config["vehicle"]["transform"]["yaw"],
                    roll=ego_config["vehicle"]["transform"]["roll"],
                ),
            )
        self.ego_vehicle = self.world.spawn_actor(bp, spawn_point)

        # Get the vehicle transform
        self.ego_vehicle_transform = spawn_point
        self.ego_vehicle_predicted_transform = spawn_point

        # Add the sensors
        attachment_types = {
            "Rigid": carla.AttachmentType.Rigid,
            "SpringArm": carla.AttachmentType.SpringArm,
            "SpringArmGhost": carla.AttachmentType.SpringArmGhost,
        }
        for sensor in ego_config["sensors"]:
            # Check if the sensor is enabled
            if not ego_config["sensors"][sensor]["enabled"]:
                continue

            # Get the blueprint
            bp = blueprint_library.find(ego_config["sensors"][sensor]["blueprint"])
            for key, value in ego_config["sensors"][sensor]["attributes"].items():
                bp.set_attribute(key, str(value))

            # Spawn the sensor
            spawn_point = carla.Transform(
                carla.Location(
                    x=ego_config["sensors"][sensor]["transform"]["x"],
                    y=ego_config["sensors"][sensor]["transform"]["y"],
                    z=ego_config["sensors"][sensor]["transform"]["z"],
                ),
                carla.Rotation(
                    pitch=ego_config["sensors"][sensor]["transform"]["pitch"],
                    yaw=ego_config["sensors"][sensor]["transform"]["yaw"],
                    roll=ego_config["sensors"][sensor]["transform"]["roll"],
                ),
            )
            sensor_name = ego_config["sensors"][sensor]["blueprint"] + "_" + sensor
            self.sensors[sensor_name] = self.world.spawn_actor(
                bp,
                spawn_point,
                attach_to=self.ego_vehicle,
                attachment_type=attachment_types[
                    ego_config["sensors"][sensor]["attachment_type"]
                ],
            )

            # Listen on the sensor
            if ego_config["sensors"][sensor]["blueprint"] == "sensor.other.gnss":
                self.sensors[sensor_name].listen(
                    lambda measurement: self.sensor_callback(
                        measurement, "sensor.other.gnss"
                    )
                )
            elif ego_config["sensors"][sensor]["blueprint"] == "sensor.other.imu":
                self.sensors[sensor_name].listen(
                    lambda measurement: self.sensor_callback(
                        measurement, "sensor.other.imu"
                    )
                )

    def get_sensor_callback_map(self):
        """
        get_sensor_callback_map get the sensor callback map.

        :return: The sensor callback map
        :rtype: dict
        """

        return {
            "sensor.other.gnss": self.gnss_measurement,
            "sensor.other.imu": self.imu_measurement,
            "sensor.camera.rgb": self.camera_rgb_measurement,
        }

    def sensor_callback(self, measurement, name):
        """
        sensor_callback sensor callback.

        :param measurement: The sensor measurement
        :type measurement: carla.SensorData
        :param name: The name of the sensor
        :type name: str
        """

        # Add the measurement to the queue
        self.sensor_queue.put((name, measurement))

    def gnss_measurement(self, measurement):
        """
        gnss_measurement GNSS measurement callback.

        :param measurement: The GNSS measurement
        :type measurement: carla.GnssMeasurement
        """

        # Pack the data
        builder = fb.Builder(0)
        GNSS.GNSSStart(builder)
        GNSS.GNSSAddTransform(
            builder,
            TransformationMatrix.CreateTransformationMatrix(
                builder,
                [elem for row in measurement.transform.get_matrix() for elem in row],
            ),
        )
        GNSS.GNSSAddLongitude(builder, measurement.longitude)
        GNSS.GNSSAddLatitude(builder, measurement.latitude)
        GNSS.GNSSAddAltitude(builder, measurement.altitude)
        gnss = GNSS.GNSSEnd(builder)
        builder.Finish(gnss)

        # Send the packet
        self.frontend.send_multipart(
            [
                self.topics["DrivingSimulator"]["Sensors"]["GNSS"].encode("utf-8"),
                builder.Output(),
            ],
            zmq.DONTWAIT,
        )

    def imu_measurement(self, measurement):
        """
        imu_measurement IMU measurement callback.

        :param measurement: The IMU measurement
        :type measurement: carla.ImuMeasurement
        """

        # Pack the data
        builder = fb.Builder(0)
        IMU.IMUStart(builder)
        IMU.IMUAddCompass(builder, measurement.compass)
        IMU.IMUAddTransform(
            builder,
            TransformationMatrix.CreateTransformationMatrix(
                builder,
                [elem for row in measurement.transform.get_matrix() for elem in row],
            ),
        )
        IMU.IMUAddAcceleration(
            builder,
            Vec3.CreateVec3(
                builder,
                measurement.accelerometer.x,
                -measurement.accelerometer.y,
                measurement.accelerometer.z,
            ),
        )
        IMU.IMUAddAngularVelocity(
            builder,
            Vec3.CreateVec3(
                builder,
                measurement.gyroscope.x,
                -measurement.gyroscope.y,
                -measurement.gyroscope.z,
            ),
        )
        imu = IMU.IMUEnd(builder)
        builder.Finish(imu)

        # Send the packet
        self.frontend.send_multipart(
            [
                self.topics["DrivingSimulator"]["Sensors"]["IMU"].encode("utf-8"),
                builder.Output(),
            ],
            zmq.DONTWAIT,
        )

    def camera_rgb_measurement(self, measurement):
        """
        camera_rgb_measurement camera RGB measurement callback.

        :param measurement: The camera RGB measurement
        :type measurement: carla.Image
        """

        # Pack the data
        builder = fb.Builder(0)
        Camera.CameraStart(builder)
        Camera.CameraAddTransform(
            builder,
            TransformationMatrix.CreateTransformationMatrix(
                builder,
                [elem for row in measurement.transform.get_matrix() for elem in row],
            ),
        )
        Camera.CameraAddType(builder, CameraType.CameraType.RGB)
        Camera.CameraAddFov(builder, measurement.fov)
        Camera.CameraAddWidth(builder, measurement.width)
        Camera.CameraAddHeight(builder, measurement.height)
        Camera.CameraAddRawData(builder, measurement.raw_data)
        camera = Camera.CameraEnd(builder)
        builder.Finish(camera)

        # Send the packet
        self.frontend.send_multipart(
            [
                self.topics["DrivingSimulator"]["Sensors"]["Camera"].encode("utf-8"),
                builder.Output(),
            ],
            zmq.DONTWAIT,
        )

    def recv(self):
        """
        get_controls get the controls from the ZMQ socket.
        """

        # Receive the ICockpit and IEnvironmentInteractions messages
        msg_cockpit = None
        msg_environment_interactions = None
        while True:
            try:
                msg = self.backend.recv_multipart(zmq.DONTWAIT)
                topic = msg[0].decode("utf-8")
                if topic == self.topics["DrivingSimulator"]["Cockpit"]["ICockpit"]:
                    msg_cockpit = msg
                elif (
                    topic
                    == self.topics["DrivingSimulator"]["EnvironmentInteractions"][
                        "IEnvironmentInteractions"
                    ]
                ):
                    msg_environment_interactions = msg
            except zmq.error.Again:
                if msg_cockpit:
                    self.cockpit = ICockpit.ICockpit.GetRootAs(msg_cockpit[1], 0)
                if msg_environment_interactions:
                    self.environment_interactions = (
                        IEnvironmentInteractions.IEnvironmentInteractions.GetRootAs(
                            msg_environment_interactions[1], 0
                        )
                    )
                break

    def predict(self):
        """
        predict predict the transform at the next frame.
        """

        # Get the vehicle data
        self.ego_vehicle_predicted_transform = self.ego_vehicle_transform
        vehicle_velocity = self.ego_vehicle.get_velocity()
        vehicle_angular_velocity = self.ego_vehicle.get_angular_velocity()
        vehicle_acceleration = self.ego_vehicle.get_acceleration()

        # Project forward using the velocity, angular velocity, acceleration,
        # and frequency
        self.ego_vehicle_predicted_transform.location.x += (
            vehicle_velocity.x * self.timestep
            + vehicle_acceleration.x * self.half_timestep_squared
        )
        self.ego_vehicle_predicted_transform.location.y += (
            vehicle_velocity.y * self.timestep
            + vehicle_acceleration.y * self.half_timestep_squared
        )
        self.ego_vehicle_predicted_transform.location.z += (
            vehicle_velocity.z * self.timestep
            + vehicle_acceleration.z * self.half_timestep_squared
        )
        self.ego_vehicle_predicted_transform.rotation.roll += (
            vehicle_angular_velocity.x * self.timestep
        )
        self.ego_vehicle_predicted_transform.rotation.pitch += (
            vehicle_angular_velocity.y * self.timestep
        )
        self.ego_vehicle_predicted_transform.rotation.yaw += (
            vehicle_angular_velocity.z * self.timestep
        )

    def follow(self):
        """
        follow follow the ego vehicle with the spectator.
        """

        # Compute the spectator transform
        transform = np.matmul(
            self.ego_vehicle_predicted_transform.get_matrix(),
            self.spectator_relative_transform.get_matrix(),
        )

        # Compute the spectator transform
        self.spectator_transform = carla.Transform(
            carla.Location(x=transform[0, 3], y=transform[1, 3], z=transform[2, 3]),
            carla.Rotation(
                pitch=np.rad2deg(np.arcsin(transform[2, 0])),
                yaw=np.rad2deg(np.arctan2(transform[1, 0], transform[0, 0])),
                roll=np.rad2deg(np.arctan2(-transform[2, 1], transform[2, 2])),
            ),
        )

    def send_egovehicle(self):
        """
        send_egovehicle send the ego vehicle data.
        """

        # TODO: figure out in which RF the ego measurements are given

        self.ego_vehicle_fb.sensors = None
        self.ego_vehicle_fb.ecu = None
        self.ego_vehicle_fb.powertrain = None
        self.ego_vehicle_fb.chassis = None
        self.ego_vehicle_fb.steeringSystem = None

        # Extract the vehicle data
        ego_controls = self.ego_vehicle.get_control()
        ego_acceleration = self.ego_vehicle.get_acceleration()
        ego_velocity = self.ego_vehicle.get_velocity()
        ego_angular_velocity = self.ego_vehicle.get_angular_velocity()
        ego_position = self.ego_vehicle.get_location()
        ego_transform = self.ego_vehicle.get_transform().get_matrix()
        ego_rotation = self.ego_vehicle.get_transform().rotation
        ego_acceleration.z += 9.81

        # Carla has no idea how to give you proper data, it always gives you the states in the global frame
        ego_transform_np = np.array(ego_transform)
        ego_rotation_np = ego_transform_np[0:3, 0:3]
        ego_position_np = ego_transform_np[3, 0:3]
        ego_acceleration_np = np.array([ego_acceleration.x, ego_acceleration.y, ego_acceleration.z, 0.0])
        ego_velocity_np = np.array([ego_velocity.x, ego_velocity.y, ego_velocity.z, 0.0])
        ego_angular_velocity_np = np.array([ego_angular_velocity.x, ego_angular_velocity.y, ego_angular_velocity.z, 0.0])

        # This inverts the transformation matrix
        ego_rotation_np = ego_rotation_np.T
        ego_position_np = -ego_rotation_np @ ego_position_np
        ego_transform_np[0:3, 0:3] = ego_rotation_np
        ego_transform_np[3, 0:3] = ego_position_np

        # Get real data
        ego_acceleration_np = ego_transform_np @ ego_acceleration_np.T
        ego_velocity_np = ego_transform_np @ ego_velocity_np.T
        ego_angular_velocity_np = ego_transform_np @ ego_angular_velocity_np.T

        # Update data
        ego_acceleration.x = ego_acceleration_np[0]
        ego_acceleration.y = ego_acceleration_np[1]
        ego_acceleration.z = ego_acceleration_np[2]
        ego_velocity.x = ego_velocity_np[0]
        ego_velocity.y = ego_velocity_np[1]
        ego_velocity.z = ego_velocity_np[2]
        ego_angular_velocity.x = ego_angular_velocity_np[0]
        ego_angular_velocity.y = ego_angular_velocity_np[1]
        ego_angular_velocity.z = ego_angular_velocity_np[2]

        # Sensors
        imu_transform = EGO_TransformationMatrix.TransformationMatrixT()
        imu_transform.data = [elem for row in ego_transform for elem in row]
        acceleration = EGO_Vec3.Vec3T()
        acceleration.x = ego_acceleration.x
        acceleration.y = -ego_acceleration.y
        acceleration.z = ego_acceleration.z
        angular_velocity = EGO_Vec3.Vec3T()
        angular_velocity.x = -ego_angular_velocity.x
        angular_velocity.y = ego_angular_velocity.y
        angular_velocity.z = -ego_angular_velocity.z
        angular_acceleration = EGO_Vec3.Vec3T()
        angular_acceleration.x = 0.0
        angular_acceleration.y = 0.0
        angular_acceleration.z = 0.0
        imu = EGO_IMU.IMUT()
        imu.transform = imu_transform
        imu.compass = -ego_rotation.yaw * np.pi / 180.0 + np.pi / 2.0
        imu.acceleration = acceleration
        imu.angularVelocity = angular_velocity
        imu.angularAcceleration = angular_acceleration

        sensors = ISensors.ISensorsT()
        sensors.imu = imu

        # ECU
        ecu = IECU.IECUT()
        ecu.ecuGear = -1 if ego_controls.reverse else ego_controls.gear
        ecu.ecuThrottle = ego_controls.throttle
        ecu.ecuBrake = ego_controls.brake
        ecu.ecuClutch = 0.0

        # Powertrain
        combustion_engine = CombustionEngine.CombustionEngineT()
        combustion_engine.outputShaftSpeed = np.clip(ego_velocity.x * 160.0, 0.0, 8000.0) # 8000 rpm at 180 km/h

        powertrain = IPowertrain.IPowertrainT()
        powertrain.combustionEngineOutput = combustion_engine

        # Chassis
        chassis = IChassis.IChassisT()
        chassis.u = ego_velocity.x
        chassis.v = -ego_velocity.y
        chassis.w = ego_velocity.z
        chassis.omega_x = -ego_angular_velocity.x
        chassis.omega_y = ego_angular_velocity.y
        chassis.omega = -ego_angular_velocity.z
        chassis.x = ego_position.x
        chassis.y = -ego_position.y
        chassis.z = ego_position.z
        chassis.psi = -ego_rotation.yaw * np.pi / 180.0
        chassis.phi = ego_rotation.roll * np.pi / 180.0
        chassis.mu = -ego_rotation.pitch * np.pi / 180.0
        chassis.uDot = ego_acceleration.x
        chassis.vDot = -ego_acceleration.y
        chassis.wDot = ego_acceleration.z

        # SteeringSystem
        steering_system = ISteeringSystem.ISteeringSystemT()
        steering_system.steeringTorqueFeedback = -np.clip(-ego_acceleration.y * 0.5, -5.0, 5.0) # 5 Nm at 10 m/s^2

        # EgoVehicle
        self.ego_vehicle_fb.sensors = sensors
        self.ego_vehicle_fb.ecu = ecu
        self.ego_vehicle_fb.powertrain = powertrain
        self.ego_vehicle_fb.chassis = chassis
        self.ego_vehicle_fb.steeringSystem = steering_system

        builder = fb.Builder(0)
        ego_vehicle = self.ego_vehicle_fb.Pack(builder)
        builder.Finish(ego_vehicle)

        # Send the ego vehicle data
        self.frontend.send_multipart(
            [
                self.topics["DrivingSimulator"]["EgoVehicle"]["IEgoVehicle"].encode(
                    "utf-8"
                ),
                builder.Output(),
            ],
            zmq.DONTWAIT,
        )

    def advance(self):
        """
        advance advance the simulation by applying the controls to the ego
        vehicle.

        :return: The predicted and actual transforms (None if not the primary
           server)
        :rtype: tuple
        """

        # Initialise a carla batch
        batch = []

        # Get the controls
        self.recv()

        # Check if we got the environment_interactions message, if so and it is not running,
        # we need to change back to the default physics engine
        if self.environment_interactions is not None:
            if not self.environment_interactions.Running():
                if self.external_vehicle_model:
                    self.ego_vehicle.restore_physx_physics()
                    self.external_vehicle_model = False

            self.environment_interactions = None

        if not self.external_vehicle_model:
            # Compute the prediction of the vehicle transform
            self.predict()

            # Follow the ego vehicle with the spectator
            self.follow()

            # Apply the spectator transform (in the batch)
            batch.append(
                carla.command.ApplyTransform(
                    self.spectator.id, self.spectator_transform
                )
            )

        #controls emergency brake
        if self.eme_brake_zmq.poll(15, zmq.POLLIN):
            self.eme_brake = self.eme_brake_zmq.recv()

        if self.eme_brake.decode("utf-8") == '0':
            # Apply controls to the vehicle (in the batch)
            if self.cockpit is not None:
                # Check that the minimum controls are available
                if self.cockpit.SteeringWheel() is None:
                    return
                if self.cockpit.Pedals() is None:
                    return

                if self.cockpit.SteeringWheel().Buttons() is not None:
                    # Check if the user wants enable/disable the external vehicle model
                    if (
                        self.cockpit.SteeringWheel().Buttons().SpecialLeft()
                        == SpecialButtonLeft.SpecialButtonLeft.PRESSED
                    ) and self.btn_presses[
                        "special_button_left"
                    ] == SpecialButtonLeft.SpecialButtonLeft.RELEASED:
                        if not self.external_vehicle_model:
                            self.ego_vehicle.enable_zmq_physics(
                                self.frontend_tcp,
                                self.backend_tcp,
                                True,
                                self.spectator_relative_transform,
                            )
                            self.external_vehicle_model = True
                        elif self.external_vehicle_model:
                            self.ego_vehicle.restore_physx_physics()
                            self.external_vehicle_model = False

                    # Update button states
                    self.btn_presses["special_button_left"] = (
                        self.cockpit.SteeringWheel().Buttons().SpecialLeft()
                    )

                # Do not apply the controls if the external vehicle model is enabled
                if not self.external_vehicle_model:
                    # Handle the reverse gear
                    if self.cockpit.Shifter() == GearShift.GearShift.DOWN:
                        self.reverse = True
                    elif self.cockpit.Shifter() == GearShift.GearShift.UP:
                        self.reverse = False

                    # Add the vehicle control to the batch
                    batch.append(
                        carla.command.ApplyVehicleControl(
                            self.ego_vehicle.id,
                            carla.VehicleControl(
                                throttle=self.cockpit.Pedals().Throttle(),
                                steer=-self.cockpit.SteeringWheel().Angle() / (3 * np.pi),
                                brake=(
                                    self.cockpit.Pedals().Brake()
                                    if self.cockpit.Pedals().Brake() > 0.1
                                    else 0.0
                                ),
                                hand_brake=self.cockpit.Handbrake() > 0.5,
                                reverse=self.reverse,
                            ),
                        )
                    )

                    self.send_egovehicle()

                # Reset the cockpit
                self.cockpit = None
        else:
            print("Breaking!!!")
            batch.append(
                carla.command.ApplyVehicleControl(
                    self.ego_vehicle.id,
                    carla.VehicleControl(
                        throttle = 0,
                        steer = 0,
                        brake = 1,
                        hand_brake = 0,
                        reverse = self.reverse,
                    )
                )
            )

            self.send_egovehicle()
            self.cockpit = None

        # Apply the batch
        commands = self.client.apply_batch_sync(batch)

        # Check if the batch was successful
        for command in commands:
            if command.has_error():
                print(command.error)
                return

        # Tick the world
        self.world.tick()

        # Get the actual vehicle transform
        self.ego_vehicle_transform = self.ego_vehicle.get_transform()

        # If we are using an external vehicle model, we have perfect following
        if self.external_vehicle_model:
            self.ego_vehicle_predicted_transform = self.ego_vehicle_transform

        self.ego_sumo.send_pyobj({'x': self.ego_vehicle_transform.location.x,
                                  'y': self.ego_vehicle_transform.location.y,
                                  'yaw': self.ego_vehicle_transform.rotation.yaw})

        # Update the traffic vehicles transforms
        self.traffic_transforms = [vehicle.get_transform() for vehicle in list(self.traffic.values())]

    def send_to_slaves(self):
        """
        send_to_slaves send the traffic transforms to the slaves.
        """

        # Send the traffic transforms to the slaves
        self.master_frontend.send_pyobj(self.fill_message())

    def wait_for_slaves(self):
        """
        wait_for_slaves wait for the slaves to be ready.
        """

        # Wait for the slaves
        for _ in range(self.slaves_number):
            try:
                self.master_backend.recv()
            except zmq.Again:
                break

    def signal_slaves(self):
        """
        signal_slaves signal the slaves.
        """

        self.master_frontend.send(b"")

    def run(self):
        """
        run run the master.
        """
        # Create a pygame clock
        pg.init()
        pg_clock = pg.time.Clock()

        # Get the sensor callback map
        callbacks = self.get_sensor_callback_map()

        # Precompute the number of sensors
        num_sensors = len(self.sensors)

        # Wait for the slaves to be ready
        self.wait_for_slaves()
        self.signal_slaves()

        # Define the traffic variables
        spawn_timestamp = None
        np.random.seed(self.traffic_seed)

        # Main loop
        print("Simulation started, CTRL-C to exit.")
        running = True
        while running:
            print(f"FPS: {int(pg_clock.get_fps()):3d}", end="\r")

            try:
                # Tick the application
                pg_clock.tick_busy_loop(self.fps)

                # Try to read traffic information
                try:
                    message = self.traffic_backend.recv_pyobj(zmq.DONTWAIT)

                    if message["state"] == -1:
                        self.destroy_traffic()
                        spawn_timestamp = None
                        print("Traffic dead")
                    elif (message["state"] == 0 and len(message) > 1) :
                        message_timestamp = message["timestamp"]
                        if not spawn_timestamp or ((message_timestamp - spawn_timestamp).seconds < 10) :
                            vehicles = message["vehicles"]
                            self.spawn_traffic(vehicles) #new vehicles spawn
                            self.despawn_traffic(vehicles) #arrived vehicles get delated
                            self.update_traffic(vehicles) #position update
                            spawn_timestamp = message_timestamp

                            self.traffic_frontend.send(b"")
                        # check on timestamp difference for discard multiple initialization messages
                        else :
                            # if the traffic has crashed clean the old traffic and ON THE NEXT TICK spawn the new one
                            print("Crashed, cleaning up old traffic ")
                            self.destroy_traffic()
                            spawn_timestamp = None
                except zmq.Again:
                    pass


                # Advance the simulation
                self.advance()

                # Send the traffic transforms to the slaves
                self.send_to_slaves()

                # Wait for the sensor queue
                try:
                    for _ in range(num_sensors):
                        sensor = self.sensor_queue.get(True, 1.0)
                        callbacks[sensor[0]](sensor[1])
                except Empty:
                    pass

                # Wait for the slaves
                self.wait_for_slaves()

            except KeyboardInterrupt:
                running = False

        pg.quit()
