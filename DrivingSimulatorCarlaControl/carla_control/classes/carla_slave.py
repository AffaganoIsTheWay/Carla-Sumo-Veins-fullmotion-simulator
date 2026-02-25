"""

 ██████╗ █████╗ ██████╗ ██╗      █████╗
██╔════╝██╔══██╗██╔══██╗██║     ██╔══██╗
██║     ███████║██████╔╝██║     ███████║
██║     ██╔══██║██╔══██╗██║     ██╔══██║
╚██████╗██║  ██║██║  ██║███████╗██║  ██║
 ╚═════╝╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝╚═╝  ╚═╝

███████╗██╗      █████╗ ██╗   ██╗███████╗
██╔════╝██║     ██╔══██╗██║   ██║██╔════╝
███████╗██║     ███████║██║   ██║█████╗
╚════██║██║     ██╔══██║╚██╗ ██╔╝██╔══╝
███████║███████╗██║  ██║ ╚████╔╝ ███████╗
╚══════╝╚══════╝╚═╝  ╚═╝  ╚═══╝  ╚══════╝

Class to handle a Carla slave instance.

"""

# 1# IMPORTS

# Standard imports
import logging
from time import sleep

# Third-party imports
import carla
import numpy as np
import zmq

# Local imports
from carla_control.classes.camera_transforms import CameraTransform


class CarlaSlave:
    """
    CarlaSlave carla slave instance.
    """

    def __init__(self, settings, ego_config, slave_type):
        """
        __init__ constructor.

        :param settings: The settings TOML
        :type settings: dict
        :param ego_config: The ego vehicle config JSON
        :type ego_config: dict
        :param slave_type: The type of the slave
        :type slave_type: str
        """

        # Initialize internal ZMQ connection with the slaves
        self.context = zmq.Context()
        self.master_frontend = self.context.socket(zmq.SUB)
        self.master_frontend.connect(settings["master"]["frontend_tcp"])
        self.master_frontend.subscribe("")
        self.master_backend = self.context.socket(zmq.PUB)
        self.master_backend.connect(settings["master"]["backend_tcp"])

        # Initialize internal ZMQ connection with master and slaves
        self.traffic_frontend = self.context.socket(zmq.PUB)
        self.traffic_frontend.connect(settings["simulation"]["traffic"]["frontend_tcp"])

        self.traffic_backend = self.context.socket(zmq.SUB)
        self.traffic_backend.setsockopt(zmq.CONFLATE, 1)
        self.traffic_backend.connect(settings["simulation"]["traffic"]["backend_tcp"])
        self.traffic_backend.subscribe("")

        # Traffic seed
        self.traffic_seed = settings["simulation"]["traffic"]["tm_seed"]

        # Initialise carla
        self.client = carla.Client(
            settings["slaves"][slave_type]["carla_host"],
            settings["slaves"][slave_type]["carla_port"],
        )
        self.client.set_timeout(settings["slaves"][slave_type]["carla_timeout"])

        # Get the world
        self.world = self.client.load_world(
            settings["simulation"]["environment"]["town"]
        )

        # Reset the traffic lights
        self.world.reset_all_traffic_lights()

        # Enable synchronous mode
        carla_settings = self.world.get_settings()
        carla_settings.synchronous_mode = True
        carla_settings.fixed_delta_seconds = 1.0 / settings["simulation"]["fps"]
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
            self.spawn_points = [carla.Transform(location=carla.Location(x=0, y=0, z=258))]

        # # Generate traffic
        # self.traffic = self.generate_traffic(
        #     settings["simulation"]["traffic"]["tm_seed"],
        #     settings["simulation"]["traffic"]["number_of_vehicles"],
        #     settings["simulation"]["traffic"]["number_of_walkers"],
        # )
        # for entity in self.traffic:
        #     entity.set_simulate_physics(False)
        self.traffic = {}

        # Spawn the ego vehicle
        self.ego_vehicle = None
        self.parse_ego_config(ego_config)

        # Get the spectator
        self.spectator = self.world.get_spectator()
        self.spectator_origin = self.spectator.get_transform()
        self.spectator_transform = self.spectator_origin
        self.spectator_relative_transform = CameraTransform().get_camera_transform(
            slave_type
        )

        # Save the settings
        self.settings = settings

    def close(self):
        """
        __del__ destructor.
        """

        # Destroy actors
        self.ego_vehicle.destroy()

        # Destory traffic
        for entity in self.traffic:
            entity.destroy()

        # Reset the spectator
        self.spectator.set_transform(self.spectator_origin)

        # Update the world
        self.world.tick()

        # Disable synchronous mode
        carla_settings = self.world.get_settings()
        carla_settings.synchronous_mode = False
        self.world.apply_settings(carla_settings)

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

        # Disable the physics of the vehicle
        self.ego_vehicle.set_simulate_physics(False)

    def get_actor_blueprints(self, filter_bp, generation):
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


    def follow(self, predicted_transform):
        """
        follow follow the ego vehicle with the spectator.

        :param predicted_transform: The predicted transform of the ego vehicle
        :type predicted_transform: carla.Transform
        """

        # Compute the spectator transform
        transform = np.matmul(
            predicted_transform.get_matrix(),
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

    def advance(self, predicted_transform, actual_transform, traffic_transforms):
        """
        advance advance the simulation by copying the master ego vehicle.

        :param predicted_transform: The predicted transform, default None
        :type predicted_transform: carla.Transform
        :param actual_transform: The actual transform, default None
        :type actual_transform: carla.Transform
        :return: The predicted and actual transforms (None if not the primary
           server)
        :rtype: tuple
        """

        # Initialise a carla batch
        batch = []

        # Follow the ego vehicle with the spectator
        self.follow(predicted_transform)

        # Apply the spectator transform (in the batch)
        batch.append(
            carla.command.ApplyTransform(self.spectator.id, self.spectator_transform)
        )

        # Apply the actual transform to the vehicle (in the batch)
        batch.append(
            carla.command.ApplyTransform(self.ego_vehicle.id, actual_transform)
        )

        # Apply the batch
        commands = self.client.apply_batch_sync(batch)

        # Check if the batch was successful
        for command in commands:
            if command.has_error():
                print(command.error)
                return

        # Tick the world
        self.world.tick()

    def recv_from_master(self):
        """
        recv_from_master receive the message from the master.

        :return: The message from the master
        :rtype: list
        """

        message = self.master_frontend.recv_pyobj()

        ego_vehicle_transform = carla.Transform(
            carla.Location(
                x=message["ego_vehicle"]["transform"]["x"],
                y=message["ego_vehicle"]["transform"]["y"],
                z=message["ego_vehicle"]["transform"]["z"],
            ),
            carla.Rotation(
                pitch=message["ego_vehicle"]["transform"]["pitch"],
                yaw=message["ego_vehicle"]["transform"]["yaw"],
                roll=message["ego_vehicle"]["transform"]["roll"],
            ),
        )
        ego_vehicle_predicted_transform = carla.Transform(
            carla.Location(
                x=message["ego_vehicle"]["predicted_transform"]["x"],
                y=message["ego_vehicle"]["predicted_transform"]["y"],
                z=message["ego_vehicle"]["predicted_transform"]["z"],
            ),
            carla.Rotation(
                pitch=message["ego_vehicle"]["predicted_transform"]["pitch"],
                yaw=message["ego_vehicle"]["predicted_transform"]["yaw"],
                roll=message["ego_vehicle"]["predicted_transform"]["roll"],
            ),
        )
        traffic_transforms = [
            carla.Transform(
                carla.Location(
                    x=traffic_transform["x"],
                    y=traffic_transform["y"],
                    z=traffic_transform["z"],
                ),
                carla.Rotation(
                    pitch=traffic_transform["pitch"],
                    yaw=traffic_transform["yaw"],
                    roll=traffic_transform["roll"],
                ),
            )
            for traffic_transform in message["traffic"]
        ]

        return (
            ego_vehicle_predicted_transform,
            ego_vehicle_transform,
            traffic_transforms,
        )

    def signal_master(self):
        """
        signal_master signal the master.
        """

        self.master_backend.send(b"")

    def run(self):
        """
        run runs the slave coordinated by the master.

        :param master: The master of the simulation
        :type master: CarlaMaster
        """
        # Define the traffic variables
        spawn_timestamp = None

        # Keep signaling the master until a response comes back
        signaling = True
        while signaling:
            self.signal_master()
            try:
                self.master_frontend.recv(zmq.DONTWAIT)
                signaling = False
            except zmq.Again:
                sleep(1e-3)

        # Main loop
        np.random.seed(self.traffic_seed)
        print("Simulation started")
        running = True
        while running:
            try:

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
                        elif ((message_timestamp - spawn_timestamp).seconds > 10) :
                            print("Crashed, cleaning up old traffic ")
                            self.destroy_traffic()
                            spawn_timestamp = None

                except zmq.Again:
                    pass


                # Receive the data from the master
                data = self.recv_from_master()

                # Advance the simulation
                self.advance(data[0], data[1], data[2])

                # Signal the master
                self.signal_master()
            except KeyboardInterrupt:
                running = False
