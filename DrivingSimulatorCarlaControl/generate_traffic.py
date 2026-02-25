"""
This file reads data from the dummy_send.py file, and applies it to the current
vehicle in the simulation. It is expected to be run on a the same machine as the
as the dummy_send.py file, and apply the changes to all the different carla
servers.
"""

# 1# IMPORTS

# Standard imports
import argparse
import logging
import os

# Third-party imports
import carla
import zmq
import tomli
import numpy as np
from time import sleep
import datetime

# Config
SUMO_TRAFFIC_ADDR = "tcp://10.196.16.101:5555"
SUMO_SYNC_ADDR = "tcp://10.196.16.101:5570"

# 1# FUNCTIONS

def parse_arguments():
    """
    parse_arguments parse the arguments of the program.

    :return: The parsed arguments
    :rtype: argparse.Namespace
    """

    dirname = os.path.dirname(os.path.abspath(__file__))
    default_settings = os.path.join(dirname, "settings.toml")

    # Parse arguments
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-s",
        "--settings",
        type=str,
        default=default_settings,
        help="Path to the settings TOML file",
    )
    parser.add_argument(
        "--slaves_num", type=int, default=0, help="Number of slaves to expect"
    )
    parser.add_argument(
        "-n","--number_of_vehicles", type=int, default=-1, help="Number of vehicle to spawn"
    )
    return parser.parse_args()

def get_actor_blueprints(world, filter_bp, generation):
        bps = world.get_blueprint_library().filter(filter_bp)

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
            print("  Warning! Actor Generation is not valid. No actor will be spawned.")
            return []

def create_traffic_actors_element(world, number_of_vehicles, settings):
    """
    create_traffic_actors_element creates the traffic actors element.

    :param world: The carla world
    :type world: carla.World
    :param settings: The settings
    :type settings: dict
    :return: The spawn points list, blueprint list and transform list
    :rtype: tuple

    """

    blueprints = get_actor_blueprints(world, "vehicle.*", "All")
    blueprints = [x for x in blueprints if x.get_attribute("base_type") == "car"]
    blueprints = sorted(blueprints, key=lambda bp: bp.id)

    # Get spawn points
    spawn_points = world.get_map().get_spawn_points()
    if not spawn_points:
        spawn_points = [carla.Transform(location=carla.Location(x=0, y=0, z=258))]
    spawn_points = spawn_points[1:]
    np.random.seed(settings["simulation"]["traffic"]["tm_seed"])
    np.random.shuffle(spawn_points)

    blueprints_list = []
    transform_list = []

    for n, transform in enumerate(spawn_points):
        if n >= number_of_vehicles:
            break
        blueprint = np.random.choice(blueprints)
        blueprints_list.append(blueprint)
        transform_list.append(transform)

    # Return spawn points list, blueprint list and transform list
    return spawn_points, blueprints_list

def transform_to_dict(transform):
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

def create_message(state, vehicles, number_of_vehicles=None, spawn_points=None, blueprints_list=None):
    """
    create_message creates the message.

    """


    # STATE 1: initialization
    # STATE 0: running
    # STATE -1: destroy
    if state == 0:
        return {
            "state": 0,
            "vehicles": vehicles,
            "timestamp": datetime.datetime.now(),
        }
    elif state == 1:
        return {"state": 1}
    elif state == -1:
        return {"state": -1}

    return 0

# Function used to syncronize Sumo simulation and Carla
def tick_sumo(sumo_sync):
    recived = True
    while recived:
        sumo_sync.send(b"tick")
        try:
            sumo_sync.recv()
            recived = False
        except zmq.Again:
            print("Sumo not responding, retring...")
            sleep(1)
            
        except KeyboardInterrupt:
            break

def main():
    """
    main main function of the program.
    """
    args = parse_arguments()

    # Get the settings
    with open(args.settings, "rb") as f:
        settings = tomli.load(f)

    # Connect to the carla client
    client = carla.Client(settings["master"]["carla_host"],
                          settings["master"]["carla_port"])
    client.set_timeout(settings["master"]["carla_timeout"])

    # Get the world
    world = client.get_world()

    # Enable the traffic manager
    traffic_manager = client.get_trafficmanager(
        settings["simulation"]["traffic"]["tm_port"])
    traffic_manager.set_random_device_seed(
        settings["simulation"]["traffic"]["tm_seed"])
    traffic_manager.set_hybrid_physics_mode(
        settings["simulation"]["traffic"]["hybrid_physics_mode"])
    traffic_manager.set_hybrid_physics_radius(
        settings["simulation"]["traffic"]["hybrid_physics_radius"])

    # Initialize ZMQ
    context = zmq.Context()

    # Initialize internal ZMQ connection with master and slaves
    traffic_frontend = context.socket(zmq.SUB)
    traffic_frontend.bind(settings["simulation"]["traffic"]["frontend_tcp"])
    traffic_frontend.subscribe("")

    traffic_backend = context.socket(zmq.PUB)
    traffic_backend.bind(settings["simulation"]["traffic"]["backend_tcp"])

    # Inizialize ZMQ connection with Sumo client
    traffic_sub = context.socket(zmq.SUB)
    traffic_sub.setsockopt(zmq.CONFLATE, 1)
    traffic_sub.bind(SUMO_TRAFFIC_ADDR)
    traffic_sub.subscribe("")

    sumo_sync = context.socket(zmq.REQ)
    sumo_sync.setsockopt(zmq.RCVTIMEO, 1000)
    sumo_sync.bind(SUMO_SYNC_ADDR)

     # if number_of_vehicles not setted by arguments retrive from settings
    if (args.number_of_vehicles == -1):
        number_of_vehicles = settings["simulation"]["traffic"]["number_of_vehicles"]
    else:
        number_of_vehicles = args.number_of_vehicles

    [spawn_points, blueprints_list] = create_traffic_actors_element(world, number_of_vehicles, settings)

    print("Initializing " + str(args.slaves_num + 1) + " simulation")

    # sleep needed for proper zmq connection bt. all the actors
    sleep(1)

    print("Spawned, start to control. CTRL-C to stop")

    # Wait for user input
    while True:

        tick_sumo(sumo_sync)

        try:
            if traffic_sub.poll(15, zmq.POLLIN):
                vehicles = traffic_sub.recv_pyobj()
                #tick_sumo(sumo_sync)
                traffic_backend.send_pyobj(create_message(0, vehicles))
            world.wait_for_tick()
        except KeyboardInterrupt:
            break

    # Send closing message
    traffic_backend.send_pyobj(create_message(-1, settings))

# 1# MAIN

if __name__ == "__main__":
    main()
