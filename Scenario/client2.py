import os
import sys
sys.path.append(os.path.join(os.environ['SUMO_HOME'], 'tools'))
import traci
import zmq
import time

PORT = 9999

# ZMQ
context = zmq.Context()

# SUMO → CARLA (traffico)
traffic_pub = context.socket(zmq.PUB)
traffic_pub.connect("tcp://10.196.36.90:5555")

# SUMO ← CARLA (sync)
sync_rep = context.socket(zmq.REP)
sync_rep.connect("tcp://10.196.36.90:5570")

# SUMO ← CARLA (egoVehicle)
traffic_egoVehicle = context.socket(zmq.SUB)
traffic_egoVehicle.setsockopt(zmq.CONFLATE, 1)
traffic_egoVehicle.connect("tcp://10.196.36.90:5580")
traffic_egoVehicle.subscribe("")

def tick_carla(sync_rep):
    sync_rep.recv()
    sync_rep.send(b"ok")

def run_traci_client():
    try:
        traci.init(PORT)
        traci.setOrder(2)
        
        while traci.simulation.getMinExpectedNumber() > 0:

            tick_carla(sync_rep)

            vehicles = []
            
            for veh_id in traci.vehicle.getIDList():
                if veh_id == "hero":
                    continue

                x, y, z = traci.vehicle.getPosition3D(veh_id)
                yaw = traci.vehicle.getAngle(veh_id)
                vehicles.append({"id": veh_id
                                 ,'x':x
                                 ,'y':(328.26 - y)
                                 ,'z':z,'pitch':0.0,'yaw':(yaw - 90),'roll':0.0})
            
            traffic_pub.send_pyobj(vehicles)
            
            traci.simulationStep()

            egoVehicle = None
            egoVehicle = traffic_egoVehicle.recv_pyobj()
            if egoVehicle is not None:
                traci.vehicle.moveToXY(vehID="hero",
                                       edgeID="",
                                       laneIndex=0,
                                       x=egoVehicle["x"],
                                       y=(328.26 - egoVehicle["y"]),
                                       angle=(egoVehicle["yaw"] + 90),
                                       keepRoute=2, matchThreshold=1000)
            
    except traci.exceptions.TraCIException as e:
        print(f"\nERRORE")
        print(f"Dettagli: {e}")
        sys.exit(1)

    finally:
        traci.close()

if __name__ == "__main__":
    run_traci_client()
