import os
import sys
sys.path.append(os.path.join(os.environ['SUMO_HOME'], 'tools'))
import traci

PORT = 9999

traci.start(["sumo-gui", "-c", "Town01.sumo.cfg", "--num-clients", "3", "--collision.action", "warn", "--verbose"], port=PORT)
traci.setOrder(0)
while traci.simulation.getMinExpectedNumber() > 0:
    traci.simulationStep()
traci.close()