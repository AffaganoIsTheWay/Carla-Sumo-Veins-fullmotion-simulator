#! /usr/bin/bash

source ../../../omnetpp-6.2.0/setenv

opp_run -m -u Cmdenv -n .:../../src/veins --image-path=../../images -l /home/giovanni/CarSimulation/veins-5.3.1/src/veins omnetpp.ini
