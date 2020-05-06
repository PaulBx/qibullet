#!/usr/bin/env python
# coding: utf-8


import os
import time
import pybullet
import pybullet_data
from qibullet import SimulationManager


def main():
    simulation_manager = SimulationManager()
    client = simulation_manager.launchSimulation(gui=True)
    pepper = simulation_manager.spawnPepper(client, spawn_ground_plane=True)

    pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())

    print("ON LANCE LES MÉTHODES SHOWLASER ET SUSBSCRIBELASER")
    pepper.showSonar(True)
    pepper.subscribeSonar()
    print("ON A FINIT")
    time.sleep(100)


if __name__ == "__main__":
    main()
