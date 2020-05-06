#!/usr/bin/env python
# coding: utf-8

import math
import os
import time
import pybullet
import pybullet_data
from qibullet import SimulationManager


def main():
    simulation_manager = SimulationManager()
    client = simulation_manager.launchSimulation(gui=True)
    pepper = simulation_manager.spawnPepper(client, spawn_ground_plane=True)

    pybullet.setAdditionalSearchPath("/home/tp/g5_raymond_bouyssoux/urdf")
    
    #Ajout du panier (pour la peluche)
    caisse_rouge=pybullet.loadURDF(
          "caisse_rouge.urdf",
          basePosition=[1, 1, 0.02])

    pepper.showSonar(True)
    pepper.subscribeSonar()
    time.sleep(2)
    warn_av_gauche = 0
    warn_av_droit = 0
    warn_ar_gauche = 0
    warn_ar_droit = 0

    while True:
        warn_av_gauche = 0
        warn_av_droit = 0
        warn_ar_gauche = 0
        warn_ar_droit = 0
        list_Front = pepper.getFrontSonarValue()
        list_Back = pepper.getBackSonarValue()

        for dist in list_Front:
            if (dist < 1.325):
                if (list_Front.index(dist) <= 4):
                    warn_av_gauche = 1
                else:
                    warn_av_droit = 1

        for dist in list_Back:
            if (dist < 1.325):
                if (list_Back.index(dist) <= 4):
                    warn_ar_droit = 1
                else:
                    warn_ar_gauche = 1

        if (warn_av_gauche == 1):
            print("Attention danger devant à gauche")
        if (warn_av_droit == 1):
            print("Attention danger devant à droite")
        if (warn_ar_droit == 1):
            print("Attention danger derrière à droite")
        if (warn_ar_gauche == 1):
            print("Attention danger derrière à gauche")
        pepper.move(0, 0, math.pi)
        time.sleep(5)


if __name__ == "__main__":
    main()
