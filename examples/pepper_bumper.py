#!/usr/bin/env python
# coding: utf-8
"""
Principe du programme : Test de l'ajout des bumpers simulé sur le robot pepper
Ce programme va afficher les rayons des bumpers et ensuite va traiter les valeurs de distances reçues par ceux-ci, indiquant alors à l'utilisateur lorsque le robot rentre en contact avec un objet posé au sol
"""

import math
import os
import time
import pybullet
import pybullet_data
from qibullet import SimulationManager


def main():
    """
    Fonction de test rapide sur les bumpers du robot pepper
    """
    simulation_manager = SimulationManager() # launch simulation
    client = simulation_manager.launchSimulation(gui=True)
    pepper = simulation_manager.spawnPepper(client, spawn_ground_plane=True) # spawn pepper robot

    pybullet.setAdditionalSearchPath("/home/tp/g5_raymond_bouyssoux/urdf")
        
    #Ajout d'un obstacle
    caisse_rouge=pybullet.loadURDF(
          "caisse_rouge.urdf",
          basePosition=[1, 0.6, 0.02])

    pepper.showBumper(True)
    pepper.subscribeBumper()
    time.sleep(4)

    while True:
        """
        Boucle while infinie permettant de faire avancer le robot en ligne droite et de regarder à tout instant s'il y a contact avec l'un des bumpers
        """
        warn_av_gauche = 0
        warn_av_droit = 0
        warn_ar = 0
        list_Right_Front = pepper.getRightFrontBumperValue()
        list_Left_Front = pepper.getLeftFrontBumperValue()
        list_Back = pepper.getBackBumperValue()

        for dist in list_Right_Front:
            if (dist < 0.05):
                warn_av_droit = 1

        for dist in list_Left_Front:
            if (dist < 0.05):
                warn_av_gauche = 1

        for dist in list_Back:
            if (dist < 0.05):
                warn_ar = 1

        if (warn_av_gauche == 1):
            print("Collision devant à gauche")
        if (warn_av_droit == 1):
            print("Collision devant à droite")
        if (warn_ar == 1):
            print("Collision derrière")
        if (warn_av_gauche == 0 & warn_av_droit == 0 & warn_ar == 0):
            print("Pas d'obstacles proches des bumpers")

        pepper.moveTo(0.1, 0, 0)
        time.sleep(2)


if __name__ == "__main__":
    main()
