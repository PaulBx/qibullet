#!/usr/bin/env python
# coding: utf-8

import time
import math
import atexit
import weakref
import pybullet
import threading

RAY_MISS_COLOR = [0, 1, 0]
RAY_HIT_COLOR = [1, 0, 0]
NUM_RAY = 15
RAY_LENGTH = 1.325  # The theoretical length is 0.15 to 2.5 (took the average for simulation)
SONAR_ANGLE = 60 # effective cone (60°)
SONAR_POSITION = [
    [0.06990, 0, -0.10230],  # Front sonar
    [-0.07180, 0, -0.05340],  # Back sonar
]
ANGLE_LIST_POSITION = [
    math.radians(SONAR_ANGLE/2),  # Front laser
    math.radians(SONAR_ANGLE/2) + math.pi,  # Back laser
]

NUM_SONAR = len(SONAR_POSITION)
SONAR_FRAMERATE = 42000 # frequency 42kHz


class Sonar:
    """
    Class representing a virtual laser
    """
    _instances = set()

    def __init__(
            self,
            robot_model,
            sonar_id,
            physicsClientId=0,
            display=False):
        """
        Constructor

        Parameters:
            robot_model - The pybullet model of the robot.
            laser_id - The id of the link (Link type)
            onto which the Lasers' are attached.
            physicsClientId - The id of the simulated instance in which the
            lasers are to be spawned
            display - boolean that allow the display of the laser
        """
        self.robot_model = robot_model
        self.physics_client = physicsClientId
        self.sonar_thread = threading.Thread()
        self.ray_from = []
        self.ray_to = []
        self.ray_ids = []
        self.sonar_value = [0] * NUM_RAY * NUM_SONAR
        self.sonar_id = sonar_id
        self.display = display
        self._instances.add(weakref.ref(self))
        self._scan_termination = False
        atexit.register(self._terminateScan)

    @classmethod
    def _getInstances(cls):
        """
        INTERNAL CLASSMETHOD, get all of the Sonar instances
        """
        dead = set()

        for ref in cls._instances:
            obj = ref()

            if obj is not None:
                yield obj
            else:
                dead.add(ref)

        cls._instances -= dead

    def isActive(self):
        """
        Check if the sonars are subscribed or not (if the sonar thread process
        is active or not)

        Returns:
            boolean - True if the sonars are subscribed, false otherwise
        """
        return self.laser_thread.isAlive()

    def subscribe(self):
        """
        Subscribe to the sonar scan (this will activate the sonar scan
        process).
        """
        # No need to subscribe to the sonar scan if the sonars are activated
        if self.isActive():
            return

        self._scan_termination = False
        self._initializeRays()
        self.sonar_thread = threading.Thread(target=self._sonarScan)
        self.sonar_thread.start()

    def unsubscribe(self):
        """
        Unsubscribe from the sonar scan (this will deactivate the sonar scan
        process)
        """
        if self.isActive():
            self._terminateScan()

    def showSonar(self, display):
        """
        Display debug lines that simulate the sonar
        """
        self.display = display

    def getFrontSonarValue(self):
        """
        Return a list of the front sonar value (clockwise)
        """
        return self.laser_value[:NUM_RAY]

    def getBackSonarValue(self):
        """
        Return a list of the back sonar value (clockwise)
        """
        return self.sonar_value[NUM_RAY:2*NUM_RAY]

    def _initializeRays(self):
        """
        INTERNAL METHOD, initialize the sonar and all variables needed
        """
        for index in range(NUM_SONAR):
            angle = ANGLE_LIST_POSITION[index]
            for i in range(NUM_RAY):
                self.ray_from.append([
                    SONAR_POSITION[index][0],
                    SONAR_POSITION[index][1],
                    SONAR_POSITION[index][2]])
                self.ray_to.append(
                    [SONAR_POSITION[index][0] + (RAY_LENGTH) *
                     math.cos(float(i) *
                     math.radians(-SONAR_ANGLE)/NUM_RAY + angle),
                     SONAR_POSITION[index][1] + (RAY_LENGTH) *
                     math.sin(float(i) *
                     math.radians(-SONAR_ANGLE)/NUM_RAY + angle),
                     SONAR_POSITION[index][2]])

    def _sonarScan(self):
        """
        INTERNAL METHOD, a loop that simulate the sonar and update the distance
        value of each sonar
        """
        lastLidarTime = time.time()

        while not self._scan_termination:
            nowLidarTime = time.time()

            if (nowLidarTime-lastLidarTime > 1/SONAR_FRAMERATE):
                results = pybullet.rayTestBatch(
                    self.ray_from,
                    self.ray_to,
                    parentObjectUniqueId=self.robot_model,
                    parentLinkIndex=self.sonar_id,
                    physicsClientId=self.physics_client)

                for i in range(NUM_RAY*len(SONAR_LIST_POSITION)):
                    hitObjectUid = results[i][0]
                    hitFraction = results[i][2]
                    hitPosition = results[i][3]
                    self.sonar_value[i] = hitFraction * RAY_LENGTH

                    if self.display:
                        if not self.ray_ids:
                            self._createDebugLine()

                        if (hitFraction == 1.):
                            pybullet.addUserDebugLine(
                                self.ray_from[i],
                                self.ray_to[i],
                                RAY_MISS_COLOR,
                                replaceItemUniqueId=self.ray_ids[i],
                                parentObjectUniqueId=self.robot_model,
                                parentLinkIndex=self.sonar_id,
                                physicsClientId=self.physics_client)
                        else:
                            localHitTo = [self.ray_from[i][0]+hitFraction*(
                                        self.ray_to[i][0]-self.ray_from[i][0]),
                                         self.ray_from[i][1]+hitFraction*(
                                        self.ray_to[i][1]-self.ray_from[i][1]),
                                         self.ray_from[i][2]+hitFraction*(
                                        self.ray_to[i][2]-self.ray_from[i][2])]
                            pybullet.addUserDebugLine(
                                self.ray_from[i],
                                localHitTo,
                                RAY_HIT_COLOR,
                                replaceItemUniqueId=self.ray_ids[i],
                                parentObjectUniqueId=self.robot_model,
                                parentLinkIndex=self.sonar_id,
                                physicsClientId=self.physics_client)

                    else:
                        if self.ray_ids:
                            self._resetDebugLine()

                lastLidarTime = nowLidarTime

    def _createDebugLine(self):
        """
        INTERNAL METHOD, create all debug lines needed for simulating the
        lasers
        """
        for i in range(NUM_RAY * NUM_LASER):
            self.ray_ids.append(pybullet.addUserDebugLine(
                self.ray_from[i],
                self.ray_to[i],
                RAY_MISS_COLOR,
                parentObjectUniqueId=self.robot_model,
                parentLinkIndex=self.sonar_id,
                physicsClientId=self.physics_client))

    def _resetDebugLine(self):
        """
        INTERNAL METHOD, remove all debug lines
        """
        for i in range(len(self.ray_ids)):
            pybullet.removeUserDebugItem(self.ray_ids[i], self.physics_client)

        self.ray_ids = []

    def _terminateScan(self):
        """
        INTERNAL METHOD, called when unsubscribing from the active laser, when
        Python is exitted or when the SimulationManager resets/stops a
        simulation instance
        """
        self._scan_termination = True

        if self.sonar_thread.isAlive():
            self.sonar_thread.join()
