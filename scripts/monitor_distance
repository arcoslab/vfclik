#!/usr/bin/python
# Copyright (c) 2009-2011 Technische Universitaet Muenchen,
# Intelligent Autonomous Systems (Prof. Beetz)
# Authors: Federico Ruiz-Ugalde <memeruiz at gmail.com>
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <http://www.gnu.org/licenses/>.

import sys
import yarp
from numpy import array
from vfl.vfl import length
from PyKDL import Frame
from PyKDL import diff
from math import sqrt
from math import pi
from arcospyu.config_parser import ConfigFileParser

import signal


def signal_handler(sig, frame):
    print("Terminating ", __file__)
    global stop
    stop = True


stop = False
signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)


def change_ps_name(name):
    """Change process name as used by ps(1) and killall(1)."""
    try:
        import ctypes
        libc = ctypes.CDLL('libc.so.6')
        libc.prctl(15, name, 0, 0, 0)
    except Exception:
        pass


change_ps_name('monitor_distance.py')

config_parser = ConfigFileParser(sys.argv)
options, args, config = config_parser.get_all()

yarp.Network.init()

robotbn = config.robotarm_portbasename
base_name = robotbn + "/dmonitor"
currentPosIn = yarp.BufferedPortBottle()
currentpospn = base_name + "/currentPosIn"
currentPosIn.open(currentpospn)
objectsInPort = yarp.BufferedPortBottle()
objectsinpn = base_name + "/objectsIn"
objectsInPort.open(objectsinpn)
objectsInPort.setStrict()
distOutPort = yarp.BufferedPortBottle()
distoutpn = base_name + "/distOut"
distOutPort.open(distoutpn)
track_error_in_port = yarp.BufferedPortBottle()
track_error_inpn = base_name + "/track_error_in"
track_error_in_port.open(track_error_inpn)
tracking_state_port = yarp.BufferedPortBottle()
tracking_statepn = base_name + "/tracking_state"
tracking_state_port.open(tracking_statepn)

objects = {}


def orientLength(final, current):
    finalKDLFrame = Frame()
    currentKDLFrame = Frame()
    for i in range(3):
        for j in range(3):
            finalKDLFrame.M[i, j] = final[i, j]
            currentKDLFrame.M[i, j] = current[i, j]
    tw = diff(currentKDLFrame, finalKDLFrame)
    return 180.0 * sqrt(tw.rot[0]**2 + tw.rot[1]**2 + tw.rot[2]**2) / (pi)


track_error_xyz = 0.0
track_error_rot = 0.0
# meters.  Acceptable distance to goal where we stop checking for
# command following
distanceXYZ_th = 0.02
# radians. Error threshold
track_error_xyz_th = 0.1
# angles.  Acceptable distance to goal where we stop checking for command
# following
distanceOrient_th = 1.0
# radians. Error Threshold
track_error_rot_th = 0.1

tracking_buffer = []
tracking_buffer_size = 20
last_tracking_xyz_state = "on goal"
last_tracking_rot_state = "on goal"
tracking_xyz_state = "on goal"
tracking_rot_state = "on goal"

while not stop:
    # if no goal position have been set, don't do anything
    yarp.Time.delay(0.010)  # slow down the reading loop
    objectsInbottle = objectsInPort.read(False)
    if (objectsInbottle and (objectsInbottle.size() >= 2)):
        if objectsInbottle.get(0).toString() == "add":
            print("Add command")
            # add the object to the dictionary
            if objectsInbottle.size() == 3:
                objectsInbottleList = objectsInbottle.get(2).asList()
                if objectsInbottleList.size() == 16:
                    # We got the right amount of data now do something
                    print("Adding something")
                    objects[objectsInbottle.get(1).asInt()] = map(
                        yarp.Value.asDouble,
                        map(objectsInbottleList.get, range(16)))
            pass
        if objectsInbottle.get(0).toString() == "remove":
            print("Removing object")
            # remove object from the dictionary
            if objectsInbottle.get(1).asInt() in objects:
                del objects[objectsInbottle.get(1).asInt()]

    # calculate distance
    track_error_in_bottle = track_error_in_port.read(False)
    if track_error_in_bottle:
        track_error_xyz = track_error_in_bottle.get(0).asDouble()
        track_error_rot = track_error_in_bottle.get(1).asDouble()
    currentPosInbottle = currentPosIn.read(False)
    currentPosReceived = 1
    if (currentPosInbottle and (currentPosInbottle.size() == 16)):
        for i in range(currentPosInbottle.size()):
            if (not (currentPosInbottle.get(i).isDouble()
                     or currentPosInbottle.get(i).isInt())):
                currentPosReceived = 0
                print("We got something wrong as current position")
    else:
        currentPosReceived = 0
        yarp.Time_delay(0.005)  # Do not eat CPU while waiting for bottles

    if (currentPosReceived):
        currentPose = map(yarp.Value.asDouble,
                          map(currentPosInbottle.get, range(16)))
        currentXYZ = array([currentPose[3], currentPose[7], currentPose[11]])
        poseHomo = array([
            currentPose[0:4], currentPose[4:8], currentPose[8:12],
            currentPose[12:16]
        ])
        objectDistances = []
        if len(objects) > 0:
            distOutbottle = distOutPort.prepare()
            distOutbottle.clear()
            for i in objects:
                distanceXYZ = length(currentXYZ - array(
                    [objects[i][3], objects[i][7], objects[i][11]]))
                distanceOrient = orientLength(
                    array([
                        objects[i][0:4], objects[i][4:8], objects[i][8:12],
                        objects[i][12:16]
                    ]), poseHomo)
                objectDistances.append([i, distanceXYZ, distanceOrient])
                distOutbottleList = distOutbottle.addList()
                distOutbottleList.addDouble(i)
                distOutbottleList.addDouble(distanceXYZ)
                distOutbottleList.addDouble(distanceOrient)
                if i == 0:
                    # for the goal now we calculate if we are tracking well
                    rot_state = "on goal"
                    xyz_state = "on goal"
                    if (distanceXYZ > distanceXYZ_th) and (track_error_xyz >
                                                           track_error_xyz_th):
                        xyz_state = "not follow"
                    if (distanceXYZ > distanceXYZ_th) and (track_error_xyz <
                                                           track_error_xyz_th):
                        xyz_state = "follow"
                    if (distanceXYZ < distanceXYZ_th):
                        xyz_state = "on goal"
                    if (distanceOrient > distanceOrient_th) and (
                            track_error_rot > track_error_rot_th):
                        rot_state = "not follow"
                    if (distanceOrient > distanceOrient_th) and (
                            track_error_rot < track_error_rot_th):
                        rot_state = "follow"
                    if (distanceOrient < distanceOrient_th):
                        rot_state = "on goal"
                    tracking_buffer.append([xyz_state, rot_state])
                    if len(tracking_buffer) > tracking_buffer_size:
                        tracking_buffer.pop(0)
                        tracking_xyz_state = max(
                            ["on goal", "follow", "not follow"],
                            key=list(zip(*tracking_buffer)[0]).count)
                        if last_tracking_xyz_state != tracking_xyz_state:
                            last_tracking_xyz_state = tracking_xyz_state
                            tracking_state_bottle = (
                                tracking_state_port.prepare()
                            )
                            tracking_state_bottle.clear()
                            tracking_state_bottle.addString("xyz")
                            tracking_state_bottle.addString(tracking_xyz_state)
                            tracking_state_port.writeStrict()
                        tracking_rot_state = max(
                            ["on goal", "follow", "not follow"],
                            key=list(zip(*tracking_buffer)[1]).count)
                        if last_tracking_rot_state != tracking_rot_state:
                            last_tracking_rot_state = tracking_rot_state
                            tracking_state_bottle = (
                                tracking_state_port.prepare()
                            )
                            tracking_state_bottle.clear()
                            tracking_state_bottle.addString("rot")
                            tracking_state_bottle.addString(tracking_xyz_state)
                            tracking_state_port.writeStrict()

            distOutPort.write()

track_error_in_port.close()
tracking_state_port.close()
currentPosIn.close()
objectsInPort.close()
distOutPort.close()
yarp.Network.fini()
