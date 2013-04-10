#!/usr/bin/env python
# Copyright (c) 2009 Technische Universitaet Muenchen, Informatik Lehrstuhl IX.
# Author: Federico Ruiz Ugalde <memeruiz at gmail.com>
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
from arcospyu.config_parser import ConfigFileParser
from arcospyu.yarp_tools.yarp_comm_helpers import recur, yarp_queryname_blocking, yarp_connect_blocking
from arcospyu.dprint import iprint, dprint, eprint, dcprint, wprint

import signal
def signal_handler(sig, frame):
    dprint("Terminating ")
    global stop
    stop=True

stop=False
signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)


def change_ps_name(name):
    """Change process name as used by ps(1) and killall(1)."""
    try:
        import ctypes
        libc = ctypes.CDLL('libc.so.6')
        libc.prctl(15, name, 0, 0, 0)
    except:
        pass

config_parser=ConfigFileParser(sys.argv)
options, args, config = config_parser.get_all()

yarp.Network.init()

robotbn=config.robotarm_portbasename
base_name = robotbn + "/ofeeder"
paramPort = yarp.BufferedPortBottle()
parampn=base_name + "/param"
paramPort.setStrict(True)
paramPort.open(parampn)
objectPort = yarp.BufferedPortBottle()
objectpn=base_name + "/object"
objectPort.open(objectpn)
objectPort.setStrict(True)
object_f_port = yarp.BufferedPortBottle()  # port forwarding /object
object_fpn=base_name + "/objectf"
object_f_port.open(object_fpn)
objectOutPort = yarp.BufferedPortBottle()
objectoutpn=base_name + "/objectOut"
objectOutPort.open(objectoutpn)

yconnect=yarp.Network.connect
cstyle=yarp.ContactStyle()
cstyle.persistent=True
yarp_connect_blocking(parampn, robotbn+"/vectorField/param", timeout=100)
yconnect(parampn, robotbn+"/vectorField/param", cstyle)
yconnect(objectoutpn, robotbn+"/dmonitor/objectsIn", cstyle)
while not yarp_queryname_blocking(robotbn+"/vectorField/param", timeout=10):
    wprint("Not connected")
    pass

objects = {}
#object 0 is always the goal, the rest are obstacles

init_pose=False
while not stop:
    objectbottle = objectPort.read(False)
    #pending=objectPort.getPendingReads()
    #for i in xrange(pending):
    #    objectbottle = objectPort.read(False)
    if not objectbottle and (not init_pose):
        yarp.Time_delay(0.005)
        continue
    if init_pose:
        dprint("Setting first goal")
        objectbottle=yarp.Bottle()
        recur(objectbottle, config.initial_vf_pose)
        init_pose=False

    objectfbottle = object_f_port.prepare()
    objectfbottle.clear()
    for i in range(objectbottle.size()):
        objectfbottle.add(objectbottle.get(i))
    object_f_port.writeStrict()
    if (objectbottle and (objectbottle.size() >= 2)):
        dprint("PARAMETER: ", objectbottle.toString())
        # We receive new parameter values!
        objectAction = objectbottle.get(0)
        if objectAction.toString() == "set":
                # object data:
                # 0: set
                # 1: "goal", "goalAndNormal", "ObstacleP", "ObstacleH"
                # 2: Parameter list or object number for an obstacle
                # 3: parameter list for an obstacle
            if objectbottle.size() == 3:
                if objectbottle.get(1).toString() == "goal":
                    # set normal attractor
                    # 0-15: goal position
                    parambottle = objectbottle.get(2).asList()
                    params = [parambottle.get(i).asDouble()
                              for i in range(parambottle.size())]
                    if len(params) == 16:
                        params.append(0.03)
                    if len(params) == 17:
                        objects[0] = params
                    else:
                        dprint("Wrong number of values, expected 16")
                if objectbottle.get(1).toString() == "goalAndNormal":
                    # set normal attractor, funnel attractor and obstacle
                    # attractor
                    parambottle = objectbottle.get(2).asList()
                    # parameter list
                    # 0-15: goal position
                    # 16-18: direction Axis
                    # 19-20: angle, length
                    # 21: slow down distance
                    params = [parambottle.get(i).asDouble()
                              for i in range(parambottle.size())]
                    if len(params) == 21:
                        params.append(0.03)
                    if len(params) == 22:
                        objects[0] = params
                    else:
                        dprint("Wrong number of values, expected 21")
            elif objectbottle.size() == 4:
                if objectbottle.get(1).toString() == "ObstacleP":
                    #set decay repeller
                    parambottle = objectbottle.get(3).asList()
                    #parameter list
                    #0-15: repeller position, rotation ignored for now
                    #16, 17: radius, order of decay
                    if parambottle.size() == 18:
                        params = [parambottle.get(i).asDouble()
                                  for i in range(parambottle.size())]
                        objects[objectbottle.get(2).asInt() + 1] = \
                            ['ObstacleP'] + params
                    else:
                        dprint("Wrong number of values, expected 18")
                if objectbottle.get(1).toString() == "ObstacleH":
                    #set hemisphere repeller
                    parambottle = objectbottle.get(3).asList()
                    #parameter list
                    #0-15: repeller position, rotation ignored for now
                    #16-18: normal Axis, point in the repelling direction
                    #19, 20: safe distance, order of decay
                    if parambottle.size() == 21:
                        params = [parambottle.get(i).asDouble()
                                  for i in range(parambottle.size())]
                        objects[objectbottle.get(2).asInt() + 1] = \
                            ['ObstacleH'] + params
                    else:
                        dprint("Wrong number of values, expected 21")
            else:
                dprint("Wrong number of values, expected 3 or 4")
        elif objectAction.toString() == "remove":
            #object data:
            #0: "remove"
            #1: object number
            objectNum = objectbottle.get(1).asInt()
            #remove object with number
            if objectNum + 1 in objects:
                del objects[objectNum + 1]
                parambottle = paramPort.prepare()
                parambottle.clear()
                parambottle.addString("remove")
                parambottle.addInt(5 + objectNum)
                paramPort.writeStrict()
                objectOutbottle = objectOutPort.prepare()
                objectOutbottle.clear()
                objectOutbottle.addString("remove")
                objectOutbottle.addInt(objectNum + 1)
                objectOutPort.writeStrict()

            else:
                dprint("Object doesn't exist, doing nothing")
                continue
        else:
            dprint("Action not recognized")
            
        if 0 in objects:
            dprint("There is a goal, adding objects to the vector field")
            for objectNum in objects:
                dprint("Sending object: ", objectNum)
                paramlist = objects[objectNum]
                if objectNum == 0:
                    objectOutbottle = objectOutPort.prepare()
                    objectOutbottle.clear()
                    objectOutbottle.addString("add")
                    objectOutbottle.addInt(objectNum)
                    objectOutList = objectOutbottle.addList()
                    for i in range(16):
                        objectOutList.addDouble(paramlist[i])
                    objectOutPort.writeStrict()
                    # send goal
                    if len(paramlist) == 17:  # normal goal
                        dprint("sending normal goal")
                        parambottle = paramPort.prepare()
                        parambottle.clear()
                        parambottle.addString("add")
                        parambottle.addInt(1)  # Object number
                        parambottle.addDouble(1)  # Direction force
                        parambottle.addInt(1)  # vectorField type
                        parambottleList = parambottle.addList()
                        for i in range(17):
                            # attractor position + slowdown distance
                            parambottleList.addDouble(paramlist[i])
                        paramPort.writeStrict()
                        for i in range(2):
                            parambottle = paramPort.prepare()
                            parambottle.clear()
                            parambottle.addString("remove")
                            parambottle.addInt(i + 2)
                            paramPort.writeStrict()
                    elif len(paramlist) == 22:  # goal with approach vector
                        dprint("sending goal with approach")
                        # adding normal attractor
                        parambottle = paramPort.prepare()
                        parambottle.clear()
                        parambottle.addString("add")
                        parambottle.addInt(1)
                        parambottle.addDouble(1)
                        parambottle.addInt(1)
                        parambottleList = parambottle.addList()
                        for i in range(16):
                            parambottleList.addDouble(paramlist[i])
                        parambottleList.addDouble(paramlist[21])
                        paramPort.writeStrict()
                        # adding funnel attractor
                        parambottle = paramPort.prepare()
                        parambottle.clear()
                        parambottle.addString("add")
                        parambottle.addInt(2)
                        parambottle.addDouble(30)
                        parambottle.addInt(5)
                        parambottleList = parambottle.addList()
                        for i in range(3):
                            # position
                            parambottleList.addDouble(paramlist[i * 4 + 3])
                        for i in range(3):
                            # axis
                            parambottleList.addDouble(paramlist[16 + i])
                        parambottleList.addDouble(paramlist[19])  # cut angle
                        parambottleList.addDouble(10)  # angle decay order
                        parambottleList.addDouble(paramlist[20])  # cut dist
                        parambottleList.addDouble(2)  # distance decay order
                        paramPort.writeStrict()
                        # adding near goal repeller
                        goalToVObsD = 0.05  # 5cm
                        parambottle = paramPort.prepare()
                        parambottle.clear()
                        parambottle.addString("add")
                        parambottle.addInt(3)
                        parambottle.addDouble(-10)  # negative: repeller
                        parambottle.addInt(2)  # decay repeller
                        parambottleList = parambottle.addList()
                        goalposition = array([paramlist[3],
                                              paramlist[7],
                                              paramlist[11]])
                        axis = array(paramlist[16:19])
                        axis = axis / length(axis)
                        vobstaclepos = goalposition + axis * goalToVObsD
                        for i in range(3):
                            # position (a bit far away from the goal in the
                            # approach direction)
                            parambottleList.addDouble(vobstaclepos[i])
                        # obstacle radius
                        parambottleList.addDouble(paramlist[20] + goalToVObsD)
                        parambottleList.addDouble(0.001)  # safe distance
                        parambottleList.addDouble(5)  # decay order
                        paramPort.writeStrict()
                    else:
                        # should never occur
                        dprint("wrong number of parameters")
                else:
                    objectOutbottle = objectOutPort.prepare()
                    objectOutbottle.clear()
                    objectOutbottle.addString("add")
                    objectOutbottle.addInt(objectNum)
                    objectOutList = objectOutbottle.addList()
                    for i in range(16):
                        objectOutList.addDouble(paramlist[i + 1])
                    objectOutPort.writeStrict()
                    # send repellers
                    if paramlist[0] == "ObstacleP":  # Point decay repeller
                        dprint("Sending point obstacle")
                        parambottle = paramPort.prepare()
                        parambottle.clear()
                        parambottle.addString("add")
                        parambottle.addInt(4 + objectNum)  # obstacle number
                        parambottle.addDouble(-10)  # negative repeller
                        parambottle.addInt(2)  # decay repeller
                        parambottleList = parambottle.addList()
                        for i in range(3):
                            parambottleList.addDouble(paramlist[i * 4 + 3 + 1])  # position
                        parambottleList.addDouble(paramlist[16 + 1])  # obstacle radius
                        parambottleList.addDouble(0.001)  # safe distance
                        parambottleList.addDouble(paramlist[17 + 1])  # decay order
                        paramPort.writeStrict()
                    if paramlist[0] == "ObstacleH":  # hemisphere repeller
                        dprint("Sending table repeller")
                        parambottle = paramPort.prepare()
                        parambottle.clear()
                        parambottle.addString("add")
                        parambottle.addInt(4 + objectNum)  # obstacle number
                        parambottle.addDouble(-50)  # negative repeller
                        parambottle.addInt(4)  # hemisphere repeller
                        parambottleList = parambottle.addList()
                        for i in range(3):
                            parambottleList.addDouble(paramlist[i * 4 + 3 + 1])  # position
                        for i in range(3):
                            parambottleList.addDouble(paramlist[i + 16 + 1])  # normal
                        parambottleList.addDouble(paramlist[19 + 1])  # safe distance
                        parambottleList.addDouble(paramlist[20 + 1])  # decay order
                        paramPort.writeStrict()
        else:
            dprint("Not setting repellers, waiting for a goal. Please set a goal, to apply all the repellers")


paramPort.close()
objectPort.close()
object_f_port.close()
objectOutPort.close()
yarp.Network.fini()
