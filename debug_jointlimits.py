#!/usr/bin/env python
# Copyright (c) 2009 Technische Universitaet Muenchen, Informatik Lehrstuhl IX.
# Author: Alexis Maldonado Herrera <maldonad at cs.tum.edu>
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
sys.path.append("../lafik")
from lafik import Lafik
from math import pi
import yarp
import sys
import signal
def signal_handler(sig, frame):
    print "Terminating ", __file__
    global stop
    stop=True

stop=False
signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)

from arcospyu.config_parser import ConfigFileParser
config_parser=ConfigFileParser(sys.argv)
options, args, config = config_parser.get_all()

yarp.Network.init()
baseportname = config.robotarm_portbasename + "/debug"
qinPort = yarp.BufferedPortBottle()
qinPort.open(baseportname + "/qin")
qdistOutPort = yarp.BufferedPortBottle()
qdistOutPort.open(baseportname + "/qdist")


robot=Lafik(config)

while not stop:
    qinbottle = qinPort.read(False)
    if qinbottle and qinbottle.size() == robot.numJnts:
        robot.jntsList=map(yarp.Value.asDouble,map(qinbottle.get,range(qinbottle.size())))
        distances=[]
        for i in zip(robot.jntsList,robot.joint_limits):
            distances.append(robot.distToCenter(i[1],i[0]))
        distances2=[x*100 for x in distances]
        qdistOutbottle=qdistOutPort.prepare()
        qdistOutbottle.clear()
        for i in distances2:
            qdistOutbottle.addDouble(i)
        qdistOutPort.write()
    yarp.Time.delay(0.1)

qdistOutPort.close()
qinPort.close()
yarp.Network.fini()




