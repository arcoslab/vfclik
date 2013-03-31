#!/usr/bin/python
# Copyright (c) 2009 Technische Universitaet Muenchen, Informatik Lehrstuhl IX.
# Authors: Federico Ruiz-Ugalde <ruizf at in.tum.de>  Alexis Maldonado <maldonad at in.tum.de>
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

import yarp
from numpy import array
import sys
import signal
def signal_handler(sig, frame):
    print "Terminating ", __file__
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

change_ps_name('jointPController.py')

from arcospyu.config_parser import ConfigFileParser
config_parser=ConfigFileParser(sys.argv)
options, args, config = config_parser.get_all()

kp=1.5 #0.8
delta=0.087 #distance to have reached

yarp.Network.init()
basename="/"+config.arm_type+"/"+config.arm_instance+"/jpctrl"
inPort=yarp.BufferedPortBottle()
inPort.open(basename+"/in")
refPort=yarp.BufferedPortBottle()
refPort.open(basename+"/ref")
outPort=yarp.BufferedPortBottle()
outPort.open(basename+"/out")

atGoalPort=yarp.BufferedPortBottle()
atGoalPort.open(basename+"/at_goal")


#yarp.Network.connect("/bridge/encoders",basename+"/in")
#yarp.Network.connect(basename+"/out","/bridge/jointcmd")

def check_limits(ref,cur_pos):
    limits=config.updateJntLimits(cur_pos)
    ref_out=list(ref)
    for i in range(len(limits)):
        if (ref[i] < limits[i][0]):
            print "Limiting low", i
            ref_out[i] = limits[i][0]
        elif (ref[i] > limits[i][1]):
            print "Limiting high", i
            ref_out[i] = limits[i][1]
    return(ref_out)

ref=config.initial_joint_pos
waitRef=False
refbottle=yarp.Bottle()
while not stop:
    while waitRef:
        #blocking first time
        refbottle=refPort.read()
        if config.nJoints==refbottle.size():
            dataOk=True
            for value in map(refbottle.get,xrange(refbottle.size())):
                if ((not value.isDouble()) and (not value.isInt())):
                    dataOk=False
            if dataOk:
                waitRef=False
                break
    else:
        refbottle=refPort.read(False)
    if refbottle:
        ref=array(map(yarp.Value.asDouble,map(refbottle.get,xrange(config.nJoints))))
        print "New reference:", ref
    inbottle = inPort.read(False)
    if inbottle:
        indata = array(map(yarp.Value.asDouble,map(inbottle.get,xrange(config.nJoints))))
        ref=check_limits(ref,indata) # for the icub we have to check in every cycle the new limits (they change according to the current position, and limit the ref accordingly
        error = ref - indata
        #print "Error:", error
        outqdot = error * kp

        outbottle=outPort.prepare()
        outbottle.clear()
        map(outbottle.addDouble,outqdot)
        outPort.write(True)

        #Calculating if the target was reached
        joint_reached=[x<delta for x in error]
        all_reached = True
        for i in joint_reached:
            all_reached = all_reached and i

        reach_bottle = atGoalPort.prepare()
        reach_bottle.clear()
        if (all_reached):
            reach_bottle.addInt(1)
        else:
            reach_bottle.addInt(0)
        atGoalPort.write(True)
    else:
        yarp.Time.delay(0.01)

inPort.close()
refPort.close()
outPort.close()
atGoalPort.close()
yarp.Network.fini()
