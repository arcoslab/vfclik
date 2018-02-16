#!/usr/bin/env python
# Copyright (c) 2009-2011 Technische Universitaet Muenchen, Intelligent Autonomous Systems (Prof. Beetz)
# Authors: Federico Ruiz-Ugalde <memeruiz at gmail.com>, Alexis Maldonado <maldonado@tum.de>
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

'''Vector Field executor.
Gets as parameters objects which can be repellers and attractors depending
on the attraction sign given.
From param gets attraction points in position and orientation and repeller
points in orientation
'''

import sys

from math import sqrt
from numpy import dot
#import Gnuplot
from math import sin
from math import acos
import yarp
from vfl.vectorFieldClass import VectorField
from vfl.vectorFieldClass import ScalarField
from vfl.vectorFields import goalObstacleVField2
from vfl.vectorFields import trapezoidWeight
from numpy import array
from numpy import zeros
from arcospyu.robot_tools import Lafik
import vfl.vfl as vf
from arcospyu.yarp_tools.yarp_comm_helpers import sendListPort, readListPort, listToKdlFrame, kdlFrameToList
from arcospyu.config_parser import ConfigFileParser
from arcospyu.dprint import dprint, eprint, wprint

from math import sqrt
import PyKDL

import signal
stop=False
def signal_handler(sig, frame):
    dprint("Terminating.")
    global stop
    stop=True

signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)

config_parser=ConfigFileParser(sys.argv)
options, args, config = config_parser.get_all()

yarp.Network.init()

qdotOutPort = yarp.BufferedPortBottle()
qInPort = yarp.BufferedPortBottle()
toolPort = yarp.BufferedPortBottle()
paramPort = yarp.BufferedPortBottle() # goal and repellers
posePort = yarp.BufferedPortBottle()
pose_no_tool_Port = yarp.BufferedPortBottle()
weightPort = yarp.BufferedPortBottle() #Joint and Task space weights
tracking_error_port = yarp.BufferedPortBottle()
maxvel_port = yarp.BufferedPortBottle()

#ports for visualizing
pose_in_port = yarp.BufferedPortBottle()
vector_port = yarp.BufferedPortBottle()
goal_port = yarp.BufferedPortBottle()

robotbn=config.robotarm_portbasename
portbasename=robotbn+"/vectorField"
qdotoutpn=portbasename + "/qdotOut"
qinpn=portbasename + "/qIn"
toolpn=portbasename + "/tool"
parampn=portbasename + "/param"
posepn=portbasename + "/pose"
pose_no_toolpn=portbasename + "/pose_no_tool"
weightpn=portbasename + "/weight"
tracking_errorpn=portbasename + "/track_error"
pose_inpn=portbasename + "/pose_in"
vectorpn=portbasename + "/vector_out"
goalpn=portbasename + "/goal_out"
maxvelpn=portbasename + "/max_vel"

paramPort.setStrict(True)
toolPort.setStrict(True)
weightPort.setStrict(True)
maxvel_port.setStrict(True)

qdotOutPort.open(qdotoutpn)
qInPort.open(qinpn)
toolPort.open(toolpn)
paramPort.open(parampn)
posePort.open(posepn)
pose_no_tool_Port.open(pose_no_toolpn)
weightPort.open(weightpn)
tracking_error_port.open(tracking_errorpn)
pose_in_port.open(pose_inpn)
vector_port.open(vectorpn)
goal_port.open(goalpn)
maxvel_port.open(maxvelpn)



yconnect=yarp.Network.connect

cstyle=yarp.ContactStyle()
cstyle.persistent=True
yconnect(posepn, robotbn+"/dmonitor/currentPosIn", cstyle)
yconnect(tracking_errorpn, robotbn+"/dmonitor/track_error_in", cstyle)
yconnect(qdotoutpn, robotbn+"/bridge/vectorfieldcmd", cstyle)

config_max_vel = 0.41
#speedScale=0.12  #iCub
speedScale = 0.20  #Kimp - Rosie
min_rot = 0.09
stopDistance = 0.005
r_stopDistance = 0.03
slowDownDistance = 0.03
#table=[-0.04]
numJntsArm = config.nJoints

vectorFields = {}
vfDB=vf.vectorFieldLibrary()
vfparams = []
vftemp = vfDB[0]()
vftemp.setParams([])
totalVF = VectorField(vftemp.getVector)
totalSF = ScalarField(vftemp.getScalar)

lafik = Lafik(config)
oldtoolFrame = [1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1]

frame_list = []
cmd_buffer = []
cmd_buffer_size = 4
frame_list_size = 5
check_delay = 4
first_cycle = True

def get_weight_matrix(wbottle,n_vars):
    '''get_weight_matrix(wbottle,n_vars)
    Returns a numpy matrix of size (n_vars,n_vars) with the weights in the diagonal
    wbottle is a yarp bottle with the format string floats*nvars
    the string is 't' for task space (cartesian), or 'j' for joint space
    '''
    if (wbottle.size() == n_vars+1):
        weights = zeros((n_vars, n_vars))
        for i in range(n_vars):
            weights[i][i] = wbottle.get(i + 1).asDouble()
        return(weights)
    else:
        dprint("WARNING: Wrong size of %s weights. Ignored" %wbottle.get(0).asString())
        return(None)


start_attractor = False
first_arm_data = True

reporting_port_counter = 0
counter_test=0

while not stop:

    # Adjust new max_vel value
    botin = maxvel_port.read(False)
    if (botin):
        new_max_vel = botin.get(0).asDouble()
        if (new_max_vel >= 0.0) and (new_max_vel <= config_max_vel):
            global speedScale
            speedScale = new_max_vel
        else:
            dprint("Value of speedScale not between 0.0 and config_max_vel. Ignoring")

    #Param Receiving
    parambottle = paramPort.read(False)
    #pending=paramPort.getPendingReads()
    #for i in xrange(pending):
    #    parambottle = paramPort.read(False)
    if (start_attractor or (parambottle and (parambottle.size() >= 1))):
        dprint("setting vectorfield")
        #eprint("COUNTER: ", counter_test)
        counter_test+=1
#      print "PARAMETER: ",parambottle.toString()
        #We receive new parameter values!
        if not start_attractor:
            paramAction = parambottle.get(0)
        else:
            paramAction = yarp.Value_makeString("start_attractor")
            #paramAction.fromString()
        if paramAction.toString() == "start_attractor":
            dprint("Adding start attractor")
            force = 1.0
            tVF = 1
            params = first_arm_frame + [0.05]
            vectorFields[1] = [force, tVF, params]
            dprint("New vector field", vectorFields[1])
            start_attractor = False
        if paramAction.toString() == "add":
            if (parambottle.size() == 5):
                #Add attractor/repeller point
                #0: "add", 1: vectorField number 2: force (positive attractor, negative repeller) 
                # 3: type of vectorField 
                # 4: vectorField parameters (list)
                # We have to check if there is another attractor/repeller in the same point. Only Repellers 
                # can be with repellers and attractors with attractors
                force = parambottle.get(2).asDouble()
                tVF = parambottle.get(3).asInt()
                if tVF in vfDB:
                    if force <= 0:
                        #print "Repeller. Force: ", force
                        paramsbottle = parambottle.get(4).asList()
                        params = map(yarp.Value.asDouble, map(paramsbottle.get, range(paramsbottle.size())))
                        #print "Params: ", params
                        vectorFields[parambottle.get(1).asInt()] = [force, tVF, params]
                    else:
                        #print "Attractor. Force: ", force
                        paramsbottle = parambottle.get(4).asList()
                        params = map(yarp.Value.asDouble,map(paramsbottle.get, range(paramsbottle.size())))
                        #print "Params: ", params
                        vectorFields[parambottle.get(1).asInt()] = [force, tVF, params]
                        dprint("Normal, vector field number", parambottle.get(1).asInt(), "vectorfield params", vectorFields[parambottle.get(1).asInt()])
                else:
                    dprint("Unknown vector field type, ignoring")
            else:
                dprint("Wrong number of values, expected 5, ignoring")
        if paramAction.toString() == "remove":
            if parambottle.size() == 2:
                if parambottle.get(1).asInt() in vectorFields:
                    del vectorFields[parambottle.get(1).asInt()]
                else:
                    pass
#            print "Vectorfield doesn't exist, doing nothing"
            else:
                dprint("Wrong number of values, expected 2")
        vftemp = vfDB[0]()
        vftemp.setParams([])
        totalVFtemp = VectorField(vftemp.getVector)
        totalSFtemp = ScalarField(vftemp.getScalar)
        for vfNum in vectorFields:
            vfForce = vectorFields[vfNum][0]
            vfType = vectorFields[vfNum][1]
            vfFunction = vfDB[vfType]()
            vfparams = vectorFields[vfNum][2]
            #print "Adding Function: ",vfFunction, " with parameters: ",vfparams
            vfFunction.setParams(vfparams)
            vftemp = VectorField(vfFunction.getVector)
            sftemp = ScalarField(vfFunction.getScalar)
            totalVFtemp += vftemp * vfForce
            totalSFtemp *= sftemp
        #totalVF=totalVFtemp
        totalVF = totalVFtemp.normCart()
        totalSF = totalSFtemp


    #Get the Weights
    weightbottle = weightPort.read(False)
    if (weightbottle and (weightbottle.size() >= 1)):
        #print "WEIGHTS: ",weightbottle.toString()
        weight_type = weightbottle.get(0).asString()


        if (weight_type == 't'):   #Task space
            n_task_vars = 6
            weights = get_weight_matrix(weightbottle, n_task_vars)
            if (weights != None):
                lafik.set_tweights(weights)
        elif (weight_type == 'j'):  #Joint space
            weights = get_weight_matrix(weightbottle,config.nJoints)
            if (weights != None):
                lafik.set_jweights(weights)

    #getting angle positions from the robot
    qInbottle = qInPort.read(False)
    if (qInbottle and (qInbottle.size() == numJntsArm)):
        q = map(yarp.Value.asDouble, map(qInbottle.get, range(qInbottle.size())));
        lafik.jntsList = q
        frame = lafik.frame
        kdlframe = lafik.kdlframe

        #tool calculation goes here
        toolFrame = readListPort(toolPort)
        if (not toolFrame) or (len(toolFrame) != 16):
        #  print "Got here"
            toolFrame = oldtoolFrame
        else:
            oldtoolFrame = toolFrame
        #print "toolFrame: ", toolFrame, "length: ", len(toolFrame)

        #Calculating new frame with respect of the tool
        kdltoolframe = listToKdlFrame(toolFrame)
        newkdlframe = kdlframe * kdltoolframe
        diff = PyKDL.diff(newkdlframe, kdlframe)
        frame = kdlFrameToList(newkdlframe)
#      if first_cycle:
#          last_frame=frame
#          first_cycle=False
        if first_arm_data:
            first_arm_frame = frame
            first_arm_data = False
            start_attractor = True

        sendListPort(posePort, frame)
        sendListPort(pose_no_tool_Port, kdlFrameToList(kdlframe))
        #print "Frame: ", frame
        velvector = totalVF.getVector(frame)
        scalars = totalSF.getScalar(frame)
        velPosvector = speedScale * scalars[0] * velvector[0:3]
        velRotvector = speedScale * scalars[1] * velvector[3:6]

        #calculating discrepancy
        cmd_buffer.append([velPosvector, velRotvector])
        if len(cmd_buffer) > cmd_buffer_size:
            cmd_buffer.pop(0)
        frame_list.append(newkdlframe)
        if len(frame_list) > frame_list_size:
            frame_list.pop(0)
            ext_diff = PyKDL.diff(frame_list[len(frame_list)-2], frame_list[len(frame_list)-1])
            #print "External vel", array([ext_diff.vel[i] for i in range(3)])
            ext_vel_mag = sqrt(dot(array([ext_diff.vel[i] for i in range(3)]), array([ext_diff.vel[i] for i in range(3)])))
            cmd_vel_mag = sqrt(dot(cmd_buffer[len(cmd_buffer) - check_delay][0], cmd_buffer[len(cmd_buffer) - check_delay][0]))
            if ext_vel_mag > 0:
                ext_vel = array([ext_diff.vel[i] for i in range(3)]) / ext_vel_mag
            else:
                ext_vel = array([1.0, 0, 0])
            if cmd_vel_mag > 0: 
                cmd_vel = cmd_buffer[len(cmd_buffer)-check_delay][0] / cmd_vel_mag
            else:
                cmd_vel = array([1.0, 0, 0])
            #print "External speed", ext_vel, "Commanded speed", cmd_vel
            vel_dot = dot(cmd_vel, ext_vel)
            if vel_dot > 1.0:
                vel_dot = 1.0
            elif vel_dot < -1.0:
                vel_dot = -1.0
            vel_diff_angle = sqrt((acos(vel_dot)) ** 2)

            ext_rot_mag = sqrt(dot(array([ext_diff.rot[i] for i in range(3)]), array([ext_diff.rot[i] for i in range(3)])))
            cmd_rot_mag = sqrt(dot(cmd_buffer[len(cmd_buffer) - check_delay][1], cmd_buffer[len(cmd_buffer) - check_delay][1]))
            if ext_rot_mag > 0:
                ext_rot = array([ext_diff.rot[i] for i in range(3)]) / ext_rot_mag
            else:
                ext_rot = array([1.0, 0, 0])
            if cmd_rot_mag > 0: 
                cmd_rot = cmd_buffer[len(cmd_buffer) - check_delay][1] / cmd_rot_mag
            else:
                cmd_rot = array([1.0, 0, 0])
            #print "External rot", ext_rot, "Commanded rot", cmd_rot
            rot_dot = dot(cmd_rot,ext_rot)
            if rot_dot > 1.0:
                rot_dot = 1.0
            if rot_dot < -1.0:
                rot_dot = -1
            rot_diff_angle = sqrt((acos(rot_dot)) ** 2)
            
            loop_freq = 150  # conversion factor between external speed
            tracking_th = 0.10
            ext_vel_mag_corr = ext_vel_mag * loop_freq
            ext_rot_mag_corr = ext_rot_mag * loop_freq
            cmd_rot_mag_corr = cmd_rot_mag / 5.0
            cmd_vel_mag_corr = cmd_vel_mag * 1.0
            ext_int_diff = abs((cmd_vel_mag_corr + cmd_rot_mag_corr) - (ext_vel_mag_corr + ext_rot_mag_corr))
            arm_tracking =  ext_int_diff < tracking_th
            
            tracking_error_bottle = tracking_error_port.prepare()
            tracking_error_bottle.clear()
            tracking_error_bottle.addDouble(vel_diff_angle)
            tracking_error_bottle.addDouble(rot_diff_angle)
            tracking_error_bottle.addDouble(ext_vel_mag_corr)
            tracking_error_bottle.addDouble(ext_rot_mag_corr)
            tracking_error_bottle.addDouble(cmd_vel_mag_corr)
            tracking_error_bottle.addDouble(cmd_rot_mag_corr)
            tracking_error_bottle.addDouble(ext_int_diff)
            tracking_error_bottle.addInt(int(arm_tracking))
            tracking_error_port.write()

#      last_frame=frame


        reporting_port_counter = reporting_port_counter + 1

        if reporting_port_counter > 20:
            reporting_port_counter = 0

            #sending vector out
            vector_out_bottle = vector_port.prepare()
            vector_out_bottle.clear()
            for i in velPosvector.tolist() + velRotvector.tolist():
                vector_out_bottle.addDouble(i)
            vector_port.write()

            #Sending the goal
            goal_out_bottle = goal_port.prepare()
            goal_out_bottle.clear()

            if 1 in vectorFields.keys():
                param_goal = vectorFields[1][2]
                for i in param_goal:
                    goal_out_bottle.addDouble(i)

                goal_port.write()


        #adjusting for new tool frame
        tw = PyKDL.Twist(PyKDL.Vector(*(velPosvector.tolist())), PyKDL.Vector(*(velRotvector.tolist())))
        tw = tw.RefPoint(diff.vel)

        qdot = lafik.getIKV(tw.vel,tw.rot)
        qdotOutbottle = qdotOutPort.prepare()
        qdotOutbottle.clear()
        #for k in range(3):
        #  qdotOutbottle.addDouble(speedScale*scalars[0]*velvector[k])
        #for k in range(3):
        #  qdotOutbottle.addDouble(speedScale*scalars[1]*velvector[k+3])
        for i in qdot:
            qdotOutbottle.addDouble(i)
        qdotOutPort.write()

    #for visualizing
    pose_in_bottle = pose_in_port.read(False)
    if (pose_in_bottle and (pose_in_bottle.size() == 16)):
        pose_in = map(yarp.Value.asDouble,map(pose_in_bottle.get,range(pose_in_bottle.size())));
#       lafik.jntsList=q
#       frame=lafik.frame
#       kdlframe=lafik.kdlframe

#       #tool calculation goes here
#       toolFrame=readListPort(toolPort)
#       if (not toolFrame) or (len(toolFrame)!=16):
#       #  print "Got here"
#         toolFrame=oldtoolFrame
#       else:
#         oldtoolFrame=toolFrame
#       #print "toolFrame: ", toolFrame, "length: ", len(toolFrame)

#       #Calculating new frame with respect of the tool
#       kdltoolframe=listToKdlFrame(toolFrame)
#       newkdlframe=kdlframe*kdltoolframe
#       diff=PyKDL.diff(newkdlframe,kdlframe)
#       frame=kdlFrameToList(newkdlframe)

#       sendListPort(posePort,frame)
        #print "Frame: ", frame
        velvector = totalVF.getVector(pose_in)
        scalars = totalSF.getScalar(pose_in)
        velPosvector = speedScale * scalars[0] * velvector[0:3]
        velRotvector = speedScale * scalars[1] * velvector[3:6]

        vector_out_bottle = vector_port.prepare()
        vector_out_bottle.clear()
        for i in velPosvector.tolist() + velRotvector.tolist():
            vector_out_bottle.addDouble(i)
        vector_port.write()

# #adjusting for new tool frame
#       tw=PyKDL.Twist(PyKDL.Vector(*(velPosvector.tolist())),PyKDL.Vector(*(velRotvector.tolist())))
#       tw=tw.RefPoint(diff.vel)

#       qdot=lafik.getIKV(tw.vel,tw.rot)
#       qdotOutbottle = qdotOutPort.prepare()
#       qdotOutbottle.clear()
#       #for k in range(3):
#       #  qdotOutbottle.addDouble(speedScale*scalars[0]*velvector[k])
#       #for k in range(3):
#       #  qdotOutbottle.addDouble(speedScale*scalars[1]*velvector[k+3])
#       for i in qdot:
#         qdotOutbottle.addDouble(i)
#       qdotOutPort.write()

    yarp.Time.delay(0.001005)  # 0.001

qdotOutPort.close() 
qInPort.close()
paramPort.close()
toolPort.close()
posePort.close()
pose_in_port.close()
goal_port.close()
pose_no_tool_Port.close()
weightPort.close()
tracking_error_port.close()
vector_port.close()
maxvel_port.close()
yarp.Network.fini();

