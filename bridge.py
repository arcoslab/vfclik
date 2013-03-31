#!/usr/bin/python
# Copyright (c) 2009 Technische Universitaet Muenchen, Informatik Lehrstuhl IX.
# Author: Ingo Kresse <kresse at in.tum.de>
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
yarp.Network.init()
import time
from math import pi

import command_mixer
import sys
from arcospyu.config_parser import ConfigFileParser
import ctypes
import numpy
from math import sqrt
icubname="icub"

import signal
def signal_handler(sig   , frame):
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

change_ps_name('bridge.py')

config_parser=ConfigFileParser(sys.argv)
config_parser.parser.add_option("-s", "--simulation", action="store_true", dest="sim", default=False,help="Simulation")
options, args, config = config_parser.get_all()

config_max_vel = config.max_vel
max_vel = config.max_vel

# other options
synchronized=False
yarpbridgeportbasename=config.robotarm_portbasename+"/bridge"
encoders_portname = yarpbridgeportbasename+"/encoders"
vectorfieldcmd_portname = yarpbridgeportbasename+"/vectorfieldcmd"
nullcmd_portname = yarpbridgeportbasename+"/nullcmd"
jointcmd_portname = yarpbridgeportbasename+"/jointcmd"
mechanismcmd_portname = yarpbridgeportbasename+"/mechanismcmd"
weight_portname = yarpbridgeportbasename+"/weights"
current_weight_portname = yarpbridgeportbasename+"/current_weights"
maxvel_portname = yarpbridgeportbasename+"/max_vel"
xtra1cmd_portname = yarpbridgeportbasename + "/xtra1cmd"
xtra2cmd_portname = yarpbridgeportbasename + "/xtra2cmd"

rate = config.rate
direct_control=False
try: config.force_control
except AttributeError:
    pass
else:
#  force_control=config.force_control
    pass

# Arm bridges should (at least) implement these methods:
#
#class Arm_Bridge():
#  def __init__(self): # constructor
#  def open(self): # open the (network) connection to the arm
#  def close(self): # close the connection to the arm
#  def set_vel(self, qdot): # send velocities
#  def read_pos(self): # read current position (blocking), emptying queues if necessary

def construct_arm_bridge(name,sim):
    if name == "lwr":
        return LWR_Bridge(sim)
    if name == "powercube":
        return Powercube_Bridge(sim)
    if name == "icub":
        return ICUB_Bridge(sim)
    else:
        print 'warning: do not know class "%"' % name
        return None # this will crash!

class LWR_Bridge():
    def __init__(self,sim):
        self.sim=sim #todo: for now not used. LWR_Bridge works for simulation and for the real robot fine
        self.nJoints = config.nJoints
        self.last_q = self.nJoints*[0.0]
        self.last_qcmded = []

    def __del__(self):
        self.close()

    def open(self):
        # yarp ports for reading
        self.qin_port = yarp.BufferedPortBottle()
        self.qin_port.open(config.robotarm_portbasename+config.qin_portname)
        self.qcmded_port = yarp.BufferedPortBottle()
        self.qcmded_port.open(config.robotarm_portbasename+config.qcmded_portname)
        # yarp port for writing
        self.qcmd_port = yarp.BufferedPortBottle()
        self.qcmd_port.open(config.robotarm_portbasename+config.qcmd_portname)


    def close(self):
        self.qin_port.close()
        self.qcmded_port.close()
        self.qcmd_port.close()


    def read_pos(self):
        bottle = self.qin_port.read(True)
        if(bottle.size() == self.nJoints):
            self.last_q = ut_bottle2list(bottle)

        bottle = self.qcmded_port.read(False)
        if bottle != None and bottle.size() == self.nJoints:
            self.last_qcmded = ut_bottle2list(bottle)
        elif self.last_qcmded == []:
            self.last_qcmded = self.last_q

        return self.last_q


    def set_vel(self, qdot):
        global direct_control
        if len(qdot) != self.nJoints or self.last_qcmded == []:
            print 'got %d velocities, expected %d. Ignoring input' % (len(qdot), self.nJoints)

        # limit joint velocities
        global max_vel
        leading_vel = max(map(abs, qdot))
        if (leading_vel > max_vel):
            ### print "LIMIT: Limiting to %3.1f deg/s, ratio = %2.2f" % (max_vel*180.0/pi, leading_vel)
            ratio = max_vel/leading_vel
        else:
            ratio = 1.0

        qdot_lim = [v*ratio for v in qdot]

        cmd = self.nJoints*[0.0]
        for i in range(self.nJoints):
            if direct_control:
                cmd[i]=qdot_lim[i]
            else:
                cmd[i] = -self.last_qcmded[i] + self.last_q[i] + qdot_lim[i]

        # sending command
        cmd_bottle = self.qcmd_port.prepare()
        cmd_bottle.clear()
        for v in cmd:
            cmd_bottle.addDouble(v)
        self.qcmd_port.write()


class Powercube_Bridge():
    def __init__(self,sim):
        self.sim=sim
        if(not self.sim):
            self.pyplayerc=__import__("pyplayerc")
        self.nJoints = config.nJoints
        self.last_q = self.nJoints*[0.0]
        print 'b21r'

    def __del__(self):
        self.close()

    def open(self):
        if not self.sim:
            print "Connecting to player"
            self.client = self.pyplayerc.Client(None, config.player_actarray_server, config.player_actarray_port)
            self.client.connect()
            self.client.set_replace_rule(-1,-1,-1,-1,False)
            self.client.datamode(self.pyplayerc.PLAYERC_DATAMODE_PUSH)
            self.actArray = self.pyplayerc.Actarray(self.client,config.player_actarray_interface)
            self.actArray.subscribe(self.pyplayerc.PLAYERC_OPEN_MODE)
            self.client.read()
            print "Sending to initial position in joint space"
            poscmd_t=ctypes.c_float*len(config.initial_joint_pos)
            poscmd=poscmd_t(*(config.initial_joint_pos))
            self.actArray.multi_position_cmd(poscmd,len(config.initial_joint_pos))
            #time.sleep(10)
            angleerror=3.0*pi/180
            done=False
            while not done:
                self.client.read()
                currentpos=numpy.array([self.actArray.actuators_data[i].position for i in xrange(self.nJoints)])
                distance=sqrt(reduce(lambda x,y: x+y,[i**2 for i in (numpy.array(config.initial_joint_pos)-currentpos)]))
                if distance<angleerror:
                    done=True
            print "Robot ready in initial joint position"
        else:
        # yarp ports for reading
            self.qin_port = yarp.BufferedPortBottle()
            self.qin_port.open(yarpbridgeportbasename+"/qin")
        # yarp port for writing
            self.qcmd_port = yarp.BufferedPortBottle()
            self.qcmd_port.open(yarpbridgeportbasename+"/qcmd")

    def close(self):
        if sim:
            self.qin_port.close()
            self.qcmd_port.close()
        else:
            self.actArray.unsubscribe()
            del self.actArray
            self.client.disconnect()
            del self.client

    def read_pos(self):
        if self.sim:
            bottle = self.qin_port.read(True)
            if(bottle.size() == self.nJoints):
                self.last_q = ut_bottle2list(bottle)
        else:
            self.client.read()
            self.last_q=[self.actArray.actuators_data[i].position for i in xrange(self.nJoints)]
        return self.last_q

    def set_vel(self,qdot):
        # limit joint velocities
        max_vel = config.max_vel
        leading_vel = max(map(abs, qdot))
        if (leading_vel > max_vel):
            ### print "LIMIT: Limiting to %3.1f deg/s, ratio = %2.2f" % (max_vel*180.0/pi, leading_vel)
            ratio = max_vel/leading_vel
        else:
            ratio = 1.0
        qdot_lim = [v*ratio for v in qdot]
        shoulder_vel=qdot_lim[0]
        if (shoulder_vel > config.max_vel_shoulder_pos):
            ratio=abs(config.max_vel_shoulder_pos/shoulder_vel)
            print "Limiting shoulder speed"
        elif (shoulder_vel < config.max_vel_shoulder_neg):
            ratio=abs(config.max_vel_shoulder_neg/shoulder_vel)
            print "Limiting shoulder speed"
        qdot_lim=[i*ratio for i in qdot_lim]
        if self.sim:
            cmd_bottle = self.qcmd_port.prepare()
            cmd_bottle.clear()
            for v in qdot_lim:
                cmd_bottle.addDouble(v)
            self.qcmd_port.write()
        else:
            speedcmd_t=ctypes.c_float*len(qdot_lim)
            speedcmd=speedcmd_t(*qdot_lim)
            self.actArray.multi_speed_cmd(speedcmd,len(qdot_lim))

class ICUB_Bridge():
    def __init__(self,sim):
        self.sim=sim
        self.nJoints = config.nJoints
        self.last_q = self.nJoints*[0.0]
        print "iCub"
        self.options_arm=yarp.Property()
        s=yarpbridgeportbasename+"/devcontrol_arm"
        self.options_arm.put("local",s)
        s="/"+icubname+"/"+config.arm_instance+"_arm"
        self.options_arm.put("remote",s)
        self.options_arm.put("robot","icub")
        self.options_arm.put("device","remote_controlboard") 
        self.options_torso=yarp.Property()
        s=yarpbridgeportbasename+"/devcontrol_torso"
        self.options_torso.put("local",s)
        s="/"+icubname+"/torso"
        self.options_torso.put("remote",s)
        self.options_torso.put("robot","icub")
        self.options_torso.put("device","remote_controlboard")

        # TODO: MOVE THIS PORT TO SOMEWHERE MORE GENERAL
        self.icub_torso_cjoints=config.icub_torso_cjoints
        # TODO: Why is this not being read correctly?
        self.icub_torso_num_joints=3
        self.icub_arm_num_joints=7

        self.torso_cjoints_port=yarp.BufferedPortBottle()
        cjoints_portname=yarpbridgeportbasename+"/torso_cjoints:i"
        self.torso_cjoints_port.open(cjoints_portname)

        self.control_weights_port=yarp.BufferedPortBottle()
        control_weights_portname=yarpbridgeportbasename+"/control_weights:o"
        self.control_weights_port.open(control_weights_portname)

    def __del__(self):
        self.close()

    def open(self):
        if not self.sim:
            print "Connecting to the iCub"
            self.dd_arm=yarp.PolyDriver(self.options_arm)
            self.dd_torso=yarp.PolyDriver(self.options_torso)

            self.enc_arm=self.dd_arm.viewIEncoders()
            self.velcmd_arm=self.dd_arm.viewIVelocityControl()
            self.poscmd_arm=self.dd_arm.viewIPositionControl()
            self.icub_arm_num_joints=self.velcmd_arm.getAxes()

            self.enc_torso=self.dd_torso.viewIEncoders()
            self.velcmd_torso=self.dd_torso.viewIVelocityControl()
            self.poscmd_torso=self.dd_torso.viewIPositionControl()
            self.icub_torso_num_joints=self.velcmd_torso.getAxes()
            #self.icub_torso_num_joints=3

            print "Torso num joints:", self.icub_torso_num_joints

            print "Sending to initial position in joint space"
            print config.initial_joint_pos
            self.joint_list=range(self.icub_torso_num_joints-1,-1,-1)+range(self.icub_torso_num_joints,self.nJoints)
            print "JointList", self.joint_list

            for n,i in enumerate(self.joint_list):
                if i<self.icub_torso_num_joints:
                    self.poscmd_torso.positionMove(i,config.initial_joint_pos[n]*180.0/pi)
                else:
                    self.poscmd_arm.positionMove(i-self.icub_torso_num_joints,config.initial_joint_pos[i]*180.0/pi)


            angleerror=3.0*pi/180
            done=False
            while not done:
                currentposDV_torso=yarp.DVector(self.icub_torso_num_joints)
                currentposDV_arm=yarp.DVector(self.icub_arm_num_joints)
                self.enc_torso.getEncoders(currentposDV_torso)
                self.enc_arm.getEncoders(currentposDV_arm)
#        print "torso",
#        for i in currentposDV_torso: print i,
#        print 
#        print "arm",
#        for i in currentposDV_arm: print i,
#        print 
                currentpos=numpy.array([i*pi/180.0 for i in currentposDV_torso]+[i*pi/180.0 for i in currentposDV_arm][0:(self.nJoints-self.icub_torso_num_joints)])
                temp=currentpos[0]
                currentpos[0]=currentpos[2]
                currentpos[2]=temp
                print "Current pos", currentpos
                print "Init pos", config.initial_joint_pos
                distance=sqrt(reduce(lambda x,y: x+y,[i**2 for i in (numpy.array(config.initial_joint_pos)-currentpos)]))
                print "Distance", distance
                if distance<angleerror:
                    done=True
            print "Robot ready in initial joint position"
        else:
        # yarp ports for reading
            self.qin_port = yarp.BufferedPortBottle()
            self.qin_port.open(yarpbridgeportbasename+"/qin")
        # yarp port for writing
            self.qcmd_port = yarp.BufferedPortBottle()
            self.qcmd_port.open(yarpbridgeportbasename+"/qcmd")

    def close(self):
        self.torso_cjoints_port.close()
        self.control_weights_port.close()

        if sim:
            self.qin_port.close()
            self.qcmd_port.close()
        else:
            pass

    def read_pos(self):

        #Added at VVV09
        bin=self.torso_cjoints_port.read(False)
        if (bin):
            print "Received a torso_joints bottle"
            print bin.toString()
            cjoints=[]
            for i in range(bin.size()):
                cjoints.append(bin.get(i).asInt())
            self.set_torso_cjoints(cjoints)

        if self.sim:
            bottle = self.qin_port.read(True)
            if(bottle.size() == self.nJoints):
                self.last_q = ut_bottle2list(bottle)
        else:
            encDV_arm=yarp.DVector(self.icub_arm_num_joints)
            encDV_torso=yarp.DVector(self.icub_torso_num_joints)
            self.enc_torso.getEncoders(encDV_torso)
            self.enc_arm.getEncoders(encDV_arm)
            self.last_q=[i*pi/180.0 for i in encDV_torso]+[i*pi/180.0 for i in encDV_arm][0:self.nJoints-self.icub_torso_num_joints]
            temp=self.last_q[0]
            self.last_q[0]=self.last_q[2]
            self.last_q[2]=temp
        return self.last_q


    #Additional method at VVV09 to deal with torso joints (activation/deactivation)
    def set_torso_cjoints(self,cjoints):
        #print "Torso num joints: %d  arm_num_joints: %d"%(self.icub_torso_num_joints,self.icub_arm_num_joints)
        #hand_joints=9
        #num_weights=self.icub_torso_num_joints+self.icub_arm_num_joints-hand_joints
        num_weights=10 #FIXME In the simulator, the size needs to come from somewhere else
        weights=[1.0]*num_weights

        if (len(cjoints)==self.icub_torso_num_joints):
            for i in range(self.icub_torso_num_joints):
                if (cjoints[i]==1):
                    self.icub_torso_cjoints[i]=True
                    weights[i]=1.0
                else:
                    self.icub_torso_cjoints[i]=False
                    if (not self.sim):
                        self.velcmd_torso.velocityMove(i,0.0) #Stop the torso joint
                    else:
                        pass
                        #stop the simulator too

                    weights[i]=0.0
            #tell the Vector Field about the new weights
            bout=self.control_weights_port.prepare()
            bout.clear()
            bout.addString("j")
            for i in weights:
                bout.addDouble(i)
            print "Sending weights: ",
            print bout.toString()
            self.control_weights_port.write(True)
        else:
            print("WARNING: set_torso_cjoints received a vector of controlled joints of a wrong length")

    def set_vel(self,qdot):
        # limit joint velocities
        max_vel = config.max_vel
        leading_vel = max(map(abs, qdot))
        if (leading_vel > max_vel):
            ### print "LIMIT: Limiting to %3.1f deg/s, ratio = %2.2f" % (max_vel*180.0/pi, leading_vel)
            ratio = max_vel/leading_vel
        else:
            ratio = 1.0
        qdot_lim = [v*ratio for v in qdot]
        if self.sim:
            cmd_bottle = self.qcmd_port.prepare()
            cmd_bottle.clear()
            for v in qdot_lim:
                cmd_bottle.addDouble(v)
            self.qcmd_port.write()
        else:
#      print "Motion to send"
            for n,i in enumerate(self.joint_list):
                if i<self.icub_torso_num_joints:
                    #only control the joints that are active
                    if self.icub_torso_cjoints[i]:
                        #print "torso j:",i,"speed:",qdot_lim[n]*180.0/pi
                        self.velcmd_torso.velocityMove(i,qdot_lim[n]*180.0/pi)
                    else:
                        pass
                        #print("Did not send vel to torso[%d]"%(i))


                else:
#          print "arm j:",i-self.icub_torso_num_joints, "speed:", qdot_lim[i]*180.0/pi
                    self.velcmd_arm.velocityMove(i-self.icub_torso_num_joints,qdot_lim[i]*180.0/pi)

def ut_list2bottle(list, bottle):
    bottle.clear()
    for i in list:
        bottle.addDouble(i)
    return bottle

def ut_bottle2list(bottle):
    return [bottle.get(i).asDouble() for i in range(bottle.size())]

def ut_writebottle(port, list):
    bottle = port.prepare()
    bottle.clear()
    for i in list:
        bottle.addDouble(i)
    port.write()

def ut_readbottle(port, blocking=False, size=-1):
    bottle = self.ports[p].read(blocking)
    if(bottle and (size == -1 or bottle.size() == size) ):
        return [bottle.get(i).asDouble() for i in range(size)]
    else:
        return None




def main():
    global stop
    bridge = construct_arm_bridge(config.arm_type,options.sim)

    encoders_port = yarp.BufferedPortBottle()
    encoders_port.open(encoders_portname)

    vectorfieldcmd_port = yarp.BufferedPortBottle()
    vectorfieldcmd_port.open(vectorfieldcmd_portname)

    nullcmd_port = yarp.BufferedPortBottle()
    nullcmd_port.open(nullcmd_portname)

    jointcmd_port = yarp.BufferedPortBottle()
    jointcmd_port.open(jointcmd_portname)

    mechanismcmd_port = yarp.BufferedPortBottle()
    mechanismcmd_port.open(mechanismcmd_portname)
    
    xtra1cmd_port = yarp.BufferedPortBottle()
    xtra1cmd_port.open(xtra1cmd_portname)
    
    xtra2cmd_port = yarp.BufferedPortBottle()
    xtra2cmd_port.open(xtra2cmd_portname)

    weight_port = yarp.BufferedPortBottle()
    weight_port.open(weight_portname)
    weight_port.setStrict(True)

    current_weight_port = yarp.BufferedPortBottle()
    current_weight_port.open(current_weight_portname)
    current_weight_port.setStrict(True)

    maxvel_port = yarp.BufferedPortBottle()
    maxvel_port.open(maxvel_portname)
    maxvel_port.setStrict(True)
    


    bridge.open()

    mixer = command_mixer.CommandMixer([vectorfieldcmd_port, nullcmd_port, jointcmd_port, mechanismcmd_port, xtra1cmd_port, xtra2cmd_port], weight_port, config.nJoints, 2.0, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    global direct_control

    while(not stop):

        # if there is no controller, then avoid drifting.
        direct_control = all([w == 0 for w in mixer.weights])

        initTime=time.time()

        # read data from arm and publish data to yarp
        q = bridge.read_pos()
        ut_writebottle(encoders_port, q)

        # Adjust new max_vel value (Max rot speed for all joints)
        botin = maxvel_port.read(False)
        if (botin):
            new_max_vel = botin.get(0).asDouble()
            if (new_max_vel >= 0.0) and (new_max_vel <= config_max_vel):
                global max_vel
                max_vel = new_max_vel
            else:
                print("Value of max_vel not between 0.0 and config.max_vel. Ignoring")

        # collect velocities from yarp loop and write to robot
        bridge.set_vel(mixer.read())
        ut_writebottle(current_weight_port, mixer.weights)

        endTime = time.time()
        #print 'processing time: %4.4fs' % (endTime-beginTime)

        # timing calculations
        finalTime=time.time()
        #print '%(time).105g' % {'time': finalTime}
        runTime=finalTime-initTime
        sleepTime=rate-runTime
        #print "Run Time: ", runTime, "Sleep Time: ", sleepTime
        if sleepTime>0:
            time.sleep(sleepTime)
            #pass
        #else:
        #  print "Warning cycle too slow (%4.4fs)" % (runTime)

    encoders_port.close()
    vectorfieldcmd_port.close()
    nullcmd_port.close()
    jointcmd_port.close()
    mechanismcmd_port.close()
    xtra1cmd_port.close()
    xtra2cmd_port.close()
    weight_port.close()
    current_weight_port.close()
    maxvel_port.close()
    bridge.close()


if __name__ == "__main__":
    main()
