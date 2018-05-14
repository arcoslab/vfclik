#!/usr/bin/env python
# Copyright (c) 2009 Technische Universitaet Muenchen, Informatik Lehrstuhl IX.
# Authors: Alexis Maldonado Herrera <maldonad at cs.tum.edu>
#          Federico Ruiz-Ugalde <memeruiz@gmail.com>
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
import time
from builtins import range
from numpy import array
from numpy import pi
from time import sleep
from arcospyu.yarp_tools.yarp_comm_helpers import yarp_connect_blocking, \
    new_port
from arcospyu.kdl_helpers.kdl_helpers import frame_to_list

cstyle=yarp.ContactStyle()
cstyle.persistent=True

class HandleArmNew:
    def __init__(self, namespace="/0", module_name="/handle_arm", arm_namespace="/0", robot="/lwr", arm="/right"):
        self.module_name=module_name
        self.namespace=namespace
        self.arm_namespace=arm_namespace
        self.arm_object_port_name=arm_namespace+robot+arm+"/ofeeder/object"
        self.arm_stiffness_port_name=arm_namespace+robot+arm+"/robot/stiffness"
        self.arm_pose_port_name=arm_namespace+robot+arm+"/vectorField/pose"
        self.arm_distout_port_name=arm_namespace+robot+arm+"/dmonitor/distOut"
        self.arm_tool_port_name=arm_namespace+robot+arm+"/vectorField/tool"
        self.arm_bridge_weight_port_name=arm_namespace+robot+arm+"/bridge/weight"
        self.arm_vf_weight_port_name=arm_namespace+robot+arm+"/vectorField/weight"
        self.arm_bridge_encoders_port_name=arm_namespace+robot+arm+"/bridge/encoders"
        self.arm_joint_ref_port_name=arm_namespace+robot+arm+"/jpctrl/ref"

        self.object_port_name=namespace+self.module_name+arm+"/object"
        self.stiffness_port_name=namespace+self.module_name+arm+"/stiffness"
        self.pose_port_name=namespace+self.module_name+arm+"/pose"
        self.distout_port_name=namespace+self.module_name+arm+"/distOut"
        self.tool_port_name=namespace+self.module_name+arm+"/tool"
        self.bridge_weight_port_name=namespace+self.module_name+arm+"/bridge/weight"
        self.vf_weight_port_name=namespace+self.module_name+arm+"/vectorField/weight"
        self.bridge_encoders_port_name=namespace+self.module_name+arm+"/encoders"
        self.joint_ref_port_name=namespace+self.module_name+arm+"/joint_ref"

        self.object_port=yarp.BufferedPortBottle()
        self.stiffness_port=yarp.BufferedPortBottle()
        self.pose_port=yarp.BufferedPortBottle()
        self.distout_port=yarp.BufferedPortBottle()
        self.tool_port=yarp.BufferedPortBottle()
        self.bridge_weight_port=yarp.BufferedPortBottle()
        self.vf_weight_port=yarp.BufferedPortBottle()
        self.bridge_encoders_port=yarp.BufferedPortBottle()
        self.joint_ref_port=yarp.BufferedPortBottle()

        self.object_port.open(self.object_port_name)
        self.stiffness_port.open(self.stiffness_port_name)
        self.pose_port.open(self.pose_port_name)
        self.distout_port.open(self.distout_port_name)
        self.tool_port.open(self.tool_port_name)
        self.bridge_weight_port.open(self.bridge_weight_port_name)
        self.vf_weight_port.open(self.vf_weight_port_name)
        self.bridge_encoders_port.open(self.bridge_encoders_port_name)
        self.joint_ref_port.open(self.joint_ref_port_name)

        yarp.Network.connect(self.object_port_name, self.arm_object_port_name, cstyle)
        yarp.Network.connect(self.stiffness_port_name, self.arm_stiffness_port_name, cstyle)
        yarp.Network.connect(self.arm_pose_port_name, self.pose_port_name, cstyle)
        yarp.Network.connect(self.arm_distout_port_name, self.distout_port_name, cstyle)
        yarp.Network.connect(self.tool_port_name, self.arm_tool_port_name, cstyle)
        yarp.Network.connect(self.bridge_weight_port_name, self.arm_bridge_weight_port_name, cstyle)
        yarp.Network.connect(self.vf_weight_port_name, self.arm_vf_weight_port_name, cstyle)
        yarp.Network.connect(self.arm_bridge_encoders_port_name, self.bridge_encoders_port_name, cstyle)
        yarp.Network.connect(self.joint_ref_port_name, self.arm_joint_ref_port_name, cstyle)

        while not (yarp.Network.isConnected(self.object_port_name, self.arm_object_port_name) and
                   yarp.Network.isConnected(self.stiffness_port_name, self.arm_stiffness_port_name) and
                   yarp.Network.isConnected(self.arm_pose_port_name, self.pose_port_name) and
                   yarp.Network.isConnected(self.arm_distout_port_name, self.distout_port_name) and
                   yarp.Network.isConnected(self.tool_port_name, self.arm_tool_port_name) and
                   yarp.Network.isConnected(self.bridge_weight_port_name, self.arm_bridge_weight_port_name) and
                   yarp.Network.isConnected(self.vf_weight_port_name, self.arm_vf_weight_port_name) and
                   yarp.Network.isConnected(self.arm_bridge_encoders_port_name, self.bridge_encoders_port_name) and
                   yarp.Network.isConnected(self.joint_ref_port_name, self.arm_joint_ref_port_name)):
            print "arm handler: waiting for initial connections"
            sleep(0.01)
        self.current_slowdown_distance = 0.1

    def set_vf_tool(self, toolframe):
        pass

    def set_stiffness(self, stiffness):
        pass

    def go_cart(self, frame):
        self.cart_goal=frame
        bottle = self.object_port.prepare()
        bottle.clear()
        bottle.addString("set")
        bottle.addString("goal")
        frame_list_bottle = bottle.addList()
        for i in self.cart_goal:
            frame_list_bottle.addDouble(i)
        frame_list_bottle.addDouble(self.current_slowdown_distance)
        self.object_port.writeStrict()
        self.set_cartesian_control()


    def _write_yarp_port(self, port, data, strict=True):
        bottle=port.prepare()
        bottle.clear()
        for i in data:
            if type(i)==float:
                bottle.addDouble(i)
            elif type(i)==int:
                bottle.addInt(i)
            elif type(i)==str:
                bottle.addString(i)
        if strict:
            port.writeStrict()
        else:
            port.write()

    def go_joint(self, angles):
        self.joint_goal=angles
        self._write_yarp_port(self.joint_ref_port, angles)
        self.set_joint_control()

    def go_xyz(self, xyz):
        pass

    def go_rot(self, rot):
        pass

    def get_cart_pose(self):
        bottle=self.pose_port.read(True)
        pose = map(yarp.Value.asDouble, map(bottle.get, range(bottle.size())))
        return(pose)

    def get_dist_cart_goal(self):
        pending = self.distout_port.getPendingReads()
        for i in range(pending):
            print("GOAL dist: ", self.distout_port.read(False))
        while True:
            dists=self.distout_port.read(True)
            for i in xrange(dists.size()):
                item=dists.get(i).asList()
                if item.get(0).asInt()==0:
                    #main goal
                    xyzdist=item.get(1).asDouble()
                    rotdist=item.get(2).asDouble()*pi/180.0
                    print "Cart distance: ", xyzdist, rotdist
                    return([xyzdist, rotdist])

    def get_dist_joint_goal(self):
        bottle=self.bridge_encoders_port.read(True)
        cur_angles=[]
        for i in xrange(bottle.size()):
            cur_angles.append(bottle.get(i).asDouble())
        return([i-j for i,j in zip(self.joint_goal, cur_angles)])

    def get_joint_angles(self):
        pass

    def set_controller_mixer(self, cart=True, joint=False, null=False):
        data=[]
        if cart:
            data.append(1)
        else:
            data.append(0)
        if null:
            data.append(1)
        else:
            data.append(0)
        if joint:
            data.append(1)
        else:
            data.append(0)
        data.append(0) # an extra channel?
        self._write_yarp_port(self.bridge_weight_port, data)

    def set_cartesian_control(self):
        self.set_controller_mixer(cart=True, null=True)


    def set_joint_control(self):
        self.set_controller_mixer(cart=False, null=False, joint=True)

    

class HandleArm(object):
    def __init__(self, arm_portbasename, namespace="", handlername="/HandlerArm"):

        prename = namespace + arm_portbasename
        full_name = prename + handlername

        self.outp = yarp.BufferedPortBottle()
        outp_name = full_name + "/toObjectFeeder"
        self.outp.open(outp_name)

        self.stiffness_port = yarp.BufferedPortBottle()
        stiffness_port_name = full_name + "/stiffness"
        self.stiffness_port.open(stiffness_port_name)

        self.goaldistp = yarp.BufferedPortBottle()
        goaldistp_name = full_name + "/fromGoalDistance"
        self.goaldistp.open(goaldistp_name)

        self.posep = yarp.BufferedPortBottle()
        posep_name = full_name + "/pose:i"
        self.posep.open(posep_name)

        yarp_connect_blocking(outp_name, prename + "/ofeeder/object")
        yarp_connect_blocking(stiffness_port_name,
                              prename + "/robot/stiffness")
        yarp_connect_blocking(prename + "/dmonitor/distOut", goaldistp_name)
        yarp_connect_blocking(prename + "/vectorField/pose", posep_name)

        self.current_frame = [
            1.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 1.0
        ]
        self.current_slowdown_distance = 0.1

        self.toolp = new_port(full_name + "/toToolin", 'out',
                              "%s/vectorField/tool" % (prename))
        self.toolp.setStrict(True)

        self.goal_threshold = 0.01

    def __del__(self):
        self.goaldistp.close()
        self.outp.close()

    def setTool(self, toolframe):
        ''' Receives a PyKDL Frame and sends it to VF as a tool
        '''

        print("Setting the tool of the arm to: ", toolframe)
        #        pp_F(toolframe)

        bout = self.toolp.prepare()
        bout.clear()
        tool_list = frame_to_list(toolframe)
        for i in tool_list:
            bout.addDouble(i)
        self.toolp.write(True)

    def set_stiffness(self, stiffness):
        bottle = self.stiffness_port.prepare()
        bottle.clear()
        for i in stiffness:
            bottle.addDouble(i)
        self.stiffness_port.write(True)

    def sendFrame(self):
        bout = self.outp.prepare()
        bout.clear()
        bout.addString("set")
        bout.addString("goal")
        frame_list = bout.addList()
        for i in self.current_frame:
            frame_list.addDouble(i)
        frame_list.addDouble(self.current_slowdown_distance)

        self.outp.write(True)

    def gotoPos(self, pos):
        print("Going here: ")
        print(pos)

        self.current_frame[3] = pos[0]
        self.current_frame[7] = pos[1]
        self.current_frame[11] = pos[2]

        self.sendFrame()

    def setOrient(self, orient):
        print("Setting orientation: ")
        print(orient)

        for i in range(3):
            for j in range(3):
                self.current_frame[j + 4 * i] = orient[j + i * 3]

        self.sendFrame()

    def getPose(self, blocking=True):
        pose_b = self.posep.read(blocking)
        pose = map(yarp.Value.asDouble, map(pose_b.get, range(pose_b.size())))
        return (pose)

    def gotoPose(self, pos, orient):

        self.current_frame[3] = pos[0]
        self.current_frame[7] = pos[1]
        self.current_frame[11] = pos[2]

        for i in range(3):
            for j in range(3):
                self.current_frame[j + 4 * i] = orient[j + i * 3]

        self.sendFrame()

    def gotoFrame(self, frame, wait=10.0, goal_precision=[]):
        '''frame: 16 value array

        wait: in seconds
        goal_precision: list of two values: trans precision and rot precision
        '''
        for i in range(len(frame)):
            self.current_frame[i] = frame[i]
        self.sendFrame()
        init_time = time.time()
        cur_time = init_time
        difference = array([0.0, 0.0])
        result = False
        pending = self.goaldistp.getPendingReads()
        for i in range(pending):
            print("GOAL dist: ", self.goaldistp.read(False))
        if len(goal_precision) == 2:
            if wait > 0.0:
                first_read = True
                while cur_time - init_time < wait:
                    # we check for goal finishing

                    bin = self.goaldistp.read(False)
                    if bin and not first_read:
                        for i in range(bin.size()):
                            line = bin.get(i).asList()
                            if (line.get(0).asInt() == 0):
                                pos_dist = line.get(1).asDouble()
                                orient_dist = (
                                    line.get(2).asDouble()) * pi / 180.0
                        # compare q vs js, if q is within tolerance, then break
                        print("Cartesian distance:", pos_dist, orient_dist)
                        difference = array([pos_dist, orient_dist])
                        if pos_dist < goal_precision[0] and \
                           orient_dist < goal_precision[1]:
                            result = True
                            break
                    first_read = False
                    sleep(0.01)
                    cur_time = time.time()
        return (result, difference)

    def gotThere(self):
        bin = self.goaldistp.read(True)
        dist = 1000.0

        if (bin):
            for i in range(bin.size()):
                line = bin.get(i).asList()
                if (line.get(0).asInt() == 0):
                    dist = line.get(1).asDouble()

            done = (dist < self.goal_threshold)
            if (done):
                return True
        return False

    def gotoPosBlocking(self, pos, timeout=20):
        self.gotoPos(pos)
        import time
        startTime = time.time()

        # ignore the first reports
        for i in range(10):
            a = self.gotThere()

        while ((time.time() - startTime) < timeout):
            a = self.gotThere()

            if (a):
                print("gotoPosBlocking GotThere")
                return (True)

        print("gotoPosBlocking TIMEOUT")
        return (False)

    def gotoPosBlockingGrasp(self, pos, timeout=20):
        self.gotoPos(pos)
        import time
        startTime = time.time()

        # ignore the first reports
        for i in range(10):
            a = self.gotThere()

        while ((time.time() - startTime) < timeout):
            a = self.gotThere()

            if (a):
                print("gotoPosBlocking GotThere")
                return (True)

        print("gotoPosBlocking TIMEOUT")
        return (False)


class HandleBridge(object):
    def __init__(self,
                 arm_portbasename,
                 handlername="HandlerArmBridge",
                 torso=True):
        self.torso = torso

        prename = arm_portbasename
        full_name = prename + "/" + handlername

        self.outp = yarp.BufferedPortBottle()
        outp_name = full_name + "/toBridge_weights"
        self.outp.open(outp_name)

        if self.torso:
            self.torso_port = yarp.BufferedPortBottle()
            torso_port_name = full_name + "/to_torso_cjoints"
            self.torso_port.open(torso_port_name)
            yarp_connect_blocking(torso_port_name,
                                  prename + "/bridge/torso_cjoints:i")

        self.VFW_port = yarp.BufferedPortBottle()
        VFW_port_name = full_name + "/to_VF_weight:o"
        self.VFW_port.open(VFW_port_name)

        self.encoders_port = yarp.BufferedPortBottle()
        encoders_port_name = full_name + "/encoders:i"
        self.encoders_port.open(encoders_port_name)

        yarp_connect_blocking(outp_name, prename + "/bridge/weights")
        yarp_connect_blocking(VFW_port_name, prename + "/vectorField/weight")
        yarp_connect_blocking(prename + "/bridge/encoders", encoders_port_name)

    def read_joint_angles(self):
        bottle = self.encoders_port.read()
        data = map(yarp.Value.asDouble, map(bottle.get, range(bottle.size())))
        return (data)

    def joint_controller(self):
        bout = self.outp.prepare()
        bout.clear()
        bout.addInt(0)
        bout.addInt(0)
        bout.addInt(1)
        bout.addInt(0)
        self.outp.write(True)

    def cartesian_controller(self):
        bout = self.outp.prepare()
        bout.clear()
        bout.addInt(1)
        bout.addInt(1)
        bout.addInt(0)
        bout.addInt(0)
        self.outp.write(True)

    def torso_joints(self, cjoints):
        if self.torso:
            bout = self.torso_port.prepare()
            bout.clear()
            for i in cjoints:
                bout.addInt(i)
            self.torso_port.write(True)
        else:
            print("There's no torso")

    def set_VFW(self, type_of="joint", weights=[1] * 7):  # back compatibility
        print("deprecated, use set_weights instead")
        self.set_weights(type_of, weights)

    def set_weights(self, type_of="joint", weights=[1] * 7):
        bout = self.VFW_port.prepare()
        bout.clear()
        if type_of == "task":
            bout.addString('t')
        else:
            bout.addString('j')
        for w in weights:
            bout.addDouble(w)
        self.VFW_port.write(True)


class HandleJController(object):
    def __init__(self, arm_portbasename, handlername="HandlerArmJoint"):
        prename = arm_portbasename
        full_name = prename + "/" + handlername

        self.outp = yarp.BufferedPortBottle()
        self.inp = yarp.BufferedPortBottle()
        outp_name = full_name + "/to_js"
        inp_name = full_name + "/q"
        self.outp.open(outp_name)
        self.inp.open(inp_name)

        yarp_connect_blocking(outp_name, prename + "/jpctrl/ref")
        yarp_connect_blocking(prename + "/bridge/encoders", inp_name)

    def __del__(self):
        self.inp.close()
        self.outp.close()

    def set_ref_js(self, js, wait=0.0, goal_precision=[]):
        bout = self.outp.prepare()
        bout.clear()
        for i in js:
            bout.addDouble(i)

        self.outp.write(True)
        # return if already in goal or if waiting time is bigger than wait.
        init_time = time.time()
        cur_time = init_time
        difference = array([0.0] * len(js))
        result = False
        if len(goal_precision) == len(js):
            if wait != 0.0:
                goal_precision_np = array(goal_precision)
                while (cur_time - init_time < wait) or wait == -1:
                    # we check for goal finishing
                    bin = self.inp.read(False)
                    if bin:
                        q = array(
                            map(yarp.Value.asDouble,
                                map(bin.get, range(bin.size()))))
                        # compare q vs js, if q is within tolerance, then break

                        difference = js - q
                        if ((((js - goal_precision_np) <= q) *
                             ((js + goal_precision_np) >= q)).all()):

                            result = True
                            break
                    sleep(0.01)
                    cur_time = time.time()
        return (result, difference)


def main():
    return (False)


if __name__ == "__main__":
    main()
