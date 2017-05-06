#!/usr/bin/env python
# Copyright (c) 2009 Technische Universitaet Muenchen, Informatik Lehrstuhl IX.
# Authors: Alexis Maldonado Herrera <maldonad at cs.tum.edu> Federico Ruiz-Ugalde <memeruiz@gmail.com>
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

import yarp, time
from numpy import array
from numpy import pi
from time import sleep
from arcospyu.yarp_tools.yarp_comm_helpers import yarp_connect_blocking, new_port
from arcospyu.kdl_helpers.kdl_helpers import frame_to_list
class HandleArm(object):
    
    def __init__(self,arm_portbasename, handlername="/HandlerArm"):
        import imp
        prename=arm_portbasename
        full_name=prename+ handlername

        self.outp=yarp.BufferedPortBottle()
        outp_name=full_name +"/toObjectFeeder"
        self.outp.open(outp_name)

        self.stiffness_port=yarp.BufferedPortBottle()
        stiffness_port_name=full_name +"/stiffness"
        self.stiffness_port.open(stiffness_port_name)

        self.goaldistp=yarp.BufferedPortBottle()
        goaldistp_name=full_name + "/fromGoalDistance"
        self.goaldistp.open(goaldistp_name)
        #self.goaldistp.setStrict()

        self.posep=yarp.BufferedPortBottle()
        posep_name=full_name+"/pose:i"
        self.posep.open(posep_name)

        yarp_connect_blocking(outp_name,prename+"/ofeeder/object")
        yarp_connect_blocking(stiffness_port_name,prename+"/robot/stiffness")
        yarp_connect_blocking(prename+"/dmonitor/distOut" , goaldistp_name)
        yarp_connect_blocking(prename+"/vectorField/pose", posep_name)


        self.current_frame=[1.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0]
        self.current_slowdown_distance=0.1
        
        self.toolp = new_port(full_name + "/toToolin", 'out', "%s/vectorField/tool"%(prename))
        self.toolp.setStrict(True)

        self.goal_threshold=0.01

    def __del__(self):
        self.goaldistp.close()
        self.outp.close()
        
    def setTool(self, toolframe):
        ''' Receives a PyKDL Frame and sends it to VF as a tool
        '''

        print "Setting the tool of the arm to: ", toolframe
#        pp_F(toolframe)
        
        bout = self.toolp.prepare() 
        bout.clear()
        tool_list = frame_to_list( toolframe )
        for i in tool_list:
            bout.addDouble(i)
        self.toolp.write(True)

        
    def set_stiffness(self,stiffness):
        bottle=self.stiffness_port.prepare()
        bottle.clear()
        for i in stiffness:
            bottle.addDouble(i)
        self.stiffness_port.write(True)
        
    def sendFrame(self):
        bout=self.outp.prepare()
        bout.clear()
        bout.addString("set")
        bout.addString("goal")
        frame_list=bout.addList()
        for i in self.current_frame:
            frame_list.addDouble(i)
        frame_list.addDouble(self.current_slowdown_distance)

        self.outp.write(True)

    def gotoPos(self,pos):
        print "Going here: ",
        print pos

        self.current_frame[3]=pos[0]
        self.current_frame[7]=pos[1]
        self.current_frame[11]=pos[2]

        self.sendFrame()


    def setOrient(self,orient):
        print "Setting orientation: ",
        print orient

        for i in range(3):
            for j in range(3):
                self.current_frame[j+4*i]=orient[j+i*3]

        self.sendFrame()

    def getPose(self, blocking=True):
        pose_b=self.posep.read(blocking)
        pose=map(yarp.Value.asDouble,map(pose_b.get,range(pose_b.size())))
        return(pose)

    def gotoPose(self,pos,orient):
        
        self.current_frame[3]=pos[0]
        self.current_frame[7]=pos[1]
        self.current_frame[11]=pos[2]

        for i in range(3):
            for j in range(3):
                self.current_frame[j+4*i]=orient[j+i*3]
        
        self.sendFrame()

    def gotoFrame(self,frame,wait=10.0,goal_precision=[]):
        '''frame: 16 value array
        wait: in seconds
        goal_precision: list of two values: trans precision and rot precision
        '''
        for i in range(len(frame)):
            self.current_frame[i]=frame[i]
        self.sendFrame()
        init_time=time.time()
        cur_time=init_time
        difference=array([0.0,0.0])
        result=False
        pending=self.goaldistp.getPendingReads()
        for i in xrange(pending):
            print "GOAL dist: ", self.goaldistp.read(False)
            #while not self.goaldistp.read(False):
            #    print "SDFASDFASDF: ", i
            #    sleep(0.01)
        if len(goal_precision)==2:
            if wait>0.0:
                goal_precision_np=array(goal_precision)
                first_read=True
                while cur_time-init_time<wait:
                    #we check for goal finishing


                    bin=self.goaldistp.read(False)
                    #print "cycling" 
                    if bin and not first_read:
                        #print " Received something"
                        for i in xrange(bin.size()):
                            l=bin.get(i).asList()
                            if (l.get(0).asInt()==0):
                                pos_dist=l.get(1).asDouble()
                                orient_dist=(l.get(2).asDouble())*pi/180.0
                        #compare q vs js, if q is within tolerance, then break
                        print "Cartesian distance:" , pos_dist, orient_dist
                        difference=array([pos_dist,orient_dist])
                        if pos_dist<goal_precision[0] and orient_dist<goal_precision[1]:
                            result=True
                            break
                    first_read=False
                    sleep(0.01)
                    cur_time=time.time()
        return(result,difference)
        

    
    def gotThere(self):
        bin=self.goaldistp.read(True)
        dist=1000.0

        if (bin):
            for i in range(bin.size()):
                l=bin.get(i).asList()
                if (l.get(0).asInt()==0):
                    dist=l.get(1).asDouble()
                    #print "Got dist=%f" %(dist)
                    
            
            done=( dist < self.goal_threshold)
            if (done):
                return True
        return False



        
    def gotoPosBlocking(self,pos,timeout=20):
        self.gotoPos(pos)
        import time
        startTime=time.time()
        
        #ignore the first reports
        for i in range(10):
            a=self.gotThere()

        while((time.time()-startTime) < timeout):
            a=self.gotThere()

            if (a):
                print "gotoPosBlocking GotThere"
                return(True)

        print "gotoPosBlocking TIMEOUT"
        return(False)

    def gotoPosBlockingGrasp(self,pos,timeout=20):
        self.gotoPos(pos)
        import time
        startTime=time.time()
        
        #ignore the first reports
        for i in range(10):
            a=self.gotThere()

        while((time.time()-startTime) < timeout):
            a=self.gotThere()

            if (a):
                print "gotoPosBlocking GotThere"
                return(True)

        print "gotoPosBlocking TIMEOUT"
        return(False)



    def __del__(self):
        self.outp.close()



#from thing_tools import HandleBridge

class HandleBridge(object):
    
    def __init__(self,arm_portbasename, handlername="HandlerArmBridge",torso=True):
        import imp
        self.torso=torso
       
        prename=arm_portbasename
        full_name=prename+ "/" + handlername

        self.outp=yarp.BufferedPortBottle()
        outp_name=full_name +"/toBridge_weights"
        self.outp.open(outp_name)
        
        if self.torso:
            self.torso_port=yarp.BufferedPortBottle()
            torso_port_name=full_name + "/to_torso_cjoints"
            self.torso_port.open(torso_port_name)
            yarp_connect_blocking(torso_port_name,prename+"/bridge/torso_cjoints:i")

        self.VFW_port=yarp.BufferedPortBottle()
        VFW_port_name=full_name + "/to_VF_weight:o"
        self.VFW_port.open(VFW_port_name)

        self.encoders_port=yarp.BufferedPortBottle()
        encoders_port_name=full_name+"/encoders:i"
        self.encoders_port.open(encoders_port_name)

        yarp_connect_blocking(outp_name,prename+"/bridge/weights")
        yarp_connect_blocking(VFW_port_name,prename+"/vectorField/weight")
        yarp_connect_blocking(prename+"/bridge/encoders",encoders_port_name)


    def read_joint_angles(self):
        bottle=self.encoders_port.read()
        data=map(yarp.Value.asDouble,map(bottle.get,xrange(bottle.size())))
        return(data)

    def joint_controller(self):
        bout=self.outp.prepare()
        bout.clear()
        bout.addInt(0)
        bout.addInt(0)
        bout.addInt(1)
        bout.addInt(0)
        self.outp.write(True)

    def cartesian_controller(self):
        bout=self.outp.prepare()
        bout.clear()
        bout.addInt(1)
        bout.addInt(1)
        bout.addInt(0)
        bout.addInt(0)
        self.outp.write(True)
        

    def torso_joints(self,cjoints):
        if self.torso:
            bout=self.torso_port.prepare()
            bout.clear()
            for i in cjoints:
                bout.addInt(i)
            self.torso_port.write(True)
        else:
            print "There's no torso"

    def set_VFW(self, type_of="joint", weights=[1]*7): #back compatibility
        print "deprecated, use set_weights instead"
        self.set_weights(type_of, weights)
            
    def set_weights(self,type_of="joint",weights=[1]*7):
        bout=self.VFW_port.prepare()
        bout.clear()
        if type_of=="task":
            bout.addString('t')
        else:
            bout.addString('j')
        for w in weights:
            bout.addDouble(w)
        self.VFW_port.write(True)

class HandleJController(object):

    def __init__(self,arm_portbasename, handlername="HandlerArmJoint"):
        import imp

        prename=arm_portbasename
        full_name=prename+ "/" + handlername

        self.outp=yarp.BufferedPortBottle()
        self.inp=yarp.BufferedPortBottle()
        outp_name=full_name +"/to_js"
        inp_name=full_name + "/q"
        self.outp.open(outp_name)
        self.inp.open(inp_name)

        yarp_connect_blocking(outp_name,prename+"/jpctrl/ref")
        yarp_connect_blocking(prename+"/bridge/encoders",inp_name)

    def __del__(self):
        self.inp.close()
        self.outp.close()

    def set_ref_js(self,js,wait=0.0,goal_precision=[]):
        bout=self.outp.prepare()
        bout.clear()
        for i in js:
            bout.addDouble(i)
        #print "HandleJController: sending joints: ",
        #print bout.toString()
        self.outp.write(True)
        #return if already in goal or if waiting time is bigger than wait.
        init_time=time.time()
        cur_time=init_time
        difference=array([0.0]*len(js))
        result=False
        if len(goal_precision)==len(js):
            if wait!=0.0:
                goal_precision_np=array(goal_precision)
                while (cur_time-init_time<wait) or wait==-1:
                    #we check for goal finishing
                    bin=self.inp.read(False)
                    if bin:
                        q=array(map(yarp.Value.asDouble,map(bin.get, range(bin.size()))))
                        #compare q vs js, if q is within tolerance, then break
                        
                        difference=js-q
                        #print "Difference", difference, (js-goal_precision_np)<=q, (js+goal_precision_np)>=q
                        if (((js-goal_precision_np)<=q)*((js+goal_precision_np)>=q)).all():
                            result=True
                            break
                    sleep(0.01)
                    cur_time=time.time()
        return(result,difference)


def main():
    return(False)



if __name__ == "__main__":
    main()


