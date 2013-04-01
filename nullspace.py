#!/usr/bin/env python
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

from numpy.linalg import svd,norm
from numpy import sum,where
from numpy import *

import scipy as Sci
import scipy.linalg

import yarp

import sys
from arcospyu.config_parser import ConfigFileParser
config_parser=ConfigFileParser(sys.argv)
options, args, config = config_parser.get_all()
from arcospyu import lafik

import signal
def signal_handler(sig, frame):
    print "Terminating ", __file__
    global stop
    stop=True

stop=False
signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)

yarp.Network.init()


def change_ps_name(name):
    """Change process name as used by ps(1) and killall(1)."""
    try:
        import ctypes
        libc = ctypes.CDLL('libc.so.6')
        libc.prctl(15, name, 0, 0, 0)
    except:
        pass    
    
change_ps_name('nullspace.py')



rob = lafik.Lafik(config)

gain = 0.5

nJoints = config.nJoints

def matrixrank(A,tol=1e-8):
    s = svd(A,compute_uv=0)
    return sum( where( s>tol, 1, 0 ) )


# return all nullspace vectors, given the robot jacobian and the projection into the task space that we care about
# (a base of the nullspace of the function f: x -> P*J*x)
def restrict(P, J):
  global nJoints
  pJ = P*J
  pJt = linalg.pinv(pJ)
  return (eye(nJoints) - pJt*pJ)

def sign(n):
  if n < 0:
    return -1
  elif n == 0:
    return 0
  else:
    return 1

sig=[1]*nJoints
lastvec=mat(zeros((nJoints,nJoints)))

def nullspace(P, J):
  global sig,lastvec
  B = restrict(P, J)
  u,s,vh = svd(B.T)
  i=0
  while s[i] >= 1e-8:
    ## print sig[i]
    ## print u[:, i].T
    if norm(sig[i]*u[:, i] - lastvec[:, i]) > norm(sig[i]*u[:, i]+lastvec[:, i]):
      sig[i] = -sig[i]
    u[:, i] = u[:, i]*sig[i]
    lastvec[:, i] = u[:, i]
    i += 1
  #print "rank=%d" % ( i )
  return u[:, 0:i].T

def move_in_nullspace(P,J,control):
  global nJoints
  ns = nullspace(P,J)
  #### ns = restrict(P,J)[0:4]
  n = min(nJoints,len(control), ns. shape[0])
  qdot = mat(zeros((nJoints)))
  for i in range(n):
    qdot += ns[i,:]*control[i]
  return [qdot[0,i] for i in range(nJoints)]

def check_limits(q, qdot, limits):
  scale = 0.3
  margin = 0.0
  n = len(limits)
  for i in range(n):
    d =  q[i] + scale*qdot[i]
    if d < limits[i][0]+margin or d > limits[i][1]-margin:
      print "limit %d: (q=%f qdot=%f l=[%f .. %f])" % (i, q[i], qdot[i], limits[i][0], limits[i][1])
      return [0]*n
  return qdot

    

def main():
  global gain
  #P = matrix('1 0 0 0 0 0; 0 1 0 0 0 0; 0 0 1 0 0 0; 0 0 0 1 0 0; 0 0 0 0 1 0; 0 0 0 0 0 0')
  P = mat(eye(6))
  control = [0]*4

  qin_port = yarp.BufferedPortBottle()
  qdotout_port = yarp.BufferedPortBottle()
  control_port = yarp.BufferedPortBottle() # expects four-float-bottles

  basename=config.robotarm_portbasename+"/nullspace"
  qin_port.open(basename+'/qin')
  qdotout_port.open(basename+'/qdotout')
  control_port.open(basename+'/control')

#  limits = rob.get_limits()

  while not stop:
    bin = qin_port.read(False)
    if bin:
      q = [bin.get(i).asDouble() for i in range(bin.size())]
      for i in range(bin.size()):
        rob.jnt_pos[i] = bin.get(i).asDouble()
      limits=rob.get_limits()
    
      bcontrol = control_port.read(False)
      if bcontrol != None:
        control = [bcontrol.get(i).asDouble() for i in range(bcontrol.size())]
    
      J = matrix(rob.jac_list())
      qdot = move_in_nullspace(P,J,control)

      ##print "tsk-disturbance:"
      ##print J*mat(qdot).T
    
      qdot = check_limits(q, qdot, limits)

      bout = qdotout_port.prepare()
      bout.clear()
      for i in qdot:
        bout.addDouble(i*gain)
      qdotout_port.write()

    else:
        yarp.Time_delay(0.01)

  qin_port.close()
  qdotout_port.close()
  control_port.close()


if __name__ == "__main__":
    main()

