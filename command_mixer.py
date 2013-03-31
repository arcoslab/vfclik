#!/usr/bin/python

import time, sys
import yarp
#sys.path.append("../python/config_parser")
#from config_parser import ConfigFileParser
#config_parser=ConfigFileParser(sys.argv)
#config_parser.parser.add_option("-s", "--simulation", action="store_true", dest="sim", default=False,help="Simulation")
#options, args, config = config_parser.get_all()

#TODO; recover after seeing NaNs

basename = '/mixer'

class CommandMixer:
  def __init__(self, ports, weight_port, n, guard_time,weights):
    self.nChannels = n
    self.ports = ports
    self.weight_port = weight_port
    if len(ports) != len(weights):
      print 'wrong number of initial weights. Resetting to zeros.'
      self.weights = [0.0]*len(ports)
    else:
      self.weights = weights
    self.guard_time = guard_time
    self.last_command = [[0.0] * n] * len(self.ports)
    self.last_command_time = [time.time()] * len(self.ports)

  def read(self):
    # update weights
    if(self.weight_port):
      bottle = self.weight_port.read(False)
      if(bottle):
        l = min(bottle.size(), len(self.ports))
        for i in range(l):
          self.weights[i] = bottle.get(i).asDouble()

    # update inputs 
    for p in range(len(self.ports)):
      bottle = self.ports[p].read(False)
      if(bottle and bottle.size() == self.nChannels):
        # update port data
        self.last_command_time[p] = time.time()
        self.last_command[p] = [bottle.get(i).asDouble() for i in range(self.nChannels)]
      elif(time.time() - self.last_command_time[p] > self.guard_time):
        # reset port data
        self.last_command[p] = [0.0] * self.nChannels
      elif bottle:
        print 'wrong length for data bottle'

    import math
    for i in range(len(self.last_command)):
      for j in range(len(self.last_command[i])):
        if math.isnan(self.last_command[i][j]):
          print 'nan: %d - %d' % (i, j)

    # compute weighted sum of all port data
    result = [0.0] * self.nChannels
    for (v,w) in zip(self.last_command, self.weights):
      for i in range(len(v)):
        result[i] += v[i]*w
    return result


def ut_writebottle(port, list):
  bottle = port.prepare()
  bottle.clear()
  for i in list:
    bottle.addDouble(i)
  port.write()

  

def main():

  in1 = yarp.BufferedPortBottle()
  in2 = yarp.BufferedPortBottle()
  out = yarp.BufferedPortBottle()
  ctr = yarp.BufferedPortBottle()

  in1.open(basename+'/in1')
  in2.open(basename+'/in2')
  out.open(basename+'/out')
  ctr.open(basename+'/control')

  mixer = CommandMixer([in1, in2], ctr, config.nJoints, 2)

  while(True):
    joints = mixer.read()
    ut_writebottle(out, joints)
    time.sleep(0.08)



if __name__ == "__main__":
    main()

