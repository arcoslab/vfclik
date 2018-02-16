#!/usr/bin/env python
# Copyright (c) 2009 Technische Universitaet Muenchen, Informatik Lehrstuhl IX.
# Author: Ingo Kresse <kresse at in.tum.de>
#         Federico Ruiz Ugalde <memeruiz at gmail.com>
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

import time
import sys
import os
import yarp
import optparse

from arcospyu.config_parser import ConfigFileParser

# TODO; recover after seeing NaNs

basename = '/mixer'


class CommandMixer:
    def __init__(self, ports, weight_port, n, guard_time, weights):
        self.nChannels = n
        self.ports = ports
        self.weight_port = weight_port
        if len(ports) != len(weights):
            print('wrong number of initial weights. Resetting to zeros.')
            self.weights = [0.0] * len(ports)
        else:
            self.weights = weights
        self.guard_time = guard_time
        self.last_command = [[0.0] * n] * len(self.ports)
        self.last_command_time = [time.time()] * len(self.ports)

    def read(self):
        # update weights
        if (self.weight_port):
            bottle = self.weight_port.read(False)
            if (bottle):
                l_bottle = min(bottle.size(), len(self.ports))
                for i in range(l_bottle):
                    self.weights[i] = bottle.get(i).asDouble()

        # update inputs
        for p in range(len(self.ports)):
            bottle = self.ports[p].read(False)
            if (bottle and bottle.size() == self.nChannels):
                # update port data
                self.last_command_time[p] = time.time()
                self.last_command[p] = [
                    bottle.get(i).asDouble() for i in range(self.nChannels)
                ]
            elif (time.time() - self.last_command_time[p] > self.guard_time):
                # reset port data
                self.last_command[p] = [0.0] * self.nChannels
            elif bottle:
                # TODO: raise
                print('wrong length for data bottle')

        import math
        for i in range(len(self.last_command)):
            for j in range(len(self.last_command[i])):
                if math.isnan(self.last_command[i][j]):
                    print('nan: %d - %d' % (i, j))

        # compute weighted sum of all port data
        result = [0.0] * self.nChannels
        for (v, w) in zip(self.last_command, self.weights):
            for i in range(len(v)):
                result[i] += v[i] * w
        return result


def ut_writebottle(port, list):
    bottle = port.prepare()
    bottle.clear()
    for i in list:
        bottle.addDouble(i)
    port.write()


def main():
    parser = optparse.OptionParser("usage: %prog [options]")
    parser.add_option("-r", "--robot", dest="robot",
                      default="lwr", type="string",
                      help="robot name")
    parser.add_option("-i", "--instance", dest="instance",
                      default="right", type="string",
                      help="instance")
    parser.add_option("-d", "--config_dir", dest="config_dir",
                      default="../config_data/lwr/",
                      type="string",
                      help="config data directory")

    (options, args) = parser.parse_args(sys.argv[1:])
    config_filename = (options.config_dir + "config-" +
                       options.robot + "-" + options.instance +
                       ".py")

    if not os.path.exists(config_filename):
        print("config file not found")
        sys.exit(-1)

    config_parser = ConfigFileParser(config_filename)
    options, args, config = config_parser.get_all()

    in1 = yarp.BufferedPortBottle()
    in2 = yarp.BufferedPortBottle()
    out = yarp.BufferedPortBottle()
    ctr = yarp.BufferedPortBottle()

    in1.open(basename + '/in1')
    in2.open(basename + '/in2')
    out.open(basename + '/out')
    ctr.open(basename + '/control')

    mixer = CommandMixer([in1, in2], ctr, config.nJoints, 2)

    while (True):
        joints = mixer.read()
        ut_writebottle(out, joints)
        time.sleep(0.08)


if __name__ == "__main__":
    main()
