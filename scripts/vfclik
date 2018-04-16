#!/usr/bin/env python
# Copyright (c) 2013 Federico Ruiz Ugalde
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
import os
from time import sleep
from arcospyu.dprint import dprint
from arcospyu.pmanager import PManager

# signal handling
import signal


def main():
    def signal_handler(sig, frame):
        pmanager.stop()
        exit(1)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    # parsing args
    import optparse
    parser = optparse.OptionParser("usage: %prog [options]")
    parser.add_option(
        "-r",
        "--robot",
        dest="robot",
        default="lwr",
        type="string",
        help="robot name")
    parser.add_option(
        "-i",
        "--instance",
        dest="instance",
        default="right",
        type="string",
        help="instance")
    parser.add_option(
        "-d",
        "--config_dir",
        dest="config_dir",
        default="../config_data/lwr/",
        type="string",
        help="config data directory")
    parser.add_option(
        "-s",
        "--simulation",
        action="store_true",
        dest="sim",
        default=False,
        help="Simulation")
    (options, args) = parser.parse_args(sys.argv[1:])
    config_filename = (options.config_dir + "config-" + options.robot + "-" +
                       options.instance + ".py")
    print("Config filename: ", config_filename)
    if not os.path.exists(config_filename):
        print("Config filename: ", config_filename, " not found, exiting")
        sys.exit(-1)

    # starting yarp modules
    processes_args = [
        ["vf", "-c", config_filename],
        ["object_feeder", "-c", config_filename],
        ["monitor_distance", "-c", config_filename],
        ["nullspace", "-c", config_filename],
        ["joint_p_controller", "-c", config_filename],
        ["debug_jointlimits", "-c", config_filename]
    ]
    if options.sim:
        processes_args += [[
            "joint_sim", "-c",
            os.getcwd() + "/" + config_filename
        ]]
        processes_args += [["bridge", "-c", config_filename, "-s"]]
    else:
        processes_args += [["bridge", "-c", config_filename]]

    dprint("Starting processes")
    pmanager = PManager(processes_args)
    pmanager.start()
    sleep(1)

    dprint("Looping")
    pmanager.monitor()
    exit()


if __name__ == "__main__":
    main()
