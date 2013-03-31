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

def main():
    import sys, os
    from arcospyu.dprint import iprint, dprint, eprint, dcprint
    #from arcospyu import dprint as dp
    #sys.path.append("../../arcospyu/arcospyu/dprint")
    #import dprint
    #d=dprint.Dprint()
    #dprint=d.dprint
    dprint("testt")
    #exit()
    from arcospyu.mypopen import MyPopen
    from arcospyu.pmanager import PManager

    #signal handling
    import signal
    def signal_handler(sig, frame):
        #dprint("Sending signal ", sig, " to subprocesses")
        #[process.send_signal(signal.SIGTERM) for process in processes]
        #dprint("Waiting processes to terminate")
        #for process in processes:
        #    print "Waiting for process: ", process.args[0], " to terminate"
        #    process.wait_and_kill(3)
        #    print "Process: ", process.args[0], " terminated"
        #print "Processes terminated, exiting"
        pmanager.stop()
        exit(1)
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    #parsing args
    import optparse
    parser=optparse.OptionParser("usage: %prog [options]")
    parser.add_option("-r", "--robot", dest="robot", default="lwr", type="string", help="robot name")
    parser.add_option("-i", "--instance", dest="instance", default="right", type="string", help="instance")
    parser.add_option("-d", "--config_dir", dest="config_dir", default="../config_data/lwr/", type="string", help="config data directory")
    parser.add_option("-s", "--simulation", action="store_true", dest="sim", default=False,help="Simulation")
    (options, args)=parser.parse_args(sys.argv[1:])
    config_filename=options.config_dir+"config-"+options.robot+"-"+options.instance+".py"
    print "Config filename: ", config_filename
    if not os.path.exists(config_filename):
        print "Config filename: ", config_filename, " not found, exiting"
        sys.exit(-1)
    yarp_base_portname="/"+options.robot+"/"+options.instance

    #starting yarp modules
    processes_args=[["./vf.py", "-c", config_filename],
                    ["./object_feeder.py", "-c", config_filename],
                    ["./monitor_distance.py", "-c", config_filename],
                    ["./bridge.py", "-c", config_filename, "-s"],
                    ["./nullspace.py", "-c", config_filename],
                    ["./joint_p_controller.py", "-c", config_filename],
                    ["./debug_jointlimits.py", "-c", config_filename],
                    ["../roboview/roboview", config_filename, yarp_base_portname],
                    ["../yarp_tools/bar_vis.py", yarp_base_portname]]
    if options.sim:
        processes_args+=[["../models/robot/joint_sim.py", "-c", os.getcwd()+"/"+config_filename]]

    print processes_args

    dprint("testetstetest")
    pmanager=PManager(processes_args)
    pmanager.start()
    #processes=[]
    #for process_args in processes_args:
    #    processes.append(MyPopen(process_args))
    #    processes[-1].args=process_args

    pmanager.monitor()
    exit()

    from time import sleep

    while True:
        print "looping"
        sleep(1)
        pass

if __name__=="__main__":
    main()
