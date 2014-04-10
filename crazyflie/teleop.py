#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  Copyright (C) 2013 Bitcraze AB
#
#  Crazyflie Nano Quadcopter Client
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.

#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
#  MA  02110-1301, USA.

"""
Headless client for the Crazyflie.
"""

import sys
import os
import rospy
import std_msgs.msg

from cfclient.utils.input import JoystickReader
from crazyflie.msg import CFJoy as CFJoyMsg

#Not yet fixed in Ubuntu 12.04: https://bitbucket.org/pygame/pygame/issue/63
if os.name == 'posix':
    print('Disabling standard output for libraries!')
    stdout = os.dup(1)
    os.dup2(os.open('/dev/null', os.O_WRONLY), 1)
    sys.stdout = os.fdopen(stdout, 'w')

#TODO: necessary?
# set SDL to use the dummy NULL video driver,
#   so it doesn't need a windowing system.
os.environ["SDL_VIDEODRIVER"] = "dummy"


class ControllerNotFound(Exception):
    pass


class Teleop():
    """Crazyflie Joystick Controller"""

    def __init__(self):
        """Initialize the joystick reader"""
        self._althold = False

        self._jr = JoystickReader(do_device_discovery=False)

        self._jr.device_error.add_callback(self._input_dev_error)
        self._jr.althold_updated.add_callback(self._althold_updated)
        self._jr.input_updated.add_callback(self._input_updated)

        self._command_pub = rospy.Publisher('cfjoy', CFJoyMsg)

    def start_controller(self, input_config, input_device=0):
        """Set up the device reader"""

        # Frequency of joystick messages is handled by joystick driver
        # TODO: make that configurable or write handler for ROS joystick
        # Frequency of sending to flie is handled by crazyflie_node

        #TODO: connect to a config message from the flie?
        self._jr.setAltHoldAvailable(False)

        devs = self._jr.getAvailableDevices()
        if len(devs) < 1:
            raise ControllerNotFound("Device count: %d" % len(devs))

        dev_name = devs[input_device]["name"]
        rospy.loginfo("Using [%s] for input with [%s]", dev_name, input_config)
        self._jr.start_input(dev_name, input_config)

    def controller_connected(self):
        """ Return True if a controller is connected"""
        return True if (len(self._jr.getAvailableDevices()) > 0) else False

    def list_controllers(self):
        """List the available controllers"""
        for dev in self._jr.getAvailableDevices():
            print("Controller #%s: %s" % (dev["id"], dev["name"]))

    def _althold_updated(self, enabled):
        #TODO: enabled is a string! Stop that!
        self._althold = (enabled == 'True')
        rospy.logdebug("Althold set: enabled=%i", self._althold)

    def _input_updated(self, roll, pitch, yaw, thrust):
        #Sent from joystickreader:
        #self.input_updated.call(trimmed_roll, trimmed_pitch, yaw, thrust)
        msg = self._create_joy_msg(roll, pitch, yaw, thrust, self._althold);
        self._command_pub.publish(msg);
        rospy.logdebug("Updated: r=%.4f, p=%.4f, y=%.4f, t=%.4f, h=%i",
                       roll, pitch, yaw, thrust, self._althold)

    @staticmethod
    def _input_dev_error(message):
        """Callback for an input device error"""
        rospy.logerr("Error when reading device: %s", message)
        rospy.signal_shutdown("Error when reading device: %s" % message)
        sys.exit(-1)

    @staticmethod
    def _create_joy_msg(roll, pitch, yaw, thrust, althold):
        h = std_msgs.msg.Header()
        h.stamp = rospy.Time.now()
        return CFJoyMsg(h, roll, pitch, yaw, thrust, althold)


def parse_args():
    """Parse command line arguments"""
    import argparse

    parser = argparse.ArgumentParser(prog="crazyflie_teleop")
    parser.add_argument("-i", "--input", type=str,
                        default="PS3_Mode_2", action="store",
                        help="Input mapping to use for the controller,"
                             "defaults to PS3_Mode_2")
    parser.add_argument("-c", "--controller", type=int,
                        default=0, action="store",
                        help="Use controller with specified id,"
                             "defaults to 0")
    parser.add_argument("--controllers", action="store_true",
                        dest="list_controllers",
                        help="Only display available controllers and exit")
    (args, unused) = parser.parse_known_args()

    return args


def main():
    """Main Crazyflie teleop"""
    args = parse_args()
    rospy.init_node('crazyflie_teleop')

    teleop = Teleop()

    if args.list_controllers:
        teleop.list_controllers()
    else:
        try:
            teleop.start_controller(input_config=args.input, input_device=args.controller)
            rospy.spin()
        except rospy.ROSInterruptException:
            pass
        except ControllerNotFound:
            #TODO: hate the redundancy
            rospy.logerr("No input-device connected, exiting!")
            rospy.signal_shutdown("No input-device connected, exiting!")
