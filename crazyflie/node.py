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

import cflib.crtp
from cflib.crazyflie import Crazyflie
from crazyflie.msg import CFJoy as CFJoyMsg
from crazyflie.msg import CFAltHold as CFAltHoldMsg

# Fixes a warning about logging during CF initialization, Remove if possible
import logging
logging.basicConfig()

#Not yet fixed in Ubuntu 12.04: https://bitbucket.org/pygame/pygame/issue/63
if os.name == 'posix':
    print('Disabling standard output for libraries!')
    stdout = os.dup(1)
    os.dup2(os.open('/dev/null', os.O_WRONLY), 1)
    sys.stdout = os.fdopen(stdout, 'w')

# set SDL to use the dummy NULL video driver,
#   so it doesn't need a windowing system.
os.environ["SDL_VIDEODRIVER"] = "dummy"


class Node():
    """Crazyflie Node"""

    def __init__(self):
        """Initialize the Crazyflie Controller"""
        self._old_hold = None
        self._joy = CFJoyMsg()

        cflib.crtp.init_drivers()

        self._cf = Crazyflie(ro_cache=sys.path[0]+"/cflib/cache",
                             rw_cache=sys.path[1]+"/cache")

        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.param.add_update_callback("imu_sensors", "HMC5883L",
                                           self._althold_detection)

        self._althold_pub = rospy.Publisher('althold', CFAltHoldMsg)
        rospy.Subscriber("cfjoy", CFJoyMsg, self._receive_cmd)

    def connect(self, link_uri, xmode=False):
        """Connect to a Crazyflie on the given link uri"""
        if xmode:
            rospy.loginfo("Client side X-mode: %s", xmode)
            self._cf.commander.set_client_xmode(xmode)

        self._cf.open_link(link_uri)
        rospy.loginfo("Connected to Crazyflie: %s", link_uri)

    def disconnect(self):
        self._cf.close_link()
        rospy.loginfo("Disconnected from Crazyflie: %s", self._cf.link_uri)

    def update_crazyflie(self):
        joy = self._joy
        rospy.logdebug("Sending: roll=%.4f, pitch=%.4f, yaw=%.4f, thrust=%.4f, hold=%i",
                       joy.roll, joy.pitch, joy.yaw, joy.thrust, joy.hold)
        if joy.hold != self._old_hold:
            self._cf.param.set_value("flightmode.althold", str(joy.hold))
            self._old_hold = joy.hold
        self._cf.commander.send_setpoint(joy.roll, joy.pitch, joy.yaw, joy.thrust)

    def _althold_detection(self, available):
        msg = self._create_althold_msg(available)
        self._althold_pub.publish(msg)
        # cb=(lambda name, found:
        #     self._jr.setAltHoldAvailable(eval(found))))

    def _receive_cmd(self, joy):
        rospy.logdebug("Received: roll=%.4f, pitch=%.4f, yaw=%.4f, thrust=%.4f, hold=%i",
                       joy.roll, joy.pitch, joy.yaw, joy.thrust, joy.hold)
        self._joy = joy

    @staticmethod
    def _connection_failed(link, message):
        """Callback for a failed Crazyflie connection"""
        rospy.logerr("Connection failed on %s: %s", link, message)
        rospy.signal_shutdown("Connection failed on %s: %s" % (link, message))
        sys.exit(-1)

    @staticmethod
    def _create_althold_msg(available):
        h = std_msgs.msg.Header()
        h.stamp = rospy.Time.now()
        return CFAltHoldMsg(h, available)


def parse_args():
    """Parse command line arguments"""
    import argparse

    parser = argparse.ArgumentParser(prog="crazyflie_node")
    parser.add_argument("-u", "--uri", type=str,
                        default="radio://0/10/250K", action="store",
                        help="URI to use for connection to the Crazyradio"
                             " dongle, defaults to radio://0/10/250K")
    parser.add_argument("-x", "--x-mode", action="store_true",
                        dest="xmode",
                        help="Enable client-side X-mode")
    (args, unused) = parser.parse_known_args()

    return args


def main():
    """Main Crazyflie node"""

    args = parse_args()
    rospy.init_node('crazyflie_node')

    node = Node()

    try:
        node.connect(link_uri=args.uri)
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            node.update_crazyflie()
            r.sleep()
    except rospy.ROSInterruptException:
        pass
    finally:
        node.disconnect()
