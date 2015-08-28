#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2011-2013 Bitcraze AB
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
Used for sending control setpoints to the Crazyflie
"""

__author__ = 'Bitcraze AB'
__all__ = ['Commander']

from cflib.crtp.crtpstack import CRTPPacket, CRTPPort
import struct
import math


class Commander():
    """
    Used for sending control setpoints to the Crazyflie
    """

    def __init__(self, crazyflie=None):
        """
        Initialize the commander object. By default the commander is in
        +-mode (not x-mode).
        """
        self._cf = crazyflie
        self._x_mode = False
        
        self._compass_mode = False
        self._compass_heading = 0
        self._heading_adjustment = 0
        self._heading_lock_value = 0
        self._heading_lock_mode = False
        
        self._mission_enable = False
        self._mission_mode = False
        self._mission_heading_value = 0
        self._mission_distance_value = 0

    def set_client_xmode(self, enabled):
        """
        Enable/disable the client side X-mode. When enabled this recalculates
        the setpoints before sending them to the Crazyflie.
        """
        self._x_mode = enabled
    
    def set_client_compass_mode(self, enabled):
        """Enable/disable the client side compass-mode. When enabled this recalculates the setpoints before sending them to the Crazyflie."""
        self._compass_mode = enabled
    
    def set_client_compass_heading(self, heading):
        """Set the client side compass heading,"""
        self._compass_heading = heading
    
    def set_heading_lock_mode(self, state):
        """Set the client side headng lock mode"""
        self._heading_lock_mode = state
    
    def set_heading_lock_value(self, value):
        """Set the client side headng lock value"""
        self._heading_lock_value = value
    
    def set_mission_enable(self, value):
        """Set the client side mission enable value"""
        self._mission_enable = value
    
    def set_mission_heading_value(self, value):
        """Set the client side headng lock value"""
        self._mission_heading_value = value
    
    def set_mission_distance_value(self, value):
        """Set the client side headng lock value"""
        self._mission_distance_value = value

    def send_setpoint(self, roll, pitch, yaw, thrust):
        """
        Send a new control setpoint for roll/pitch/yaw/thust to the copter

        The arguments roll/pitch/yaw/trust is the new setpoints that should
        be sent to the copter
        """
        
        if self._heading_lock_mode:

            yaw = self._heading_lock_value - self._compass_heading
            
            if yaw > 180:
                yaw += -360
            elif yaw < -180:
                yaw += 360
            
            yaw = yaw/2
            
            print str(self._compass_heading) + " " + str(self._heading_lock_value) + " " + str(yaw)
                
                
        if self._mission_enable:

            yaw = self._mission_heading_value - self._compass_heading
                        
            if yaw > 180:
                yaw += -360
            elif yaw < -180:
                yaw += 360
        
        if self._x_mode:
            roll = 0.707 * (roll - pitch)
            pitch = 0.707 * (roll + pitch)

        pk = CRTPPacket()
        pk.port = CRTPPort.COMMANDER
        pk.data = struct.pack('<fffH', roll, -pitch, yaw, thrust)
        self._cf.send_packet(pk)
