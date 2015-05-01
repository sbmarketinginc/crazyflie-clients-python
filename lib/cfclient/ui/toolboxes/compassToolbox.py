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
#  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.

"""
A detachable toolbox
"""

__author__ = 'Bitcraze AB'
__all__ = ['compassToolbox']
import sys, time
from PyQt4 import QtCore, QtGui, uic
from PyQt4.QtCore import Qt, pyqtSlot, pyqtSignal
from cfclient.ui.widgets.compass import Compass
from cfclient.utils.config import Config
import logging
logger = logging.getLogger(__name__)
from cfclient.utils.logconfigreader import LogVariable, LogConfig
import math
from math import pi, sin, cos
import numpy
from numpy import linalg
import string
console_class = uic.loadUiType(sys.path[0] + "/cfclient/ui/toolboxes/compassToolbox.ui")[0]


class compassToolbox(QtGui.QWidget, console_class):
    """Compass toolbox"""
    update = pyqtSignal(str)
    _mag_data_signal = pyqtSignal(int, object, object)
    _motor_data_signal = pyqtSignal(int, object, object)
    _imu_data_signal = pyqtSignal(int, object, object)
    connectionFinishedSignal = pyqtSignal(str)
    disconnectedSignal = pyqtSignal(str)

    def __init__(self, helper, *args):
	# TODO Check for compass throw error message if not found
        super(compassToolbox, self).__init__(*args)
        self.setupUi(self)
        self.helper = helper
	#Setup Compass Widget
        self.compass = Compass()
        self.gridLayout_3.addWidget(self.compass, 2, 0)
	# Connect UI signals
        self.headingLockRadioButton.clicked.connect(self.headingLockRadioButtonClicked)
        self.headingLockDial.valueChanged.connect(self.headingLockDialValueChanged)
        self.headingLockSpinBox.valueChanged.connect(self.headingLockSpinBoxValueChanged)
        self.headingLockCurrentPushButton.clicked.connect(self.headingLockCurrentPushButtonClicked)
        self.intelligentOrientationRadioButtton.clicked.connect(self.intelligentOrientationRadioButtonClicked)
        self.intelligentOrientationDial.valueChanged.connect(self.intelligentOrientationDialValueChanged)
        self.intelligentOrientationSpinBox.valueChanged.connect(self.intelligentOrientationSpinBoxValueChanged)
        self.intelligentOrientationCurrentPushButton.clicked.connect(self.intelligentOrientationCurrentPushButtonClicked)
        self.calibratePushButton.clicked.connect(self.calibratePushButtonClicked)
        self.helper.cf.connected.add_callback(self.connectionFinishedSignal.emit)
        self.helper.cf.disconnected.add_callback(self.disconnectedSignal.emit)
 	#Incoming Signals
        self._mag_data_signal.connect(self._mag_data_received)
        self._motor_data_signal.connect(self._motor_data_received)
        self._imu_data_signal.connect(self._imu_data_received)
        self.disconnectedSignal.connect(self.disconnected)
        self.connectionFinishedSignal.connect(self.connected)
    # Initialise variables
        self.connection_status = 0
        self._heading_lock_mode = False
        self._heading_lock_value = 0
        self._intelligent_orientation_mode = False
        self._intelligent_orientation_value = 0
        self._current_heading = 0
    # Calibration Variables
        self._calibrating = 0
        self._calibration_data_x = []
        self._calibration_data_y = []
        self._calibration_data_z = []
        self._calibration_index = 0
        
        if(Config().get("ellipsoid_center")):
            self._magn_ellipsoid_center = Config().get("ellipsoid_center")
        else:
            self._magn_ellipsoid_center = [0.0, 0.0, 0.0]
        
        if (Config().get("ellipsoid_transform")):
            self._magn_ellipsoid_transform = Config().get("ellipsoid_transform")
        else:
            self._magn_ellipsoid_transform = [[0.966092, 0.00760439, 0.0283891], [0.00760439, 0.907735, -0.00426615], [0.0283891, -0.00426615, 0.976183]]
        
        #self._qx = [0.067946222436498283, -0.25034004667098259, 8.3336994198409666, -0.17762637163222378]
        #self._qy = [-0.13945102271766135, 2.9074808469097495, 1.6764850422889934, 0.19244505046927501]
        #self._qz = [0.018800599305554239, -0.79590273035713055, -3.1033531112103478, 0.13550993988096199]

        #self._magn_ellipsoid_transform = [[0.0,0.0,0.0], [0.0,0.0,0.0], [0.0,0.0,0.0]]
        self._qx = [0.0,0.0,0.0,0.0]
        self._qy = [0.0,0.0,0.0,0.0]
        self._qz = [0.0,0.0,0.0,0.0]

        self.apitch = 0
        self.aroll = 0
        self._motor_power = 0

    def start_logging(self):
        print "Start Logging I"
        
        #Start Logging
        lg = LogConfig("Magnetometerh", 50)
        lg.add_variable("magh.x")
        lg.add_variable("magh.y")
        lg.add_variable("magh.z")
        self.helper.cf.log.add_config(lg)
        if (lg.valid):
            lg.data_received_cb.add_callback(self._mag_data_signal.emit)
            lg.start()
        else:
            logger.warning("Could not setup logconfiguration after "
                           "connection!")

        lg = LogConfig("Stabal", 100)
        lg.add_variable("stabilizer.roll", "float")
        lg.add_variable("stabilizer.pitch", "float")
        lg.add_variable("stabilizer.yaw", "float")
        lg.add_variable("stabilizer.thrust", "uint16_t")
        self.helper.cf.log.add_config(lg)
        if (lg.valid):
            lg.data_received_cb.add_callback(self._imu_data_signal.emit)
            lg.start()
        else:
            logger.warning("Could not setup logconfiguration after connection!")


    def loggingError(self, log_conf, msg):
        logger.warning("Callback of error in LogEntry :(")

    def connected(self, linkURI):
        self.connection_status = 1
        self.start_logging()

    def disconnected(self, linkURI):# what else needs done in here ?
        self.connection_status = 0
        self.intelligentOrientationDial.setValue(0)
        self.intelligentOrientationSpinBox.setValue(0)
        self.headingLockDial.setValue(0)
        self.headingLockSpinBox.setValue(0)
        self.compass.setAngle(0)
        self.headingLcdNumber.display(0)

	# Calibration Functions
    def calibrate(self, x, y, z):
        H = numpy.array([x, y, z, -y**2, -z**2, numpy.ones([len(x)])])
        H = numpy.transpose(H)
        w = x**2
        (X, residues, rank, shape) = linalg.lstsq(H, w)
        OSx = X[0] / 2
        OSy = X[1] / (2 * X[3])
        OSz = X[2] / (2 * X[4])
        A = X[5] + OSx**2 + X[3] * OSy**2 + X[4] * OSz**2
        B = A / X[3]
        C = A / X[4]
        SCx = numpy.sqrt(A)
        SCy = numpy.sqrt(B)
        SCz = numpy.sqrt(C)
        return ([OSx, OSy, OSz], [SCx, SCy, SCz])


	#Recieving Logging Data
    def _imu_data_received(self, timestamp, data, logconf):
        self.aroll = data["stabilizer.roll"]
        self.apitch = data["stabilizer.pitch"]


    def _motor_data_received(self, timestamp, data, logconf):
        self.motor_power = data["motor.m1"] + data["motor.m2"] + data["motor.m3"] + data["motor.m4"]

    def _mag_data_received(self, timestamp, data, logconf):
        if self._calibrating:
            num_of_cal_samples = 1500
            self._calibration_data_x.append(int(data["magh.x"]))
            self._calibration_data_y.append(int(data["magh.y"]))
            self._calibration_data_z.append(int(data["magh.z"]))
            self._calibration_index += 1
            self.calibratePushButton.setText('Calibrating : ' + str(num_of_cal_samples-self._calibration_index))
            if (self._calibration_index > num_of_cal_samples):
                self._calibration_index = 0
                self._calibrating = False
                (offsets, scale) = self.calibrate(numpy.array(self._calibration_data_x), numpy.array(self._calibration_data_y), numpy.array(self._calibration_data_z))
                self._magn_ellipsoid_center = offsets
                self._calibration_data_x = []
                self._calibration_data_y = []
                self._calibration_data_z = []
                self.calibratePushButton.setText('Calibrate')
                Config().set("ellipsoid_center", self._magn_ellipsoid_center)
                Config().set("ellipsoid_transform", self._magn_ellipsoid_transform)
                print self._magn_ellipsoid_center
                print self._magn_ellipsoid_transform
                #print scale

        def mv(a, b):			# matrix by vector multiplication
            out = [0,0,0]
            for x in range(0, 3):
                out[x] = a[x][0] * b[0] + a[x][1] * b[1] + a[x][2] * b[2];
            return out

        def adj(qs, power):		        # calculate adjustments related to how much power is sent to the motors      
            p = float(power) / float(40000) # 10k * 4 motors
            return qs[0]*p**3+qs[1]*p**2+qs[2]*p+qs[3]

        x, y, z = data['magh.x'], data['magh.y'], data['magh.z']
        x = x - self._magn_ellipsoid_center[0]
        y = y - self._magn_ellipsoid_center[1]
        z = z - self._magn_ellipsoid_center[2]
        x, y, z = mv(self._magn_ellipsoid_transform, [x, y, z])
        x = x + adj(self._qx, self._motor_power)
        y = y + adj(self._qy, self._motor_power)
        z = z + adj(self._qz, self._motor_power)
        x, y, z = y, x, z * -1        # correct magnetometer orientation relative to the CF orientation
        cosRoll = cos(math.radians(self.aroll))	        # calculate tilt-compensated heading angle        
        sinRoll = sin(math.radians(self.aroll))  
        cosPitch = cos(math.radians(self.apitch)) 
        sinPitch = sin(math.radians(self.apitch))
        Xh = x * cosPitch + z * sinPitch
        Yh = x * sinRoll * sinPitch + y * cosRoll - z * sinRoll * cosPitch
        heading = math.atan2(Yh, Xh)
        self._current_heading = math.degrees(heading) * -1  # for some reason getting inverted sign here
        self.compass.setAngle(self._current_heading)	    # update compass widget
        self.headingLcdNumber.display(int(self._current_heading))

        if self._intelligent_orientation_mode:
            adjusted_heading = self._current_heading - self._intelligent_orientation_value
            self.helper.cf.commander.set_client_compass_heading(adjusted_heading)# plus offset
            print adjusted_heading
        else:
            self.helper.cf.commander.set_client_compass_heading(self._current_heading)

        print self.aroll
       

    #Define UI events
    def headingLockRadioButtonClicked(self,status):
        if status:
            self._heading_lock_mode = True
            self.helper.cf.commander.set_heading_lock_mode(True)
        elif not status:
            self._heading_lock_mode = False
            self.helper.cf.commander.set_heading_lock_mode(False)
            #print status

    def headingLockDialValueChanged(self, change):
        self.headingLockSpinBox.setValue(change)
        self._heading_lock_value = change
        self.helper.cf.commander.set_heading_lock_value(change)

    def headingLockSpinBoxValueChanged(self, change):
        self.headingLockDial.setValue(change)
        self._heading_lock_value = change
        self.helper.cf.commander.set_heading_lock_value(change)

    def headingLockCurrentPushButtonClicked(self):
        self.headingLockDial.setValue(self._current_heading)
        self.headingLockSpinBox.setValue(self._current_heading)
        self._heading_lock_value = self._current_heading #redundant ?
        self.helper.cf.commander.set_heading_lock_value(self._current_heading)

    def intelligentOrientationRadioButtonClicked(self,change):
        if change:
            self.helper.cf.commander.set_client_compass_mode(True)
            self._intelligent_orientation_mode = True
        elif not change:
            self.helper.cf.commander.set_client_compass_mode(False)
            self._intelligent_orientation_mode = False
	
    def intelligentOrientationDialValueChanged(self, change):
        self.intelligentOrientationSpinBox.setValue(change)
        self._intelligent_orientation_value = change

    def intelligentOrientationSpinBoxValueChanged(self, change):
        self.intelligentOrientationDial.setValue(change)
        self._intelligent_orientation_value = change

    def intelligentOrientationCurrentPushButtonClicked(self):
        self.intelligentOrientationDial.setValue(self._current_heading)
        self.intelligentOrientationSpinBox.setValue(self._current_heading)
        self._intelligent_orientation_value = self._current_heading

    def calibratePushButtonClicked(self):
        self._calibrating = True
        self._calibration_index = 0

    def getName(self):
        return 'CompassToolbox'
    
    def enable(self):
        print "Enabling Toolbox"
    
    def disable(self):
        print "Disabling Toolbox"
    
    def preferedDockArea(self):
        return Qt.RightDockWidgetArea

