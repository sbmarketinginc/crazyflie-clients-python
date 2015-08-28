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
This tab plots different logging data defined by configurations that has been
pre-configured.
"""
import math

__author__ = 'Bitcraze AB'
__all__ = ['GpsTab']

import logging
import sys

logger = logging.getLogger(__name__)

from PyQt4 import QtCore, QtGui, uic
from PyQt4.QtCore import pyqtSlot, pyqtSignal, QThread, Qt, QUrl
from PyQt4.QtGui import QMessageBox
from PyQt4.QtWebKit import *

from cfclient.ui.tab import Tab

from cflib.crazyflie.log import LogConfig, Log
from cflib.crazyflie.param import Param
import math


gps_tab_class = uic.loadUiType(sys.path[0] + "/cfclient/ui/tabs/gpsTab.ui")[0]

maphtml = '''
    <!DOCTYPE html>
    <html>
    <head>
    <style>
    html, body, #map-canvas {
    height: 100%;
    margin: 0px;
    padding: 0px
    }
    </style>
    <script src="http://maps.googleapis.com/maps/api/js?v=3.exp&sensor=true&libraries=drawing"></script>
    <script>
    function initialize() {
    var myLatLng = new google.maps.LatLng(40.361782, -80.002687);
    var mapOptions = {
    center: new google.maps.LatLng(40.361782, -80.002687),
    zoom: 19,
    mapTypeId: google.maps.MapTypeId.ROADMAP
    };
    marker = new google.maps.Marker( {position: myLatLng, map: map} );
    var map = new google.maps.Map(document.getElementById('map-canvas'), mapOptions);
    
    marker.setMap( map );
    //moveMarker( map, marker );
    
    var drawingManager = new google.maps.drawing.DrawingManager({
    drawingMode: google.maps.drawing.OverlayType.POLYGON,
    drawingControl: true,
    drawingControlOptions: {
    position: google.maps.ControlPosition.TOP_CENTER,
    drawingModes: [google.maps.drawing.OverlayType.POLYGON]
    },
    polygonOptions: {editable: true, draggable: true},
    });
    drawingManager.setMap(map);
    
    var thePolygon = null;
    
    google.maps.event.addListener(drawingManager, 'polygoncomplete', function (polygon) {
    if (thePolygon)
    thePolygon.setMap(null);
    thePolygon = polygon;
    polygon.getPath().forEach(function (xy, i) {
    self.polygoncomplete(xy.lat(), xy.lng(), i);
    });
    });
    }
    
    
    function moveMarker( lat, lng ) {
    marker.setPosition( new google.maps.LatLng( (insertDecimal(lat)), (insertDecimal(lng)) ) );
    };
    
    function insertDecimal(num) {
    return num;
    }
    
    google.maps.event.addDomListener(window, 'load', initialize);
    
    self.polygoncomplete(1.08, 1.08, 1);
    
    
    </script>
    </head>
    <body>
    <div id="map-canvas"></div>
    </body>
    </html>
    '''


class GpsTab(Tab, gps_tab_class):
    """Tab for plotting logging data"""

    _connected_signal = pyqtSignal(str)
    _disconnected_signal = pyqtSignal(str)
    _log_data_signal = pyqtSignal(int, object, object)
    _log_error_signal = pyqtSignal(object, str)
    _param_updated_signal = pyqtSignal(str, str)


    def __init__(self, tabWidget, helper, *args):
        super(GpsTab, self).__init__(*args)
        self.setupUi(self)

        self.tabName = "GPS"
        self.menuName = "GPS"

        self.tabWidget = tabWidget
        self._helper = helper
        self._cf = helper.cf
        self._got_home_point = False
        self._line = ""
        
        self._web_view.setHtml(maphtml)
        self._web_view.page().mainFrame().addToJavaScriptWindowObject('self', self)

        # Always wrap callbacks from Crazyflie API though QT Signal/Slots
        # to avoid manipulating the UI when rendering it
        self._connected_signal.connect(self._connected)
        self._disconnected_signal.connect(self._disconnected)
        self._log_data_signal.connect(self._log_data_received)
        self._param_updated_signal.connect(self._param_updated)
        
        # Connect the Crazyflie API callbacks to the signals
        self._helper.cf.connected.add_callback(self._connected_signal.emit)
        self._helper.cf.disconnected.add_callback(self._disconnected_signal.emit)
        self._mission_enable_box.clicked.connect(
            lambda enable:
            self._helper.cf.param.set_value("gpsDest.mission_enable", str(enable))
        )
                                               
        self._helper.cf.param.add_update_callback(group="gpsDest", name="mission_enable", cb=self._param_updated_signal.emit)

    def _connected(self, link_uri):
        """Callback when the Crazyflie has been connected"""
        
        logger.debug("Crazyflie connected to {}".format(link_uri))
        
        gps_conf = LogConfig("gps", 100)
        gps_conf.add_variable("gps.fixType")
        gps_conf.add_variable("gps.lat")
        gps_conf.add_variable("gps.lon")
        gps_conf.add_variable("gps.hMSL")
        gps_conf.add_variable("gps.hAcc")
        gps_conf.add_variable("gps.gSpeed")
        gps_conf.add_variable("gps.heading")
        
        self._helper.cf.log.add_config(gps_conf)
        if gps_conf.valid:
            gps_conf.data_received_cb.add_callback(self._log_data_signal.emit)
            gps_conf.start()

    def _disconnected(self, link_uri):
        """Callback for when the Crazyflie has been disconnected"""
        
        logger.debug("Crazyflie disconnected from {}".format(link_uri))
        
        self._mission_enable_box.setEnabled(False)
    
    def _param_updated(self, name, value):
        """Callback when the registered parameter get's updated"""
        
        logger.debug("Updated {0} to {1}".format(name, value))
        
        if not self._mission_enable_box.isEnabled():
            self._mission_enable_box.setEnabled(True)
        
        self._mission_enable_box.setChecked(eval(str(value)))
        self._mission_enable = eval(str(value))


    def _log_data_received(self, timestamp, data, log_conf):
        """Callback when the log layer receives new data"""
        
        self._current_lat = float(data["gps.lat"])/10000000
        self._current_lng = float(data["gps.lon"])/10000000
        
        self._current_lat = float(40.361852)
        self._current_lng = float(-80.002581)
        
        self._gps_fixtype_box.setText("{}".format(data["gps.fixType"]))
        self._gps_lat_box.setText("{}".format(self._current_lat))
        self._gps_lng_box.setText("{}".format(self._current_lng))
        self._gps_hMSL_box.setText("{}".format(data["gps.hMSL"]))
        self._gps_hAcc_box.setText("{}".format(data["gps.hAcc"]))
        self._gps_gSpeed_box.setText("{}".format(data["gps.gSpeed"]))
        self._gps_heading_box.setText("{}".format(data["gps.heading"]))
        frame = self._web_view.page().mainFrame()
        frame.evaluateJavaScript('moveMarker(' + str(self._current_lat) + ', ' + str(self._current_lng) + ');')
        self._waypoint_distance(self._current_lat, self._current_lng, self._mission_lat, self._mission_lng)
        self._waypoint_heading(self._current_lat, self._current_lng, self._mission_lat, self._mission_lng)
        
        """if(self._mission_enable==1):
            self._helper.cf.commander.set_mission_current_gps(self._current_lat, self._current_lng, self._mission_lat, self._mission_lng)"""
        
        
        self._helper.cf.commander.set_mission_enable(self._mission_enable)
    
    def _logging_error(self, log_conf, msg):
        """Callback from the log layer when an error occurs"""
        
        QMessageBox.about(self, "Example error",
                          "Error when using log config"
                          " [{0}]: {1}".format(log_conf.name, msg))
    
    def _set_mission_waypoint(self, lat, lng):
        self._mission_lat = lat
        self._mission_lng = lng
        self._gps_mission_lat_box.setText("{}".format(self._mission_lat))
        self._gps_mission_lng_box.setText("{}".format(self._mission_lng))
    
    def _set_mission_control(self, curLat, curLng, misLat, misLng):
        self._helper.cf.commander.set_mission_commands(heading, distance)
    
    def _waypoint_distance(self, gpsLat, gpsLng, wayLat, wayLng):
        dlat = wayLat - gpsLat
        dlong = wayLng - gpsLng
        way_distance = math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5
        self._waypoint_distance_box.setText("{}".format(way_distance))
        self._helper.cf.commander.set_mission_distance_value(way_distance)
    
    def _waypoint_heading(self, gpsLat, gpsLng, wayLat, wayLng):
        off_x = wayLng - gpsLng
        off_y = wayLat - gpsLat
        bearing = 90.00 + math.atan2(-off_y, off_x) * 57.2957795
        
        if (bearing > 180):
            bearing += -360.00
        
        self._waypoint_bearing_box.setText("{}".format(bearing))
        self._helper.cf.commander.set_mission_heading_value(bearing)

    @pyqtSlot(float, float, int)
    def polygoncomplete(self, lat, lng, i):
        if i == 0:
            print "Point #{} ({}, {})".format(i, lat, lng)
        if i == 1:
            self._set_mission_waypoint(lat, lng)
            print "Point #{} ({}, {})".format(i, lat, lng)
        
        if i == 2:
            print "Point #{} ({}, {})".format(i, lat, lng)