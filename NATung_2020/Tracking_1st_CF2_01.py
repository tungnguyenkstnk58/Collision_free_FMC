# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2014 Bitcraze AB
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
Anh Tung Nguyen, email: tung.kstn@gmail.com
This is Program controlling Multi-Crazyflies based on Formation Control
"""
import logging
import time
import threading
import sys
import termios
import contextlib
import csv

from threading import Timer
from math import cos, sin, asin, atan2, sqrt, acos

import cflib.crtp  # noqa
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

@contextlib.contextmanager
def raw_mode(file):
    """ Implement termios module for keyboard detection """
    old_attrs = termios.tcgetattr(file.fileno())
    new_attrs = old_attrs[:]
    new_attrs[3] = new_attrs[3] & ~(termios.ECHO | termios.ICANON)
    try:
        termios.tcsetattr(file.fileno(), termios.TCSADRAIN, new_attrs)
        yield
    finally:
        termios.tcsetattr(file.fileno(), termios.TCSADRAIN, old_attrs)

def logging_error(Logconf, msg):
    # Error message to be shown in the svent of logging errors
    print ("Error when logging %s" % logconf.name)

class formationControl(threading.Thread):
    """ A controller thread, taking references and mearsurements from the UAV 
    computing a control input"""

    def __init__(self, link_uri):
        threading.Thread.__init__(self)
        #self.link_uri = link_uri
        self.daemon = True
        self.timePrint = 0.0

        # Initialize the crazyflie object and add some callbacks
        self._cf = Crazyflie(rw_cache='./cache')
        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)
        self._cf.open_link(link_uri)

        self.rate           = 20 # (Hz) Rate of each sending control input
        self.statefb_rate   = 50 # (ms) Time between two consecutive state feedback

        # Logged states 
        self.position = [0.0, 0.0, 0.0] # [m] in the global frame of reference

        # Limits
        self.vel_x_limit    = (-0.5, 0.5)       # [m/s]
        self.vel_y_limit    = (-0.5, 0.5)       # [m/s]
        self.vel_z_limit    = (-0.2, 0.2)       # [m/s]
        self.vel_yaw_limit  = (-200.0, 200.0)   # [Degree/s]

        # Control setting
        self.isEnabled = False
        self.printDis  = True

    def _run_controller(self):
        """ Main control loop
        # System parameters"""
        timeSampling            = 1.0/float(self.rate)  # [s] sampling time
        delta                   = 1                     # [] if quad knows reference -> 1, otherwise 0

        # Control Parameters
        gp                      = 2.0     # for position
        alpha_z                 = 1.0

        # Reference parameters
        v0                      = [0.0, 0.0, 0.0]   # Reference velocity
        fp                      = [0.0, 0.0, 0.0]   # [m] the formation shape
        coef_init               = 1.0               # initialize altitude
        rot_mat                 = [[0.0, 0.0, 0.0], # Rotation matrix for formation
                                   [0.0, 0.0, 0.0], # shape in 3D space
                                   [0.0, 0.0, 0.0]]

        # Set the current reference to the current positional estimate
        time.sleep(5)
        x_0, y_0, z_0 = self.position
        print('Initial position reference: ({}, {}, {})'.format(x_0, y_0, z_0))
        x_d, y_d, z_d = [x_0, y_0, z_0 + 0.3]

        # Unlock the controller
        self.isEnabled = True

        # Begin while loop for sending the control signal
        self._cf.commander.send_hover_setpoint(0, 0, 0, 0.3)
        time.sleep(1.0)
        timeBeginCtrl = time.time()
        while True:
            timeStart = time.time()
            # Change the reference velocity
            if time.time() > timeBeginCtrl + 2.0:
                v0          = [0.05, -0.05, 0.0]
                rot_mat     = [[1.0, 0.0, 0.0], # Rotation matrix for formation
                               [0.0, 1.0, 0.0], # shape in 3D space
                               [0.0, 0.0, 1.0]]
                coef_init   = 0.0
                alpha_z     = 0.0
            # Landing
            if time.time() > timeBeginCtrl + 10.0:
                self._cf.commander.send_velocity_world_setpoint(0.0, 0.0, 0.0, 0.0)
                self._cf.close_link()

            # Get the reference
            x_d = x_d + v0[0] * timeSampling
            y_d = y_d + v0[1] * timeSampling
            z_d = z_d + v0[2] * timeSampling

            # Get roll, pitch, yaw
            x, y , z           = self.position

            # Get eta
            eta_px = x  - (rot_mat[0][0] * fp[0] + rot_mat[0][1] * fp[1] + rot_mat[0][2] * fp[2]) - coef_init * x_0
            eta_py = y  - (rot_mat[1][0] * fp[0] + rot_mat[1][1] * fp[1] + rot_mat[1][2] * fp[2]) - coef_init * y_0
            eta_pz = z  - (rot_mat[2][0] * fp[0] + rot_mat[2][1] * fp[1] + rot_mat[2][2] * fp[2]) - coef_init * z_0

            # Get v
            v_x = - gp * delta * (eta_px - x_d) 
            v_y = - gp * delta * (eta_py - y_d) 
            v_z = - gp * alpha_z * delta * (eta_pz - z_d)

            # Saturation
            v_x = self.saturation(v_x, self.vel_x_limit)
            v_y = self.saturation(v_y, self.vel_y_limit)
            v_z = self.saturation(v_z, self.vel_z_limit) 
            #self.datalog_Pos.write(str([time.time(), self.position]) + "\n")
            
            if self.isEnabled:
                # Get thrust and attitude desired
                self._cf.commander.send_velocity_world_setpoint(v_x, v_y, v_z, 0.0)
                message = ('ref:({}, {}, {})\n'.format(x_d, y_d, z_d) + 
                           'pos:({}, {}, {})\n'.format(x, y, z) + 
                           'control:({}, {}, {})\n'.format(v_x, v_y, v_z))                           
            else:
                self._cf.close_link()
                #print('Force disconnecting')

            self.print_at_period(2.0, message) 
            self.loop_sleep(timeStart)
        # End     

    def saturation(self, value, limits):
        """ Saturate a given value within limit interval"""
        if value < limits[0]:
            value = limits[0]
            #print("limit low")
        elif value > limits[1]:
            value = limits[1]
            #print("limit up")
        return value

    def enableInput(self):
        return self.isEnabled

    def print_at_period(self, period, message):
        """ Prints the message at a given period"""
        if (time.time() - period) > self.timePrint:
            self.timePrint = time.time()
            print(message)

    def _connected(self, link_uri):
        """ This callback is called from the Crazyflie API when a Crazyflie
        has been connected adn TOCs have been downloaded """
        print('connected to: %s' %link_uri)

        # Open text file for recording data
        self.datalog_Pos    = open("log_data/Cf2log_Pos","w")
        #self.datalog_Vel    = open("log_data/Cf2log_Vel","w")
        #self.datalog_Att    = open("log_data/Cf2log_Att","w")

        # Reset Kalman filter 
        #self._cf.param.set_value('kalman.resetEstimation', '1')
        time.sleep(0.1)
        self._cf.param.set_value('kalman.resetEstimation', '0')
        time.sleep(2)
        print('Wait for Kalamn filter')

        # Feedback position estimated by KF
        logPos = LogConfig(name='Kalman Position', period_in_ms=self.statefb_rate)
        logPos.add_variable('kalman.stateX','float')
        logPos.add_variable('kalman.stateY','float')
        logPos.add_variable('kalman.stateZ','float')

        # Invoke logged states
        self._cf.log.add_config(logPos)

        # Start invoking logged states
        if logPos.valid:
            # Position
            logPos.data_received_cb.add_callback(self.log_pos_callback)
            logPos.error_cb.add_callback(logging_error)
            logPos.start()
        else:
            print("One or more of the variables in the configuration was not found"+
                  "in log TOC. No logging will be possible")

        # Start in a thread 
        threading.Thread(target=self._run_controller).start()

    def log_pos_callback(self, timestamp, data, Logconf):
        """ Callback for the logging the position of the UAV in the global frame"""
        self.position = [
                data['kalman.stateX'], 
                data['kalman.stateY'],
                data['kalman.stateZ']
                        ]

        self.datalog_Pos.write(str([time.time(), self.position]) + "\n")
        #print('Position x, y, z, time', self.position, time.time())
        #print(self.rate)

    def _connection_failed(self, link_uri, msg):
        """ Callback when connection initial connection fails
        there is no Crazyflie at the specified address """
        print('Connection to %s failed: %s' % (link_uri, msg))

    def _connection_lost(self, link_uri, msg):
        """ Callback when disconected after a connection has been made """
        print('Connection to %s lost: %s' % (link_uri, msg))

    def _disconnected(self, link_uri):
        """ Callback when the Crazyflie is disconnected (called in all cases) """
        if self.printDis:
            print('Disconnected from %s' % link_uri)
            self.printDis = False

    def loop_sleep(self, timeStart):
        """ Sleeps the control loop to make it run at a specified rate """
        deltaTime = 1.0/float(self.rate) - (time.time() - timeStart)
        if deltaTime > 0:
            time.sleep(deltaTime)

    def _close_connection(self, message):
        """ This is able to close link connecting Crazyflie from keyboard """
        if message == "q":
            self.isEnabled = False
        # end

class inputThreat(threading.Thread):
        """ Create an input thread which sends a forced command """
        def __init__(self, controller):
            threading.Thread.__init__(self)
            self.valid_characters = ["q"]
            self.daemon = True
            self.controller = controller
            self.start()

        def run(self):
            with raw_mode(sys.stdin):
                try:
                    while True:
                        ch = sys.stdin.read(1)
                        if ch in self.valid_characters:
                            self.controller._close_connection(ch)

                except (KeyboardInterrupt, EOFError):
                    sys.exit(0)
                    pass


"""  The main loop"""
if __name__ == "__main__":
    # Initialize the low-level drivers(dont list the debuf drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)

    # Set a chanel - if set None, the first avaiable crazyflie is used
    myUri_1 = 'radio://0/70/2M/E7E7E7E701'
    myUri_2 = 'radio://0/70/2M/E7E7E7E702'
    myUri_3 = 'radio://0/70/2M/E7E7E7E703'

    # All Uri
    myUri       = [myUri_1,             # List all link_uri  
                   myUri_2,
                   myUri_3]   
    numCf2      = len(myUri)            # Number of Crazyflie used
    myCf2       = [0] * numCf2          # Declare each Crazyflie
    myKbcmd     = [0] * numCf2          # Declare force command from keyboard for each Crazyflie
    timeCmd     = 0.0/1000.0           # [ms] time between two consecutive sending command 

    # Start the threads
    for order_Cf2 in range(0, numCf2):
        print('Connecting to %s...' %str(myUri[order_Cf2]))
        myCf2[order_Cf2]    = formationControl(myUri[order_Cf2])
        myKbcmd[order_Cf2]  = inputThreat(myCf2[order_Cf2])
        time.sleep(0.1)







