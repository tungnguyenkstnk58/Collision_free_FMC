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
"""
Anh Tung Nguyen, email: tung.kstn@gmail.com
This is program controlling Multi-Crazyflie based on Formation Control
Date: 2020/07/14
"""
import logging
import time
import threading
import sys
import termios
import contextlib
import numpy as np 

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

    def __init__(self, link_uri, delta_ref, formation_fp, order_Cf2):
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

        self.rate           = 15 # (Hz) Rate of each sending control input
        self.statefb_rate   = 75 # (ms) Time between two consecutive state feedback

        # Logged states -, attitude(p, r, y), rotation matrix
        self.position       = [0.0, 0.0, 0.0] # [m] in the global frame of reference
        self.eta            = [0] * 3 
        self.eta_other      = [0] * 3
        self.formation_fp   = formation_fp    # Formation shape for each agent

        # Limits
        self.vel_x_limit    = (-0.2, 0.2)       # [m/s]
        self.vel_y_limit    = (-0.2, 0.2)       # [m/s]
        self.vel_z_limit    = (-0.2, 0.2)       # [m/s]
        self.vel_yaw_limit  = (-200.0, 200.0)   # [Degree/s]

        # Control setting
        self.isEnabled = True
        self.printDis  = True

        # System parameter
        self.pi         = 3.14159265359
        self.delta_ref  = delta_ref
        self.order_Cf2  = order_Cf2

    def _run_controller(self):
        """ Main control loop
        # System parameters"""
        timeSampling            = 1.0/float(self.rate)  # [s] sampling time
        delta                   = 1             # [] initial time, all Crazyflie lifted to 0.3m

        # Control Parameters
        gp                      = 1.5     # for position
        alpha_z                 = 1.0       

        # Reference parameter
        v0                      = [0.0, 0.0, 0.0]   # [m/s] reference velocity
        fp                      = self.formation_fp # [m] the formation shape
        fv                      = [0.0, 0.0, 0.0]   # [m/s] the velocity formation shape
        coef_init               = 1.0               # initialize altitude
        rot_mat                 = [[0.0, 0.0, 0.0], # Rotation matrix for formation
                                   [0.0, 0.0, 0.0], # shape in 3D space
                                   [0.0, 0.0, 0.0]]

        # Set the current reference to the current positional estimate
        time.sleep(5)
        x_0, y_0, z_0 = self.position
        print('Initial position reference: ({}, {}, {})'.format(x_0, y_0, z_0))
        x_d, y_d, z_d = [x_0, y_0, z_0 + 0.3]       # initial time, lift to 0.3m, avoid groud effect 
        eta_p_other   = [0.0] * 3
        eta_v_other   = [0.0] * 3

        # Begin while loop for sending the control signal
        self._cf.commander.send_hover_setpoint(0, 0, 0, 0.3)
        time.sleep(1.0)
        timeBeginCtrl = time.time()
        while True:
            timeStart = time.time()
            # Change the reference velocity
            if time.time() > timeBeginCtrl + 1.0:
                v0          = [0.05, -0.05, 0.0]
                rot_mat     = [[1.0, 0.0, 0.0], # Rotation matrix for formation
                               [0.0, 1.0, 0.0], # shape in 3D space
                               [0.0, 0.0, 1.0]]
                coef_init   = 0.0
                delta       = self.delta_ref    
                #alpha_z     = 0.0
            # Landing
            if time.time() > timeBeginCtrl + 10.0:
                #time.sleep(0.6)
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
            eta    = [eta_px, eta_py, eta_pz]
            self.publish_infor(eta)

            # Get eta from others
            if self.eta_other != [0.0] * 3:
                eta_p_other = self.eta_other[0:3]
            else:
                eta_p_other   = [-x_d, -y_d, -z_d]

            # Get u
            v_x = - gp * delta * (eta_px - x_d) - gp * (eta_px + eta_p_other[0])
            v_y = - gp * delta * (eta_py - y_d) - gp * (eta_py + eta_p_other[1]) 
            v_z = - gp * delta * (eta_pz - z_d) - 0 * (eta_pz + eta_p_other[2])
            #v_z = v_z * alpha_z 

            # Saturation
            v_x = self.saturation(v_x, self.vel_x_limit)
            v_y = self.saturation(v_y, self.vel_y_limit)
            v_z = self.saturation(v_z, self.vel_z_limit)
            v   = [v_x, v_y, v_z] 

            # Storage u
            self.myInput_log.write(str([time.time(), v]) + "\n")

            if self.isEnabled:
                # Get thrust and attitude desired
                self._cf.commander.send_velocity_world_setpoint(v_x, v_y, v_z, 0.0)          
                message = ('ref:({}, {}, {})\n'.format(x_d, y_d, z_d) + 
                           'pos:({}, {}, {})\n'.format(x, y, z) + 
                           'control:({}, {}, {})\n'.format(v_x, v_y, v_z))
                self.print_at_period(2.0, message)             
            else:
                self._cf.close_link()

            # Send set point
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

    def print_at_period(self, period, message):
        """ Prints the message at a given period"""
        if (time.time() - period) > self.timePrint:
            self.timePrint = time.time()
            print(message)

    def _connected(self, link_uri):
        """ This callback is called from the Crazyflie API when a Crazyflie
        has been connected adn TOCs have been downloaded """
        print('connected to: %s' %link_uri)

        # Open text file to storage data
        self.myPos_log   = open("log_data/Pos_log" + str(self.order_Cf2) + ".txt", "w")
        self.myInput_log = open("log_data/Input_log" + str(self.order_Cf2) + ".txt", "w")

        # Reset Kalman filter 
        self._cf.param.set_value('kalman.resetEstimation', '1')
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

        self.myPos_log.write(str([time.time(), self.position]) + "\n")
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

    def subscribe_other_infor(self, eta_other):
        """ This function updates the information including eta_p and eta_v from
        other Crazyflies """
        self.eta_other = eta_other

    def publish_infor(self, eta):
        """ This function publishes this Crazyflie information including eta_p and eta_v
        to others"""
        self.eta = eta

    def get_publish_infor(self):
        """ This calback take the information of this Crazyflie"""
        return self.eta

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

    # Declare channels with Crazyflies used
    myUri_1 = 'radio://0/70/2M/E7E7E7E701'
    myUri_2 = 'radio://0/70/2M/E7E7E7E702'
    myUri_3 = 'radio://0/70/2M/E7E7E7E703'
    myUri       = [myUri_1,             # List all link_uri
                   myUri_2,
                   myUri_3]
    form_shape  = [[0.0, 0.0, 0.0],      # Formation shape for each Crazyflie
                   [0.0, 0.0, 0.0],
                   [0.0, 0.0, 0.0]]       
    numCf2      = len(myUri)            # Number of Crazyflie used   
    myCf2       = [0] * numCf2          # Declare each Crazyflie
    myKbcmd     = [0] * numCf2          # Declare force command from keyboard for each Crazyflie
    matLap      = [[2.0, -1.0, -1.0],   # Declare a Laplacian matrix representing the 
                   [-1.0, 2.0, -1.0],
                   [-1.0,-1.0,  2.0]]   # communication among Crazyflie
    #matLap      = [[1.0, -1.0],
    #               [-1.0, 1.0]]
    matDel      = [1, 0, 0]             # [] if quad knows reference -> 1, otherwise 0
    eta_all     = [[0] * 3] * numCf2    # Declare eta for all Crazyflie   
    timeCom     = 75.0/1000.0           # Time each communication 50 ms

    # Start the threads
#    while True:
    for order_Cf2 in range(0, numCf2):
        print('Connecting to %s...' %str(myUri[order_Cf2]))
        myCf2[order_Cf2]    = formationControl(myUri[order_Cf2], matDel[order_Cf2], form_shape[order_Cf2], order_Cf2)
        #myKbcmd[order_Cf2]  = inputThreat(myCf2[order_Cf2])      
        #time.sleep(0.01)
    ##
    time.sleep(8.2)
    print('beginnnn')    
    while True:
        timeBeginCom = time.time() 
        for order_Cf2 in range(0, numCf2):
            eta_all[order_Cf2] = myCf2[order_Cf2].get_publish_infor()
        ##
        eta_other   = [[0] * 3] * numCf2         # Declare eta for all Crazyflie
        for order_Cf2 in range(0, numCf2):
            for order_Cf2_other in range(0, numCf2):
                if order_Cf2_other != order_Cf2:
                    e = list(np.array(eta_all[order_Cf2_other]) * matLap[order_Cf2][order_Cf2_other])
                    eta_other[order_Cf2] = list(np.array(eta_other[order_Cf2]) + np.array(e))
            ##
            myCf2[order_Cf2].subscribe_other_infor(eta_other[order_Cf2])
        ##
        deltaT = timeBeginCom + timeCom - time.time()
        if deltaT > 0:    
            time.sleep(deltaT)

    #-------------End----------