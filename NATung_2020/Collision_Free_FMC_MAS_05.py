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

        self.rate           = 5  # (Hz) Rate of each sending control input
        self.statefb_rate   = 400 # (ms) Time between two consecutive state feedback

        # Logged states -, attitude(p, r, y), rotation matrix
        self.position       = [0.0, 0.0, 0.0] # [m] in the global frame of reference
        self.col_free       = [0] * 2 
        self.col_other      = [0] * 2
        self.formation_fp   = formation_fp    # Formation shape for each agent
        self.Mtr            = [0.0, 0.0, 0.0, 0.0, 0.0] # Multiranger


        # Control setting
        self.isEnabled = True
        self.printDis  = True

        # System parameter
        self.pi         = 3.14159265359
        self.delta_ref  = delta_ref
        self.order_Cf2  = order_Cf2

        # Open text file to storage data
        self.myPos_log   = open("log_data/Pos_log" + str(self.order_Cf2), "w")
        self.myInput_log = open("log_data/Input_log" + str(self.order_Cf2), "w")

    def _run_controller(self):
        """ Main control loop
        # System parameters"""
        timeSampling            = 1.0/float(self.rate)  # [s] sampling time
        delta                   = 1             # [] initial time, all Crazyflie lifted to 0.3m

        # Control Parameters

        # Reference parameter
        v0                      = [0.0, 0.0, 0.0]   # [m/s] reference velocity
        fp                      = [0.0, 0.0, 0.0]   # [m] the formation shape
        fv                      = [0.0, 0.0, 0.0]   # [m/s] the velocity formation shape

        # Collision parameters
        col_thrs                = 0.2               
        safe_thrs               = 0.4
        col_muy                 = 2.0
        col_lda                 = -0.1
        Mtr_pre                 = self.Mtr
        Mtr_dot                 = [0.0, 0.0, 0.0, 0.0]

        # Set the current reference to the current positional estimate
        time.sleep(1)
        # Unlock the controller
        # First, we need send a signal to enable sending attitude reference and thrust force        
        self._cf.commander.send_setpoint(0, 0, 0, 0)
        # Hover at z = 0.3 durin 2 seconds
        x_0, y_0, z_0 = self.position
        yaw_0         = 0
        timeStart = time.time()
        while True:
            self._cf.commander.send_position_setpoint(x=x_0,y=y_0,z=0.4,yaw=yaw_0)
            time.sleep(0.2)
            if time.time() - timeStart > 1.0:
                break
        # Take a local coordinate
        x_0, y_0, z_0 = self.position
        yaw_d         = 0
        print('Initial position reference: ({}, {}, {})'.format(x_0, y_0, z_0))
        x_d, y_d, z_d = [x_0, y_0, z_0]       # initial time
        col_other     = [0.0] * 2

        # Begin while loop for sending the control signal
        timeBeginCtrl = time.time()
        while True:
            timeStart = time.time()
            # Change the reference velocity
            if time.time() > timeBeginCtrl + 0.5:
                v0          = [0.0, 0.0, 0.0]
                delta       = self.delta_ref 
            #if time.time() > timeBeginCtrl + 20.0:
            #    v0          = [0.02, -0.02, 0.0]
            #if time.time() > timeBeginCtrl + 35.0:
            #    v0          = [-0.02, 0.02, 0.0] 
            #if time.time() > timeBeginCtrl + 50.0:
            #    v0          = [0.0, 0.0, 0.0]                  
            if time.time() > timeBeginCtrl + 30.0:
                self._cf.commander.send_setpoint(0, 0, 0, 40000)
                self._cf.close_link()
                print('Disconnect timeout')
            
            # Get the reference
            x_d = x_d + v0[0] * timeSampling
            y_d = y_d + v0[1] * timeSampling
            z_d = z_d + v0[2] * timeSampling

            # Get fp and fv using smooth step function
            fp[0]  = self.formation_fp[0] * self.step_fcn(time.time(), timeBeginCtrl + 5.0, timeBeginCtrl + 15.0)
            fp[1]  = self.formation_fp[1] * self.step_fcn(time.time(), timeBeginCtrl + 5.0, timeBeginCtrl + 15.0)
            fp[2]  = self.formation_fp[2] * self.step_fcn(time.time(), timeBeginCtrl + 5.0, timeBeginCtrl + 15.0)
            fv[0]  = self.formation_fp[0] * self.step_dot_fcn(time.time(), timeBeginCtrl + 5.0, timeBeginCtrl + 15.0)
            fv[1]  = self.formation_fp[1] * self.step_dot_fcn(time.time(), timeBeginCtrl + 5.0, timeBeginCtrl + 15.0)
            fv[2]  = self.formation_fp[2] * self.step_dot_fcn(time.time(), timeBeginCtrl + 5.0, timeBeginCtrl + 15.0)

            # Get roll, pitch, yaw
            x, y , z           = self.position

            # Get eta
            eta_px = x  - fp[0] 
            eta_py = y  - fp[1] 
            eta_pz = z  - fp[2]
            
            # Get eta from others
            col_other = self.col_other[0:2]
            
            # Multi range dot
            Mtr_dot[0] = (self.Mtr[0] - Mtr_pre[0]) / timeSampling
            Mtr_dot[1] = (self.Mtr[1] - Mtr_pre[1]) / timeSampling
            Mtr_dot[2] = (self.Mtr[2] - Mtr_pre[2]) / timeSampling
            Mtr_dot[3] = (self.Mtr[3] - Mtr_pre[3]) / timeSampling
            # Storage multi range information
            Mtr_pre = self.Mtr

            # Collision avoidance
            u_x_c = - self.bump_fcn_dot(self.Mtr[0],col_thrs,safe_thrs,col_muy,col_lda) * Mtr_dot[0] \
                  + self.bump_fcn_dot(self.Mtr[1],col_thrs,safe_thrs,col_muy,col_lda) * Mtr_dot[1]
            u_y_c = - self.bump_fcn_dot(self.Mtr[2],col_thrs,safe_thrs,col_muy,col_lda) * Mtr_dot[2] \
                  + self.bump_fcn_dot(self.Mtr[3],col_thrs,safe_thrs,col_muy,col_lda) * Mtr_dot[3]
            u_z_c = self.bump_fcn_dot(self.Mtr[0],col_thrs,safe_thrs,col_muy,col_lda) \
                  * self.bump_fcn_dot(self.Mtr[1],col_thrs,safe_thrs,col_muy,col_lda)
            
            # collion-free force
            self.col_free   = [u_x_c, u_y_c]

            # Setpoint
            x_d_c =  x_d + fp[0] + u_x_c + col_other[0]
            y_d_c =  y_d + fp[1] + u_y_c + col_other[1]

            # Send set point
            #self._cf.commander.send_setpoint(roll_d, pitch_d, yaw_d, thrust_d)
            self._cf.commander.send_position_setpoint(x_d_c,y_d_c,0.4,yaw_d)
            #self.print_at_period(2.0, message) 
            # Storage data
            self.myPos_log.write(str(time.time()) + "," +
                                 str(eta_px - x_d) + "," +
                                 str(eta_py - y_d) + "," +
                                 str(eta_pz - z_d) + "," +
                                 str(x) + "," +
                                 str(y) + "," +
                                 str(z) + "\n")
            #self.myInput_log.write(str(time.time()) + "," +
            #                       str(u_x) + "," +
            #                       str(u_y) + "," +
            #                       str(u_z) + "\n")
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

    def bump_fcn(self, mr, threshold, muy):
        """ This is bump function"""
        if mr > threshold:
            mr_bump = 0
        else:
            mr_bump = (threshold - mr) * (threshold - mr)
            mr_bump = mr_bump/(mr + threshold * threshold * 1.0/muy)

        return mr_bump

    def bump_fcn_dot(self, mr, col_thrs, safe_thrs, muy, lda):
        """ This is the derivative of the bump function"""
        if mr > col_thrs:
            mr_bump_dot = lda * self.step_dot_fcn(mr, col_thrs, safe_thrs)
        else:
            mr_bump_dot = float(mr) + 2.0 * col_thrs * col_thrs / muy + col_thrs
            mr_bump_dot = - mr_bump_dot * (col_thrs - float(mr))
            mr_bump_dot = mr_bump_dot / ((float(mr) + col_thrs * col_thrs / muy)*(float(mr) + col_thrs * col_thrs / muy)) 

        return mr_bump_dot

    def step_fcn(self, mr, low, up):
        """ This is smooth step function """
        if mr <= low:
            step_out = 0
        elif mr > low and mr < up:
            step_out = (float(mr) - low)/(up - low)
            step_out = step_out * step_out
        else:
            step_out = 1

        return step_out

    def step_dot_fcn(self, mr, low, up):
        """ This is smooth step function """
        if mr <= low or mr >= up: 
            step_out = 0
        else:
            step_out = 2*(float(mr) - low)/(up - low)
            step_out = step_out / (up - low)

        return step_out

    def print_at_period(self, period, message):
        """ Prints the message at a given period"""
        if (time.time() - period) > self.timePrint:
            self.timePrint = time.time()
            print(message)

    def _connected(self, link_uri):
        """ This callback is called from the Crazyflie API when a Crazyflie
        has been connected adn TOCs have been downloaded """
        print('connected to: %s' %link_uri)

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

        # Feedback Multi ranger
        logMultiRa = LogConfig(name='Range', period_in_ms=self.statefb_rate)
        logMultiRa.add_variable('range.front','uint16_t')
        logMultiRa.add_variable('range.back','uint16_t')
        logMultiRa.add_variable('range.left','uint16_t')
        logMultiRa.add_variable('range.right','uint16_t')
        logMultiRa.add_variable('range.up','uint16_t')

        # Invoke logged states
        self._cf.log.add_config(logPos)
        self._cf.log.add_config(logMultiRa)

        # Start invoking logged states
        if logPos.valid:
            # Position
            logPos.data_received_cb.add_callback(self.log_pos_callback)
            logPos.error_cb.add_callback(logging_error)
            logPos.start()
            # Multi-Ranger
            logMultiRa.data_received_cb.add_callback(self.log_mtr_callback)
            logMultiRa.error_cb.add_callback(logging_error)
            logMultiRa.start()
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

        #self.myPos_log.write(str(time.time()) + "," +
        #                     str(self.position[0]) + "," +
        #                     str(self.position[1]) + "," +
        #                     str(self.position[2]) + "\n")
        #print('Position x, y, z, time', self.position, time.time())
        #print(self.rate)

    def log_mtr_callback(self, timestamp, data, Logconf):
        """ Callback for the logging the position of the UAV in the global frame"""
        self.Mtr = [
                data['range.front'] * 0.001, 
                data['range.back'] * 0.001,
                data['range.left'] * 0.001,
                data['range.right'] * 0.001,
                data['range.up'] * 0.001,
                        ]
        #print('Time, Multirange: front, back, left, right, up', time.time(), self.Mtr)

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

    def subscribe_other_infor(self, col_other):
        """ This function updates the information including eta_p and eta_v from
        other Crazyflies """
        self.col_other = col_other

    def get_publish_infor(self):
        """ This calback take the information of this Crazyflie"""
        return self.col_free

#------------End class
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

#------------End class
class consensus(threading.Thread):
    """ This class is able to exchange information among Crazyflies to 
    achieve the consensus"""
    def __init__(self, myCf2):
        threading.Thread.__init__(self)
        self.myCf2   = myCf2
        self.numCf2  = len(self.myCf2)
        self.col_all = [[0] * 2] * self.numCf2
        """self.matLap  = [[2.0, -1.0, 0.0, 0.0, 0.0, -1.0],   # Declare a Laplacian matrix representing the 
                        [-1.0, 2.0, -1.0, 0.0, 0.0, 0.0],   # communication among Crazyflie
                        [0.0,-1.0,  2.0, -1.0, 0.0, 0.0],
                        [0.0, 0.0, -1.0, 2.0, -1.0, 0.0],
                        [0.0, 0.0, 0.0, -1.0, 2.0, -1.0],
                        [-1.0, 0.0, 0.0, 0.0, -1.0, 2.0]]   """
        self.matLap  = [[0.0, 1.0],
                        [1.0, 0.0]]
        self.rate    = 5   # [Hz]
        time.sleep(2.0)
        self.start()

    def run(self):
        while True:
            timeStart = time.time()
            for i in range(0, self.numCf2):
                self.col_all[i] = self.myCf2[i].get_publish_infor()
            ##
            col_other   = [[0] * 2] * self.numCf2         # Declare eta for all Crazyflie
            for i in range(0, self.numCf2):
                for j in range(0, self.numCf2):
                    e = list(np.array(self.col_all[j]) * self.matLap[i][j])
                    col_other[i] = list(np.array(col_other[i]) + np.array(e))
                ##
                self.myCf2[i].subscribe_other_infor(col_other[i])         
            self.loop_sleep(timeStart)

    def loop_sleep(self, timeStart):
        """ Sleeps the control loop to make it run at a specified rate """
        deltaTime = 1.0/float(self.rate) - (time.time() - timeStart)
        if deltaTime > 0:
            time.sleep(deltaTime)    
#--------------End class 

"""  The main loop"""
if __name__ == "__main__":
    # Initialize the low-level drivers(dont list the debuf drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)

    # Declare channels with Crazyflies used
    myUri_1 = 'radio://0/99/2M/E7E7E7E706'
    myUri_2 = 'radio://0/99/2M/E7E7E7E702'
    myUri_3 = 'radio://0/99/2M/E7E7E7E706'
    myUri_4 = 'radio://0/99/2M/E7E7E7E704'
    myUri_5 = 'radio://1/99/2M/E7E7E7E705'
    myUri_6 = 'radio://1/99/2M/E7E7E7E701'
    myUri       = [myUri_1,             # List all link_uri
                   myUri_2,
                   myUri_3,
                   myUri_4,
                   myUri_5,
                   myUri_6]
    form_shape  = [[-0.5, 0.5, 0.0],      # Formation shape for each Crazyflie
                   [0.5, -0.6, 0.0],
                   [-0.3, 0.3, 0.0],
                   [-1.0, 0.0, 0.0],
                   [0.0, 0.1, 0.0],
                   [0.1, 0.0, 0.0]]       
    numCf2      = 2 #len(myUri)            # Number of Crazyflie used   
    myCf2       = [0] * numCf2          # Declare each Crazyflie
    myKbcmd     = [0] * numCf2          # Declare force command from keyboard for each Crazyflie
    matDel      = [1.0, 1.0,            # [] if quad knows reference -> 1, otherwise 0
                   1.0, 1.0, 
                   0.0, 0.0]            


    # Start the threads
#    while True:
    for order_Cf2 in range(0, numCf2):
        print('Connecting to %s...' %str(myUri[order_Cf2]))
        myCf2[order_Cf2]    = formationControl(myUri[order_Cf2], matDel[order_Cf2], form_shape[order_Cf2], order_Cf2)
        #myKbcmd[order_Cf2]  = inputThreat(myCf2[order_Cf2])      
        #time.sleep(0.1)
    ##
    consensus(myCf2)

    while True:
        pass


    #-------------End----------