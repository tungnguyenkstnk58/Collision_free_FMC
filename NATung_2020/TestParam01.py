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
from matplotlib import pyplot as plt 

from threading import Timer
from math import cos, sin, asin, atan2, sqrt, acos

import cflib.crtp  # noqa
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.high_level_commander import HighLevelCommander

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
        self.timeplt = time.time()

        # Initialize the crazyflie object and add some callbacks
        self._cf = Crazyflie(rw_cache='./cache')
        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)
        self._cf.open_link(link_uri)

        self.rate           = 20 # (Hz) Rate of each sending control input
        self.statefb_rate   = 10 # (ms) Time between two consecutive state feedback

        # Logged states -, attitude(p, r, y), rotation matrix
        self.position = [0.0, 0.0, 0.0] # [m] in the global frame of reference
        self.motor    = [0.0, 0.0, 0.0, 0.0]
        self.Cmd      = [0.0, 0.0, 0.0]
        self.CtrT     = [0.0, 0.0, 0.0]
        self.Mtr      = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.velocity = [0.0, 0.0, 0.0] # [m/s] in the global frame of reference
        self.attitude = [0.0, 0.0, 0.0] # [rad] attitude with inverted roll (r)
        self.rotMat   = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]

        # Limits
        self.thrust_limit = (30000,50000)
        self.roll_limit   = (-30.0, 30.0) # [Degree]
        self.pitch_limit  = (-30.0, 30.0) # [Degree]
        self.yaw_limit    = (-200.0, 200.0) # [Degree]

        # Control setting
        self.isEnabled = True
        self.printDis  = True

        # System parameter
        self.pi = 3.14159265359

    def _run_controller(self):
        """ Main control loop
        System parameters"""
        timeBegin = time.time()
        while True:
            self._cf.commander.send_position_setpoint(x=1.0, y=1.0, z=1.0, yaw=0.0)
            time.sleep(0.2)
            if time.time() - timeBegin > 10.0:
                self._cf.commander.send_position_setpoint(x=1.0, y=1.0, z=0.0, yaw=0.0)
                time.sleep(2.0)
                self._cf.close_link()


    def _connected(self, link_uri):
        """ This callback is called from the Crazyflie API when a Crazyflie
        has been connected adn TOCs have been downloaded """
        print('connected to: %s' %link_uri)

        # Open text file for recording data
        self.datalog_Pos    = open("log_data/Testparam_Pos","w")
        self.datalog_Mt    = open("log_data/Testparam_Mt","w")
        self.datalog_Att    = open("log_data/Testparam_Att","w")
        self.datalog_Cmd    = open("log_data/Testparam_Cmd","w")
        self.datalog_CtrlT    = open("log_data/Testparam_Ctr","w")

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

        # Feedback velocity estimated by KF
        logVel = LogConfig(name='Kalman Velocity', period_in_ms=self.statefb_rate)
        logVel.add_variable('kalman.statePX','float')
        logVel.add_variable('kalman.statePY','float')
        logVel.add_variable('kalman.statePZ','float')

        # Feedback Quaternion attitude estimated by KF
        logAtt = LogConfig(name='Kalman Attitude', period_in_ms=self.statefb_rate)
        logAtt.add_variable('kalman.q0','float')
        logAtt.add_variable('kalman.q1','float')
        logAtt.add_variable('kalman.q2','float')
        logAtt.add_variable('kalman.q3','float')

        # Feedback Motor
        logMt = LogConfig(name='Stabilizer', period_in_ms=self.statefb_rate)
        logMt.add_variable('motor.m1','int32_t')
        logMt.add_variable('motor.m2','int32_t')
        logMt.add_variable('motor.m3','int32_t')
        logMt.add_variable('motor.m4','int32_t')

        # Feedback Cmd
        logCmd = LogConfig(name='Controller', period_in_ms=self.statefb_rate)
        logCmd.add_variable('controller.cmd_thrust','float')
        logCmd.add_variable('controller.cmd_roll','float')
        logCmd.add_variable('controller.cmd_pitch','float')
        logCmd.add_variable('controller.cmd_yaw','float')

        # Feedback Multi ranger
        logMultiRa = LogConfig(name='Range', period_in_ms=self.statefb_rate)
        logMultiRa.add_variable('range.front','uint16_t')
        logMultiRa.add_variable('range.back','uint16_t')
        logMultiRa.add_variable('range.left','uint16_t')
        logMultiRa.add_variable('range.right','uint16_t')
        logMultiRa.add_variable('range.up','uint16_t')

        # Feedback Multi ranger
        logCtrT = LogConfig(name='control target', period_in_ms=self.statefb_rate)
        logCtrT.add_variable('ctrltarget.roll','float')
        logCtrT.add_variable('ctrltarget.pitch','float')
        logCtrT.add_variable('ctrltarget.yaw','float')

        # Invoke logged states
        self._cf.log.add_config(logPos)
        self._cf.log.add_config(logVel)
        self._cf.log.add_config(logAtt)
        self._cf.log.add_config(logMt)
        self._cf.log.add_config(logCmd)
        self._cf.log.add_config(logMultiRa)
        self._cf.log.add_config(logCtrT)

        # Start invoking logged states
        if logPos.valid and logVel.valid:
            # Position
            logPos.data_received_cb.add_callback(self.log_pos_callback)
            logPos.error_cb.add_callback(logging_error)
            logPos.start()
            # Velocity
            logVel.data_received_cb.add_callback(self.log_vel_callback)
            logVel.error_cb.add_callback(logging_error)
            logVel.start()
            # Quadternion attitude
            logAtt.data_received_cb.add_callback(self.log_att_callback)
            logAtt.error_cb.add_callback(logging_error)
            logAtt.start()
            #
            # Quadternion attitude
            logMt.data_received_cb.add_callback(self.log_mt_callback)
            logMt.error_cb.add_callback(logging_error)
            logMt.start()
            #
            # Quadternion attitude
            logCmd.data_received_cb.add_callback(self.log_cmd_callback)
            logCmd.error_cb.add_callback(logging_error)
            logCmd.start()
            # Quadternion attitude
            logMultiRa.data_received_cb.add_callback(self.log_mtr_callback)
            logMultiRa.error_cb.add_callback(logging_error)
            logMultiRa.start()
            # Quadternion attitude
            logCtrT.data_received_cb.add_callback(self.log_ctr_callback)
            logCtrT.error_cb.add_callback(logging_error)
            logCtrT.start()

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

    def log_mt_callback(self, timestamp, data, Logconf):
        """ Callback for the logging the position of the UAV in the global frame"""
        self.motor = [
                data['motor.m1'], 
                data['motor.m2'],
                data['motor.m3'],
                data['motor.m4']
                        ]

        self.datalog_Mt.write(str([time.time(), self.motor]) + "\n")
        #print('Position x, y, z, time', self.position, time.time())
        #print(self.rate)

    def log_cmd_callback(self, timestamp, data, Logconf):
        """ Callback for the logging the position of the UAV in the global frame"""
        self.Cmd = [
                data['controller.cmd_thrust'],
                data['controller.cmd_roll'], 
                data['controller.cmd_pitch'],
                data['controller.cmd_yaw']
                        ]

        self.datalog_Cmd.write(str([time.time(), self.Cmd]) + "\n")
        #print('Position x, y, z, time', self.position, time.time())
        #print(self.rate)


    def log_ctr_callback(self, timestamp, data, Logconf):
        """ Callback for the logging the position of the UAV in the global frame"""
        self.CtrT = [
                data['ctrltarget.roll'], 
                data['ctrltarget.pitch'],
                data['ctrltarget.yaw']
                        ]

        self.datalog_CtrlT.write(str([time.time(), self.CtrT]) + "\n")
        #print('Position x, y, z, time', self.position, time.time())
        #print(self.rate)

    def log_mtr_callback(self, timestamp, data, Logconf):
        """ Callback for the logging the position of the UAV in the global frame"""
        self.Mtr = [
                data['range.front'], 
                data['range.back'],
                data['range.left'],
                data['range.right'],
                data['range.up'],
                        ]

        #self.datalog_Cmd.write(str([time.time(), self.Cmd]) + "\n")
        #print('Time, Multirange: front, back, left, right, up', time.time(), self.Mtr)
        #print(self.rate)

    def log_att_callback(self, timestamp, data, Logconf):
        """ Callback for the logging the quadternion attitude of the UAV which is
        converted to roll, pitch, yaw in radians"""
        q    = [
            data['kalman.q0'], 
            data['kalman.q1'],
            data['kalman.q2'],
            data['kalman.q3']
                ]
        #Convert the quadternion to pitch roll and yaw
        yaw  = atan2(2*(q[1]*q[2]+q[0]*q[3]) , q[0]*q[0] + q[1]*q[1] - q[2]*q[2] - q[3]*q[3])
        pitch = asin(-2*(q[1]*q[3] - q[0]*q[2]))
        roll  = atan2(2*(q[2]*q[3]+q[0]*q[1]) , q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3])
        self.attitude = [roll, pitch, yaw]

        # Convert the quaternion to a rotation matrix
        R = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
        R[0][0] = q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]
        R[0][1] = 2 * q[1] * q[2] - 2 * q[0] * q[3]
        R[0][2] = 2 * q[1] * q[3] + 2 * q[0] * q[2]
        R[1][0] = 2 * q[1] * q[2] + 2 * q[0] * q[3]
        R[1][1] = q[0] * q[0] - q[1] * q[1] + q[2] * q[2] - q[3] * q[3]
        R[1][2] = 2 * q[2] * q[3] - 2 * q[0] * q[1]
        R[2][0] = 2 * q[1] * q[3] - 2 * q[0] * q[2]
        R[2][1] = 2 * q[2] * q[3] + 2 * q[0] * q[1]
        R[2][2] = q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]
        self.rotMat = R

        self.datalog_Att.write(str([time.time(), self.attitude]) + "\n")
        #print('Attitude r, p, y', self.attitude)

    def log_vel_callback(self, timestamp, data, Logconf):
        """  Callback for logging the velocity of the UAV defined w.r.t the body frame
        this subsequently rotated to the global frame"""
        PX, PY, PZ = [
                data['kalman.statePX'],
                data['kalman.statePY'],
                data['kalman.statePZ']
                    ]
        R = self.rotMat
        self.velocity[0] = R[0][0]*PX + R[0][1]*PY + R[0][2]*PZ
        self.velocity[1] = R[1][0]*PX + R[1][1]*PY + R[1][2]*PZ
        self.velocity[2] = R[2][0]*PX + R[2][1]*PY + R[2][2]*PZ

        #self.datalog_Vel.write(str([time.time(), self.velocity]) + "\n")
        #print('Velocity rx, ry, rz', self.velocity)

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
    myUri_1 = 'radio://1/70/2M/E7E7E7E702'
    myUri_2 = 'radio://0/70/2M/E7E7E7E701'
    myUri_3 = 'radio://1/70/2M/E7E7E7E703'

    # All Uri
    myUri       = [myUri_1,             # List all link_uri  
                   myUri_2,
                   myUri_3]   
    numCf2      = len(myUri)            # Number of Crazyflie used
    myCf2       = [0] * numCf2          # Declare each Crazyflie
    myKbcmd     = [0] * numCf2          # Declare force command from keyboard for each Crazyflie 

    # Figure
    

    # Start the threads
    for order_Cf2 in range(0, numCf2):
        print('Connecting to %s...' %str(myUri[order_Cf2]))
        myCf2[order_Cf2]    = formationControl(myUri[order_Cf2])
        myKbcmd[order_Cf2]  = inputThreat(myCf2[order_Cf2])
        #time.sleep(0.1)

    while True:
        pass





