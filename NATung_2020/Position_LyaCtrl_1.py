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

from threading import Timer
from math import cos, sin, asin, atan2, sqrt, acos

import cflib.crtp  # noqa
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

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

        # Rate of each loop
        self.rate = 50

        # Logged states -, attitude(p, r, y), rotation matrix
        self.position = [0.0, 0.0, 0.0] # [m] in the global frame of reference
        self.velocity = [0.0, 0.0, 0.0] # [m/s] in the global frame of reference
        self.attitude = [0.0, 0.0, 0.0] # [rad] attitude with inverted roll (r)
        self.rotMat   = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]

        # Limits
        self.thrust_limit = (40000,50000)
        self.roll_limit   = (-30.0, 30.0) # [Degree]
        self.pitch_limit  = (-30.0, 30.0) # [Degree]
        self.yaw_limit    = (-200.0, 200.0) # [Degree]

        # Control setting
        self.isEnabled = False

        # System parameter
        self.pi = 3.14159265359

    def _run_controller(self):
        """ Main control loop
        # System parameters"""
        m                       = 0.032         # [kg] actual mass with some decks
        g                       = 9.799848      # [m/s^2] gravitational acceleration
        timeSampling            = 1.0/float(self.rate)  # [s] sampling time
        number_Quad             = 1             # [] number of crazyflie used
        delta                   = 1             # [] if quad knows reference -> 1, otherwise 0

        # Control Parameters
        gv                      = 3     # for velocity
        gp                      = 5     # for position

        # Reference parameter
        v0                      = [0.05, -0.05, 0.05]   # [m/s] reference velocity
        fp                      = [0.0, 0.0, 0.0]   # [m] the formation shape
        fv                      = [0.0, 0.0, 0.0]   # [m/s] the velocity formation shape

        # Set the current reference to the current positional estimate
        # Start logging state
        self.logPos.start()
        self.logVel.start()
        self.logAtt.start()
        time.sleep(5)
        # Take an origin
        x_0, y_0, z_0 = self.position
        yaw_d         = self.attitude[2]
        # Stop logging state
        self.logPos.stop()
        self.logVel.stop()
        self.logAtt.stop()
        print('Initial position reference: ({}, {}, {})'.format(x_0, y_0, z_0))
        x_d, y_d, z_d = [x_0, y_0, z_0]

        # Unlock the controller
        # First, we need send a signal to enable sending attitude reference and thrust force
        self._cf.commander.send_setpoint(0, 0, 0, 0)

        # Begin while loop for sending the control signal
        timeBeginCtrl = time.time()
        while True:
            timeStart = time.time()
            # Change the reference velocity
            if time.time() > timeBeginCtrl + 10.0:
                v0 = [0.0, 0.0, -0.05] 
            # Landing
            if time.time() > timeBeginCtrl + 20.0:
                time.sleep(0.6)

            # Get the reference
            x_d = x_d + v0[0] * timeSampling
            y_d = y_d + v0[1] * timeSampling
            z_d = z_d + v0[2] * timeSampling

            # Start logging state
            self.logPos.start()
            self.logVel.start()
            self.logAtt.start()
            time.sleep(0.01)

            # Get roll, pitch, yaw
            roll, pitch, yaw   = self.attitude
            x, y , z           = self.position
            vx, vy, vz         = self.velocity

            # Stop logging state
            self.logPos.stop()
            self.logVel.stop()
            self.logAtt.stop()

            # Get eta
            eta_px = x - fp[0]
            eta_py = y - fp[1]
            eta_pz = z - fp[2]
            eta_vx = vx - fv[0]
            eta_vy = vy - fv[1]
            eta_vz = vz - fv[2]

            # Get u
            u_x = - gp * delta * (eta_px - x_d) - gv * delta * (eta_vx - v0[0])
            u_y = - gp * delta * (eta_py - y_d) - gv * delta * (eta_vy - v0[1])
            u_z = - gp * delta * (eta_pz - z_d) - gv * delta * (eta_vz - v0[2])
            u = [u_x, u_y, u_z]

            # Get thrust and attitude desired
            #force2thrust_calib = force2thrust/(cos(roll) * cos(pitch))
            thrust_d, roll_d, pitch_d = self.output2nd(u, yaw, m, g)

            # Send set point
            #self._cf.commander.send_setpoint(roll_d, pitch_d, yaw_d, thrust_d)
            #time.sleep(0.001)
            # Test
            #roll_rt = self.attitude[0] * 180 / self.pi 
            #pitch_rt = self.attitude[1] * 180 / self.pi
            #self.loop_sleep(timeStart) 
            #self._cf.commander.send_setpoint(0.0 , 0.0 , yaw_d, thrust_d)
            # Sleep for enough 1 rate
            #print("time, thrust force, roll_d, pitch_d, z_d:", time.time(), thrust_d, roll_d, pitch_d, z_d)
            #print(self.rate)
            message = ('ref:({}, {}, {})\n'.format(x_d, y_d, z_d) + 
                       'pos:({}, {}, {})\n'.format(x, y, z) + 
                       'att:({}, {}, {})\n'.format(roll, pitch, yaw) +
                       'control:({}, {}, {})\n'.format(roll_d, pitch_d, thrust_d))
            self.print_at_period(1.0, message)
            self.loop_sleep(timeStart)
        
        # End     

    def output2nd(self, u_out, yaw, m, g):
        """ This calculates the thrust force and attitude desired """
        # Calculate tau
        tau1 = m * (cos(yaw) * u_out[0] + sin(yaw) * u_out[1])
        tau2 = m * (-sin(yaw) * u_out[0] + cos(yaw) * u_out[1])
        tau3 = m * (u_out[2] + g)

        # Calculate thrust and attitude desired
        thrust  = sqrt(tau1 * tau1 + tau2 * tau2 + tau3 * tau3)
        roll_d  = asin(-tau2/thrust)
        pitch_d = atan2(tau1,tau3)

        # thrust in 16bit and angle to degree
        #thrust_16bit    = thrust * force2thrust
        #thrust_16bit    = int(round(thrust_16bit))
        roll_d          = roll_d * 180 / self.pi
        pitch_d         = pitch_d * 180 / self.pi

        # Saturation
        thrust_16bit_limit  = self.thrust2cmd(thrust)
        roll_d_limit        = self.saturation(roll_d, self.roll_limit)
        pitch_d_limit       = self.saturation(pitch_d, self.pitch_limit)

        return [thrust_16bit_limit, roll_d_limit, pitch_d_limit]

    def thrust2cmd(self, thrust):
        """ This is able to transform thrust in Newton to command in integer 0-65000 
        We need to solve the second order polynominal to find 
        cmd_thrust in integer corresponding to thrust in Newton
        thrust = 4 * (a * cmd_thrust ^ 2 + b * cmd_thrust + c)"""
        a           = 2.130295e-11
        b           = 1.032633e-6
        c           = 5.48456e-4
        delta       = b * b - 4 * a * (c - thrust/4.0)
        cmd_thrust  = (-b + sqrt(delta))/(2 * a)
        cmd_thrust  = int(round(cmd_thrust))
        cmd_thrust  = self.saturation(cmd_thrust, self.thrust_limit)
        return cmd_thrust

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

        # Feedback position estimated by KF
        self.logPos = LogConfig(name='Kalman Position', period_in_ms=50)
        self.logPos.add_variable('kalman.stateX','float')
        self.logPos.add_variable('kalman.stateY','float')
        self.logPos.add_variable('kalman.stateZ','float')

        # Feedback velocity estimated by KF
        self.logVel = LogConfig(name='Kalman Velocity', period_in_ms=50)
        self.logVel.add_variable('kalman.statePX','float')
        self.logVel.add_variable('kalman.statePY','float')
        self.logVel.add_variable('kalman.statePZ','float')

        # Feedback Quaternion attitude estimated by KF
        self.logAtt = LogConfig(name='Kalman Attitude', period_in_ms=50)
        self.logAtt.add_variable('kalman.q0','float')
        self.logAtt.add_variable('kalman.q1','float')
        self.logAtt.add_variable('kalman.q2','float')
        self.logAtt.add_variable('kalman.q3','float')

        # Invoke logged states
        self._cf.log.add_config(self.logPos)
        self._cf.log.add_config(self.logVel)
        self._cf.log.add_config(self.logAtt)

        # Defind callback feedback state
        self.logPos.data_received_cb.add_callback(self.log_pos_callback)
        self.logPos.error_cb.add_callback(logging_error)
        self.logVel.data_received_cb.add_callback(self.log_vel_callback)
        self.logVel.error_cb.add_callback(logging_error)
        self.logAtt.data_received_cb.add_callback(self.log_att_callback)
        self.logAtt.error_cb.add_callback(logging_error)
        
        self.logPos.start()
        self.logVel.start()
        self.logAtt.start()
        
        m                       = 0.032         # [kg] actual mass with some decks
        g                       = 9.799848      # [m/s^2] gravitational acceleration
        timeSampling            = 1.0/float(self.rate)  # [s] sampling time
        number_Quad             = 1             # [] number of crazyflie used
        delta                   = 1             # [] if quad knows reference -> 1, otherwise 0

        # Control Parameters
        gv                      = 3     # for velocity
        gp                      = 5     # for position

        # Reference parameter
        v0                      = [0.05, -0.05, 0.05]   # [m/s] reference velocity
        fp                      = [0.0, 0.0, 0.0]   # [m] the formation shape
        fv                      = [0.0, 0.0, 0.0]   # [m/s] the velocity formation shape

        # Set the current reference to the current positional estimate
        # Start logging state
        if self.logPos.valid:
            self.logPos.start()
            self.logVel.start()
            self.logAtt.start()
            time.sleep(5.0)
        else:
            print('abc')

        # Take an origin
        x_0, y_0, z_0 = self.position
        yaw_d         = self.attitude[2]
        # Stop logging state
        #self.logPos.stop()
        #self.logVel.stop()
        #self.logAtt.stop()
        print('Initial position reference: ({}, {}, {})'.format(x_0, y_0, z_0))
        x_d, y_d, z_d = [x_0, y_0, z_0]


    def log_pos_callback(self, timestamp, data, Logconf):
        """ Callback for the logging the position of the UAV in the global frame"""
        self.position = [
                data['kalman.stateX'], 
                data['kalman.stateY'],
                data['kalman.stateZ']
                        ]
        print('Position x, y, z, time', self.position, time.time())
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
        print('Disconnected from %s' % link_uri)

    def loop_sleep(self, timeStart):
        """ Sleeps the control loop to make it run at a specified rate """
        deltaTime = 1.0/float(self.rate) - (time.time() - timeStart)
        if deltaTime > 0:
            time.sleep(deltaTime)

    def set_reference(self):
        """ Set the position reference in the global frame """


        # end

"""  The main loop"""
if __name__ == "__main__":
    # Initialize the low-level drivers(dont list the debuf drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)

    # Set a chanel - if set None, the first avaiable crazyflie is used
    myUri_1 = 'radio://0/26/2M'
    #myUri_2 = 'radio://0/49/2M'
    #myUri_1 = None

    # All Uri
    #myUri = {myUri_1, myUri_2}

    # If myUri is None, scan for Crazyflies and use the first oene found
    if not myUri_1:
        print('Scanning interfaces for Crazyflies...')
        avaiable_Uri = cflib.crtp.scan_interfaces()
        print('Crazyflies found:')
        for i in avaiable_Uri:
            print(i[0])
        if len(avaiable_Uri) > 0:
            myUri_1 = avaiable_Uri[0][0]
    # Open a text file
    #log_file_text = open("LogData.txt","a")

    # Start the threads
    if myUri_1:
        print('Connecting to %s...' % str(myUri_1))
        controller = formationControl(myUri_1)
        #print('Connecting to %s...' % str(myUri_2))
        #controller_2 = formationControl(myUri_2)
    else:
        print('Could not start - no Crazyflie found')

    while True:
        pass





