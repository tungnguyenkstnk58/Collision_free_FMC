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
This program controlling Crazyflie based on Dynamic Event Triggering Mechanism
"""
import logging
import time
import threading
import sys
import termios
import contextlib

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
        self.link_uri = link_uri
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
        self.isEnabled = True
        self.printDis  = True

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
        gv                      = 3         # for velocity
        gp                      = 5         # for position
        thv                     = 1
        thp                     = 1
        gam                     = 0.2

        # Event triggered parameters
        beta                    = 5
        sigma                   = 0.4
        chi                     = 10
        chi_hat                 = -10
        trigger_on              = 1

        # Reference parameter
        v0                      = [0.1, -0.1, 0.0]   # [m/s] reference velocity
        fp                      = [0.0, 0.0, 0.0]   # [m] the formation shape
        fv                      = [0.0, 0.0, 0.0]   # [m/s] the velocity formation shape

        # Set the current reference to the current positional estimate
        time.sleep(5)
        # Hover at z = 0.3
        self._cf.commander.send_hover_setpoint(0, 0, 0, 0.3)
        time.sleep(2.0)
        x_0, y_0, z_0 = self.position
        yaw_d         = self.attitude[2]
        print('Initial position reference: ({}, {}, {})'.format(x_0, y_0, z_0))
        x_d, y_d, z_d = [x_0, y_0, z_0]

        # Unlock the controller
        # First, we need send a signal to enable sending attitude reference and thrust force
        self._cf.commander.send_setpoint(0, 0, 0, 0)

        # Begin while loop for sending the control signal
        timeBeginCtrl  = time.time()
        timUpdatechi   = time.time()
        timeUpdateCtrl = time.time()
        while True:
            timeStart = time.time()
            # Change the reference velocity
            if time.time() > timeBeginCtrl + 10.0:
                v0 = [0.0, 0.0, -0.05] 
            # Landing
            if time.time() > timeBeginCtrl + 12.0:
                time.sleep(0.6)
                self._cf.close_link()
                print('Disconnect timeout')

            # Get the reference
            x_d = x_d + v0[0] * timeSampling
            y_d = y_d + v0[1] * timeSampling
            z_d = z_d + v0[2] * timeSampling

            # Get roll, pitch, yaw
            roll, pitch, yaw   = self.attitude
            x, y , z           = self.position
            vx, vy, vz         = self.velocity

            # Get eta
            eta_px = x - fp[0]
            eta_py = y - fp[1]
            eta_pz = z - fp[2]
            eta_vx = vx - fv[0]
            eta_vy = vy - fv[1]
            eta_vz = vz - fv[2]

            # Check event triggering condition
            if sigma * chi + chi_hat <= 0:
                eta_px_trig = eta_px
                eta_py_trig = eta_py
                eta_pz_trig = eta_pz
                eta_vx_trig = eta_vx
                eta_vy_trig = eta_vy
                eta_vz_trig = eta_vz
                x_d_trig   = x_d
                y_d_trig   = y_d
                z_d_trig   = z_d
                trigger_on = 1 
            else:
                trigger_on = 0

            # Calcualte eta_wt
            eta_px_wt = eta_px_trig - eta_px
            eta_py_wt = eta_py_trig - eta_py
            eta_pz_wt = eta_pz_trig - eta_pz
            eta_vx_wt = eta_vx_trig - eta_vx
            eta_vy_wt = eta_vy_trig - eta_vy
            eta_vz_wt = eta_vz_trig - eta_vz
            x_d_wt    = x_d_trig   - x_d
            y_d_wt    = y_d_trig   - y_d
            z_d_wt    = z_d_trig   - z_d

            # Calculate zp zv q chi_hat chi_dot
            zpx       = delta * (eta_px - x_d)
            zpy       = delta * (eta_py - y_d)
            zpz       = delta * (eta_pz - z_d)
            zvx       = delta * (eta_vx - v0[0])
            zvy       = delta * (eta_vy - v0[1])
            zvz       = delta * (eta_vz - v0[2])
            q_x       = - gv * delta * eta_vx_wt - gp * delta * eta_px_wt
            q_y       = - gv * delta * eta_vy_wt - gp * delta * eta_py_wt
            q_z       = - gv * delta * eta_vz_wt - gp * delta * eta_pz_wt
            chi_hat  = gam * thp * zpx * zpx + thv * zvx * zvx - (gam * zpx + zvx) * q_x
            chi_hat += gam * thp * zpy * zpy + thv * zvy * zvy - (gam * zpy + zvy) * q_y
            chi_hat += gam * thp * zpz * zpz + thv * zvz * zvz - (gam * zpz + zvz) * q_z
            chi_dot   = - beta * chi + sigma * chi_hat
            chi       = chi + chi_dot * (time.time() - timUpdatechi)
            timUpdatechi = time.time()

            # Get u
            #timeUpdateCtrl = time.time()
            if self.isEnabled:            
                if trigger_on == 1:
                    u_x = - gp * delta * (eta_px_trig - x_d_trig) - gv * delta * (eta_vx_trig - v0[0])
                    u_y = - gp * delta * (eta_py_trig - y_d_trig) - gv * delta * (eta_vy_trig - v0[1])
                    u_z = - gp * delta * (eta_pz_trig - z_d_trig) - gv * delta * (eta_vz_trig - v0[2])
                    u = [u_x, u_y, u_z]
                    thrust_d, roll_d, pitch_d = self.output2nd(u, yaw, m, g)
                    self._cf.commander.send_setpoint(roll_d, pitch_d, yaw_d, thrust_d)
                    timeUpdateCtrl = time.time()
                    #print('trigger at:', timeUpdateCtrl)
                else:
                    if time.time() - timeUpdateCtrl > 0.4:   
                        self._cf.commander.send_setpoint(roll_d, pitch_d, yaw_d, thrust_d) 
                        timeUpdateCtrl = time.time()
                        #print('Reup at:', timeUpdateCtrl)
                    #else:
                        #print('None')
                message = ('ref:({}, {}, {})\n'.format(x_d, y_d, z_d) + 
                       'pos:({}, {}, {})\n'.format(x, y, z) + 
                       'att:({}, {}, {})\n'.format(roll, pitch, yaw) +
                       'control:({}, {}, {})\n'.format(roll_d, pitch_d, thrust_d) + 
                       'link_uri:({})\n'.format(self.link_uri))
                self.print_at_period(2.0, message)
            else:
                self._cf.commander.send_setpoint(0.0, 0.0, 0.0, 0.0)
                self._cf.close_link()
            
            # Sleep for 1 loop   
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

        self.datalog_Pos    = open("log_data/Cf2log_Pos_DET","w")

        # Feedback position estimated by KF
        logPos = LogConfig(name='Kalman Position', period_in_ms=20)
        logPos.add_variable('kalman.stateX','float')
        logPos.add_variable('kalman.stateY','float')
        logPos.add_variable('kalman.stateZ','float')

        # Feedback velocity estimated by KF
        logVel = LogConfig(name='Kalman Velocity', period_in_ms=20)
        logVel.add_variable('kalman.statePX','float')
        logVel.add_variable('kalman.statePY','float')
        logVel.add_variable('kalman.statePZ','float')

        # Feedback Quaternion attitude estimated by KF
        logAtt = LogConfig(name='Kalman Attitude', period_in_ms=20)
        logAtt.add_variable('kalman.q0','float')
        logAtt.add_variable('kalman.q1','float')
        logAtt.add_variable('kalman.q2','float')
        logAtt.add_variable('kalman.q3','float')

        # Invoke logged states
        self._cf.log.add_config(logPos)
        self._cf.log.add_config(logVel)
        self._cf.log.add_config(logAtt)

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
        self.datalog_Pos.write(str(time.time()) + "," +
                                   str(self.position[0]) + "," +
                                   str(self.position[1]) + "," +
                                   str(self.position[2]) + "\n")
        #rint('Position x, y, z, time', self.position, time.time(), self.link_uri)
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
        if self.printDis:
            print('Disconnected from %s' % link_uri)
            self.printDis = False

    def _close_connection(self, message):
        """ This is able to close link connecting Crazyflie from keyboard """
        if message == "q":
            self.isEnabled = False

    def loop_sleep(self, timeStart):
        """ Sleeps the control loop to make it run at a specified rate """
        deltaTime = 1.0/float(self.rate) - (time.time() - timeStart)
        if deltaTime > 0:
            time.sleep(deltaTime)

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

    # Figure
    

    # Start the threads
    for order_Cf2 in range(0, numCf2):
        print('Connecting to %s...' %str(myUri[order_Cf2]))
        myCf2[order_Cf2]    = formationControl(myUri[order_Cf2])
        myKbcmd[order_Cf2]  = inputThreat(myCf2[order_Cf2])
        #time.sleep(0.1)

    while True:
        pass





