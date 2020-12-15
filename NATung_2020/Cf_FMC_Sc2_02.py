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

    def __init__(self, link_uri, delta_ref, formation_fp, order_Cf2, num_Cf2):
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
        self.statefb_rate   = 200 # (ms) Time between two consecutive state feedback

        # Logged states -, attitude(p, r, y), rotation matrix
        self.position       = [0.0, 0.0, 0.0] # [m] in the global frame of reference
        self.velocity       = [0.0, 0.0, 0.0] # [m/s] in the global frame of reference
        self.attitude       = [0.0, 0.0, 0.0] # [rad] attitude with inverted roll (r)
        self.rotMat         = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
        self.pos_other      = ([0.0] * 3) * (num_Cf2 - 1)
        self.formation_fp   = formation_fp    # Formation shape for each agent
        self.ob_avoid       = [0.0, 0.0, 0.0]
        self.ob_avoid_other = [0.0, 0.0, 0.0]

        # Control setting
        self.isEnabled = True
        self.printDis  = True

        # System parameter
        self.pi         = 3.14159265359
        self.delta_ref  = delta_ref
        self.order_Cf2  = order_Cf2
        self.num_Cf2    = num_Cf2

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
        sh_start                = 0.0
        sh_end                  = 10.0
        alt_fix                 = 0.4
        alt_residue             = 0.05 * self.order_Cf2

        # Collision parameters
        col_thrs                = 0.5               
        safe_thrs               = 1.0
        col_muy                 = 0.4
        col_lda                 = -0.00
        dis_col                 = [0.0] * (self.num_Cf2 - 1)

        # Set the current reference to the current positional estimate
        time.sleep(3)
        # Unlock the controller
        # First, we need send a signal to enable sending attitude reference and thrust force        
        self._cf.commander.send_setpoint(0, 0, 0, 0)
        # Hover at z = 0.3 durin 2 seconds
        x_0, y_0, z_0 = self.position
        yaw_0         = self.attitude[2]
        timeStart = time.time()
        while True:
            self._cf.commander.send_position_setpoint(x=x_0,y=y_0,z=alt_fix+alt_residue,yaw=yaw_0)
            time.sleep(0.2)
            if time.time() - timeStart > 4.0:
                break
        # Take a local coordinate
        x_0, y_0, z_0 = self.position
        yaw_d         = self.attitude[2]
        print('Initial position reference: ({}, {}, {})'.format(x_0, y_0, z_0))
        x_d, y_d, z_d = [0.0, 0.0, 0.0]       # initial time
        pos_other     = ([0.0] * 3) * 1


        # Begin while loop for sending the control signal
        timeBeginCtrl = time.time()
        while True:
            timeStart = time.time()
            # Change the reference velocity
            if time.time() > timeBeginCtrl + 0.0:
                v0          = [0.0, 0.0, 0.0]
                delta       = self.delta_ref 
            if time.time() > timeBeginCtrl + 15.0:
                v0          = [0.2, -0.0, 0.0]
            if time.time() > timeBeginCtrl + 28.0:
                v0          = [-0.2, 0.0, 0.0] 
            if time.time() > timeBeginCtrl + 25.0:
                v0          = [0.0, 0.0, 0.0]                  
            if time.time() > timeBeginCtrl + 27.0:
                self._cf.commander.send_setpoint(0, 0, 0, 40000)
                self.myPos_log.close()
                self.myInput_log.close()
                self._cf.close_link()
                print('Disconnect timeout')
                break
            
            if self.order_Cf2 < self.num_Cf2 - 1:
                # Get the reference
                x_d = x_d + v0[0] * timeSampling
                y_d = y_d + v0[1] * timeSampling
                z_d = z_d + v0[2] * timeSampling
            else: 
                x_d, y_d, z_d = [0.0, 0.0, 0.0] 


            # Get roll, pitch, yaw
            roll, pitch, yaw   = self.attitude
            x, y , z           = self.position
            vx, vy, vz         = self.velocity

            # A loop computes a potential function producing repulsive force
            u_x_c = 0
            u_y_c = 0
            u_z_c = 0
            u_z_c1= 0
            count_near = 0
            dis_col_mess = ","

            for i in range(0, self.num_Cf2 - 2):
                # Get position from others
                pos_other   = self.pos_other[i][0:3]
            
                # Collision avoidance
                e_col_x     = x - pos_other[0]
                e_col_y     = y - pos_other[1]
                e_col_z     = z - pos_other[2]
                dis_col[i]  = sqrt(e_col_x*e_col_x + e_col_y*e_col_y + e_col_z*e_col_z) - 0.1
            
                # collion-free force
                repul_force = self.bump_fcn_dot(dis_col[i],col_thrs,safe_thrs,col_muy,col_lda)
                u_x_c       = u_x_c - repul_force * 2 * e_col_x #* vx 
                u_y_c       = u_y_c - repul_force * 2 * e_col_y #* vy
                u_z_c       = u_z_c - repul_force * 1.5 * e_col_z #* vz
                #u_z_c1      = u_z_c1+ repul_force * 2 * e_col_z #* vz 
                #u_z_c       = u_z_c + u_x_c + u_y_c - repul_force * 2 * e_col_z
                dis_col_mess = dis_col_mess + str(dis_col[i]) + ","
                if repul_force != 0.0:
                    count_near = count_near + 1
            ##    
            u_z_c = self.saturation(u_z_c, [-0.2, 0.5])
            

            ## A loop computes repulsive force to avoid obstacle
            u_x_ob = 0
            u_y_ob = 0
            u_z_ob = 0

            pos_ob     = self.pos_other[self.num_Cf2 - 2][0:3]
            e_ob_x     = x - pos_ob[0]
            e_ob_y     = y - pos_ob[1]
            e_ob_z     = z - pos_ob[2] 
            dis_ob     = sqrt(e_ob_x*e_ob_x + e_ob_y*e_ob_y + e_ob_z*e_ob_z) - 0.1
            repul_ob   = self.bump_fcn_dot(dis_ob,col_thrs,safe_thrs,col_muy,col_lda)
            u_x_ob     = - repul_ob * 1 * e_ob_x
            u_y_ob     = - repul_ob * 1 * e_ob_y
            u_z_ob     = - repul_ob * 1 * (e_ob_z + 0.03)
            u_z_ob     = self.saturation(u_z_ob, [-0.2, 0.5])
            self.ob_avoid = [u_x_ob, u_y_ob, u_z_ob]

            # Setpoint
            x_d_c = x_0 + (x_d - x_0 + self.formation_fp[0]) * self.step_fcn(time.time(), timeBeginCtrl + sh_start, timeBeginCtrl + sh_end) 
            y_d_c = y_0 + (y_d - y_0 + self.formation_fp[1]) * self.step_fcn(time.time(), timeBeginCtrl + sh_start, timeBeginCtrl + sh_end) 
            z_d_c = alt_fix + alt_residue - alt_residue * self.step_fcn(time.time(), timeBeginCtrl + sh_start, timeBeginCtrl + sh_end) 

            # Setpoint with collision-free
            x_d_cf = x_d_c + u_x_c + u_x_ob + self.ob_avoid_other[0]
            y_d_cf = y_d_c + u_y_c + u_y_ob + self.ob_avoid_other[1]
            z_d_cf = z_d_c + u_z_c + u_z_ob + self.ob_avoid_other[2]

            if self.order_Cf2 < self.num_Cf2 - 1:
                # Send set point
                self._cf.commander.send_position_setpoint(x_d_cf,y_d_cf,z_d_cf,yaw_d)
            else:
                self._cf.commander.send_position_setpoint(x_d_c,y_d_c,z_d_c,yaw_d)
            #if self.order_Cf2 == 0:
            #    self._cf.commander.send_position_setpoint(x_d_c,y_d_c,0.4,yaw_d)
            #else:
            #    self._cf.commander.send_setpoint(0, 0, 0, 20000)
            
            ## Monitoring
            message = ('ref:({}, {}, {})\n'.format(x_d_c, y_d_c, z_d) + 
                           'pos:({}, {}, {})\n'.format(x, y, z) + 
                           'pos_other:({}, {}, {})\n'.format(pos_other[0], pos_other[1], pos_other[2]) + 
                           'dis:({})\n'.format(dis_col))

            message1 = ('ob_other :({}, {}, {})\n'.format(self.ob_avoid_other[0], self.ob_avoid_other[1], self.ob_avoid_other[2]))  
            message2 = ('ob_avoid :({}, {}, {})\n'.format(u_x_ob,u_y_ob,u_z_ob))
            if self.order_Cf2 == 1: 
                self.print_at_period(0.5, message1) 
            ##
            if self.order_Cf2 == 0:
                self.print_at_period(0.5, message2)

            # Storage data
            self.myPos_log.write(str(time.time()) + "," + str(x) + "," + str(y) + "," + str(z) + "\n")
            self.myInput_log.write(str(time.time()) + "," + str(u_x_c) + "," + str(u_y_c) + "," + str(u_z_c) + dis_col_mess + "\n")
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

    def bump_fcn_dot(self, dis, col_thrs, safe_thrs, muy, lda):
        """ This is the derivative of the bump function"""
        if dis > col_thrs:
            mr_bump_dot = lda * self.step_dot_fcn(dis, col_thrs, safe_thrs)
        else:
            mr_bump_dot = 2.0 * dis + 3.0 * col_thrs * col_thrs * col_thrs / muy + col_thrs
            mr_bump_dot = - mr_bump_dot * (col_thrs - dis) * (col_thrs - dis)
            mr_bump_dot = mr_bump_dot / ((dis + col_thrs * col_thrs * col_thrs / muy)*(dis + col_thrs * col_thrs * col_thrs / muy)) 

        return mr_bump_dot

    '''def bump_fcn_dot(self, mr, col_thrs, safe_thrs, muy, lda):
        """ This is the derivative of the bump function"""
        if mr > col_thrs:
            mr_bump_dot = lda * self.step_dot_fcn(mr, col_thrs, safe_thrs)
        else:
            mr_bump_dot = float(mr) + 2.0 * col_thrs * col_thrs / muy + col_thrs
            mr_bump_dot = - mr_bump_dot * (col_thrs - float(mr))
            mr_bump_dot = mr_bump_dot / ((float(mr) + col_thrs * col_thrs / muy)*(float(mr) + col_thrs * col_thrs / muy)) 

        return mr_bump_dot'''

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

        # Feedback velocity estimated by KF
        logVel = LogConfig(name='Kalman Velocity', period_in_ms=self.statefb_rate )
        logVel.add_variable('kalman.statePX','float')
        logVel.add_variable('kalman.statePY','float')
        logVel.add_variable('kalman.statePZ','float')

        # Feedback Quaternion attitude estimated by KF
        logAtt = LogConfig(name='Kalman Attitude', period_in_ms=self.statefb_rate )
        logAtt.add_variable('kalman.q0','float')
        logAtt.add_variable('kalman.q1','float')
        logAtt.add_variable('kalman.q2','float')
        logAtt.add_variable('kalman.q3','float')

        # Invoke logged states
        self._cf.log.add_config(logPos)
        self._cf.log.add_config(logVel)
        self._cf.log.add_config(logAtt)

        # Start invoking logged states
        if logPos.valid:
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

        #self.myPos_log.write(str(time.time()) + "," +
        #                         str(self.position[0]) + "," +
        #                         str(self.position[1]) + "," +
        #                         str(self.position[2]) + "\n")
        #print('Position x, y, z, time', self.position, time.time())
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

        #self.datalog_Att.write(str([time.time(), self.attitude]) + "\n")
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

    def subscribe_other_infor(self, pos_other):
        """ This function updates the information including eta_p and eta_v from
        other Crazyflies """
        self.pos_other = pos_other

    def get_publish_infor(self):
        """ This calback take the information of this Crazyflie"""
        return self.position

    def get_publish_ob_avoid(self):
        return self.ob_avoid

    def subscribe_other_ob(self, ob_avoid):
        self.ob_avoid_other = ob_avoid

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
        self.pos_all = [[0] * 3] * self.numCf2  # Position of all Cf2
        self.ob_all  = [[0] * 3] * self.numCf2    
        self.rate    = 30   # [Hz]
        time.sleep(2.0)
        self.start()

    def run(self):
        while True:
            timeStart = time.time()
            for i in range(0, self.numCf2):
                self.pos_all[i] = self.myCf2[i].get_publish_infor()
                self.ob_all[i]  = self.myCf2[i].get_publish_ob_avoid()
            ##
            u_ob_all = [0.0, 0.0, 0.0]
            for i in range(0, self.numCf2):
                pos_other  = [[0.0] * 3] * (self.numCf2 - 1)
                u_ob_all[0] = u_ob_all[0] + self.ob_all[i][0]
                u_ob_all[1] = u_ob_all[1] + self.ob_all[i][1]
                u_ob_all[2] = u_ob_all[2] + self.ob_all[i][2]
                for j in range(0, self.numCf2):
                    if j < i:
                        pos_other[j] = self.pos_all[j]
                    if j > i:
                        pos_other[j-1] = self.pos_all[j]


                ##
                self.myCf2[i].subscribe_other_infor(pos_other) 
            ##
            for i in range(0, self.numCf2):
                self.myCf2[i].subscribe_other_ob(u_ob_all) 

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
    myUri_3 = 'radio://1/99/2M/E7E7E7E705'
    myUri_4 = 'radio://1/99/2M/E7E7E7E704'
    myUri_5 = 'radio://1/99/2M/E7E7E7E705'
    myUri_6 = 'radio://1/99/2M/E7E7E7E701'
    myUri       = [myUri_1,             # List all link_uri
                   myUri_2,
                   myUri_3,
                   myUri_4,
                   myUri_5,
                   myUri_6]
    form_shape  = [[-0.2, 0.4, 0.0],      # Formation shape for each Crazyflie
                   [-0.2, -0.4, 0.0],
                   [0.8, 0.45, 0.0],
                   [0.6, -0.4, 0.0],
                   [0.0, 0.1, 0.0],
                   [0.1, 0.0, 0.0]]       
    numCf2      = 3 #len(myUri)            # Number of Crazyflie used   
    myCf2       = [0] * numCf2          # Declare each Crazyflie
    myKbcmd     = [0] * numCf2          # Declare force command from keyboard for each Crazyflie
    matDel      = [1.0, 1.0,            # [] if quad knows reference -> 1, otherwise 0
                   1.0, 1.0, 
                   0.0, 0.0]            


    # Start the threads
#    while True:
    for order_Cf2 in range(0, numCf2):
        print('Connecting to %s...' %str(myUri[order_Cf2]))
        myCf2[order_Cf2]    = formationControl(myUri[order_Cf2], matDel[order_Cf2], form_shape[order_Cf2], order_Cf2, numCf2)
        #myKbcmd[order_Cf2]  = inputThreat(myCf2[order_Cf2])      
        #time.sleep(0.1)
    ##
    consensus(myCf2)

    while True:
        pass


    #-------------End----------