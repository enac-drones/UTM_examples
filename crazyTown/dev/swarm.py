import logging
import sys
import time
from threading import Event
import numpy as np

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.utils import uri_helper


class Vehicle():
    def __init__(self,uri:str) -> None:
        self._uri = uri
        self._position = np.zeros(3)
        self._velocity = np.zeros(3)
        self._battery = 0.0
        self.scf = SyncCrazyflie(self._uri, cf=Crazyflie(rw_cache='./cache'))
        # self.start_state_update()

    @property
    def position(self):
        self.position_update_callback()
        # self.log_conf.reset()
        return self._position

    @property
    def velocity(self):
        self.velocity_update_callback()
        return self._velocity

    @property
    def battery(self):
        self.battery_update_callback()
        return self._battery
    
    def start_state_update(self):
        self.log_conf = LogConfig(name='Position', period_in_ms=10)
        self.log_conf.add_variable('kalman.stateX', 'float')
        self.log_conf.add_variable('kalman.stateY', 'float')
        self.log_conf.add_variable('kalman.stateZ', 'float')
        # self.log_conf.add_variable('pm.vbat', 'float')

        # self.log_conf.add_variable('stateEstimate.vx', 'float')
        # self.log_conf.add_variable('stateEstimate.vy', 'float')
        # self.log_conf.add_variable('stateEstimate.vz', 'float')

        # self.log_conf.add_variable('stateEstimate.ax', 'float')
        # self.log_conf.add_variable('stateEstimate.ay', 'float')
        # self.log_conf.add_variable('stateEstimate.az', 'float')

        # self.scf.cf.log.add_config(self.log_conf)
        # self.log_conf.data_received_cb.add_callback(self.position_update_callback())
        # self.log_conf.start()

    def position_update_callback(self):
        self.log_conf = LogConfig(name='Position', period_in_ms=10)
        self.log_conf.add_variable('kalman.stateX', 'float')
        self.log_conf.add_variable('kalman.stateY', 'float')
        self.log_conf.add_variable('kalman.stateZ', 'float')
        with SyncLogger(self.scf, self.log_conf) as logger:
            for entry in logger:
                x = entry[1]['kalman.stateX']
                y = entry[1]['kalman.stateY']
                z = entry[1]['kalman.stateZ']
                # vb = entry[1]['pm.vbat']
                self._position = np.array([x,y,z])
                # self._battery = vb
                break

    def velocity_update_callback(self):
        self.log_conf = LogConfig(name='Position', period_in_ms=10)
        self.log_conf.add_variable('stateEstimate.vx', 'float')
        self.log_conf.add_variable('stateEstimate.vy', 'float')
        self.log_conf.add_variable('stateEstimate.vz', 'float')
        with SyncLogger(self.scf, self.log_conf) as logger:
            for entry in logger:
                dx = entry[1]['stateEstimate.vx']
                dy = entry[1]['stateEstimate.vy']
                dz = entry[1]['stateEstimate.vz']
                self._velocity = np.array([dx,dy,dz])
                break

    def battery_update_callback(self):
        with SyncLogger(self.scf, self.log_conf) as logger:
            for entry in logger:
                vb = entry[1]['pm.vbat']
                self._battery = vb
                break

    def shut_down(self):
        # self.log_conf.stop()
        self.scf.close_link()

class Swarm():
    def __init__(self,uris:list) -> None:
        self._uris = uris
        self._vehicles = [Vehicle(uri) for uri in uris]
        for vehicle in self._vehicles:
            vehicle.scf.open_link()
            time.sleep(0.1)
        
    def take_off(self):
        for vehicle in self._vehicles:
            vehicle.scf.cf.high_level_commander.takeoff(0.5, 1.5)
        time.sleep(1.5)
    
    def land(self):
        landing_duration = 3.0
        for vehicle in self._vehicles:
            vehicle.scf.cf.high_level_commander.land(0.0, landing_duration)
        time.sleep(landing_duration)

    def close(self):
        for vehicle in self._vehicles:
            vehicle.shut_down()
        time.sleep(0.5)