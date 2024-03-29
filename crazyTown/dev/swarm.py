import logging
import sys
import time
from threading import Event
import numpy as np
from enum import IntEnum

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.utils import uri_helper

import zmq

class VehicleState(IntEnum):
    GROUNDED = 0
    STARTED_TAKE_OFF = 1
    INFLIGHT = 2
    STARTED_LANDING = 3
    FINISHED_LANDING = 4 
    
class Vehicle():
    def __init__(self,uri:str, report_socket=None) -> None:
        self.report_socket=report_socket
        self._uri = uri
        self._position = np.zeros(3)
        self._velocity = np.zeros(3)
        self.ref_vel_enu = np.zeros(3)
        self.ref_heading = 0.0
        self._battery = 0.0
        self._takeoff_pos = np.zeros(3)
        self._landing_pos = np.zeros(3)
        self._vehicle_state = 0
        self._state_duration = 0.0
        self._hover_height = 0.4
        self._descent_height = 0.15
        self._cf = Crazyflie(rw_cache='./cache')

        # Connect some callbacks from the Crazyflie API
        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)

        # self.scf = SyncCrazyflie(self._uri, cf=self._cf)

        print('Connecting to %s' % uri)

        # Try to connect to the Crazyflie
        self._cf.open_link(uri)

        # Variable used to keep main loop occupied until disconnect
        self.is_connected = True

    def fly(self):
        # According to Vehicle_state , do different things
        match self._vehicle_state:
            case 0: #Takeoff
                self._cf.high_level_commander.takeoff(0.4, 1.5)
                self.check_state_duration(1.5, 1)
            case 1: #Hover
                self._cf.high_level_commander.go_to(self._takeoff_pos[0], self._takeoff_pos[1], self._takeoff_pos[2]+ self._hover_height, 0, 0.1)
                self.check_state_duration(1, 2)
            case 2: #Guidance with Velocity Setpoint
                self._cf.commander.send_velocity_world_setpoint( self.ref_vel_enu[0],
                                                                self.ref_vel_enu[1],
                                                                self.ref_vel_enu[2],
                                                                yawrate=self.ref_heading)
            case 4: #Descend
                self._cf.high_level_commander.go_to(self._landing_pos[0], self._landing_pos[1], self._landing_pos[2]+ self._descent_height, 0, 0.1)
                self.check_state_duration(1.5, 5) # Change to 5 after 1.5 secs
                print('Landing')
            case 5: #Land
                print('Landing')
                pass

    def check_state_duration(self, duration_s, new_state):
        self._state_duration += 0.1
        if self._state_duration > duration_s:
            self._vehicle_state = new_state
            self._state_duration = 0.0

    @property
    def position(self):
        # self.position_update_callback()
        # self.log_conf.reset()
        return self._position

    @property
    def velocity(self):
        # self.velocity_update_callback()
        return self._velocity

    @property
    def battery(self):
        # self.battery_update_callback()
        return self._battery

    def _connected(self, link_uri):
        """ This callback is called form the Crazyflie API when a Crazyflie
        has been connected and the TOCs have been downloaded."""
        print('Connected to %s' % link_uri)

        # The definition of the logconfig can be made before connecting
        self._lg_stab = LogConfig(name='Stabilizer', period_in_ms=100)
        self._lg_stab.add_variable('stateEstimate.x', 'float')
        self._lg_stab.add_variable('stateEstimate.y', 'float')
        self._lg_stab.add_variable('stateEstimate.z', 'float')
        self._lg_stab.add_variable('stateEstimate.vx', 'float')
        self._lg_stab.add_variable('stateEstimate.vy', 'float')
        self._lg_stab.add_variable('stateEstimate.vz', 'float')
        # The fetch-as argument can be set to FP16 to save space in the log packet
        self._lg_stab.add_variable('pm.vbat', 'FP16')

        # Adding the configuration cannot be done until a Crazyflie is
        # connected, since we need to check that the variables we
        # would like to log are in the TOC.
        try:
            self._cf.log.add_config(self._lg_stab)
            # This callback will receive the data
            self._lg_stab.data_received_cb.add_callback(self._stab_log_data)
            # This callback will be called on errors
            self._lg_stab.error_cb.add_callback(self._stab_log_error)
            # Start the logging
            self._lg_stab.start()
        except KeyError as e:
            print('Could not start log configuration,'
                  '{} not found in TOC'.format(str(e)))
        except AttributeError:
            print('Could not add StateEstimate log config, bad configuration.')

        # Start a timer to disconnect in 10s
        # t = Timer(180, self._cf.close_link)
        # t.start()

    def _stab_log_error(self, logconf, msg):
        """Callback from the log API when an error occurs"""
        print('Error when logging %s: %s' % (logconf.name, msg))

    def _stab_log_data(self, timestamp, data, logconf):
        """Callback from a the log API when data arrives"""
        self._position[0] = data['stateEstimate.x']
        self._position[1] = data['stateEstimate.y']
        self._position[2] = data['stateEstimate.z']
        self._velocity[0] = data['stateEstimate.vx']
        self._velocity[1] = data['stateEstimate.vx']
        self._velocity[2] = data['stateEstimate.vx']
        self._battery = data['pm.vbat']
    
        # print(f'[{timestamp}][{logconf.name}]: ', end='')
        # for name, value in data.items():
        #     print(f'{name}: {value:3.3f} ', end='')
        # print()

    def _connection_failed(self, link_uri, msg):
        """Callback when connection initial connection fails (i.e no Crazyflie
        at the specified address)"""
        print('Connection to %s failed: %s' % (link_uri, msg))
        self.is_connected = False

    def _connection_lost(self, link_uri, msg):
        """Callback when disconnected after a connection has been made (i.e
        Crazyflie moves out of range)"""
        print('Connection to %s lost: %s' % (link_uri, msg))

    def _disconnected(self, link_uri):
        """Callback when the Crazyflie is disconnected (called in all cases)"""
        print('Disconnected from %s' % link_uri)
        self.is_connected = False

    def send_report(self):
        if self.report_socket is None:
            return

        # for i, controller in enumerate(self.controllers):
        state = "idle"
        if not self._cf.is_connected():
            state = "disconnected"
        # elif self._cf.is_crashed():
        #     state = "crashed"
        # elif self._cf.is_flying():
        #     state = "flying"
        # elif self._cf.is_taking_off():
        #     state = "hovering"
        # elif self._cf.is_landing():
        #     state = "landing"
        # elif self._cf.is_charged_for_flight():
        #     state = "ready"
        # elif self._cf.is_charging():
        #     state = "charging"

        try:
            report = {
                'id': 1,
                'state': state,
                'battery': self._battery,#controller.get_charge_level(),
                'uptime': 10, #controller.up_time_ms,
                'flighttime': 100, #controller.flight_time_ms,
            }
            self.report_socket.send_json(report, zmq.NOBLOCK)
        except Exception:
            pass
    
    # def start_state_update(self):
    #     self.log_conf = LogConfig(name='Position', period_in_ms=100)
    #     self.log_conf.add_variable('kalman.stateX', 'float')
    #     self.log_conf.add_variable('kalman.stateY', 'float')
    #     self.log_conf.add_variable('kalman.stateZ', 'float')
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

    # def position_update_callback(self):
    #     self.log_conf = LogConfig(name='Position', period_in_ms=100)
    #     self.log_conf.add_variable('kalman.stateX', 'float')
    #     self.log_conf.add_variable('kalman.stateY', 'float')
    #     self.log_conf.add_variable('kalman.stateZ', 'float')
    #     with SyncLogger(self.scf, self.log_conf) as logger:
    #         for entry in logger:
    #             x = entry[1]['kalman.stateX']
    #             y = entry[1]['kalman.stateY']
    #             z = entry[1]['kalman.stateZ']
    #             # vb = entry[1]['pm.vbat']
    #             self._position = np.array([x,y,z])
    #             # self._battery = vb
    #             break

    # def velocity_update_callback(self):
    #     self.log_conf = LogConfig(name='Position', period_in_ms=100)
    #     self.log_conf.add_variable('stateEstimate.vx', 'float')
    #     self.log_conf.add_variable('stateEstimate.vy', 'float')
    #     self.log_conf.add_variable('stateEstimate.vz', 'float')
    #     with SyncLogger(self.scf, self.log_conf) as logger:
    #         for entry in logger:
    #             dx = entry[1]['stateEstimate.vx']
    #             dy = entry[1]['stateEstimate.vy']
    #             dz = entry[1]['stateEstimate.vz']
    #             self._velocity = np.array([dx,dy,dz])
    #             break

    # def battery_update_callback(self):
    #     with SyncLogger(self.scf, self.log_conf) as logger:
    #         for entry in logger:
    #             vb = entry[1]['pm.vbat']
    #             self._battery = vb
    #             break

    def shut_down(self):
        # self.log_conf.stop()
        self._cf.close_link()

class Swarm():
    def __init__(self,uris:list, report_socket=None) -> None:
        self._uris = uris
        self._vehicles = [Vehicle(uri, report_socket=report_socket) for uri in uris]
        self._state = 0
        for vehicle in self._vehicles:
            # vehicle._cf.open_link()
            time.sleep(0.1)
    
    @property
    def state(self): # FIX ME , this is hardcodded :(
        self._state = 5 if np.all([vehicle._vehicle_state == 5 for vehicle in self._vehicles]) else 0
        return self._state
    
    @state.setter
    def state(self, value):
        self._state = value

    def take_off(self):
        for vehicle in self._vehicles:
            vehicle._cf.high_level_commander.takeoff(0.4, 1.5)
        time.sleep(1.5)
    
    def fly(self):
        for vehicle in self._vehicles:
            vehicle.fly()
        

    def land(self):
        landing_duration = 3.0
        for vehicle in self._vehicles:
            vehicle._cf.high_level_commander.land(0.0, landing_duration)
        time.sleep(landing_duration)

    def close(self):
        for vehicle in self._vehicles:
            vehicle.shut_down()
        time.sleep(0.5)

if __name__ == '__main__':
    # Initialize the low-level drivers
    cflib.crtp.init_drivers()

    URI1 = uri_helper.uri_from_env(default='radio://0/80/1M/E7E7E7E701')
    # URI2 = uri_helper.uri_from_env(default='radio://0/80/1M/E7E7E7E702')
    # URI3 = uri_helper.uri_from_env(default='radio://0/80/1M/E7E7E7E703')
    # URI4 = uri_helper.uri_from_env(default='radio://0/80/1M/E7E7E7E704')
    # uris = [URI1, URI2, URI3, URI4]

    uris = [URI1] #, URI2]
    # uris = [URI1, URI2]
    swarm = Swarm(uris)
    
    try:
        swarm.take_off()

        time.sleep(2)

        # for i in range(600):
        #     print(swarm._vehicles[0].position)
        #     print(swarm._vehicles[0].velocity)
        #     time.sleep(0.1)

        for i in range(15):
            swarm._vehicles[0]._cf.commander.send_velocity_world_setpoint(0.3, 0., 0.,  yawrate=55.0)
        #     swarm._vehicles[1].scf.cf.commander.send_velocity_world_setpoint(0.3, 0., 0.,  yawrate=55.0)
        #     swarm._vehicles[2].scf.cf.commander.send_velocity_world_setpoint(0.3, 0., 0.,  yawrate=55.0)
        #     swarm._vehicles[3].scf.cf.commander.send_velocity_world_setpoint(0.3, 0., 0.,  yawrate=55.0)
        #     # print(swarm._vehicles[0].position, swarm._vehicles[1].position)
            time.sleep(0.1)
        for i in range(15):
            swarm._vehicles[0]._cf.commander.send_velocity_world_setpoint(-0.3, 0., 0.,  yawrate=55.0)
        #     swarm._vehicles[1].scf.cf.commander.send_velocity_world_setpoint(-0.3, 0., 0.,  yawrate=55.0)
        #     swarm._vehicles[2].scf.cf.commander.send_velocity_world_setpoint(-0.3, 0., 0.,  yawrate=55.0)
        #     swarm._vehicles[3].scf.cf.commander.send_velocity_world_setpoint(-0.3, 0., 0.,  yawrate=55.0)
            time.sleep(0.1)

        swarm.land()
        swarm.close()

    except KeyboardInterrupt:
        swarm.land()
        swarm.close()