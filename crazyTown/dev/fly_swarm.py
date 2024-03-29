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
# from cflib.positioning.motion_commander import MotionCommander
# from cflib.crazyflie import Commander, HighLevelCommander

# URI = uri_helper.uri_from_env(default='radio://0/80/1M/E7E7E7E7E9')
DEFAULT_HEIGHT = 0.5

# deck_attached_event = Event()

logging.basicConfig(level=logging.ERROR)


def wait_for_position_estimator(scf):
    print('Waiting for estimator to find position...')

    log_config = LogConfig(name='Kalman Variance', period_in_ms=500)
    log_config.add_variable('kalman.varPX', 'float')
    log_config.add_variable('kalman.varPY', 'float')
    log_config.add_variable('kalman.varPZ', 'float')

    var_y_history = [1000] * 10
    var_x_history = [1000] * 10
    var_z_history = [1000] * 10

    threshold = 0.001

    with SyncLogger(scf, log_config) as logger:
        for log_entry in logger:
            data = log_entry[1]

            var_x_history.append(data['kalman.varPX'])
            var_x_history.pop(0)
            var_y_history.append(data['kalman.varPY'])
            var_y_history.pop(0)
            var_z_history.append(data['kalman.varPZ'])
            var_z_history.pop(0)

            min_x = min(var_x_history)
            max_x = max(var_x_history)
            min_y = min(var_y_history)
            max_y = max(var_y_history)
            min_z = min(var_z_history)
            max_z = max(var_z_history)

            # print("{} {} {}".
            #       format(max_x - min_x, max_y - min_y, max_z - min_z))

            if (max_x - min_x) < threshold and (
                    max_y - min_y) < threshold and (
                    max_z - min_z) < threshold:
                break


def reset_estimator(scf):
    cf = scf.cf
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')

    wait_for_position_estimator(cf)


# def position_callback(timestamp, data, logconf, scf):
def position_callback(logconf, data, scf):
    x = data['kalman.stateX']
    y = data['kalman.stateY']
    z = data['kalman.stateZ']
    vb = data['pm.vbat']
    print('pos: ({}, {}, {})'.format(x, y, z))
    print('vbat: {}'.format(vb))

def position_printing(scf):
    log_conf = LogConfig(name='Position', period_in_ms=10)
    log_conf.add_variable('kalman.stateX', 'float')
    log_conf.add_variable('kalman.stateY', 'float')
    log_conf.add_variable('kalman.stateZ', 'float')
    log_conf.add_variable('pm.vbat', 'float')
    with SyncLogger(scf, log_conf) as logger:
                for entry in logger:
                    x = entry[1]['kalman.stateX'] #stateEstimate.x
                    y = entry[1]['kalman.stateY']
                    z = entry[1]['kalman.stateZ']
                    vb = entry[1]['pm.vbat']
                    # print(f'CF : {scf.cf.link_uri} pos: ({x}, {y}, {z}) Batt: {vb} V')
                    # self._positions[scf.cf.link_uri] = SwarmPosition(x, y, z)
                    break

# def take_off_simple(scf):
#     with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
#         time.sleep(3)
#         mc.stop()





from swarm import Swarm

if __name__ == '__main__':
    cflib.crtp.init_drivers()

    URI1 = uri_helper.uri_from_env(default='radio://0/80/1M/E7E7E7E701')
    URI2 = uri_helper.uri_from_env(default='radio://0/80/1M/E7E7E7E702')
    # URI3 = uri_helper.uri_from_env(default='radio://0/80/1M/E7E7E7E703')
    # URI4 = uri_helper.uri_from_env(default='radio://0/80/1M/E7E7E7E704')
    # uris = [URI1, URI2, URI3, URI4]

    uris = [URI1, URI2]
    swarm = Swarm(uris)
    
    try:
        swarm.take_off()

        time.sleep(2)

        for i in range(15):
            swarm._vehicles[0].scf.cf.commander.send_velocity_world_setpoint(0.3, 0., 0.,  yawrate=0.0)
            swarm._vehicles[1].scf.cf.commander.send_velocity_world_setpoint(0.3, 0., 0.,  yawrate=55.0)
            # swarm._vehicles[2].scf.cf.commander.send_velocity_world_setpoint(0.3, 0., 0.,  yawrate=55.0)
            # swarm._vehicles[3].scf.cf.commander.send_velocity_world_setpoint(0.3, 0., 0.,  yawrate=55.0)
            print('P', swarm._vehicles[0].position)#, 
            print('V', swarm._vehicles[0].velocity)
            time.sleep(0.1)
        for i in range(15):
            swarm._vehicles[0].scf.cf.commander.send_velocity_world_setpoint(-0.3, 0., 0.,  yawrate=0.0)
            swarm._vehicles[1].scf.cf.commander.send_velocity_world_setpoint(-0.3, 0., 0.,  yawrate=55.0)
            # swarm._vehicles[2].scf.cf.commander.send_velocity_world_setpoint(-0.3, 0., 0.,  yawrate=55.0)
            # swarm._vehicles[3].scf.cf.commander.send_velocity_world_setpoint(-0.3, 0., 0.,  yawrate=55.0)
            print('P', swarm._vehicles[0].position)#, 
            print('V', swarm._vehicles[0].velocity)
            time.sleep(0.1)

        swarm.land()
        swarm.close()

    except KeyboardInterrupt:
        swarm.land()
        swarm.close()