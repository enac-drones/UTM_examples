import logging
import sys
import time
from threading import Event

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.utils import uri_helper
# from cflib.positioning.motion_commander import MotionCommander
# from cflib.crazyflie import Commander, HighLevelCommander

# URI = uri_helper.uri_from_env(default='radio://0/80/1M/E7E7E7E7E9')
URI1 = uri_helper.uri_from_env(default='radio://0/80/1M/E7E7E7E701')
URI2 = uri_helper.uri_from_env(default='radio://0/80/1M/E7E7E7E702')
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


def position_callback(timestamp, data, logconf):
    x = data['kalman.stateX']
    y = data['kalman.stateY']
    z = data['kalman.stateZ']
    print('pos: ({}, {}, {})'.format(x, y, z))


def start_position_printing(scf):
    log_conf = LogConfig(name='Position', period_in_ms=500)
    log_conf.add_variable('kalman.stateX', 'float')
    log_conf.add_variable('kalman.stateY', 'float')
    log_conf.add_variable('kalman.stateZ', 'float')

    scf.cf.log.add_config(log_conf)
    log_conf.data_received_cb.add_callback(position_callback)
    log_conf.start()

# def take_off_simple(scf):
#     with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
#         time.sleep(3)
#         mc.stop()


if __name__ == '__main__':
    cflib.crtp.init_drivers()

    scf1 = SyncCrazyflie(URI1, cf=Crazyflie(rw_cache='./cache'))
    scf2 = SyncCrazyflie(URI2, cf=Crazyflie(rw_cache='./cache'))
    scf1.open_link()
    scf2.open_link()
    time.sleep(3)

    start_position_printing(scf1)
    start_position_printing(scf2)
    
    scf1.cf.high_level_commander.takeoff(0.5, 1.5)
    scf2.cf.high_level_commander.takeoff(0.5, 1.5)
    time.sleep(5)
    # for i in range(20):
    #     scf.cf.commander.send_position_setpoint(0.5, -0.3, 0.4, 0.0)
    #     time.sleep(0.1)
    # for i in range(20):
    #     scf.cf.commander.send_velocity_world_setpoint(0., 0., 0.,  yawrate=55.0)
    #     time.sleep(0.1)
    # for i in range(20):
    #     scf.cf.commander.send_position_setpoint(0.0, 0., 0.4, 0.0)
    #     time.sleep(0.1)
    
    scf1.cf.high_level_commander.land(0.0, 3.0)
    scf2.cf.high_level_commander.land(0.0, 3.0)
    time.sleep(3)
    scf1.close_link()
    scf2.close_link()
    time.sleep(1)

    # with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:

    #     time.sleep(1)

    #     take_off_simple(scf)