import logging
import sys
import time
from threading import Event

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils import uri_helper
from cflib.crazyflie import Commander, HighLevelCommander

URI = uri_helper.uri_from_env(default='radio://0/80/1M/E7E7E7E7E9')

DEFAULT_HEIGHT = 0.5

deck_attached_event = Event()

logging.basicConfig(level=logging.ERROR)

def take_off_simple(scf):
    with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
        time.sleep(3)
        mc.stop()


if __name__ == '__main__':
    cflib.crtp.init_drivers()

    scf = SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache'))
    scf.open_link()
    time.sleep(1)
    
    scf.cf.high_level_commander.takeoff(0.5, 1.0)
    time.sleep(3)
    for i in range(20):
        scf.cf.commander.send_position_setpoint(0.5, -0.3, 0.4, 0.0)
        time.sleep(0.1)
    for i in range(20):
        scf.cf.commander.send_velocity_world_setpoint(0., 0., 0.,  yawrate=55.0)
        time.sleep(0.1)
    for i in range(20):
        scf.cf.commander.send_position_setpoint(0.0, 0., 0.4, 0.0)
        time.sleep(0.1)
    
    scf.cf.high_level_commander.land(0.0, 1.0)
    time.sleep(3)
    scf.close_link()

    # with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:

    #     time.sleep(1)

    #     take_off_simple(scf)