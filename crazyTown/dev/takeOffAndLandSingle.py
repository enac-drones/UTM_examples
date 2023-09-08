import logging
import sys
import time
from threading import Event
import numpy as np

import cflib.crtp
from cflib.utils import uri_helper
from swarm import Swarm


if __name__ == '__main__':
    # Initialize the low-level drivers
    cflib.crtp.init_drivers()

    URI1 = uri_helper.uri_from_env(default='radio://0/80/1M/E7E7E7E701')

    uris = [URI1]
    swarm = Swarm(uris)
    time.sleep(2)

    duration_s = 3.

    takeoff_pos = swarm._vehicles[0].position.copy()

    try:
        swarm.take_off()
        time.sleep(2)

        # swarm._vehicles[0]._cf.commander.send_position_setpoint(0.5, 0.5, 0.4, 0)
        # swarm._vehicles[0]._cf.high_level_commander.land(0.0, 3)
        swarm._vehicles[0]._cf.high_level_commander.go_to(0.5, 0.5, 0.4, 0, duration_s)
        time.sleep(duration_s)

        # swarm._vehicles[0]._cf.commander.send_position_setpoint(takeoff_pos[0], takeoff_pos[1], takeoff_pos[2]+0.05, 0)
        swarm._vehicles[0]._cf.high_level_commander.go_to(takeoff_pos[0], takeoff_pos[1],takeoff_pos[2]+0.15 , 0, duration_s)
        print('Sleeping...')
        time.sleep(duration_s+2)
        print('Landing...')

        swarm.land()
        swarm.close()

    except KeyboardInterrupt:
        swarm.land()
        swarm.close()