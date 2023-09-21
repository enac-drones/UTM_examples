import logging
import sys
import time
from threading import Event
import numpy as np

import cflib.crtp
from cflib.utils import uri_helper
from swarm import Swarm

# class Flight_plan():
#     def __init__(self) -> None:
#         self.state = 0

# an enum class of Vehicle states
# class VehicleState():
#     IDLE = 0
#     TAKE_OFF = 1
#     HOVER = 2
#     GO_TO = 3
#     LAND = 4
#     LANDED = 5

def calculate_velocity(vehicle, towards_point=None):
    point = vehicle._landing_pos if towards_point is None else towards_point
    pos_err = point - vehicle.position
    pos_err_norm = np.linalg.norm(pos_err)
    # FIX ME division is dangering....
    vel_enu = pos_err / pos_err_norm * 0.5
    return vel_enu

def calculate_mid_waypoint(vehicle):
    p_diff = vehicle._landing_pos[:2] - vehicle._takeoff_pos[:2]
    p_diff_norm = np.linalg.norm(p_diff)
    unit_vec_ab = p_diff / p_diff_norm
    p_mid = vehicle._takeoff_pos[:2] + (p_diff) / 2
    e = np.array([[0, -1],
                  [-1, 0]])
    p_mid = p_mid + np.matmul(e, unit_vec_ab) * 0.5
    return np.array([p_mid[0], p_mid[1], vehicle._landing_pos[2]])

if __name__ == '__main__':
    # Initialize the low-level drivers
    cflib.crtp.init_drivers()

    URI1 = uri_helper.uri_from_env(default='radio://0/80/1M/E7E7E7E701')

    uris = [URI1]
    swarm = Swarm(uris)
    time.sleep(2)

    duration_s = 3.

    # takeoff_pos = swarm._vehicles[0].position.copy()
    destination = np.array([0.1, 0.1, 0.4])


    swarm._vehicles[0]._takeoff_pos = swarm._vehicles[0].position.copy()
    swarm._vehicles[0]._landing_pos = destination
    mid_point = calculate_mid_waypoint(swarm._vehicles[0])
    print(f'TakeOff {swarm._vehicles[0]._takeoff_pos}, Mid Point: {mid_point} Landing {swarm._vehicles[0]._landing_pos}')

    try:
        while swarm.state != 5: # Change this to enums with "LANDED"
            swarm.fly()
            for vehicle in swarm._vehicles:
                if vehicle._vehicle_state == 2: # Change this to enums with "CRUISE or ...."

                    vehicle.ref_vel_enu = calculate_velocity(vehicle, towards_point=None)


                    if np.linalg.norm(vehicle._landing_pos - vehicle.position) < 0.1:
                        vehicle._vehicle_state = 4 # Descend
            time.sleep(0.1)

        # swarm._vehicles[0]._cf.commander.send_position_setpoint(0.5, 0.5, 0.4, 0)
        # swarm._vehicles[0]._cf.high_level_commander.land(0.0, 3)
        # swarm._vehicles[0]._cf.high_level_commander.go_to(0.5, 0.5, 0.4, 0, duration_s)
        # time.sleep(duration_s)

        # swarm._vehicles[0]._cf.commander.send_position_setpoint(takeoff_pos[0], takeoff_pos[1], takeoff_pos[2]+0.05, 0)
        # swarm._vehicles[0]._cf.high_level_commander.go_to(takeoff_pos[0], takeoff_pos[1],takeoff_pos[2]+0.15 , 0, duration_s)
        # print('Sleeping...')
        # time.sleep(duration_s+2)
        # print('Landing...')

        swarm.land()
        swarm.close()

    except KeyboardInterrupt:
        swarm.land()
        swarm.close()