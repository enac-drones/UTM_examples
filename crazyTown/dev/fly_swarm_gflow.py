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


from gflow.arena import ArenaMap
from gflow.panel_flow import Flow_Velocity_Calculation
# from gflow.building import Building
from gflow.vehicle import Vehicle
from gflow.cases import Cases
from gflow.source import Source

# We are using our own Logger here for convenience
from logger import Logger

from swarm import Swarm


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

def basic_example():
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



def main():
    cflib.crtp.init_drivers()

    URI1 = uri_helper.uri_from_env(default='radio://0/80/1M/E7E7E7E701')
    # URI2 = uri_helper.uri_from_env(default='radio://0/80/1M/E7E7E7E702')
    # URI3 = uri_helper.uri_from_env(default='radio://0/80/1M/E7E7E7E703')
    # URI4 = uri_helper.uri_from_env(default='radio://0/80/1M/E7E7E7E704')
    # uris = [URI1, URI2, URI3, URI4]

    uris = [URI1] #, URI2]
    # uris = [URI1, URI2]


    case = Cases.get_case(filename='cases.json', casename='CT_demo_1v')
    # case = Cases.get_case(filename='cases.json', casename='ERF23_case_2v')
    # case = Cases.get_case(filename='cases.json', casename='ERF23_case_3v')
    vehicles_next_goal_list = [[ [0, 1.0,0.4], [1.0,0,0.4] , [0,-.8,0.4], [-1.0,0,0.4]], # V0
                               [ [0, 2.5,0.8], [-2, 0,0.8] , [0,-2.5,0.8], [ 2.5,0,0.8]], # V1
                               [ [-2, 0 ,0.8], [0,2.5,0.8] , [0,-2.5,0.8], [ 2.5,0,0.8]], # V2
                               [ [0, 2.5,0.8], [0,-2.5,0.8], [-2, 0 ,0.8], [ 2.5,0,0.8]],]# V3
    
    num_vehicles = len(case.vehicle_list)
    INIT_XYZS = np.array([vehicle.position for vehicle in case.vehicle_list])
    INIT_RPYS = np.zeros([num_vehicles,3])
    TARGET_VELS = np.zeros([num_vehicles,3])
    FLOW_VELS = np.zeros([num_vehicles,3])


    #### Initialize the logger #################################
    log = Logger(logging_freq_hz=30, #int(ARGS.simulation_freq_hz/AGGR_PHY_STEPS),
                num_drones=num_vehicles, ) #duration_sec=100 )

    print('Connecting to  crazyTown...')
    swarm = Swarm(uris)
    print('Connected to the Swarm...')

    # Generating an example Source for population
    # population = [Source(ID=0, source_strength=0.3, position=np.array([4., 4., 0.4]))]
    population = None

    # Arena = ArenaMap(building_hulls=building_hulls)
    # Arena.Inflate(radius = 0.17)
    # Arena.Panelize(size=0.01)
    # Arena.Calculate_Coef_Matrix()
    # Arena.Visualize2D()
    # Arena.Wind(0,0,info = 'unknown') # Not used for the moment !

    # vinfmag_list = [0.05, 0. , -0.05]
    # dt  = 0.02
    # hor = 3.0
    
    vehicle_list = case.vehicle_list
    Arena = case.arena
    # log.set_buildings(Arena.buildings)

    # for i, vehicle in enumerate(vehicle_list):
    #     vehicle.arena = Arena
    #     vehicle.vehicle_list = vehicle_list

    try:
        swarm.take_off()
        time.sleep(2)

        starttime= time.time()
        sim_start_time = time.time()
        while time.time()-sim_start_time < 120:
            if time.time()-starttime > 0.2:
                # print(f'Freq : {1/(time.time()-starttime):.3f}')
                starttime= time.time()
                        
                for i, vehicle in enumerate(vehicle_list):
                    vehicle.Set_Position(swarm._vehicles[i].position)
                    vehicle.Set_Velocity(swarm._vehicles[i].velocity)
                    print(vehicle.position)
                    # print(f' {i} - Vel : {vehicle.velocity[0]:.3f}  {vehicle.velocity[1]:.3f}  {vehicle.velocity[2]:.3f}')
                    # if i == 0 :
                    # vehicle.Go_to_Goal(Vinfmag=1.0) # FIXME this is only for waypoint guidance

                # Check if vehicles reached their final position
                distance_to_destination = [vehicle.distance_to_destination for vehicle in vehicle_list]
                flight_finished = True if np.all(distance_to_destination) < 0.20 else False
                
                for vehicle_nr, vehicle in enumerate(vehicle_list):
                    # print('Heyyo :', target_position, type(target_position))
                    # vehicle.Set_Next_Goal(vehicle_next_goal_list[], dynamic_sigma=True)
                    
                    # if vehicle.state :
                    if vehicle.distance_to_destination<0.50:# and vehicle_nr==1:
                        print('Changing goal set point')
                        vehicle.Set_Next_Goal(vehicles_next_goal_list[vehicle_nr][vehicle._sink_index])
                        vehicle._sink_index += 1
                        # print(f'Vehicle : {vehicle_nr} -- sink ind:{vehicle._sink_index}')
                        # The below is to have a finite waypoint
                        # vehicle._sink_index = np.clip(vehicle._sink_index, 0,len(vehicles_next_goal_list[vehicle_nr])-1)
                        # This one is to have an infinite loop of waypoints
                        vehicle._sink_index = vehicle._sink_index%len(vehicles_next_goal_list[vehicle_nr])
                        # print(f'Vehicle : {vehicle_nr} -- clipped :{vehicle._sink_index}')
                        # goal_index = goal_index%len(vehicle_next_goal_list)

                use_panel_flow = 1
                if use_panel_flow :
                    flow_vels= Flow_Velocity_Calculation(vehicle_list, Arena, external_sources=population)
                    
                    for i, vehicle in enumerate(vehicle_list):
                        V_des = flow_vels[i] #vehicle.run_flow_calc_alone()
                        mag = np.linalg.norm(V_des)
                        V_des_unit = V_des/mag
                        # V_des_unit[2] = 0.
                        mag = np.clip(mag, 0., 0.3)
                        vel_enu = V_des_unit*mag
                        vel_enu[2] = V_des[2]
                        # vel_enu[2] = 0. 
                        # print('Vel enu : ',vel_enu)
                        # swarm.tellos[i].update(swarm.tellos, )

                        # vel_enu = flow_vels[i]*limited_norm #- swarm.tellos[i].velocity_enu
                        # print(f' {i} - Flow Velocity : {flow_vels[i]}')
                        # print(f' {i} - Flow Velocity Error : {vel_enu_err}')
                        # if i==0:
                            # print(f' Velicle {i} -- Flow Vels : {vel_enu}')
                        heading = 0.
                        # Look towards where you go
                        # heading = np.arctan2(vel_enu[1],vel_enu[0])
                        
                        # vehicle.Set_Desired_Velocity(vel_enu, correction_method='None')

                        # and update the Nest Vinf:
                        # vinfmag = 0.5
                        # vehicle.propagate_simple_path(vinfmag, V_des=vel_enu)

                        # swarm.tellos[i].send_velocity_enu(vehicle.velocity_desired, heading)
                        # swarm.tellos[i].send_velocity_enu(vel_enu, heading)

                        swarm._vehicles[i].scf.cf.commander.send_velocity_world_setpoint(
                                                                            vel_enu[0],
                                                                            vel_enu[1],
                                                                            vel_enu[2],
                                                                            yawrate=heading)
                        TARGET_VELS[i]=vel_enu


        print('Finished flying...')
        swarm.land()
        swarm.close()

    except KeyboardInterrupt:
        swarm.land()
        swarm.close()


if __name__ == '__main__':
    main()

