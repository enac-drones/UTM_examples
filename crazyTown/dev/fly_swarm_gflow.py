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


from gflow.arena import ArenaMap
from gflow.panel_flow import Flow_Velocity_Calculation
# from gflow.building import Building
from gflow.vehicle import Vehicle
from gflow.cases import Cases
from gflow.source import Source

# We are using our own Logger here for convenience
from logger import Logger

from swarm import Swarm

import zmq

# URI = uri_helper.uri_from_env(default='radio://0/80/1M/E7E7E7E7E9')
DEFAULT_HEIGHT = 0.5

# deck_attached_event = Event()

logging.basicConfig(level=logging.ERROR)

def main():
    cflib.crtp.init_drivers()

    context = zmq.Context()
    socket = context.socket(zmq.PUSH)
    socket.bind("tcp://*:5555")

    URI1 = uri_helper.uri_from_env(default='radio://0/80/1M/E7E7E7E701')
    URI2 = uri_helper.uri_from_env(default='radio://0/80/1M/E7E7E7E702')
    URI3 = uri_helper.uri_from_env(default='radio://0/80/1M/E7E7E7E703')
    # URI4 = uri_helper.uri_from_env(default='radio://0/80/1M/E7E7E7E704')
    # uris = [URI1, URI2, URI3, URI4]

    # uris = [URI1] #, URI2]
    uris = [URI1, URI2, URI3]
    # uris = [URI3, URI4]


    # case = Cases.get_case(filename='cases.json', casename='CT_demo_1v')
    # case = Cases.get_case(filename='cases.json', casename='CT_demo_2v')
    case = Cases.get_case(filename='cases.json', casename='CT_demo_3v')

    vehicles_next_goal_list = [[ [0, 0.4,0.4], [1.0,0,0.6] , [0,-1.2,0.4], [-1.0,0,0.6]] , # V0
                               [ [0, 0.4,0.5], [-1, 0,0.5] , [0,-1.0,0.5], [ .9,0.2,0.5]], # V1
                               [ [-1,-0.8,0.4],[1,-0.8,0.6], [1, 0.8,0.4], [-1.0,0.8,0.6]],# V2
                               [ [0, 1.5,0.8], [0,-1.5,0.8], [-1, 0 ,0.8], [ 1.5,0,0.8]],] # V3
    
    num_vehicles = len(case.vehicle_list)
    INIT_XYZS = np.array([vehicle.position for vehicle in case.vehicle_list])
    INIT_RPYS = np.zeros([num_vehicles,3])
    TARGET_VELS = np.zeros([num_vehicles,3])
    FLOW_VELS = np.zeros([num_vehicles,3])


    #### Initialize the logger #################################
    log = Logger(logging_freq_hz=30, #int(ARGS.simulation_freq_hz/AGGR_PHY_STEPS),
                num_drones=num_vehicles, ) #duration_sec=100 )

    print('Connecting to  crazyTown...')
    swarm = Swarm(uris, report_socket=socket)
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
    
    vehicle_list = case.vehicle_list
    Arena = case.arena
    # log.set_buildings(Arena.buildings)

    try:
        swarm.take_off()
        time.sleep(2)

        starttime= time.time()
        sim_start_time = time.time()
        while time.time()-sim_start_time < 60:
            if time.time()-starttime > 0.1:
                # print(f'Freq : {1/(time.time()-starttime):.3f}')
                starttime= time.time()
                        
                for i, vehicle in enumerate(vehicle_list):
                    vehicle.Set_Position(swarm._vehicles[i].position)
                    vehicle.Set_Velocity(swarm._vehicles[i].velocity)
                    # print(vehicle.position)
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
                        # The below is to have a finite waypoint
                        # vehicle._sink_index = np.clip(vehicle._sink_index, 0,len(vehicles_next_goal_list[vehicle_nr])-1)
                        # This one is to have an infinite loop of waypoints
                        vehicle._sink_index = vehicle._sink_index%len(vehicles_next_goal_list[vehicle_nr])


                flow_vels= Flow_Velocity_Calculation(vehicle_list, Arena, external_sources=population)
                
                for i, vehicle in enumerate(vehicle_list):
                    V_des = flow_vels[i] #vehicle.run_flow_calc_alone()
                    mag = np.linalg.norm(V_des)
                    V_des_unit = V_des/mag
                    # V_des_unit[2] = 0.
                    mag = np.clip(mag, 0., 0.25)
                    vel_enu = V_des_unit*mag
                    vel_enu[2] = V_des[2]*0.5
                    # vel_enu[2] = 0. 
                    # print('Vel enu : ',vel_enu)

                    heading = 0.
                    # Look towards where you go
                    # heading = np.arctan2(vel_enu[1],vel_enu[0])
    
                    # and update the Nest Vinf:
                    # vinfmag = 0.5
                    # vehicle.propagate_simple_path(vinfmag, V_des=vel_enu)

                    swarm._vehicles[i]._cf.commander.send_velocity_world_setpoint(
                                                                        vel_enu[0],
                                                                        vel_enu[1],
                                                                        vel_enu[2],
                                                                        yawrate=heading)
                    swarm._vehicles[i].send_report()
                    TARGET_VELS[i]=vel_enu
                    FLOW_VELS[i]=V_des

                ##### Log the simulation ####################################
                for i, vehicle in enumerate(vehicle_list):
                    log.log(drone=i,
                               timestamp=time.time()-sim_start_time,
                               state= np.hstack([vehicle.position, vehicle.velocity,  np.zeros(14)]),
                            #    state= np.hstack([swarm.tellos[i].get_position_enu(), swarm.tellos[i].get_velocity_enu(), swarm.tellos[i].get_quaternion(),  np.zeros(10)]),
                            #    control=np.hstack([TARGET_VELS[i], FLOW_VELS[i], V_sum[i], 0., target_vehicle[0].position]),
                            #    control=np.hstack([TARGET_VELS[i], FLOW_VELS[i], population[0].position, sink_position]),
                               control=np.hstack([TARGET_VELS[i], FLOW_VELS[i], np.zeros(6)]),
                               sim=False
                               # control=np.hstack([TARGET_VEL[j, wp_counters[j], 0:3], np.zeros(9)])
                               )

        #### Save the simulation results ###########################
        log.save(flight_type='crazyTown')

        print('Finished flying...')
        swarm.land()
        swarm.close()

    except KeyboardInterrupt:
        log.save(flight_type='crazyTown')
        swarm.land()
        swarm.close()


if __name__ == '__main__':
    main()

