from time import sleep
import time
from djitellopy import TelloSwarm
from voliere import VolierePosition
from voliere import Vehicle as Target
import pdb
import numpy as np
import sys
import gflow


# from path_plan_w_panel import ArenaMap, Vehicle, Flow_Velocity_Calculation_0
# sys.path.append('/Users/adriandelser/Desktop/ENAC/gflow')
# from src.arena import ArenaMap
# from src.panel_flow import Flow_Velocity_Calculation
# from src.building import Building
# from src.vehicle import Vehicle
# from src.cases import Cases, Case


# from gflow.arena import ArenaMap
# from gflow.panel_flow import Flow_Velocity_Calculation
# # from gflow.building import Building
# from gflow.vehicle import Vehicle
# from gflow.cases import Cases

# from source import Source

# For pybullet :
# import pybullet as p
from time import sleep
# import pybullet_data
import pdb
import numpy as np

# from utils import add_buildings

# We are using our own Logger here for convenience
# from logger import Logger

#type hinting
from typing import List, Dict, Tuple, Optional


#should probably pull this from json or something
NEXT_GOAL_LIST:list = [[ [0,0,0.5], [2,2,0.5], [3,2,0.5]],
                [ [-2,-2,1.5], [-2,-3.5,1.5]]]


#---------- OpTr- ACID - -----IP------
AC_LIST:list = [['69', '69', '192.168.1.69']]
# AC_LIST:list = [['65', '65', '192.168.1.65'],
#             ['66', '66', '192.168.1.66']]
AC_ID_LIST:list = [[_[0], _[1]] for _ in AC_LIST]



CASE_INFO:dict = {
        "filename": 'cases.json',
        "casename": 'DASC23_case_1T',
    }

# Generating an example Source for population
# population = [Source(ID=0, source_strength=0.8, position=np.array([-1., 3., 0.4]))]
POPULATION = None

# Define the distance at which the drone is considered to have arrived at destination
ARRIVAL_DISTANCE = 0.5

USE_PANEL_FLOW = True


def initialize_id_swarm(ac_list:list):
    '''Initialise the parameters'''
    ip_list = [_[2] for _ in ac_list]
    swarm = TelloSwarm.fromIps(ip_list)

    id_list = [_[1] for _ in ac_list]
    for i, id in enumerate(id_list):
        swarm.tellos[i].set_ac_id(id)
    
    return id_list, swarm

# def initialize_case()->Case:
#     '''returns a case object from a case_info dictionary which stores the filename and casename'''
#     file_name=CASE_INFO["filename"]
#     case_name = CASE_INFO["casename"]
#     case = Cases.get_case(filename=file_name, casename=case_name)
#     return case


def connect_swarm(swarm)->None:
    print('Connecting to Tello Swarm...')
    swarm.connect()
    print('Connected to Tello Swarm...')
    return None

def initialise_voliere(swarm):
    voliere = VolierePosition(AC_ID_LIST, swarm.tellos, freq=40)
    voliere.run()
    sleep(4)

    return voliere

# def flight_over(vehicle_list:List(Vehicle)):
#     '''Returns True if all drones have reached their destination else False'''
#     distance_to_destination = [vehicle.distance_to_destination for vehicle in vehicle_list]
#     return True if np.all(distance_to_destination) < ARRIVAL_DISTANCE else False


# def update_vehicle_goals(vehicle_list:List(Vehicle)):
#     for vehicle_nr, vehicle in enumerate(vehicle_list):
#         if vehicle.distance_to_destination < ARRIVAL_DISTANCE and vehicle_nr == 1:
#             vehicle.Set_Next_Goal(NEXT_GOAL_LIST[vehicle_nr][vehicle._sink_index])
#             vehicle._sink_index += 1
#             vehicle._sink_index = np.clip(vehicle._sink_index, 0, len(NEXT_GOAL_LIST[vehicle_nr])-1)


# def calculate_and_set_flow_velocities(vehicle_list, Arena, swarm, TARGET_VELS, use_panel_flow=True):
#     if use_panel_flow:
#         flow_vels = Flow_Velocity_Calculation(vehicle_list, Arena, external_sources=POPULATION)
#         for i, vehicle in enumerate(vehicle_list):
#             V_des = flow_vels[i]
#             mag = np.linalg.norm(V_des)
#             V_des_unit = V_des / mag
#             mag = np.clip(mag, 0., 1.0)
#             vel_enu = V_des_unit * mag
#             heading = 0.
#             swarm.tellos[i].send_velocity_enu(vel_enu, heading)
#             TARGET_VELS[i] = vehicle.velocity_desired
#     else:
#         # Directly fly to the point
#         for i, vehicle in enumerate(vehicle_list):
#             swarm.tellos[i].fly_to_enu(vehicle.goal, 0.)

# def update_vehicle_states(vehicle_list:List(Vehicle), swarm):
#     for i, vehicle in enumerate(vehicle_list):
#         vehicle.Set_Position(swarm.tellos[i].get_position_enu())
#         vehicle.Set_Velocity(swarm.tellos[i].get_velocity_enu())


# def visualize_vehicles(vehicle_list, swarm, quadrotors, INIT_XYZS, trace_count):
#     vehicle_colors=[[1, 0, 0], [0, 1, 0], [0, 0, 1],[1, 1, 0], [0, 1, 1], [0.5, 0.5, 1]]

#     for i, vehicle in enumerate(vehicle_list):
#         p.resetBasePositionAndOrientation(quadrotors[i],
#                                           swarm.tellos[i].get_position_enu(),
#                                           swarm.tellos[i].get_quaternion(),
#                                           physicsClientId=physicsClient)
        
#         if trace_count > 10:
#             p.addUserDebugLine(lineFromXYZ=INIT_XYZS[i],
#                                lineToXYZ=swarm.tellos[i].get_position_enu(),
#                                lineColorRGB=vehicle_colors[i],
#                                lifeTime=1 * 1000,
#                                physicsClientId=physicsClient)
#             INIT_XYZS[i] = swarm.tellos[i].get_position_enu()
#             trace_count = 0
#         trace_count += 1
#     return trace_count


def log_simulation(log, id_list, swarm, TARGET_VELS, FLOW_VELS, sim_start_time):
    for i, vehicle in enumerate(id_list):
        log.log(drone=i,
                timestamp=time.time()-sim_start_time,
                state=np.hstack([swarm.tellos[i].get_position_enu(), swarm.tellos[i].get_velocity_enu(), swarm.tellos[i].get_quaternion(),  np.zeros(10)]),
                control=np.hstack([TARGET_VELS[i], FLOW_VELS[i], np.zeros(2), 0., np.zeros(3)]),
                sim=False)



def main():
    # PyBullet Visualization
    # visualize = False#True


    id_list, swarm = initialize_id_swarm(AC_LIST)

    # case = initialize_case()    

    # num_vehicles = len(id_list)
    # INIT_XYZS = np.array([vehicle.position for vehicle in case.vehicle_list])
    # # INIT_RPYS = np.zeros([num_vehicles,3])
    # TARGET_VELS = np.zeros([num_vehicles,3])
    # FLOW_VELS = np.zeros([num_vehicles,3])

    #### Initialize the logger #################################
    # log = Logger(logging_freq_hz=30, #int(ARGS.simulation_freq_hz/AGGR_PHY_STEPS),
    #             num_drones=num_vehicles, ) #duration_sec=100 )

    connect_swarm(swarm)

    voliere=initialise_voliere(swarm)

    # vehicle_list = case.vehicle_list
    # Arena = case.arena

    # log.set_buildings(Arena.buildings)

    print("Starting Natnet3.x interface at %s" % ("1234567"))

    # Simulation starts
    sim_start_time = time.time()
    try:

        #takeoff everyone
        swarm.takeoff()
        for i in range(50):
            print(swarm.tellos[0].get_position_enu())
            time.sleep(0.1)
            #(east, north, up)
            # swarm.tellos[0].fly_to_enu([-2, -1.5, 0.5], heading=0)
            swarm.tellos[0].send_velocity_enu([1,0,0], heading=np.pi/2)
            
        # # set start time
        # starttime= time.time()

        # #for the first 9 seconds, move the tellos to their initial positions
        # while time.time()-starttime < 9:
        #     for idx, _ in enumerate(id_list):
        #         swarm.tellos[idx].fly_to_enu(INIT_XYZS[idx], heading=0)

        # print('Finished moving !!!!!') #Finished 
        # # Main loop :
        # # trace count needed if visualising trajectories, do not comment
        # trace_count = 0

        # #reset starttime to 0
        # starttime= time.time()

        # # set while loop for 1800 seconds of starting the whole flight (ie before takeoff)
        # while time.time()-sim_start_time < 1800:
        #     #can this line not be replaced with pause? #TODO
        #     if time.time()-starttime > 0.025:
        #         # print(f'Freq : {1/(time.time()-starttime):.3f}')
        #         # reset startime to 0 once 0.025 seconds have passed. Why? #TODO
        #         starttime= time.time()
               
        #         # Check if vehicles reached their final position
        #         if flight_over(vehicle_list): break

        #         update_vehicle_goals(vehicle_list)

        #         update_vehicle_states(vehicle_list, swarm)
  
        #         calculate_and_set_flow_velocities(vehicle_list,Arena,swarm,TARGET_VELS,use_panel_flow=USE_PANEL_FLOW)

        #         # Visualise the vehicle traces
        #         # visualize_vehicles(vehicle_list, swarm,quadrotors,INIT_XYZS,trace_count) #FIXME quadrotors undefined as well as other things 
        #         # #### Log the simulation ####################################
        #         log_simulation(log, id_list, swarm, TARGET_VELS, FLOW_VELS, sim_start_time)

        # #### Save the simulation results ###########################
        # log.save(flight_type='fast_follow')
        # swarm.move_down(int(40))
        swarm.move_down(int(40))
        swarm.land()
        voliere.stop()
        swarm.end()
        sleep(1)

    except (KeyboardInterrupt, SystemExit):
        print("Shutting down natnet interfaces...")
        # log.save(flight_type='fast_follow')
        swarm.move_down(int(40))
        swarm.land()
        voliere.stop()
        swarm.end()
        # if visualize:
        #     p.disconnect(physicsClientId=physicsClient)
        sleep(1)

    except OSError:
        print("Natnet connection error")
        swarm.move_down(int(40))
        swarm.land()
        voliere.stop()
        swarm.end()
        exit(-1)

if __name__=="__main__":
    main()
