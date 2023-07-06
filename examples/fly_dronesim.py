"""Script demonstrating the joint use of velocity input.

The simulation is run by a `VelocityAviary` environment.

Example
-------
In a terminal, run as:

    $ python3 fly_dronesim.py

Notes
-----
The drones use interal INDI control to track a target velocity.

"""
import os
import time
import argparse
from datetime import datetime
import pdb
import math
import random
import numpy as np
import pybullet as p
import matplotlib.pyplot as plt

from gflow.arena import ArenaMap
from gflow.building import Building
from gflow.vehicle import Vehicle
import gflow.utils as ut
from time import sleep
import numpy as np

from dronesim.envs.BaseAviary import DroneModel, Physics
from dronesim.envs.VelocityAviary import VelocityAviary
from dronesim.utils.utils import sync, str2bool

from cases import Cases

# from gym_pybullet_drones.envs.BaseAviary import DroneModel, Physics
# from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
# from gym_pybullet_drones.envs.VisionAviary import VisionAviary
# from gym_pybullet_drones.control.DSLPIDControl import DSLPIDControl
# from gym_pybullet_drones.control.SimplePIDControl import SimplePIDControl

# from gym_pybullet_drones.utils.utils import sync, str2bool



# We are using our own Logger here for convenience
# from gym_pybullet_drones.utils.Logger import Logger
# from Logger import Logger

# from boids import Boid, magnitude

# from path_plan_w_panel import ArenaMap, Vehicle, Flow_Velocity_Calculation


# from utils import add_buildings

def main():
    #### Define and parse (optional) arguments for the script ##
    parser = argparse.ArgumentParser(description='Velocity control example using VelocityAviary')
    parser.add_argument('--drone',              default=['tello'],     type=str,    help='Drone model (default: CF2X)', metavar='', choices=DroneModel)
    parser.add_argument('--gui',                default=True,        type=str2bool,      help='Whether to use PyBullet GUI (default: True)', metavar='')
    parser.add_argument('--record_video',       default=False,       type=str2bool,      help='Whether to record a video (default: False)', metavar='')
    parser.add_argument('--plot',               default=True,        type=str2bool,      help='Whether to plot the simulation results (default: True)', metavar='')
    parser.add_argument('--user_debug_gui',     default=False,       type=str2bool,      help='Whether to add debug lines and parameters to the GUI (default: False)', metavar='')
    parser.add_argument('--aggregate',          default=True,        type=str2bool,      help='Whether to aggregate physics steps (default: False)', metavar='')
    parser.add_argument('--obstacles',          default=False,       type=str2bool,      help='Whether to add obstacles to the environment (default: True)', metavar='')
    parser.add_argument('--simulation_freq_hz', default=240,         type=int,           help='Simulation frequency in Hz (default: 240)', metavar='')
    parser.add_argument('--control_freq_hz',    default=24,          type=int,           help='Control frequency in Hz (default: 48)', metavar='')
    parser.add_argument('--flow_calc_freq_hz',  default=24,          type=int,           help='Vector field calculation frequency in Hz (default: 5)', metavar='')
    parser.add_argument('--duration_sec',       default=50,         type=int,           help='Duration of the simulation in seconds (default: 5)', metavar='')
    # parser.add_argument('--duration_sec',       default=90,         type=int,           help='Duration of the simulation in seconds (default: 5)', metavar='')
    ARGS = parser.parse_args()

    AGGR_PHY_STEPS = int(ARGS.simulation_freq_hz/ARGS.control_freq_hz) if ARGS.aggregate else 1
    PHY = Physics.PYB

    #### Initialize the simulation #############################



    # Case 8 : Arena 2 : 2 Vehicles U- Minima
    # arena_version = 2
    # vehicle_name_list =   ['V1', 'V2']
    # vehicle_source_list = [0.95, 2.6] # Source_strength
    # vehicle_imaginary_source_list = [0.45, 0.85] # Imaginary source_strength
    # vehicle_goal_list = [([1.5, -3, 0.5], 5, 0.00), ([1.51, -2, 0.5], 5, 0.00)]# goal,goal_strength all 5, safety 0.001 for V1 safety = 0 when there are sources
    # vehicle_goto_goal_list =[[0.5,-np.pi/2,0,0.5],[0.5,0,0,0] ] # altitude,AoA,t_start,Vinf=0.5,0.5,1.5
    # vehicle_pos_list = [[2., 1.2, 0.5],[1.5, -2, 0.5]]

    # Arena = ArenaMap(version = arena_version)
    # Arena.Inflate(radius = 0.2)
    # Arena.Panelize(size=0.01)
    # Arena.Calculate_Coef_Matrix()
    # # Arena.Visualize2D()
    # Arena.Wind(0,0,info = 'unknown') # Not used for the moment !


    # vehicle_list = [Vehicle(name,source, imag_source) for name , source, imag_source in zip(vehicle_name_list, vehicle_source_list, vehicle_imaginary_source_list)]
    # num_vehicles = len(vehicle_list)
    # INIT_XYZS = np.array(vehicle_pos_list)
    # INIT_RPYS = np.zeros([num_vehicles,3])
    # TARGET_VELS = np.zeros([num_vehicles,3])
    # FLOW_VELS = np.zeros([num_vehicles,3])

    # for vehicle,set_goal,goto_goal,pos in zip(vehicle_list,vehicle_goal_list,vehicle_goto_goal_list,vehicle_pos_list):
    #     vehicle.Set_Goal(set_goal[0],set_goal[1],set_goal[2])
    #     vehicle.Go_to_Goal(goto_goal[0],goto_goal[1],goto_goal[2],goto_goal[3])
    #     vehicle.Set_Position(pos)
        # pdb.set_trace()

    case = Cases()
    vehicle_list = case.Vehicle_list
    
    # current_vehicle_list = vehicle_list
    num_vehicles = len(case.Vehicle_list)

    INIT_XYZS = np.array([vehicle.position for vehicle in vehicle_list]) #np.array(vehicle_pos_list)
    INIT_RPYS = np.zeros([num_vehicles,3])

    #### Create the environment ################################
    env = VelocityAviary(drone_model=num_vehicles*ARGS.drone,
                         num_drones=num_vehicles,
                         initial_xyzs=INIT_XYZS,
                         initial_rpys=INIT_RPYS,
                         physics=Physics.PYB,
                         neighbourhood_radius=10,
                         freq=ARGS.simulation_freq_hz,
                         aggregate_phy_steps=AGGR_PHY_STEPS,
                         gui=ARGS.gui,
                         record=ARGS.record_video,
                         obstacles=ARGS.obstacles,
                         user_debug_gui=ARGS.user_debug_gui
                         )
    #### Obtain the PyBullet Client ID from the environment ####
    PYB_CLIENT = env.getPyBulletClient()
    DRONE_IDS = env.getDroneIds()


    #### Initialize the logger #################################
    # logger = Logger(logging_freq_hz=int(ARGS.simulation_freq_hz/AGGR_PHY_STEPS),
    #                 num_drones=num_vehicles )

    ### Add ground plane
    PLANE_ID = p.loadURDF("plane.urdf", physicsClientId=PYB_CLIENT)

    ### Add Buildings ################################
    # add_buildings(physicsClientId=PYB_CLIENT, version=arena_version)

    #### Compute number of control steps in the simlation ######
    PERIOD = ARGS.duration_sec
    NUM_WP = ARGS.control_freq_hz*PERIOD

    #### Run the simulation ####################################
    CTRL_EVERY_N_STEPS = int(np.floor(env.SIM_FREQ/ARGS.control_freq_hz))
    CALC_FLOW_EVERY_N_STEPS = int(np.floor(env.SIM_FREQ/ARGS.flow_calc_freq_hz))
    action = {str(i): np.array([0.0,0.0,0.0,0.0]) for i in range(num_vehicles)}
    START = time.time()
    sim_finished = False

    for i in range(0, int(ARGS.duration_sec*env.SIM_FREQ), AGGR_PHY_STEPS):

        # for index,vehicle in enumerate(case.Vehicle_list):
        #     if vehicle.state != 1:
        #         vehicle.run_simple_sim()

    # i=0
    # while sim_finished != True:
        ############################################################
        sim_finished = True if np.all([vehicle.state for vehicle in vehicle_list]) else False
        if sim_finished: break
        #### Step the simulation ###################################
        obs, reward, done, info = env.step(action)

        #### Calculate the vector field ############################
        # if i%CALC_FLOW_EVERY_N_STEPS == 0:
            # flow_vels = Flow_Velocity_Calculation(vehicle_list,Arena)

        #### Compute control at the desired frequency ##############
        if i%CTRL_EVERY_N_STEPS == 0:
            # print('obs : ',obs['0']['state'][0:3])
            # write the current position and velocity observation into vehicles
            for vehicle_nr, vehicle in enumerate(vehicle_list):

                # vehicle.position = obs[str(vehicle_nr)]['state'][0:3]
                # vehicle.velocity = obs[str(vehicle_nr)]['state'][10:13]
                vehicle.Set_Position(obs[str(vehicle_nr)]['state'][0:3])
                vehicle.Set_Velocity(obs[str(vehicle_nr)]['state'][10:13])

            # Communication Block
            # Update positions
            for index,vehicle in enumerate(vehicle_list):
                # Update only self position
                vehicle.vehicle_list[index].position = vehicle.position

                # Update the listed vehicle numbers wrt every one
                if index in [1,2,3]:
                    for list_index in range(len(vehicle.vehicle_list)):
                        vehicle.vehicle_list[list_index].position = vehicle_list[list_index].position # calling case.Vehicle is not nice here... 1 unneccessary element update


            for vehicle_nr, vehicle in enumerate(vehicle_list):
                # V_des = flow_vels[vehicle_nr] #- vehicle.velocity # FIXME Check this out !!!
                V_des = vehicle.run_flow_calc_alone()
                mag = np.linalg.norm(V_des)
                V_des_unit = V_des/mag
                V_des_unit[2] = 0.
                mag = np.clip(mag, 0., 0.4)
                mag_converted = mag/8.3 # This is Tellos max speed 30Km/h

                # print(f' X : {V_err[0]:.3f} , Y : {V_err[1]:.3f} , Z : {V_err[2]:.3f} Mag : {mag}')
                action[str(vehicle_nr)] = np.array([V_des_unit[0],V_des_unit[1],V_des_unit[2], mag_converted]) # This is not incremental ! It is direct desired action.
                # FLOW_VELS[vehicle_nr] = V_des #flow_vels[vehicle_nr]
                # TARGET_VELS[vehicle_nr] = np.array([V_des_unit*mag_converted])

        # #### Log the simulation ####################################
        # for j in range(num_vehicles):
        #     logger.log(drone=j,
        #                timestamp=i/env.SIM_FREQ,
        #                state= obs[str(j)]["state"],
        #                control=np.hstack([TARGET_VELS[j], FLOW_VELS[j], np.zeros(6)])
        #                )

        #### Vehicle Trace #################################
        if i%4 == 0:
            vehicle_colors=[[1, 0, 0], [0, 1, 0], [0, 0, 1],[1, 1, 0], [0, 1, 1], [0.5, 0.5, 1] ]
            for vehicle_nr, vehicle in enumerate(vehicle_list):
                vehicle.position = obs[str(vehicle_nr)]['state'][0:3]
                
                p.addUserDebugLine(lineFromXYZ=INIT_XYZS[vehicle_nr],
                           lineToXYZ=vehicle.position,
                           lineColorRGB=vehicle_colors[vehicle_nr],
                           lifeTime=1 * 1000,
                           physicsClientId=PYB_CLIENT)
                INIT_XYZS[vehicle_nr] = vehicle.position

        #### Printout ##############################################
        if i%env.SIM_FREQ == 0:
            env.render()

        #### Sync the simulation ###################################
        if ARGS.gui:
            sync(i, START, env.TIMESTEP)

    logger.plot()
        # i+=1
    #### Close the environment #################################
    env.close()

    #### Save the simulation results ###########################
    logger.save()

if __name__ == "__main__":
    main()

#EOF