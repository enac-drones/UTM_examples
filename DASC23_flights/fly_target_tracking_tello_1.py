from time import sleep
import time
from djitellopy import TelloSwarm
from voliere import VolierePosition
from voliere import Vehicle as Target
import pdb
import numpy as np

# from path_plan_w_panel import ArenaMap, Vehicle, Flow_Velocity_Calculation_0


from gflow.arena import ArenaMap
from gflow.panel_flow import Flow_Velocity_Calculation
# from gflow.building import Building
from gflow.vehicle import Vehicle
from gflow.cases import Cases

# from source import Source

# For pybullet :
import pybullet as p
from time import sleep
import pybullet_data
import pdb
import numpy as np

# from utils import add_buildings

# We are using our own Logger here for convenience
# from gym_pybullet_drones.utils.Logger import Logger
from logger import Logger

def main():
    # PyBullet Visualization
    visualize = False#True

    #---------- OpTr- ACID - -----IP------
    # ac_list = [['65', '65', '192.168.1.65'],]
    
    ac_list =  [['65', '65', '192.168.1.65'],
               ['66', '66', '192.168.1.66'],]

    ip_list = [_[2] for _ in ac_list]
    swarm = TelloSwarm.fromIps(ip_list)

    id_list = [_[1] for _ in ac_list]
    for i,id in enumerate(id_list):
        swarm.tellos[i].set_ac_id(id)

    case = Cases.get_case(filename='cases.json', casename='twodrones')

    # arena_version = 65 #102
    # vehicle_name_list =   ['V1']
    # vehicle_source_list = [0.95] # Source_strength
    # vehicle_imaginary_source_list = [1.5] # Imaginary source_strength
    # vehicle_goal_list = [([-3.5, 3, 1.4], 5, 0.00)]# goal,goal_strength all 5, safety 0.001 for V1 safety = 0 when there are sources
    # vehicle_goto_goal_list =[[1.4,0,0,0] ] # altitude,AoA,t_start,Vinf=0.5,0.5,1.5
    # vehicle_pos_list = [[3.5, 3, 1.4]]
    # vehicle_next_goal_list = [[3.5, 3, 1.4], [-3.5, 3, 1.4]]

    goal_index = 0

    num_vehicles = len(id_list)
    # INIT_XYZS = np.array(vehicle_pos_list)
    # INIT_RPYS = np.zeros([num_vehicles,3])
    TARGET_VELS = np.zeros([num_vehicles,3])
    FLOW_VELS = np.zeros([num_vehicles,3])


    #### Initialize the logger #################################
    log = Logger(logging_freq_hz=30, #int(ARGS.simulation_freq_hz/AGGR_PHY_STEPS),
                    num_drones=num_vehicles, ) #duration_sec=100 )

    print('Connecting to Tello Swarm...')
    swarm.connect()
    print('Connected to Tello Swarm...')

    ac_id_list = [[_[0], _[1]] for _ in ac_list]
    # ac_id_list.append(['888', '888']) # Add a moving target
    # target_vehicle = [Target('888')]
    # all_vehicles = swarm.tellos+target_vehicle

    # voliere = VolierePosition(ac_id_list, swarm.tellos+target_vehicle, freq=40)
    voliere = VolierePosition(ac_id_list, swarm.tellos, freq=40)
    voliere.run()
    sleep(4)
    # building_hulls=voliere.get_markerset_pos()
    # log.set_building_hulls(building_hulls)

    # Generating an example Source for population
    # population = [Source(ID=0, source_strength=0.8, position=np.array([-1., 3., 0.4]))]
    population = None

    # Arena = ArenaMap(building_hulls=building_hulls)
    # # Arena = ArenaMap(version = 101)
    # Arena.Inflate(radius = 0.17)
    # Arena.Panelize(size=0.01)
    # Arena.Calculate_Coef_Matrix()
    # Arena.Visualize2D()
    # Arena.Wind(0,0,info = 'unknown') # Not used for the moment !

    vinfmag_list = [0.05, 0. , -0.05]
    dt  = 0.02
    hor = 3.0
    
    vehicle_list = case.vehicle_list

    for i, vehicle in enumerate(vehicle_list):
        vehicle.arena = case.arena
        vehicle.vehicle_list = vehicle_list

    print("Starting Natnet3.x interface at %s" % ("1234567"))

    # Simulation starts
    sim_start_time = time.time()
    try:

        swarm.takeoff()

        # Main loop :
        trace_count = 0
        set_vel_time = time.time()
        flight_finished=False
        starttime= time.time()
        while time.time()-sim_start_time < 1800:
            if time.time()-starttime > 0.025:
                # print(f'Freq : {1/(time.time()-starttime):.3f}')
                starttime= time.time()
                # Check if vehicles reached their final position
                distance_to_destination = [vehicle.distance_to_destination for vehicle in vehicle_list]
                flight_finished = True if np.all(distance_to_destination) < 0.50 else False
                # print(distance_to_destination)
                # print(f' {vehicle_list[0].distance_to_destination:.3f}  -  {vehicle_list[1].distance_to_destination:.3f}  -  {vehicle_list[2].distance_to_destination:.3f}')
                if flight_finished: break

                # Get Target position to follow
                # target_position = target_vehicle[0].position
                # print(f'Target Pos : {target_position}')

                # for vehicle_nr, vehicle in enumerate(vehicle_list):
                    # print('Heyyo :', target_position, type(target_position))
                    # vehicle.Set_Next_Goal(target_position+np.array([0.,0.,-0.5]), dynamic_sigma=True)
                    
                    # # if vehicle.state :
                    # if vehicle.distance_to_destination<0.50:
                    #     print('Changing goal set point')
                    #     vehicle.Set_Next_Goal(vehicle_next_goal_list[goal_index])
                    #     goal_index += 1
                    #     goal_index = goal_index%len(vehicle_next_goal_list)
                
                # COMMUNICATION 
                for index,vehicle in enumerate(vehicle_list):
                    # Update only self position
                    
                    vehicle.vehicle_list[index].position = vehicle.position
                    if index in [1,2,3]: # FIXME : This has to be replaced with a flag in vehicle!
                        for list_index in range(len(vehicle.vehicle_list)):
                            vehicle.vehicle_list[list_index].position = vehicle_list[list_index].position # calling case.Vehicle is not nice here... 1 unneccessary element update


                # flow_vels = Flow_Velocity_Calculation(vehicle_list,Arena)
                for i, vehicle in enumerate(vehicle_list):
                    vehicle.Set_Position(swarm.tellos[i].get_position_enu())
                    vehicle.Set_Velocity(swarm.tellos[i].get_velocity_enu())
                    # print(f' {i} - Vel : {vehicle.velocity[0]:.3f}  {vehicle.velocity[1]:.3f}  {vehicle.velocity[2]:.3f}')


                use_panel_flow = 1
                if use_panel_flow :
                    # flow_vels= Flow_Velocity_Calculation(vehicle_list,Arena, external_sources=population)
                    
                    # for i,id in enumerate(id_list):
                    for i, vehicle in enumerate(vehicle_list):
                        V_des = vehicle.run_flow_calc_alone()
                        mag = np.linalg.norm(V_des)
                        V_des_unit = V_des/mag
                        V_des_unit[2] = 0.
                        mag = np.clip(mag, 0., 0.5)
                        vel_enu = V_des_unit*mag
                        # swarm.tellos[i].update(swarm.tellos, )

                        # vel_enu = flow_vels[i]*limited_norm #- swarm.tellos[i].velocity_enu
                        # print(f' {i} - Flow Velocity : {flow_vels[i]}')
                        # print(f' {i} - Flow Velocity Error : {vel_enu_err}')
                        if i==0:
                            print(f' Velicle {i} -- Flow Vels : {vel_enu}')
                        heading = 0.
                        # Look towards where you go
                        # heading = np.arctan2(vel_enu[1],vel_enu[0])
                        
                        # vehicle.Set_Desired_Velocity(vel_enu, correction_method='None')

                        # and update the Nest Vinf:
                        vinfmag = 0.5
                        vehicle.propagate_simple_path(vinfmag, vel_enu)

                        swarm.tellos[i].send_velocity_enu(vehicle.velocity_desired, heading)
                        TARGET_VELS[i]=vehicle.velocity_desired

                        # swarm.tellos[i].send_velocity_enu(vehicle.velocity_desired, heading)
                        # TARGET_VELS[i]=vehicle.velocity_desired

                else:
                    # Directly fly to the point
                    for i, vehicle in enumerate(vehicle_list):
                        swarm.tellos[i].fly_to_enu(vehicle.goal, 0.)

                
                for i, vehicle in enumerate(vehicle_list):
                    if visualize:
                        p.resetBasePositionAndOrientation(quadrotors[i],
                                                    swarm.tellos[i].get_position_enu(),
                                                    swarm.tellos[i].get_quaternion(),
                                                    physicsClientId=physicsClient)
                        vehicle_colors=[[1, 0, 0], [0, 1, 0], [0, 0, 1],[1, 1, 0], [0, 1, 1], [0.5, 0.5, 1] ]
                        if trace_count > 10:
                            p.addUserDebugLine(lineFromXYZ=INIT_XYZS[i],
                                   lineToXYZ=swarm.tellos[i].get_position_enu(),
                                   lineColorRGB=vehicle_colors[i],#[1, 0, 0],
                                   lifeTime=1 * 1000,
                                   physicsClientId=physicsClient)
                            INIT_XYZS[i] = swarm.tellos[i].get_position_enu()
                            trace_count = 0
                        trace_count +=1

                # #### Log the simulation ####################################
                for i, vehicle in enumerate(id_list):
                    log.log(drone=i,
                               timestamp=time.time()-sim_start_time,
                               state= np.hstack([swarm.tellos[i].get_position_enu(), swarm.tellos[i].get_velocity_enu(), swarm.tellos[i].get_quaternion(),  np.zeros(10)]),#obs[str(j)]["state"],
                            #    control=np.hstack([TARGET_VELS[i], FLOW_VELS[i], V_sum[i], 0., target_vehicle[0].position]),
                               control=np.hstack([TARGET_VELS[i], FLOW_VELS[i], np.zeros(2), 0., target_vehicle[0].position]),
                               sim=False
                               # control=np.hstack([TARGET_VEL[j, wp_counters[j], 0:3], np.zeros(9)])
                               )

        #### Save the simulation results ###########################
        log.save(flight_type='fast_follow')
        swarm.move_down(int(40))
        swarm.land()
        voliere.stop()
        swarm.end()

    except (KeyboardInterrupt, SystemExit):
        print("Shutting down natnet interfaces...")
        log.save(flight_type='fast_follow')
        swarm.move_down(int(40))
        swarm.land()
        voliere.stop()
        swarm.end()
        if visualize:
            p.disconnect(physicsClientId=physicsClient)
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
