
from gflow.vehicle import Vehicle
import gflow.utils as ut
from gflow.cases import Cases
from time import sleep
import numpy as np
from gflow.source import Source
from gflow.panel_flow import Flow_Velocity_Calculation

#case =  Cases.get_case(filename='cases.json', casename='default')

case =  Cases.get_case(filename='cases.json', casename='ERF_case_0')
population = [Source(ID="EX0",source_strength=0.3,position=np.array([-1, 2.5 , 0.45]))]
for vehicle in case.vehicle_list:
    vehicle.externalSource = population    
    
    
#case = Cases.get_case(filename='cases.json', casename='DASC23_case_1')
#case = Cases.get_case(filename='cases.json', casename='DASC23_case_2')
#case = Cases.get_case(filename='cases.json', casename='DASC23_case_3')
#case = Cases.get_case(filename='cases.json', casename='DASC23_case_4')
#case = Cases.get_case(filename='cases.json', casename='DASC23_case_5')


# simContinues = True
# while simContinues:

#     # Step the simulation
#     for index,vehicle in enumerate(case.vehicle_list):
#         if vehicle.state == 1:
#             simContinues = False 
#         elif vehicle.state != 1:
#             vehicle.run_simple_sim()
#             simContinues = True
#         #    break
simContinues = True
counter = 0
while simContinues:
    #print(counter)
    counter = counter + 1
    simContinues = any(vehicle.state != 1 for vehicle in case.vehicle_list)
    for index,vehicle in enumerate(case.vehicle_list):
        if vehicle.state != 1:
            if (counter % 30 == 0):
                vehicle.propagate_simple_path(vinfmag = 0, t0=0., dt=0.5, hor = 2.,reset_position=True)
            #print(counter)
            #print( " z_pos "    +  str( vehicle.altitude ) )    
            #vehicle.run_simple_sim()
            flow_vels = Flow_Velocity_Calculation(case.vehicle_list, case.arena, method='Vortex', external_sources=vehicle.externalSource)
            vehicle.Update_Velocity(flow_vels[index], case.arena)

    # Communication Block
    # Update positions
    for index,vehicle in enumerate(case.vehicle_list):
        # Update only self position
        vehicle.vehicle_list[index].position = vehicle.position

        # Update the listed vehicle numbers wrt every one
        # the numbers in the if statement within the list, separated by commas indicate which drones are providing their position
        if index in [0,1]:
            for list_index in range(len(vehicle.vehicle_list)):
                vehicle.vehicle_list[list_index].position = case.vehicle_list[list_index].position # calling case.Vehicle is not nice here... 1 unneccessary element update

        if vehicle.state == 1:
            #print('Vehicle ', str(index), 'has reached the goal', i)
            pass

arenaR = case.arena 
trajectory = ut.plot_trajectories2(case)
trajectory.show()

#EOF