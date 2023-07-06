from gflow.building import Building, RegularPolygon
from gflow.arena import ArenaMap
from gflow.vehicle import Vehicle
from gflow.cases import Cases
from copy import copy, deepcopy
import shutil
import json
import os, sys


def main():
    sides = 7
    position = (0,0)
    orientation = 0
    radius = 1

    obstacle = RegularPolygon(sides = sides, centre = position, rotation=orientation,radius=radius)
    building = Building(obstacle.points())
    Vehicle1 = Vehicle(ID="V1",source_strength=0.5,imag_source_strength=0.5)
    Vehicle1.Set_Goal(goal=[3,   0, 0.5], goal_strength = 5, safety = 0.0001)
    Vehicle1.Set_Position(pos = [ -3,  0.0001 , 0.5])
    Vehicle2 = Vehicle(ID="V2",source_strength=0.5,imag_source_strength=0.5)
    Vehicle2.Set_Goal(goal=[-3,   0, 0.5], goal_strength = 5, safety = 0.0001)
    Vehicle2.Set_Position(pos = [ 3,  0.0001 , 0.5])
    Vehicle3 = Vehicle(ID="V3",source_strength=0.5,imag_source_strength=0.5)
    Vehicle3.Set_Goal(goal=[0,   -3, 0.5], goal_strength = 5, safety = 0.0001)
    Vehicle3.Set_Position(pos = [ 0,  3 , 0.5])

    case = Cases()
    #print(f"Now changing the filename")
    case.filename = "./cases.json"
    buildings = []
    vehicles = []
    #buildings.append(building)
    vehicles.append(Vehicle1)
    vehicles.append(Vehicle2)
    vehicles.append(Vehicle3)

    case.add_case(ID="threedrones",building_list=buildings,vehicle_list=vehicles)
    #case.add_case(ID="test2",building_list=buildings,vehicle_list=vehicles)
    #print(case.cases)

def DASC_cases():
    ''' Hand generated buildings as corridors'''
    building_height = 0.75
    # buildings = [Building([[3, 2.5, building_height], [3, 3, building_height], [-0.5, 3, building_height], [-1.5, 3, building_height], [-3.5, 1.5, building_height], [-3, 1, building_height], [-1, 2.5, building_height], [-0.5, 2.5, building_height]]), 
    #             Building([[3, -0.5, building_height], [3, 0, building_height], [0, 0, building_height],[-2, -1, building_height],[-1.5, -1.8, building_height], [1, -0.5, building_height]])]
    buildings = [Building([[3, 2.5, building_height], [3, 3, building_height], [-0.5, 3, building_height], [-1.5, 3, building_height], [-3.5, 1.5, building_height], [-3, 1, building_height], [-1, 2.5, building_height], [-0.5, 2.5, building_height]]) ]
    # buildings = [Building([[3, 2.5, building_height], [3, 3, building_height], [-0.5, 3, building_height],  [-3.5, 1.5, building_height], [-3, 1, building_height], [-1, 2.5, building_height]]),
    #             Building([[3, -0.5, building_height], [3, 0, building_height], [0, 0, building_height],[-2, -1, building_height],[-1.5, -1.8, building_height], [1, -0.5, building_height]])]
    
    Vehicle1 = Vehicle(ID="V1",source_strength=0.,imag_source_strength=0.85)
    Vehicle1.Set_Goal([4, 1, 0.5], 5, 0)    
    Vehicle1.Set_Position([-3.4, -0.5 , 0.5])
    # Vehicle1.Go_to_Goal(0.5,0,0,0)

    # Intruder
    Vehicle2 = Vehicle(ID="V2",source_strength=1.0,imag_source_strength=0.85)
    Vehicle2.Set_Goal([3, -2 , 4.5], 5, 0)
    Vehicle2.Set_Position([-2, 4, 4.5])
    # Vehicle2.Go_to_Goal(2.5,0,0,0)

    vehicles = [Vehicle1,Vehicle2]

    case = Cases(filename="./cases.json")
    # case.filename = "./cases.json"

    case.add_case(ID="DASC23_case_0",building_list=buildings,vehicle_list=vehicles)
    print(f'DASC23 0th case added into {case.filename}')




    buildings = [Building([[4, -3, 0.75], [4, 0, 0.75], [2, 0, 0.75], [0,-2, 0.75], [0, -4, 0.75], [3, -4, 0.75]]),
                 Building([[4, 3.5, 0.75], [4, 4, 0.75], [-3, 4, 0.75],[-4, 3, 0.75],[-4, -4, 0.75], [-3.5, -4, 0.75] , [-3.5, 0, 0.75], [0, 3.5, 0.75]])]
    next_goal_list = [[-2,-4,0.5], [-2,-2,0.5] , [0,0,0.5] , [2,2,0.5] , [4,2,0.5]]

    Vehicle1 = Vehicle(ID="V1",source_strength=0.,imag_source_strength=0.85)
    Vehicle1.Set_Goal([4, 2, 0.5], 5, 0)    
    Vehicle1.Set_Position([-2., -4. , 0.5])
    # Vehicle1.Go_to_Goal(0.5,0,0,0)

    # Intruder
    Vehicle2 = Vehicle(ID="V2",source_strength=1.0,imag_source_strength=0.85)
    Vehicle2.Set_Goal([4, -4 , 1.5], 5, 0)
    Vehicle2.Set_Position([-4, 4, 1.5])

    vehicles = [Vehicle1,Vehicle2]

    case = Cases(filename="./cases.json")
    # case.filename = "./cases.json"

    case.add_case(ID="DASC23_case_1",building_list=buildings,vehicle_list=vehicles)
    print(f'DASC23 1st case added into {case.filename}')


    buildings = [Building([[4, 1.9, 0.75], [4, 2, 0.75], [-2, -4, 0.75], [-1.9,-4, 0.75]]),
                Building([[-4, -2, 0.75], [2, 4, 0.75], [1.9, 4, 0.75],[-4, -1.9, 0.75]])]
    next_goal_list = [[-4,-4,0.5], [-2,-2,0.5] , [0,0,0.5] , [2,2,0.5] , [4,4,0.5]]

    Vehicle1 = Vehicle(ID="V1",source_strength=0.,imag_source_strength=0.85)
    Vehicle1.Set_Goal([4, 4, 0.5], 5, 0)    
    Vehicle1.Set_Position([-4., -4. , 0.5])
    # Vehicle1.Go_to_Goal(0.5,0,0,0)

    # Intruder
    Vehicle2 = Vehicle(ID="V2",source_strength=1.0,imag_source_strength=0.85)
    Vehicle2.Set_Goal([4, -4 , 1.5], 5, 0)
    Vehicle2.Set_Position([-4, 4, 1.5])

    vehicles = [Vehicle1,Vehicle2]

    case = Cases(filename="./cases.json")
    # case.filename = "./cases.json"

    case.add_case(ID="DASC23_case_2",building_list=buildings,vehicle_list=vehicles)
    print(f'DASC23 2nd case added into {case.filename}')

    buildings = [Building([[4, -4, 0.75], [4, 3, 0.75], [3, 4, 0.75], [-3,4, 0.75] , [-3,3, 0.75] , [2,3, 0.75] , [3,2, 0.75] , [3,-4, 0.75]]),
                Building([[0, -4, 0.75], [0, -1, 0.75], [-1, 0, 0.75],[-3, 0, 0.75] , [-3,-3, 0.75] , [-2,-4, 0.75]])]
    next_goal_list = [[-2, 1.5,0.5], [0,1.5,0.5] , [1.5,0,0.5] , [1.5,-2,0.5] , [1.5,-4,0.5]]

    Vehicle1 = Vehicle(ID="V1",source_strength=0.,imag_source_strength=0.85)
    Vehicle1.Set_Goal([1.5, -4, 0.5], 5, 0)    
    Vehicle1.Set_Position([-2., 1.5 , 0.5])
    # Vehicle1.Go_to_Goal(0.5,0,0,0)

    # Intruder
    Vehicle2 = Vehicle(ID="V2",source_strength=1.0,imag_source_strength=0.85)
    Vehicle2.Set_Goal([4, 4 , 1.5], 5, 0)
    Vehicle2.Set_Position([-4, -4, 1.5])

    vehicles = [Vehicle1,Vehicle2]

    case = Cases(filename="./cases.json")
    # case.filename = "./cases.json"

    case.add_case(ID="DASC23_case_3",building_list=buildings,vehicle_list=vehicles)
    print(f'DASC23 3rd case added into {case.filename}')


    buildings = [Building([[3, -4, 0.75], [4, -3, 0.75], [4, 3, 0.75], [3,4, 0.75]]),
                Building([[0.5, 0.5, 0.75], [1, 1, 0.75], [1, 4, 0.75], [-1,4, 0.75] , [-1,1, 0.75] , [-0.5,0.5, 0.75]]),
                Building([[1, -4, 0.75], [1, -3.5, 0.75], [0.5, -3, 0.75],[-2, -3, 0.75] , [-3,-2, 0.75] , [-3,4, 0.75] , [-4,3, 0.75] , [-4,-3, 0.75] , [-3,-4, 0.75]])]
    next_goal_list = [[2, 4, 0.5], [2,2,0.5] , [2,0,0.5] , [1,-1,0.5] , [-1,-1,0.5]  , [-2,0,0.5]  , [-2,2,0.5] , [-2,4,0.5]]
    next_goal_list = [[2,-4,0.5] , [2,-2,0.5] , [-1,-1,0.5]  , [-2,0,0.5]  , [-2,2,0.5] , [-2,4,0.5]]


    Vehicle1 = Vehicle(ID="V1",source_strength=0.,imag_source_strength=0.85)
    Vehicle1.Set_Goal([-2, 4, 0.5], 5, 0)    
    Vehicle1.Set_Position([2., 4. , 0.5])
    # Vehicle1.Go_to_Goal(0.5,0,0,0)

    # Intruder
    Vehicle2 = Vehicle(ID="V2",source_strength=1.0,imag_source_strength=0.85)
    Vehicle2.Set_Goal([4, 4 , 1.5], 5, 0)
    Vehicle2.Set_Position([-4, -4, 1.5])

    vehicles = [Vehicle1,Vehicle2]

    case = Cases(filename="./cases.json")
    # case.filename = "./cases.json"

    case.add_case(ID="DASC23_case_4",building_list=buildings,vehicle_list=vehicles)
    print(f'DASC23 4th case added into {case.filename}')

    # Just a building...
    buildings = [Building([[3, -4, 0.75], [4, -3, 0.75], [4, 3, 0.75], [3,4, 0.75]]),]
    
    next_goal_list = [[2, 4, 0.5], [2,2,0.5] , [2,0,0.5] , [1,-1,0.5] , [-1,-1,0.5]  , [-2,0,0.5]  , [-2,2,0.5] , [-2,4,0.5]]
    next_goal_list = [[2,-4,0.5] , [2,-2,0.5] , [-1,-1,0.5]  , [-2,0,0.5]  , [-2,2,0.5] , [-2,4,0.5]]

    Vehicle1 = Vehicle(ID="V1",source_strength=1.0,imag_source_strength=0.85)
    Vehicle1.Set_Goal([-2, 4, 0.5], 5, 0)    
    Vehicle1.Set_Position([2., 4. , 0.5])
    # Vehicle1.Go_to_Goal(0.5,0,0,0)

    vehicles = [Vehicle1]

    case = Cases(filename="./cases.json")

    case.add_case(ID="target_tracking_1_vehicle",building_list=buildings,vehicle_list=vehicles)
    print(f'1 Vehicle Target Tracking case added into {case.filename}')


    buildings = [Building([[3, -4, 0.75], [4, -3, 0.75], [4, 3, 0.75], [3,4, 0.75]]),]

    next_goal_list = [[2, 4, 0.5], [2,2,0.5] , [2,0,0.5] , [1,-1,0.5] , [-1,-1,0.5]  , [-2,0,0.5]  , [-2,2,0.5] , [-2,4,0.5]]
    next_goal_list = [[2,-4,0.5] , [2,-2,0.5] , [-1,-1,0.5]  , [-2,0,0.5]  , [-2,2,0.5] , [-2,4,0.5]]


    Vehicle1 = Vehicle(ID="V1",source_strength=1.0,imag_source_strength=0.85)
    Vehicle1.Set_Goal([-2, 4, 0.5], 5, 0)    
    Vehicle1.Set_Position([2., 4. , 0.5])
    # Vehicle1.Go_to_Goal(0.5,0,0,0)

    # Intruder
    Vehicle2 = Vehicle(ID="V2",source_strength=1.0,imag_source_strength=0.85)
    Vehicle2.Set_Goal([4, 4 , 1.5], 5, 0)
    Vehicle2.Set_Position([-4, -4, 1.5])

    vehicles = [Vehicle1,Vehicle2]

    case = Cases(filename="./cases.json")
    # case.filename = "./cases.json"

    case.add_case(ID="target_tracking_2_vehicle",building_list=buildings,vehicle_list=vehicles)
    print(f'2 Vehicle Target Tracking case added into {case.filename}')
if __name__ == "__main__":
#    main()
    DASC_cases()
    
#EOF