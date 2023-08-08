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

def ERF_cases():
    ''' Hand generated buildings as corridors'''
    building_height = 0.75
    buildings = [Building([[3.0, 2.0, 1.2], [2.75, 1.567, 1.2], [2.25, 1.567, 1.2], [2.0, 2.0, 1.2], [2.25, 2.433, 1.2], [2.75, 2.433, 1.2]]), #AddCircularBuilding( 2.5, 2, 6, 0.5, 1.2, angle = 0)
                 Building([[1.0, 3.0, 1.5], [0.75, 2.567, 1.5], [0.25, 2.567, 1.5], [0.0, 3.0, 1.5], [0.25, 3.433, 1.5], [0.75, 3.433, 1.5]]), #AddCircularBuilding( 0.5, 3, 6, 0.5, 1.5, angle = 0)
                 Building([[1.0, 0.5, 2], [0.75, 0.067, 2], [0.25, 0.067, 2], [0.0, 0.5, 2], [0.25, 0.933, 2], [0.75, 0.933, 2]]), #AddCircularBuilding( 0.5, 0.5, 6, 0.5, 2, angle = 0)
                 Building([[-2.65, 1.5, 1.5], [-3.0, 1.15, 1.5], [-3.35, 1.5, 1.5], [-3.0, 1.85, 1.5]]), #AddCircularBuilding( -3, 1.5, 4, 0.35, 1.5, angle = 0)
                 Building([[-2.65, -1.5, 1.5], [-3.0, -1.85, 1.5], [-3.35, -1.5, 1.5], [-3.0, -1.15, 1.5]]), #AddCircularBuilding( -3, -1.5, 4, 0.35, 1.5, angle = 0)
                 Building([[-1.15, -0.2, 1.5], [-1.5, -0.55, 1.5], [-1.85, -0.2, 1.5], [-1.5, 0.15, 1.5]]), #AddCircularBuilding( -1.5, -0.2, 4, 0.35, 1.5, angle = 0)
                 Building([[1.5, -2.5, 1.2], [1, -2.5, 1.2], [1, -1.4, 1.2], [1.5, -1, 1.2]]),
                 Building([[3.5, -2.5, 1.2], [3, -2.5, 1.2], [3, -1, 1.2], [3.5, -1.4, 1.2]])]
    
		
    
    Vehicle1 = Vehicle(ID="V1",source_strength=0.5,imag_source_strength=0.85)
    Vehicle1.Set_Goal([-3, 0, 0.5], 5, 0)    
    Vehicle1.Set_Position([3, 1 , 0.5])
    # Vehicle1.Go_to_Goal(0.5,0,0,0)

    Vehicle2 = Vehicle(ID="V2",source_strength=0.5,imag_source_strength=0.85)
    Vehicle2.Set_Goal([2, 3.5, 0.45], 5, 0)
    Vehicle2.Set_Position([-2, -3 , 0.5])
    # Vehicle2.Go_to_Goal(2.5,0,0,0) 

    Vehicle3 = Vehicle("V3",source_strength=0.5,imag_source_strength=0.85)
    Vehicle3.Set_Goal([2, 0, 0.5], 5, 0)
    Vehicle3.Set_Position([-3, 3 , 0.5])
    vehicles = [Vehicle1,Vehicle2,Vehicle3]
    # Vehicle3.Go_to_Goal(2.5,0,0,0)

    case = Cases(filename="./cases.json")
    # case.filename = "./cases.json"

    case.add_case(ID="ERF_case_0",building_list=buildings,vehicle_list=vehicles)
    print(f'ERF 0th case added into {case.filename}')


    ''' Hand generated buildings as corridors'''
    building_height = 0.75
    buildings = [Building([[-0.82 ,  3.217,  1.85 ],[-1.121,  3.052,  1.85 ],[-1.451,  2.384,  1.85 ],[-1.225,  1.749,  1.85 ],[-0.806,  1.479,  1.85 ],[ 0.363,  2.253,  1.85 ]]),
                 Building([[ 1.983, -0.884,  1.839],[ 1.517, -0.333,  1.839], [ 1.303, -0.302,  1.839],[ 1.111, -0.333,  1.839],[ 0.907, -0.437,  1.839],[ 0.777, -0.566,  1.839],[ 1.948, -1.215,  1.839]]),
                 Building([[4.163, 2.803, 1.857],[2.437, 3.887, 1.857],[1.698, 3.337, 1.857],[1.794, 1.718, 1.857],[2.206, 1.389, 1.857],[3.811, 1.663, 1.857],[4.155, 2.332, 1.857]]),
                 Building([[-1.184, -1.625,  1.58 ],[-2.677, -1.711,  1.58 ],[-3.166, -3.644,  1.58 ],[-2.698, -4.168,  1.58 ],[-1.793, -4.429,  1.58 ], [-0.679, -3.848,  1.58 ],[-0.534, -3.616,  1.58 ],[-0.422, -3.32 ,  1.58 ]]),
                 Building([[-0.141,  0.489,  1.697],[-0.222,  0.651,  1.697],[-1.031,  0.426,  1.697],[-1.014,  0.239,  1.697],[-0.358, -0.024,  1.697],[-0.18 ,  0.157,  1.697],[-0.139,  0.265,  1.697]])]    
    
    Vehicle1 = Vehicle(ID="V1",source_strength=0.5,imag_source_strength=0.85)
    Vehicle1.Set_Goal([ 2.7, -2.7,  0.5], 5, 0)    
    Vehicle1.Set_Position([0.7, 3. , 0.5])
    # Vehicle1.Go_to_Goal(0.5,0,0,0)

    Vehicle2 = Vehicle(ID="V2",source_strength=0.5,imag_source_strength=0.85)
    Vehicle2.Set_Goal([-3.4,  1.9,  0.5], 5, 0)
    Vehicle2.Set_Position([3.1, -0.7,  0.5])
    # Vehicle2.Go_to_Goal(2.5,0,0,0) 

    Vehicle3 = Vehicle("V3",source_strength=0.5,imag_source_strength=0.85)
    Vehicle3.Set_Goal([ 1.9, -3.2,  0.5], 5, 0)
    Vehicle3.Set_Position([-2.3,  1.2,  0.5])
    vehicles = [Vehicle1,Vehicle2,Vehicle3]
    # Vehicle3.Go_to_Goal(2.5,0,0,0)

    case = Cases(filename="./cases.json")
    # case.filename = "./cases.json"

    case.add_case(ID="ERF_case_1",building_list=buildings,vehicle_list=vehicles)
    print(f'ERF 1th case added into {case.filename}')

def DASC_cases():
    ''' Hand generated buildings as corridors'''
    building_height = 0.75
    # buildings = [Building([[3, 2.5, building_height], [3, 3, building_height], [-0.5, 3, building_height], [-1.5, 3, building_height], [-3.5, 1.5, building_height], [-3, 1, building_height], [-1, 2.5, building_height], [-0.5, 2.5, building_height]]), 
    #             Building([[3, -0.5, building_height], [3, 0, building_height], [0, 0, building_height],[-2, -1, building_height],[-1.5, -1.8, building_height], [1, -0.5, building_height]])]
    buildings = [Building([[4, -0.1, 0.75], [4, 0, 0.75], [2, 0, 0.75], [0,-2, 0.75], [0, -4, 0.75], [0.1, -4, 0.75], [0.1, -2, 0.75], [2, -0.1, 0.75]]),
                 Building([[4, 3.5, 0.75], [4, 3.6, 0.75], [0, 3.6, 0.75],[-3.6, 0, 0.75],[-3.6, -4, 0.75], [-3.5, -4, 0.75] , [-3.5, 0, 0.75], [0, 3.5, 0.75]])]
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




    buildings = [Building([[4, -0.1, 0.75], [4, 0, 0.75], [2, 0, 0.75], [0,-2, 0.75], [0, -4, 0.75], [0.1, -4, 0.75], [0.1, -2, 0.75], [2, -0.1, 0.75]]),
                 Building([[4, 3.5, 0.75], [4, 3.6, 0.75], [0, 3.6, 0.75],[-3.6, 0, 0.75],[-3.6, -4, 0.75], [-3.5, -4, 0.75] , [-3.5, 0, 0.75], [0, 3.5, 0.75]])]
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

    buildings = [Building([[3.1, -4, 0.75], [3.1, 2, 0.75], [2, 3.1, 0.75], [-3,3.1, 0.75] , [-3,3, 0.75] , [2,3, 0.75] , [3,2, 0.75] , [3,-4, 0.75]]),
                 Building([[0, -4, 0.75], [0, -1, 0.75], [-1, 0, 0.75],[-3, 0, 0.75] , [-3,-0.1, 0.75] , [-1,-0.1, 0.75], [-0.1,-1, 0.75], [-0.1,-4, 0.75]])]
    next_goal_list = [[-2, 1.5,0.5], [0,1.5,0.5] , [1.5,0,0.5] , [1.5,-2,0.5] , [1.5,-4,0.5]]

    Vehicle1 = Vehicle(ID="V1",source_strength=0.,imag_source_strength=0.85)
    Vehicle1.Set_Goal([1.5, -4, 0.5], 5, 0)    
    Vehicle1.Set_Position([-2., 1.5 , 0.5])
    # Vehicle1.Go_to_Goal(0.5,0,0,0)

    # Intruder
    Vehicle2 = Vehicle(ID="V2",source_strength=1.0,imag_source_strength=0.85)
    Vehicle2.Set_Goal([4, 4, 1.5], 5, 0)
    Vehicle2.Set_Position([-1, -1 , 1.5])
    #Vehicle2.Set_Goal([-4, -4, 1.5], 5, 0)
    #Vehicle2.Set_Position([4, 4 , 1.5])

    vehicles = [Vehicle1,Vehicle2]

    case = Cases(filename="./cases.json")
    # case.filename = "./cases.json"

    case.add_case(ID="DASC23_case_3",building_list=buildings,vehicle_list=vehicles)
    print(f'DASC23 3rd case added into {case.filename}')


    buildings = [Building([[3, -4, 0.75], [3.1, -3, 0.75], [3.1, 3, 0.75], [3,4, 0.75]]),
                 Building([[0.5, 0.5, 0.75], [1.1, 1, 0.75], [1.1, 4, 0.75], [1, 4, 0.75], [1, 1, 0.75], [0.5, 0.6, 0.75], [-1.5, 0.6, 0.75], [-1.9, 1, 0.75], [-1.9, 4, 0.75], [-2,4, 0.75] , [-2,1, 0.75] , [-1.5,0.5, 0.75]]),
                 Building([[1, -4, 0.75], [1, -3.9, 0.75],[-3, -3.9, 0.75] , [-3.9,-3, 0.75] , [-3.9,4, 0.75] , [-4,3, 0.75] , [-4,-3, 0.75] , [-3,-4, 0.75]])]
    next_goal_list = [[2, 4, 0.5], [2,2,0.5] , [2,0,0.5] , [1,-1,0.5] , [-1,-1,0.5]  , [-2,0,0.5]  , [-2,2,0.5] , [-2,4,0.5]]
    next_goal_list = [[2,-4,0.5] , [2,-2,0.5] , [-1,-1,0.5]  , [-2,0,0.5]  , [-2,2,0.5] , [-2,4,0.5]]


    Vehicle1 = Vehicle(ID="V1",source_strength=0.,imag_source_strength=0.85)
    Vehicle1.Set_Goal([-3, 3.5, 0.5], 5, 0)    
    Vehicle1.Set_Position([2., 3.5 , 0.5])
    # Vehicle1.Go_to_Goal(0.5,0,0,0)

    # Intruder
    Vehicle2 = Vehicle(ID="V2",source_strength=1.0,imag_source_strength=0.85)
    Vehicle2.Set_Goal([4, 3.5 , 1.5], 5, 0)
    Vehicle2.Set_Position([-4, -3.5, 1.5])

    vehicles = [Vehicle1,Vehicle2]

    case = Cases(filename="./cases.json")
    # case.filename = "./cases.json"

    case.add_case(ID="DASC23_case_4",building_list=buildings,vehicle_list=vehicles)
    print(f'DASC23 4th case added into {case.filename}')

    buildings = [Building([[3.1, -3, 0.75], [3.1, 3, 0.75], [3,4, 0.75], [3,-3, 0.75] , [2,-3.9, 0.75]  , [-3, -3.9, 0.75] , [-3.9,-3, 0.75] , [-3.9,4, 0.75] , [-4,3, 0.75] , [-4,-3, 0.75] , [-3,-4, 0.75], [2,-4, 0.75] ]),
                Building([[0.5, 0.5, 0.75], [1.1, 1, 0.75], [1.1, 4, 0.75], [1, 4, 0.75], [1, 1, 0.75], [0.5, 0.6, 0.75], [-1.5, 0.6, 0.75], [-1.9, 1, 0.75], [-1.9, 4, 0.75], [-2,4, 0.75] , [-2,1, 0.75] , [-1.5,0.5, 0.75]])]
    #next_goal_list = [[2, 4, 0.5], [2,2,0.5] , [2,0,0.5] , [1,-1,0.5] , [-1,-1,0.5]  , [-2,0,0.5]  , [-2,2,0.5] , [-2,4,0.5]]
    #next_goal_list = [[2,-4,0.5] , [2,-2,0.5] , [-1,-1,0.5]  , [-2,0,0.5]  , [-2,2,0.5] , [-2,4,0.5]]


    Vehicle1 = Vehicle(ID="V1",source_strength=0.,imag_source_strength=0.85)
    Vehicle1.Set_Goal([-3, 3.5, 0.5], 5, 0)    
    Vehicle1.Set_Position([2., 3.5 , 0.5])
    # Vehicle1.Go_to_Goal(0.5,0,0,0)

    # Intruder
    Vehicle2 = Vehicle(ID="V2",source_strength=2.5,imag_source_strength=0.85)
    Vehicle2.Set_Goal([4, 1.5 , 1.5], 5, 0)
    Vehicle2.Set_Position([-4.5, -3.5, 1.5])

    vehicles = [Vehicle1, Vehicle2]

    case = Cases(filename="./cases.json")
    # case.filename = "./cases.json"

    case.add_case(ID="DASC23_case_5",building_list=buildings,vehicle_list=vehicles)
    print(f'DASC23 5th case added into {case.filename}')

if __name__ == "__main__":
#    main()
    ERF_cases()
    
#EOF