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

# ==============================================================
# ==============================================================

# ==============================================================
# ==============================================================

def crazyTown():
    # ===============
    buildings=[]

    Vehicle1 = Vehicle(ID="V1",source_strength=0.5,imag_source_strength=0.85)
    Vehicle1.Set_Goal([0.5, 0.3, 0.4], 5, 0)    
    Vehicle1.Set_Position([-0.1, 0 , 0.4])
    # Vehicle1.Go_to_Goal(0.5,0,0,0)

    vehicles = [Vehicle1]
    
    case = Cases(filename="./cases.json")
    # case.filename = "./cases.json"

    case.add_case(ID="CT_demo_1v",building_list=buildings,vehicle_list=vehicles)
    print(f'CrazyTown single vehicle demo added into {case.filename}')


if __name__ == "__main__":
#    main()
    # ERF_cases()
    crazyTown()
    
#EOF