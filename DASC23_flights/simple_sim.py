import gflow.utils.plot_utils as ut
from gflow.cases import Cases, Case
# from time import time, sleep
from gflow.utils.simulation_utils import run_simulation, set_new_attribute
from scenebuilder.gui_sim import InteractivePlot
from scenebuilder.observer_utils import Observer
import sys

import time
from djitellopy import TelloSwarm
from voliere import VolierePosition
from voliere import Vehicle as Target
import numpy as np

from running_utils import initialize_id_swarm, connect_swarm, initialise_voliere, xyz_to_enu, run_real_case


#should probably pull this from json or something
NEXT_GOAL_LIST:list = [[ [0,0,0.5], [2,2,0.5], [3,2,0.5]],
                [ [-2,-2,1.5], [-2,-3.5,1.5]]]


#---------- OpTr- ACID - -----IP------
AC_LIST:list = [['68', '68', '192.168.1.68']]
                # ['69', '69', '192.168.1.69']]
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


class CaseMaker(Observer):
    def __init__(self) -> None:
        self.gui = InteractivePlot()
        self.gui.add_observer(self)
        # self.gui.draw_scene()

    def call(self, event:str, case:Case):
        print("Called!")
        print(f"{case.vehicle_list=}, {case.buildings =}")
        if event != "generate_case":
            raise NotImplementedError
        if len(case.vehicle_list) != len(AC_LIST):
            print("Number of vehicles does not match!")
            return

        # set_new_attribute(case, "source_strength", new_attribute_value=1)
        set_new_attribute(case, "sink_strength", new_attribute_value=5)
        set_new_attribute(case, "max_speed", new_attribute_value=0.5)
        set_new_attribute(case, "imag_source_strength", new_attribute_value=5)
        set_new_attribute(case, "source_strength", new_attribute_value=2)
        set_new_attribute(case, "dynamics_type", new_attribute_value="radius")
        set_new_attribute(case, "turn_radius", new_attribute_value=0)
        case.building_detection_threshold = 10
        # set_new_attribute(case, "")

        self.fly(case)
    
    def show(self)->None:
        self.gui.draw_scene()
        
    def fly(self, case:Case):
        print(case, case.name)
        delta_t = case.vehicle_list[0].delta_t
        update_frequency = 50  # Hz
        update_time_period = max(int(1 / (update_frequency * delta_t)), 1)

        ######################################
        # this variable controls how many time steps occur between every communication of position
        # update_time_period = 10
        ######################################

        # print(f"update every = {update_time_period}")

        id_list, swarm = initialize_id_swarm(AC_LIST)
        connect_swarm(swarm)
        voliere=initialise_voliere(swarm,AC_ID_LIST)
        print("Starting Natnet3.x interface at %s" % ("1234567"))

        INIT_XYZS = [np.append(vehicle.position[:2],0.5) for vehicle in case.vehicle_list]
        print(f"initial positions are {INIT_XYZS}")


        # for i in range(1000):
        #     print(swarm.tellos[0].get_position_enu())
        #     time.sleep(0.1)
        # sys.exit()

        try:
            #takeoff everyone
            print(swarm.tellos[0].get_position_enu())
            swarm.takeoff()
            time.sleep(2)
            # for i in range(50):
            #     print(swarm.tellos[0].get_position_enu())
            #     time.sleep(0.1)
            starttime= time.time()

            #for the first 9 seconds, move the tellos to their initial positions
            print(swarm.tellos[0].get_position_enu())
            # raise OSError
            while time.time()-starttime < 9:
                for idx, _ in enumerate(id_list):
                    swarm.tellos[idx].fly_to_enu(INIT_XYZS[idx], heading=0)
            print('Finished moving !') #Finished 
            result = run_real_case(case=case,swarm=swarm,t = 500,update_every=1,stop_at_collision=True,max_avoidance_distance=20)


            # swarm.move_down(int(40))
            swarm.land()
            voliere.stop()
            swarm.end()
            time.sleep(1)

        # except KeyboardInterrupt:
        #     sys.exit()

        except (KeyboardInterrupt, SystemExit):
            print("Shutting down natnet interfaces...")
            # log.save(flight_type='fast_follow')
            swarm.move_down(int(40))
            swarm.land()
            voliere.stop()
            swarm.end()
            # if visualize:
            #     p.disconnect(physicsClientId=physicsClient)
            time.sleep(1)

        except OSError:
            print("Natnet connection error")
            swarm.move_down(int(40))
            swarm.land()
            voliere.stop()
            swarm.end()
            exit(-1)





        case.max_avoidance_distance = 10




if __name__ == "__main__":
    case_maker = CaseMaker()
    case_maker.show()
    
#EOF