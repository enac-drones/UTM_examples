
import numpy as np
import utm
from shapelysmooth import chaikin_smooth

class Guidance():
    def __init__(self, trajectory=None) -> None:
        self.trajectory = trajectory
        self.flight_altitude = 1. #m
        self.waypoint_index = 0
        self.waypoint_reached_distance = 20. #meters
        self.destination_reached = False

    def reset(self, new_trajectory):
        self.waypoint_index = 0
        self.destination_reached = False
        self.trajectory = new_trajectory

    # Start from zeroth waypoint and check if the next waypoint is reached and 
    # Increment the waypoint index if the waypoint has been reached
    def update_waypoint_index(self, aircraft_position):
        waypoint = self.trajectory[self.waypoint_index]
        print('Waypoint : ', waypoint)
        distance = np.linalg.norm(aircraft_position[:2] - waypoint[:2]) # FIXME : checking only 2D , we may want to convert this to 3D...
        if distance < self.waypoint_reached_distance:
            self.waypoint_index += 1
            print("NEXT WAYPOINT : ", self.waypoint_index)
            if self.waypoint_index >= len(self.trajectory):
                self.waypoint_index = len(self.trajectory)-1
                self.destination_reached = True

    # Calculate velocity vector between aircraft position and the next point on the trajectory
    def calculate_velocity_vector(self, aircraft_position):
        self.update_waypoint_index(aircraft_position)
        waypoint = np.hstack([self.trajectory[self.waypoint_index][:2], self.flight_altitude])  #FIXME : 2D-3D issues... 
        velocity_vector = waypoint - aircraft_position
        velocity_unit_vector = self.normalize_vector(velocity_vector)
        return velocity_unit_vector

    def normalize_vector(self, vector):
        if np.linalg.norm(vector) > 0.1: # FIXME : 0.1 is going to create problems !!!
            unit_vector = vector / np.linalg.norm(vector)
        else:
            unit_vector = np.array([0,0,0])
        return unit_vector
    

    
    # def find_closest_point(self, aircraft_position):
    #     # Find the closest point on the trajectory to the aircraft position
    #     closest_point = None
    #     closest_distance = float('inf')
    #     for point in self.trajectory:
    #         distance = np.linalg.norm(point - aircraft_position)
    #         if distance < closest_distance:
    #             closest_distance = distance
    #             closest_point = point

    #     return closest_point, closest_distance
        