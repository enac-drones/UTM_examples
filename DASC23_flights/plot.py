
import os, sys
import numpy as np
import time
import matplotlib.pyplot as plt
import argparse

from scipy.spatial.transform import Rotation
from gflow.arena import ArenaMap
import pyclipper


import pdb

# sys.path.insert(0, '../')
# from path_plan_w_panel import ArenaMap

def read(filename):
    with np.load(filename, 'rb', allow_pickle=True) as log:
        timestamp= log['timestamps']
        controls = log['controls']
        states = log['states']
        building_hulls = log['building_hulls']
    return timestamp, states, controls, building_hulls

def plot_wind_states(timestamp, states, controls):

    # fig_0 = plt.figure(figsize=(5,5) )
    # for _v in range(states.shape[0]):
    #     time = timestamp[_v]
    #     pos_e = states[_v,0,:]
    #     pos_n = states[_v,1,:]
    #     pos_u = states[_v,2,:]

    #     # plt.scatter(pos_e, pos_n)
    #     plt.grid()
    #     plt.plot(pos_e, pos_n)

    fig_1 = plt.figure(figsize=(10,5) )
    for _v in range(states.shape[0]):
        time = timestamp[_v]
        vel_e = states[_v,3,:]
        vel_n = states[_v,4,:]
        vel_u = states[_v,5,:]

        plt.grid()
        plt.plot(time,vel_e, label='Vel_E')
        plt.plot(time,vel_n, label='Vel_N')
        # plt.plot(time,vel_u)

    fig_2 = plt.figure(figsize=(10,5) )
    for _v in range(states.shape[0]):
        time = timestamp[_v]
        vel_e  = controls[_v,6,:]
        vel_we = controls[_v,7,:]
        vel_n  = controls[_v,8,:]
        vel_wn = controls[_v,9,:]

        plt.grid()
        plt.plot(time,vel_e , label='Vel_E')
        plt.plot(time,vel_n , label='Vel_N')
        plt.plot(time,vel_we, label='Wind_Vel_E')
        plt.plot(time,vel_wn, label='Wind_Vel_N')
        plt.legend()

    plt.show()


def plot_trajectory_2(timestamp, states, controls, building_hulls):
    fig4 = plt.figure(figsize=(12,12))
    minx = -5 # min(min(building.vertices[:,0].tolist()),minx)
    maxx = 5 # max(max(building.vertices[:,0].tolist()),maxx)
    miny = -5 # min(min(building.vertices[:,1].tolist()),miny)
    maxy = 5 # max(max(building.vertices[:,1].tolist()),maxy)
    #plt.figure(figsize=(20,10))
    plt.grid(color = 'k', linestyle = '-.', linewidth = 0.5)
    # for index in range(building_hulls.shape[0]):
    #     plt.plot(     np.hstack((building_hulls[index][:,0],building_hulls[index][0,0]))  , np.hstack((building_hulls[index][:,1],building_hulls[index][0,1] )) ,'salmon', alpha=0.5 )
    #     plt.fill(     np.hstack((building_hulls[index][:,0],building_hulls[index][0,0]))  , np.hstack((building_hulls[index][:,1],building_hulls[index][0,1] )) ,'salmon', alpha=0.5 )


    for building in building_hulls:
        building_arr = np.array(building)
        safetyfac = 1.1
        rad = 0.0 #0.17
        rad = rad * safetyfac
        scale = 1e6
        pco = pyclipper.PyclipperOffset()
        pco.AddPath( (building_arr[:,:2] * scale).astype(int).tolist() , pyclipper.JT_MITER, pyclipper.ET_CLOSEDPOLYGON)

        inflated  =  np.array ( pco.Execute( rad*scale )[0] ) / scale
        height = building_arr[0,2]
        points = np.hstack(( inflated , np.ones((inflated.shape[0],1)) *height ))
        Xavg = np.mean(points[:,0:1])
        Yavg = np.mean(points[:,1:2])
        angles = np.arctan2( ( Yavg*np.ones(len(points[:,1])) - points[:,1] ) , ( Xavg*np.ones(len(points[:,0])) - points[:,0] ) )
        sorted_angles = sorted(zip(angles, points), reverse = True)
        points_sorted = np.vstack([x for y, x in sorted_angles])
        building_arr = points_sorted
        # plt.plot( np.hstack((building_arr[:,0],building_arr[0,0]))  , np.hstack((building_arr[:,1],building_arr[0,1] )) ,'salmon', alpha=0.8 )
        # plt.fill( np.hstack((building_arr[:,0],building_arr[0,0]))  , np.hstack((building_arr[:,1],building_arr[0,1] )) ,'salmon', alpha=0.8 )

    for building in building_hulls:
        building_arr = np.array(building)
        #pdb.set_trace()
        plt.plot(     np.hstack((building_arr[:,0],building_arr[0,0]))  , np.hstack((building_arr[:,1],building_arr[0,1])) ,'m', alpha=1 )
        plt.fill(     np.hstack((building_arr[:,0],building_arr[0,0]))  , np.hstack((building_arr[:,1],building_arr[0,1] )) ,'m', alpha=1 )

    for _v in range(states.shape[0]):
        # if _v != 2 : continue
        # k1 = 7300
        # k2 = 7800
        k1=0
        k2=len(timestamp[_v])
        time = timestamp[_v]
        pos_e = states[_v,0,k1:k2]
        pos_n = states[_v,1,k1:k2]
        pos_u = states[_v,2,k1:k2]
        desired_e = controls[_v,0,k1:k2]
        desired_n = controls[_v,1,k1:k2]

        # plt.scatter(pos_e, pos_n)
        plt.plot(pos_e, pos_n)
        # pdb.set_trace()
        n=20
        scale = 0.5
        pos_e = pos_e[::n]
        pos_n = pos_n[::n]
        desired_e = desired_e[::n]*scale
        desired_n = desired_n[::n]*scale
        for i in range(len(pos_e)):
            plt.arrow(pos_e[i], pos_n[i], desired_e[i], desired_n[i], head_width=0.03, head_length=0.03, fc='k', ec='k')

    # Target position is recorded in each vehicle's control dictionary
    # So we can take vehicle 0's control dict and obtain the positions
    target_e = controls[0,9,k1:k2]
    target_n = controls[0,10,k1:k2]

    # plt.scatter(pos_e, pos_n)
    plt.plot(target_e, target_n, '--r')

    # plt.plot(Vehicle1.path[:,0],Vehicle1.path[:,1],'r',linewidth = 2)
    # plt.plot(Vehicle1.path[0,0],Vehicle1.path[0,1],'o')
    # plt.plot(Vehicle1.goal[0],Vehicle1.goal[1],'x')
    # plt.plot(Vehicle2.path[:,0],Vehicle2.path[:,1],'b',linewidth = 2)
    # plt.plot(Vehicle2.path[0,0],Vehicle2.path[0,1],'o')
    # plt.plot(Vehicle2.goal[0],Vehicle2.goal[1],'x') 
    #Steamline_Calculator(Vehicle_list,0,Arena,resolution = 30)
    plt.xlim([minx, maxx])
    plt.ylim([miny, maxy])
    # plt.show()
    plt.show()


def plot_trajectory(timestamp, states, controls, building_hulls):

    # fig_0 = plt.figure(figsize=(5,5) )
    # for _v in range(states.shape[0]):
    #     time = timestamp[_v]
    #     pos_e = states[_v,0,:]
    #     pos_n = states[_v,1,:]
    #     pos_u = states[_v,2,:]

    #     # plt.scatter(pos_e, pos_n)
    #     plt.grid()
    #     plt.plot(pos_e, pos_n)

    fig_1 = plt.figure(figsize=(10,5) )
    for _v in range(states.shape[0]):
        time = timestamp[_v]
        vel_e = states[_v,3,:]
        vel_n = states[_v,4,:]
        vel_u = states[_v,5,:]

        plt.grid()
        plt.plot(time,vel_e)
        plt.plot(time,vel_n)
        plt.plot(time,vel_u)

    # fig_1_1 = plt.figure(figsize=(10,5) )
    # for _v in range(states.shape[0]):
    #     time = timestamp[_v]
    #     phi  = states[_v,9,:]
    #     theta= states[_v,10,:]
    #     psi  = states[_v,11,:]


    #     plt.grid()
    #     plt.plot(time,phi, label='Phi')
    #     plt.plot(time,theta, label='Theta')
    #     plt.plot(time,psi, label='Psi')


    fig_1_2 = plt.figure(figsize=(10,5) )
    # pdb.set_trace()
    euler = np.zeros((states.shape[0],3,states.shape[2]))
    for _v in range(1):
    # for _v in range(states.shape[0]):
        time = timestamp[_v]
        qx = states[_v,6,:]
        qy = states[_v,7,:]
        qz = states[_v,8,:]
        qw = states[_v,9,:]

        plt.plot(time,qx, label='qx')
        plt.plot(time,qy, label='qy')
        plt.plot(time,qz, label='qz')
        plt.plot(time,qw, label='qw')
        plt.grid(); plt.legend()
        i=0
        for _qx,_qy,_qz,_qw in zip(qx,qy,qz,qw):
            quat = np.array([_qx,_qy,_qz,_qw])
            rot = Rotation.from_quat(quat)
            euler[_v,:,i] = rot.as_euler('xyz', degrees=True)
            i+=1

    fig_1_3 = plt.figure(figsize=(10,5) )
    plt.plot(time,euler[0,0,:], label='phi')
    plt.plot(time,euler[0,1,:], label='theta')
    plt.plot(time,euler[0,2,:], label='psi')
    plt.grid(); plt.legend()


    fig_1_4 = plt.figure(figsize=(10,5) )
    plt.plot(euler[0,2,:],euler[0,0,:], label='phi')
    plt.plot(euler[0,2,:],euler[0,1,:], label='theta')
    plt.xlabel('psi')
    plt.grid(); plt.legend()


    fig_2 = plt.figure(figsize=(10,5) )
    for _v in range(states.shape[0]):
        time = timestamp[_v]
        vel_e = controls[_v,0,:]
        vel_n = controls[_v,1,:]
        vel_u = controls[_v,2,:]

        plt.grid()
        plt.plot(time,vel_e)
        plt.plot(time,vel_n)
        plt.plot(time,vel_u)

    fig_2_5 = plt.figure(figsize=(10,5) )
    for _v in range(states.shape[0]):
        time = timestamp[_v]
        pos_e = states[_v,0,:]
        pos_n = states[_v,1,:]
        pos_u = states[_v,2,:]

        plt.plot(time, pos_e)
        plt.plot(time, pos_n)
        plt.plot(time, pos_u)

    fig_3 = plt.figure(figsize=(10,5) )
    for _v in range(states.shape[0]):
        time = timestamp[_v]
        vel_e = controls[_v,3,:]
        vel_n = controls[_v,4,:]
        vel_u = controls[_v,5,:]

        plt.grid()
        plt.plot(time,vel_e)
        plt.plot(time,vel_n)
        plt.plot(time,vel_u)

    # Arena = ArenaMap(version = 65)
    # Arena = ArenaMap(version = None, building_hulls=building_hulls)

    # Arena.Inflate(radius = 0.2)
    # ArenaR = ArenaMap(version = 65)

    fig4 = plt.figure(figsize=(5,5))
    minx = -5 # min(min(building.vertices[:,0].tolist()),minx)
    maxx = 5 # max(max(building.vertices[:,0].tolist()),maxx)
    miny = -5 # min(min(building.vertices[:,1].tolist()),miny)
    maxy = 5 # max(max(building.vertices[:,1].tolist()),maxy)
    #plt.figure(figsize=(20,10))
    plt.grid(color = 'k', linestyle = '-.', linewidth = 0.5)
    # for index in range(building_hulls.shape[0]):
    #     plt.plot(     np.hstack((building_hulls[index][:,0],building_hulls[index][0,0]))  , np.hstack((building_hulls[index][:,1],building_hulls[index][0,1] )) ,'salmon', alpha=0.5 )
    #     plt.fill(     np.hstack((building_hulls[index][:,0],building_hulls[index][0,0]))  , np.hstack((building_hulls[index][:,1],building_hulls[index][0,1] )) ,'salmon', alpha=0.5 )
    for building in building_hulls:
        building_arr = np.array(building)
        #pdb.set_trace()
        plt.plot(     np.hstack((building_arr[:,0],building_arr[0,0]))  , np.hstack((building_arr[:,1],building_arr[0,1])) ,'salmon', alpha=0.85 )
        plt.fill(     np.hstack((building_arr[:,0],building_arr[0,0]))  , np.hstack((building_arr[:,1],building_arr[0,1] )) ,'salmon', alpha=0.85 )
    # for buildingR in ArenaR.buildings:
    #     plt.plot(     np.hstack((buildingR.vertices[:,0],buildingR.vertices[0,0]))  , np.hstack((buildingR.vertices[:,1],buildingR.vertices[0,1] )) ,'m' )
    #     plt.fill(     np.hstack((buildingR.vertices[:,0],buildingR.vertices[0,0]))  , np.hstack((buildingR.vertices[:,1],buildingR.vertices[0,1] )) ,'m' )
    for _v in range(states.shape[0]):
        time = timestamp[_v]
        pos_e = np.array(states[_v,0,:])
        pos_n = np.array(states[_v,1,:])
        pos_u = np.array(states[_v,2,:])
        desired_e = np.array(controls[_v,0,:])
        desired_n = np.array(controls[_v,1,:])

        # plt.scatter(pos_e, pos_n)
        plt.plot(pos_e, pos_n)
        n=20
        # plt.arrow(pos_e[::n], pos_n[::n], desired_e[::n], desired_n[::n], head_width=0.03, head_length=0.03, fc='k', ec='k')

    # plt.plot(Vehicle1.path[:,0],Vehicle1.path[:,1],'r',linewidth = 2)
    # plt.plot(Vehicle1.path[0,0],Vehicle1.path[0,1],'o')
    # plt.plot(Vehicle1.goal[0],Vehicle1.goal[1],'x')
    # plt.plot(Vehicle2.path[:,0],Vehicle2.path[:,1],'b',linewidth = 2)
    # plt.plot(Vehicle2.path[0,0],Vehicle2.path[0,1],'o')
    # plt.plot(Vehicle2.goal[0],Vehicle2.goal[1],'x') 
    #Steamline_Calculator(Vehicle_list,0,Arena,resolution = 30)
    plt.xlim([minx, maxx])
    plt.ylim([miny, maxy])
    # plt.show()
    plt.show()


def plot_wind_diff(timestamp, states, controls, timestamp2, states2, controls2):
    Arena = ArenaMap(version = 102)
    Arena.Inflate(radius = 0.2)
    ArenaR = ArenaMap(version = 102)
    minx = -5 # min(min(building.vertices[:,0].tolist()),minx)
    maxx = 5 # max(max(building.vertices[:,0].tolist()),maxx)
    miny = -5 # min(min(building.vertices[:,1].tolist()),miny)
    maxy = 5 # max(max(building.vertices[:,1].tolist()),maxy)
    colorlist = ['r','b','g']
    labellist1 = ['_nolegend_','Simulation','_nolegend_']
    labellist2 = ['_nolegend_','Experiment','_nolegend_']
    labellist3 = ['_nolegend_','Velocity Vector','_nolegend_']
    plt.figure(figsize=(5,5))
    plt.grid(color = 'k', linestyle = '-.', linewidth = 0.5)
    for building in Arena.buildings:
        plt.plot(     np.hstack((building.vertices[:,0],building.vertices[0,0]))  , np.hstack((building.vertices[:,1],building.vertices[0,1] )) ,'salmon', alpha=0.5 )
        plt.fill(     np.hstack((building.vertices[:,0],building.vertices[0,0]))  , np.hstack((building.vertices[:,1],building.vertices[0,1] )) ,'salmon', alpha=0.5 )
    for buildingR in ArenaR.buildings:
        plt.plot(     np.hstack((buildingR.vertices[:,0],buildingR.vertices[0,0]))  , np.hstack((buildingR.vertices[:,1],buildingR.vertices[0,1] )) ,'m' )
        plt.fill(     np.hstack((buildingR.vertices[:,0],buildingR.vertices[0,0]))  , np.hstack((buildingR.vertices[:,1],buildingR.vertices[0,1] )) ,'m' )
    for _v in range(states.shape[0]):
        time = timestamp[_v]

        pos_e  = states[_v,0,:]
        pos_n  = states[_v,1,:]

        pos_e2 = states2[_v,0,:]
        pos_n2 = states2[_v,1,:]

        vel_e = states[_v,3,:]
        vel_n = states[_v,4,:]

        cont_vel_e = controls[_v,0,:]
        cont_vel_n = controls[_v,1,:]

        X = pos_e[1::100]
        Y = pos_n[1::100]
        U = cont_vel_e[1::100]
        V = cont_vel_n[1::100]

        plt.plot(pos_e2, pos_n2, color=colorlist[_v], linewidth = 0.5, linestyle ='--',alpha = 0.5, label=labellist1[_v])
        plt.plot(pos_e,  pos_n,  color=colorlist[1], linewidth = 0.5, label=labellist2[_v])
        # plt.quiver(X, Y, U, V, color='k', linewidth = 1, units='xy', scale=1, width = 0.075, headwidth = 2, headlength = 3, headaxislength = 3, alpha = 1, label=labellist3[_v])
        plt.legend(loc = 'lower left',fontsize='small')
    plt.xlabel('East-direction --> (m)')
    plt.ylabel('North-direction --> (m)')
    plt.xlim([minx, maxx])
    plt.ylim([miny, maxy])
    plt.show()



def main():
    parser = argparse.ArgumentParser(description='Flight Log Plotter')
    parser.add_argument('--file',              default=None,     type=str,       help='Flight log filename', metavar='')
    parser.add_argument('--f1',              default=None,     type=str,       help='Flight log filename', metavar='')
    parser.add_argument('--f2',              default=None,     type=str,       help='Flight log filename', metavar='')
    parser.add_argument('--save',              default=False,    type=bool,      help='Whether to save the plots to a file or not)', metavar='')
    
    ARGS = parser.parse_args()

    # filename = 'save-voliere-flight-02.28.2022_17.48.34.npy'
    
    # timestamp, states, controls = read(ARGS.file)

    # plot_trajectory(timestamp, states, controls)

    timestamp, states, controls, building_hulls = read(ARGS.f1)
    #timestamp2, states2, controls2 = read(ARGS.f2)
    #plot_wind_diff(timestamp, states, controls, timestamp2, states2, controls2)
    # plot_wind_states(timestamp, states, controls)
    # plot_trajectory(timestamp, states, controls, building_hulls)
    plot_trajectory_2(timestamp, states, controls, building_hulls)



    # plot_trajectory2(timestamp, states, controls, Vehicle_list=vehicle_list, building_hulls=building_hulls)
  




class plot_trajectories2:
    '''Same as above but trying to use the FuncAnimation for the play button implementation which supposedly uses less CPU'''
    def __init__(self, Arena=None, ArenaR=None, Vehicle_list=None, building_hulls=None):
        self.Arena = Arena
        self.ArenaR = ArenaR
        self.Vehicle_list = Vehicle_list
        self.bulding_hulls = building_hulls
        #plt.close('all')
        self.fig,self.ax = self.plot_setup()
        self.slider = self.create_slider(fig = self.fig)
        self.play_button = self.create_button(fig=self.fig)
        self.info_box = self.create_info_box(ax = self.ax)
        self.plot_buildings(ax=self.ax,Arena = self.Arena, ArenaR = self.ArenaR)
        self.time_steps_max=self.get_max_timesteps(self.Vehicle_list)

        #plot_list is a list that will hold the references to each trajectory plot
        self.plot_list = []
        #list to store (the references to) the drone icon plots 
        self.drone_list = []
        #same as above, but display these only when certain separation minima are not respected
        self.warning_circles = []
        #initialize empty array to store vehicle positions
        self.positions = np.zeros((len(Vehicle_list), 3))
        self.initial_plot(Vehicle_list=self.Vehicle_list,ax= self.ax)

        #this variable will increase by 1 every time a collision is detected
        self.conflict_iterator = 0
        self.conflicts = {}
        self.conflicts["addressed"] = []
        #create new set to hold the conflicts that have already caused the simulation to pause:
        #addressed_conflicts = set()
        #connect the action of pressing the spacebar to the result we want it to have
        self.fig.canvas.mpl_connect('key_press_event', self.on_press)
        # Create the animation object, it starts automatically against my will BUG
        self.anim = FuncAnimation(self.fig, self.animate, interval=50, frames = 101,init_func=None,blit = False,repeat=True, repeat_delay=1000)
    
        #tell the slider to update the plot when it is moved
        self.slider.on_changed(self.update)
        #call the play function when the button is pressed (to play or pause the animation)
        self.play_button.on_clicked(self.play)
        
        #again this line updates the plot, without it, the animation starts by default even though we have anim.event_source.stop()
        #perhaps there is a better way of doing this that I am not aware of. 
        #plt.pause(0.1)
        #the line below also appears to do the trick, and better
        self.fig.canvas.draw()

        #these three lines are a bit of a hack to stop the simulation which for some reason automatically starts BUG
        self.animation_running = True
        self.play(event=None)
        self.slider.set_val(0.0)
        #fig.canvas.draw_idle()

    def plot_setup(self):
        fig = plt.figure(figsize=(8,8))
        ax = fig.add_subplot(111)
        fig.subplots_adjust(bottom=0.1, top=0.9)
        
        ax.set_xlim([-5, 5])
        ax.set_ylim([-5, 5])
        ax.set_box_aspect(1)
        ax.set_xlabel('East --> (m)')
        ax.set_ylabel('North --> (m)')
        ax.grid(color = 'k', linestyle = '-', linewidth = 0.5,which = "major")
        ax.grid(color = 'k', linestyle = ':', linewidth = 0.5,which = "minor")
        #ax.grid(True)
        ax.minorticks_on()
        return fig,ax
    
    def create_slider(self,fig):
        # Create axes for sliders
        #variable inside add_axes is left, bottom, width, height
        ax_prog = fig.add_axes([0.3, 0.92, 0.4, 0.05])
        ax_prog.spines['top'].set_visible(True)
        ax_prog.spines['right'].set_visible(True)

        # Create slider object to iterate through the plot
        slider = Slider(ax=ax_prog, label='Progress ',valinit=5.0, valstep=0.001, valmin=0, valmax=1.0, valfmt=' %1.1f ', facecolor='#cc7000')
        return slider
    
    def create_button(self, fig):
        # create the play button axis object
        play_ax = fig.add_axes([0.8, 0.92, 0.1, 0.05])
        #create a play button at the location of the axis object
        play_button = Button(play_ax, 'Play', color='0.8', hovercolor='0.95')
        return play_button
    
    def create_info_box(self, ax):
        # these are matplotlib.patch.Patch properties
        bounding_box = dict(boxstyle='round', facecolor='wheat',edgecolor = 'g', alpha=1)
        textstr = 'Safe'
        # place a text box in upper left in axes coords
        info_box = ax.text(0.05, 0.95, textstr, transform=ax.transAxes, fontsize=14,
                verticalalignment='top', bbox=bounding_box,color = 'g')
        return info_box
    
    def plot_buildings(self,ax,Arena,ArenaR):
        for building in Arena.buildings:
            #print(f"building vertices are {(building.vertices[:,0],building.vertices[0,0])}")
            ax.plot(     np.hstack((building.vertices[:,0],building.vertices[0,0]))  , np.hstack((building.vertices[:,1],building.vertices[0,1] )) ,'salmon', alpha=0.5 )
            ax.fill(     np.hstack((building.vertices[:,0],building.vertices[0,0]))  , np.hstack((building.vertices[:,1],building.vertices[0,1] )) ,'salmon', alpha=0.5 )
        for buildingR in ArenaR.buildings:
            ax.plot(     np.hstack((buildingR.vertices[:,0],buildingR.vertices[0,0]))  , np.hstack((buildingR.vertices[:,1],buildingR.vertices[0,1] )) ,'m' )
            ax.fill(     np.hstack((buildingR.vertices[:,0],buildingR.vertices[0,0]))  , np.hstack((buildingR.vertices[:,1],buildingR.vertices[0,1] )) ,'m' )
        return None
        
    def get_max_timesteps(self,Vehicle_list):
        #these few lines obtain the maximum length of any path for all the drones, aka the time taken for the last drone to reach its destination
        #maybe there's a neater way of doing this but it isn't computationally expensive so for now it's fine
        list_of_point_lengths = []
        for i in range(len(Vehicle_list)):
            n_points = len(Vehicle_list[i].path[:,0])
            list_of_point_lengths.append(n_points)
        #time_steps_max stores the maximum amount of timesteps for any drone to reach its destination. time_steps_max*dt = t_max
        return max(list_of_point_lengths)
    
    def initial_plot(self,Vehicle_list,ax):
        for _v in range(len(Vehicle_list)):
            #define the current coordinates of the drone
            x, y, z = Vehicle_list[_v].path[-1,0],Vehicle_list[_v].path[-1,1],Vehicle_list[_v].path[-1,2]
            self.positions[_v] = [x,y,z]
            #print(positions, type(positions))
            # Plot default data
            #note that the comma operator is mandatory, it turns the ax.plot object into a tuple
            f_d, = ax.plot(Vehicle_list[_v].path[:self.time_steps_max,0],Vehicle_list[_v].path[:self.time_steps_max,1], linewidth = 2)
            #the line below is for adding an icon to the current vehicle position
            #also the 'x' at the end is the position marker, can be changed to other things
            drone_icon, = ax.plot(x,y,'x')

            #plot the default warning circles
            # define the circle
            warning_circle = plt.Circle((x, y), 0.25, fill=False, edgecolor="r", linewidth=2)
            #add the circle to the plot
            ax.add_artist(warning_circle)

            self.warning_circles.append(warning_circle)

            self.plot_list.append(f_d)
            #append the references to the drone icons to the drone icon list
            self.drone_list.append(drone_icon)
            #the following two lines plot the start and end points
            ax.plot(Vehicle_list[_v].path[0,0],Vehicle_list[_v].path[0,1],'')
            ax.plot(Vehicle_list[_v].goal[0],Vehicle_list[_v].goal[1],'*')
        return None
    # Update values
    def update(self,val):
        # self.time_steps_max stores the maximum number of iterations it took for a drone to reach its destination. This is equivalent to time if ...
        # ...travelling at a constant speed. plot_until therefore is the current time value of the simulation.
        plot_until = int(np.floor(self.slider.val*self.time_steps_max))
        for i in range(len(self.plot_list)):
            # we are now telling the code to plot the drone trajectories only until the value of the slider (or time)
            self.plot_list[i].set_data(self.Vehicle_list[i].path[:plot_until,0],self.Vehicle_list[i].path[:plot_until,1])

            if plot_until == 0:
                # here is what we plot when the drones are at their starting positions
                # if t=0 we want to draw the drone icons at their starting points
                x,y,z = self.Vehicle_list[i].path[0,:3]
                self.warning_circles[i].set_edgecolor("b")
            elif plot_until < len(self.Vehicle_list[i].path[:,0]):
                #here is what we plot during drone transit
                # if t < the time it takes for this drone to reach its destination, show the drone icon at its current position
                x,y,z = self.Vehicle_list[i].path[plot_until-1,:3]
                self.warning_circles[i].set_fill(False)
                self.warning_circles[i].set_edgecolor("g")
            else:
                # here is where we plot what we want once the drone is at its destination
                # if t >= time taken for the drone to reach its destination, ensure the drone icon is shown at its destination
                # to avoid indexing errors
                x,y,z = self.Vehicle_list[i].path[-1,:3]
                # plt.Circle object has severable attributes that can be dynamically modified
                self.warning_circles[i].set_fill(True)
                self.warning_circles[i].set_facecolor("skyblue")
                self.warning_circles[i].set_edgecolor("b")
            self.drone_list[i].set_data(x,y)
            self.warning_circles[i].center = (x,y)
            self.positions[i] = [x,y,z]
        #now detect and handle collisions
        self.collision_handling()
        return self.plot_list + self.drone_list + self.warning_circles

    def collision_handling(self):
        '''Figure out if there is a collision and what to display when collisions are encountered'''
        distance_matrix = np.sqrt(np.sum((self.positions[:, np.newaxis] - self.positions) ** 2, axis=-1))
        # set collision threshold
        collision_threshold = 0.5
        for i in range(distance_matrix.shape[0]):
            for j in range(i+1, distance_matrix.shape[1]):
                if distance_matrix[i,j] < collision_threshold:
                    #print(f"Collision detected between drones {i} and {j}")
                    self.warning_circles[i].set_edgecolor("r")
                    self.warning_circles[j].set_edgecolor("r")
                    #conflicts.update([i,j])
                    if self.conflict_iterator in self.conflicts.keys():
                        self.conflicts[self.conflict_iterator].append((i,j))
                    else:
                        self.conflicts[self.conflict_iterator]=[(i,j)]
        #the line below checks if there is there are any conflicts at the current iteration                
        if self.conflict_iterator in self.conflicts.keys():
            # these are matplotlib.patch.Patch properties
            bounding_box = dict(boxstyle='round', facecolor='skyblue',edgecolor = 'r', alpha=1)
            #self.bounding_box['edgecolor'] = 'r'
            #self.bounding_box['facecolor'] = 'skyblue'
            textstr = f"Conflict detected!\nDrones {[drones for drones in self.conflicts[self.conflict_iterator]]}"
            # place a text box in upper left in axes coords
            #info_box = ax.text(0.05, 0.95, textstr, transform=ax.transAxes, fontsize=14,
            #        verticalalignment='top', bbox=bounding_box,color = 'g')
            self.info_box.set_text(textstr)
            self.info_box.set_c("k")
            self.info_box.set_bbox(bounding_box)
            #print(f"there is a conflict, slider value = {val}")

            for conflict in self.conflicts[self.conflict_iterator]:
                if int(self.conflict_iterator-1) in self.conflicts.keys():
                    #print(self.conflict_iterator,self.conflicts[self.conflict_iterator],self.conflicts[self.conflict_iterator-1])
                    if conflict not in self.conflicts[self.conflict_iterator-1]:
                        #print("Conflict already addressed, not stopping simulation.")
                        self.stop()
                else:
                    #print("No conflicts previously, should stop now")
                    self.stop()
        else:
            #no conflicts, set box to safe
            bounding_box = dict(boxstyle='round', facecolor='wheat',edgecolor = 'g', alpha=1)
            textstr = f"Safe"
            #info_box = ax.text(0.05, 0.95, textstr, transform=ax.transAxes, fontsize=14,
            #        verticalalignment='top', bbox=bounding_box,color = 'g')
            self.info_box.set_text(textstr)
            self.info_box.set_bbox(bounding_box)
            self.info_box.set_c("k")
        
        self.conflict_iterator+=1
        return None
    #anim.event_source.stop()
    def play(self,event):
        '''Function that is called every time the play button is pressed. It will alternate between play and pause and start/stop the animation'''
        #running is not a default attribute of the FuncAnimation object, but we have defined it ourselves lower down
        if self.animation_running:
            self.anim.event_source.stop()
            self.play_button.label.set_text("Play")
            self.animation_running = False
        else:
            self.anim.event_source.start()
            self.play_button.label.set_text("Pause")
            self.animation_running = True
        # this line seems to update the plot. Without it, the Play and Pause will not update until the mouse leaves the button area.
        #plt.pause(0.1)
        self.fig.canvas.draw_idle()
        return None

    #anim.event_source.stop()
    def stop(self):
        '''function that can be called in the code to stop the animation'''
        #running is not a default attribute of the FuncAnimation object, but we have defined it ourselves lower down
        #self.animation_running:
        self.anim.event_source.stop()
        self.play_button.label.set_text("Play")
        self.animation_running = False
        # else:
        #     self.anim.event_source.start()
        #     self.play_button.label.set_text("Pause")
        #     self.animation_running = True
        # this line seems to update the plot. Without it, the Play and Pause will not update until the mouse leaves the button area.
        #plt.pause(0.1)
        self.fig.canvas.draw_idle()
        return None

    def on_press(self,event):
        '''Allows the user to pause and play the animation with the spacebar'''
        sys.stdout.flush()
        if event.key == ' ':
            self.play(event=None)
            self.fig.canvas.draw()
        return None

    


    def animate(self,i):
        '''Function that updates the slider and calls the update function. i iterates from 0 to the number of frames'''
        #obtain the slider value to 2dp
        current_slider_value = round(self.slider.val,2)
        #set i to the slider value so that the simulation stops when the slider reaches the end
        #it is 100x the slider value because the slider goes from 0 to 1 and the i from 0 to 100
        i = int(100*current_slider_value)
        #print(f"i={i}")
        #increment the slider by 0.01 for every frame
        self.slider.set_val((current_slider_value + 0.01) % (self.slider.valmax+0.01))
        #stop the animation when the slider reaches the end
        if i==99:
            #calling the play function while the animation is running stops the animation
            self.play(event=None)
        #nothing to return I'm pretty sure :)
        return None

    def show(self):
        #show the plot
        plt.show()
        return None


if __name__ == '__main__':
    main()
