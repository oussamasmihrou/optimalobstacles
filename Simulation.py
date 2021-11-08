from Obstacle import Obstacle
from Exit import Exit
from GridGraph import GridGraph
from FastMarching import fast_marching_method
from Agent import Agent
import random as rd
import numpy as numpy
from datetime import datetime
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib import animation
from pylab import show,imshow
import os
from os import startfile

def example_simulation(obstacles):
    obstacles = obstacles
    name = ('GA - Output')
    time_simulation = 90
    exits = [Exit((99,40),2,10)]
    #agents = Square_agents((20,20),10,10,6) # 100 pedestrian population
    agents = Square_agents((20,20),10,3,6) # 30 pedestrian population
    
    size_scene =(100,100)
    dt = .2 # time steps in seconds, 400 time steps on total
    evacuation_time = launch_simulation(obstacles,exits,agents,time_simulation,dt,name,size_scene)
    return evacuation_time
def launch_simulation(obstacles,exits,agents,time_simulation,dt,name,size_scene):
    history_agents = []
    ##print ("Initialization of the agents...")
    # nodes per unity
    Start_time = datetime
    precision = 0.5
    navigation_maps = {}
    for agent in agents:
        if agent.size not in navigation_maps:
            debug_precision = precision
            #to make sure that precision*agent.size is an integer
            if not precision*agent.size == int(precision*agent.size):
                debug_precision = 1
            graph = GridGraph(size_scene,debug_precision)
            graph.prepare_graph_for_fast_marching(obstacles,exits,agent)
            fast_marching_method(graph, graph.to_node(agent.position))
            navigation_maps[agent.size] = graph
            #shows the distance map to the exit after applying the fast marching method
            imshow(graph.distances,interpolation='nearest',origin='lower')
            ##show()
    for agent in agents:
        agent.navigation_map = navigation_maps[agent.size]
    loading_bar = 0
    ##print ("Simulation running...")
    #start simulation
    for t in range(int(time_simulation/dt)):
        
        #initialize patches for the agents for one frame of the animation
        patches_agents = []
        
        numpy.random.seed(1)
        pick_agent = numpy.random.choice(len(agents),len(agents),replace=False)
        for i in pick_agent:
            #update the speed of the agent
            agents[i].update_speed(agents,obstacles)
            #update the position of the agent
            agents[i].update_position(agents,obstacles,exits,dt,size_scene)
            #add patch of the agent for one frame
            patches_agents.append(patches.Circle(agents[i].position,agents[i].size,color=[agents[i].get_color_agent(),0,0]))
             
        
        history_agents.append(patches_agents)
        
        agents = [v for i, v in enumerate(agents) if not v.has_reached_exit]
        #print(update_time_move_agents(time_move,agents,dt))
        if len(agents) == 0 :
            break;
    return (t+1)*dt 

    #display_simulation(history_agents, obstacles, exits, size_scene,dt,3,name)
def display_simulation(history_agents,obstacles, exits, size_scene,dt,speed,name):
    fig = plt.figure()
    ax = fig.add_subplot(111, aspect='equal')
    ax.set_xlim([0,size_scene[0]])
    ax.set_ylim([0,size_scene[1]])

    for obstacle in obstacles:
        ax.add_patch(patches.Rectangle((obstacle.position[0],obstacle.position[1]),obstacle.width,obstacle.height))

    for exit_ in exits:
        ax.add_patch(patches.Rectangle((exit_.position[0],exit_.position[1]),exit_.width,exit_.height,color='black'))
    
    def init():
        for i in range(len(history_agents)):
            for j in range(len(history_agents[i])):
                history_agents[i][j].set_visible(False)
        return []

    def animate(i):
        patches = []
        if i>0:
            for j in range(len(history_agents[i-1])):
                history_agents[i-1][j].set_visible(False)
        for j in range(len(history_agents[i])):
            history_agents[i][j].set_visible(True)
            patches.append(ax.add_patch(history_agents[i][j]))
        return patches
    
    
    #interval doesn't work
    anim = animation.FuncAnimation(fig, animate, init_func=init,frames=len(history_agents), interval=dt*1000/float(speed))
    
    def save_simulation(namee):
        print ("Saving the simulation...")
        
        # Set up formatting for the movie files
        Writer = animation.writers['ffmpeg']
        writer = Writer(fps=1/float(dt)*speed, metadata=dict(artist='Me'), bitrate=1800)
        filename = os.path.abspath('')+f"\Videos\{namee}.mp4"
        anim.save(filename, writer)
        startfile(filename)
    
    save_simulation(name)
    
    ##print ('Simulation saved under the name:',name)
def Square_agents(leader_coordinates,Nb_columns,Nb_lines,distancing): # the distancing is the distances between two Agents,must be greater than 2*size 
    List_agents = []
    for i in range(Nb_lines):
        for j in range(Nb_columns):
            Mass = rd.choices([1,2], weights=[1,3], cum_weights=None, k=1)[0] 
            List_agents.append(Agent((leader_coordinates[0]+i*distancing, leader_coordinates[1]+j*distancing),Mass)) 
    return List_agents

if __name__ == 'main':
    print('importing succeded')