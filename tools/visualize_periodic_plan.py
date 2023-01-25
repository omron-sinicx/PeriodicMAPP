#!/usr/bin/env python3

# Copyright (c) 2023 OMRON SINIC X Corporation
# Author: Kazumi Kasaura

import matplotlib.pyplot as plt 
import matplotlib.patches as patches
import sys
import matplotlib.animation as animation
import math
import numpy as np
import matplotlib.path

time_span = float(sys.argv[2])

fig, ax = plt.subplots(figsize = (7,7))
if len(sys.argv) > 3 and sys.argv[3] != "-":
    with open(sys.argv[3], "r") as map_file:
        n_holes = int(map_file.readline())
        n_vertices = int(map_file.readline())
        vertices = []
      
        for i in range(n_vertices):
            x, y = map(float, map_file.readline().split())
            vertices.append([x,y])
        xlim = [min([p[0] for p in vertices]),max([p[0] for p in vertices])]
        ylim = [min([p[1] for p in vertices]),max([p[1] for p in vertices])]
        plt.xlim(xlim[0], xlim[1])
        plt.ylim(ylim[0], ylim[1])
        ax.add_patch(patches.Rectangle([xlim[0],ylim[0]],xlim[1]-xlim[0],ylim[1]-ylim[0], color = 'k'))
        ax.add_patch(patches.Polygon(vertices, facecolor = 'w', edgecolor = 'k'))
        for j in range(n_holes):
            n_vertices = int(map_file.readline())
            vertices = []
            for i in range(n_vertices):
                x, y = map(float, map_file.readline().split())
                vertices.append([x,y])
            ax.add_patch(patches.Polygon(vertices, color = 'k'))

if len(sys.argv) > 4 and sys.argv[4] != "-":
    with open(sys.argv[4], "r") as roadmap_file:
        n, m, k = map(int, roadmap_file.readline().split())
        
        points = []
        for i in range(n):
            point = list(map(float, roadmap_file.readline().split()))
            points.append(point)
  
        plt.scatter([p[0] for p in points], [p[1] for p in points], s = 1.0)
  
        for i in range(m):
            u, v = list(map(int, roadmap_file.readline().split()))
            path = matplotlib.path.Path([points[u], points[v]])
            ax.add_patch(patches.PathPatch(path, linewidth = 0.1, fill = False))
  
        tasks = []
  
        for i in range(k):
            tasks.append(list(map(int, roadmap_file.readline().split())))
  
        agent_size = float(roadmap_file.readline())
     
        colors = plt.cm.get_cmap('hsv', k)
        for i in range(k):
            s,t = tasks[i]
            #ax.add_patch(patches.Circle(points[s], radius=agent_size, color = colors(i)))
            #ax.add_patch(patches.Circle(points[t], fill = False, radius=agent_size, color = colors(i)))

        xlim = [min([point[0] for point in points]), max([point[0] for point in points])]
        ylim = [min([point[1] for point in points]), max([point[1] for point in points])]
            
    ax.invert_yaxis()
    ax.axis('off')
    fig.subplots_adjust(left=0, right=1, bottom=0, top=1)
    ax.set_xticklabels([])
    ax.set_yticklabels([])
    r = 0.3
    fig.set_figwidth(r * (xlim[1]-xlim[0]))
    fig.set_figheight(r * (ylim[1]-ylim[0]))
        
with open(sys.argv[1], "r") as plan_file:
    plan=[]
    line = plan_file.readline().split()
    items=0
    #number_of_agents=int(line[0])
    #items+=1
    cycle = int(line[items])
    agent_size = float(line[items+1])
    if agent_size == 0.0:
        agent_size=0.05
    period = float(line[items+2])
    number_of_groups = int(line[items+3])
    number_of_agents = number_of_groups * cycle
    #start_times = [0.] + list(map(float, plan_file.readline().split()))

    makespan = 0.0
    routes = []
    
    for i in range(number_of_agents):
        route = []
        line = plan_file.readline().split()
        route_length = int(line[0])
        #delay = int(line[1])
        #group = int(line[2])
        #time_0 = delay * period + start_times[group]
        time_0 = (i % cycle) * period
        raw_route = []
        for j in range(route_length):
            new_line = list(map(float, plan_file.readline().split()))
            raw_route.append(new_line)
        span = raw_route[-1][2] - raw_route[0][2]
        for new_line in raw_route:
            route.append([time_0 + (new_line[2]-raw_route[0][2]), new_line[:2]])
            #route.append([time_0 + (new_line[2]-raw_route[0][2])/span, new_line[:2]])
        makespan = max(makespan, route[len(route)-1][0])
        routes.append(route)
        # print(route)

    xlim = [min([min([t[1][0] for t in route]) for route in routes]),
            max([max([t[1][0] for t in route]) for route in routes])]
    ylim = [min([min([t[1][1] for t in route]) for route in routes]),
            max([max([t[1][1] for t in route]) for route in routes])]
    plt.xlim(xlim[0]-agent_size, xlim[1]+agent_size)
    plt.ylim(ylim[0]-agent_size, ylim[1]+agent_size)
    r = 1.0
    fig.set_figwidth(r * (xlim[1]-xlim[0] + 2*agent_size))
    fig.set_figheight(r * (ylim[1]-ylim[0] + 2* agent_size))

    period = cycle * period

    makespan = math.ceil(makespan/period)*period
    agent_circles = []

    colors = plt.cm.get_cmap('jet', number_of_agents)
    # colors = ['r', 'g', 'b']

    
    def init():
        global steps
        global routes
        

        #init_patches = []
        
        #for agent in range(number_of_agents):
            #position = routes[agent][0][1]

            #init_patches.append(ax.add_patch(patches.Circle(position, radius = agent_size,fill = False, linewidth = 0.3, linestyle = ':', color = colors(agent), alpha = 0.5)))
            #position = routes[agent][len(routes[agent])-1][1]

            #init_patches.append(ax.add_patch(patches.Circle(position, radius = agent_size,fill = False, linewidth = 0.3, linestyle = '-', color = colors(agent), alpha = 0.5)))

            #path = matplotlib.path.Path([vertex[1] for vertex in routes[agent]])
            #init_patches.append(ax.add_patch(patches.PathPatch(path, linewidth = 0.3, color = colors(agent), fill = False)))

            
        return []
    
    agent_circles = []
    
    def plot(frame):
        global routes
        global agent_circles
        for circle in agent_circles:
            circle.remove()

        t = time_span * frame

        agent_circles = []
        for agent in range(number_of_agents):
            for p in range(math.ceil((routes[agent][0][0]-t)/period), math.floor((routes[agent][-1][0]-t)/period)+1):
                st = t + p * period
                step = 0
                while step < len(routes[agent]) and routes[agent][step][0] <= st:
                    step+=1
                if 0<step and step < len(routes[agent]):
                    r = (st - routes[agent][step-1][0]) / (routes[agent][step][0] - routes[agent][step-1][0])
                    position = (1-r) * np.array(routes[agent][step-1][1]) + r * np.array(routes[agent][step][1])

                    agent_circles.append(ax.add_patch(patches.Circle(position, radius = agent_size, color = colors(agent))))
        return agent_circles
    
    ani = animation.FuncAnimation(fig, plot, frames = range(math.ceil(makespan / time_span)), interval=100, init_func = init, blit = True)
    plt.show()
    if len(sys.argv) > 5:
        ani.save(sys.argv[5], writer='ffmpeg')
        #ani.save(sys.argv[5], writer='imagemagick')
