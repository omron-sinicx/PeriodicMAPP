import matplotlib
matplotlib.rcParams['pdf.fonttype'] = 42
matplotlib.rcParams['ps.fonttype'] = 42

import matplotlib.pyplot as plt
plt.rcParams["font.size"] =30

import matplotlib.pyplot as plt 
import matplotlib.patches as patches
import sys
import math
import numpy as np
import matplotlib.path

fig, ax = plt.subplots()
if len(sys.argv) > 2 and sys.argv[2] != "-":
    with open(sys.argv[2], "r") as map_file:
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
        ax.set_facecolor('k')
        ax.add_patch(patches.Rectangle([xlim[0],ylim[0]],xlim[1]-xlim[0],ylim[1]-ylim[0], color = 'k'))
        ax.add_patch(patches.Polygon(vertices, facecolor = 'w', edgecolor = 'k'))
        for j in range(n_holes):
            n_vertices = int(map_file.readline())
            vertices = []
            for i in range(n_vertices):
                x, y = map(float, map_file.readline().split())
                vertices.append([x,y])
            ax.add_patch(patches.Polygon(vertices, color = 'k'))
        if len(sys.argv) > 3:
            ax.set_title('('+sys.argv[3]+')')
        XL=xlim[1]-xlim[0]
        YL=ylim[1]-ylim[0]
        LS=1.0
        RS=0.50
        BS=0.75
        TS=0.75
        fig.subplots_adjust(left=LS/(LS+RS+XL), right=1.0-RS/(LS+RS+XL), bottom=BS/(BS+TS+YL), top=1.0-TS/(BS+TS+YL))
        r = 0.7
        fig.set_figwidth(r * (xlim[1]-xlim[0]+LS+RS))
        fig.set_figheight(r * (ylim[1]-ylim[0]+TS+BS))

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
    #plt.xlim(xlim[0]-agent_size-0.1, xlim[1]+agent_size+0.1)
    #plt.ylim(ylim[0]-agent_size-0.1, ylim[1]+agent_size+0.1)
    #r = 1.0
    #fig.set_figwidth(r * (xlim[1]-xlim[0] + 2*agent_size+0.2))
    #fig.set_figheight(r * (ylim[1]-ylim[0] + 2* agent_size+0.2))

    period = cycle * period

    #makespan = math.ceil(makespan/period)*period


#if cycle == 1:
#    colors = ('red', 'blue')
#    max_order=5
#    diff=0.5
#elif cycle == 2:
#    colors = ('red', 'orange', 'blue', 'cyan')
#    max_order=6
#    diff=0.5
#else:
#    colors = ('red', 'orange', 'yellow', 'blue', 'cyan', 'purple')
#    max_order=6
#    diff=0.5
colors = []
color_gen = plt.cm.get_cmap('jet', number_of_agents)
for i in range(number_of_agents):
    colors.append(color_gen(i))
diff=0.5

from matplotlib.path import Path
from matplotlib.patches import PathPatch
from matplotlib.markers import MarkerStyle

#ax.axis('off')
#fig.subplots_adjust(left=0.00, right=1.0, bottom=0.00, top=1.00)

if len(sys.argv) > 4:
    for i in range(number_of_groups):      
        s = routes[i*cycle][0][1]
        t = routes[i*cycle][-1][1]
        ax.add_patch(patches.Circle(s, fill=False, radius=agent_size, color = colors[i*cycle], alpha=1.0))
        plt.text(s[0]-0.5*agent_size, s[1]-0.5*agent_size, 's'+str(i+1), fontsize = 30, color = colors[i*cycle], alpha=1.0)
        ax.add_patch(patches.Circle(t, fill = False, radius=agent_size, color = colors[i*cycle], alpha=1.0))
        plt.text(t[0]-0.5*agent_size, t[1]-0.5*agent_size, 'g'+str(i+1), fontsize = 30, color = colors[i*cycle], alpha=1.0)
    plt.show()
    exit(0)
    
for agent in range(number_of_agents):
    path = Path([point[1] for point in routes[agent]])
    ax.add_patch(PathPatch(path, color = colors[agent], fill=False))
    for i in range(10):
        t = 10*i+5
        p=routes[agent][t][1]
        q=routes[agent][t+1][1]
        m=MarkerStyle('>')
        m._transform.rotate(math.atan2(q[1]-p[1],q[0]-p[0]))
        ax.scatter([p[0]],[p[1]],marker=m, color=colors[agent])
    step=0
    i=0
    while True:
        st=period*(i+diff/cycle)
        while step < len(routes[agent]) and routes[agent][step][0] <= st:
            step+=1
        if step == len(routes[agent]):
            break
        if 0<step and step < len(routes[agent]):
            r = (st - routes[agent][step-1][0]) / (routes[agent][step][0] - routes[agent][step-1][0])
            position = (1-r) * np.array(routes[agent][step-1][1]) + r * np.array(routes[agent][step][1])

            ax.add_patch(patches.Circle(position, alpha=0.3, radius = agent_size, color = colors[agent]))
            # plt.text(position[0]-0.2, position[1]-0.2, str(i*cycle-agent%cycle+1), fontsize = 40, color='k', alpha=0.5)
        i+=1
plt.savefig('plan_figures/'+sys.argv[1].split('/')[-2]+'_plan.pdf')
plt.show()
