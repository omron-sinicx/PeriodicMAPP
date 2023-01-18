import sys, copy
import math
with open(sys.argv[1], "r") as plan_file:
    plan=[]
    line = plan_file.readline().split()
    items=0
    #number_of_agents=int(line[0])
    #items+=1
    cycle = int(line[items])
    agent_size = float(line[items+1])
    if agent_size == 0.0:
        agent_size=0.1
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
        time_0 = 0.0
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

waiting_place = [[] for i in range(number_of_groups)]
last_place = []
D=int(sys.argv[3])
for p in range(D):
    d=p+1
    for i in range(number_of_groups):
        initial = copy.copy(routes[i*cycle][0][1])
        if initial[0] + initial[1] > 0:
            if initial[0] > initial[1]:
                initial[0]+=d
            else:
                initial[1]+=d
        else:
            if initial[0] > initial[1]:
                initial[1]-=d
            else:
                initial[0]-=d
        
        waiting_place[i].append(initial)
d = D
for i in range(number_of_groups):
    goal = copy.copy(routes[i*cycle][-1][1])
    if goal[0] +goal[1] > 0:
        if goal[0] > goal[1]:
            goal[0]+=d
        else:
            goal[1]+=d
    else:
        if goal[0] > goal[1]:
            goal[1]-=d
        else:
            goal[0]-=d
    last_place.append(goal)

number_of_waiting = [0 for i in range(number_of_groups)]
times = [0 for i in range(number_of_groups)]

with open(sys.argv[2], "r") as dist_file:
    number_of_arrivals=int(dist_file.readline())
    print(number_of_arrivals, agent_size)
    for counter in range(number_of_arrivals):
        line = dist_file.readline().split()
        group = int(line[1])
        t = float(line[0])
        while number_of_waiting[group] > 0 and (times[group]-number_of_waiting[group]+1)*period -1<= t + (D-number_of_waiting[group]-1):
            number_of_waiting[group]-=1
        if number_of_waiting[group]>=D:
            print('0')
            continue
        if number_of_waiting[group]>0:
            number_of_waiting[group]+=1
            times[group]+=1
        else:
            number_of_waiting[group]=1
            times[group]=math.ceil((t+D)/period)
        #print(group, t, number_of_waiting[group], times[group], file=sys.stderr)
        order = times[group] % cycle
        agent = group * cycle + order
        w = number_of_waiting[group]
        print(len(routes[agent])+2+2*w)
        print(t, '\n', waiting_place[group][D-1][0], waiting_place[group][D-1][1])
        print(t+(D-w), '\n', waiting_place[group][w-1][0], waiting_place[group][w-1][1])
        while w>0:
            w-=1
            print((times[group]-w)*period-1, '\n', waiting_place[group][w][0], waiting_place[group][w][1])
            if w>0:
                print((times[group]-w)*period, '\n', waiting_place[group][w-1][0], waiting_place[group][w-1][1])
        for point in routes[agent]:
            print(times[group] * period + point[0], '\n', point[1][0], point[1][1])
        print(times[group] * period + routes[agent][-1][0] + D, '\n', last_place[group][0], last_place[group][1])
