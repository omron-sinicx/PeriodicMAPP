densities=(0.25, 0.5, 0.75, 1.0, 1.25, 1.5, 1.75, 2.0, 2.25, 2.5)
#densities=(2.0,)
iteration=10

planners =("PSIPP", "Periodic-1", "Periodic-2", "Periodic-3")
#planners =("CCBS",)

suc_ratess = []
tpss = []
costss = []

D = 1000

import math

for planner in planners:
    suc_rates = []
    costs = []
    tps = []
    for density in densities:
        for it in range(iteration):
            sum_n = 0
            success = 0.
            sum_cost = 0.
            makespan = 0.
            with open("results/"+planner+"-"+str(density)+ "-" + str(it) + ".txt", "r") as plan_file:
                n = int(plan_file.readline().split()[0])
                sum_n+=n
                for t in range(n):
                    route = []
                    route_length = int(plan_file.readline())
                    for j in range(route_length):
                        if 0 < j and j < route_length-100:
                            plan_file.readline()
                            plan_file.readline()
                            continue
                        time = float(plan_file.readline())
                        point = list(map(float, plan_file.readline().split()))
                        route.append([time, point])
                    if route_length > 1:
                        success+=1
                        sum_cost+=route[-2][0]-route[0][0]-1000.
                        if route[-2][0] < 1000:
                            print("results/"+planner+"-"+str(density)+ "-" + str(it) + ".txt", t)
                        dist_x = route[-2][1][0]-route[0][1][0]
                        dist_y = route[-2][1][1]-route[0][1][1]
                        if dist_x <= -D:
                            dist_x+=D
                        elif dist_x >= D:
                            dist_x-=D
                        elif dist_y <= -D:
                            dist_y+=D
                        elif dist_y >= D:
                            dist_y-=D
                        sum_cost -= math.sqrt(dist_x*dist_x+dist_y*dist_y)
                        m = -2;
                        while abs(route[-2][1][0]-dist_x - route[m][1][0]) < 1e-6 and abs(route[-2][1][1]-dist_y - route[m][1][1]) < 1e-6:
                            m-=1
                        makespan = max(makespan, route[m][0]-1000.)
                        
                        
            suc_rates.append(success/sum_n)
            tps.append(success/makespan)
            costs.append(sum_cost/success)
    suc_ratess.append(suc_rates)
    tpss.append(tps)
    costss.append(costs)
print(suc_ratess)
print(tpss)
print(costss)
#exit(0)
#import matplotlib.pyplot as plt
#fig = plt.figure()
#for i in range(len(planners)):
#    plt.plot(densities, tpss[i], label=planners[i])
#plt.legend()
#plt.show()
#fig = plt.figure()
#for i in range(len(planners)):
#    plt.plot(densities, costss[i], label=planners[i])
#plt.legend()
#plt.show()
