densities=(0.25, 0.5, 0.75, 1.0, 1.25, 1.5, 1.75, 2.0, 2.25, 2.5)
planners =("FCFS", "M=1", "M=2", "M=3")
envs=("1x1", "2x1", "2x2","2x2large", "6small", "6")
env_names = ('(a)','(b)', '(c)', '(d)', '(e)', '(f)')
N = len(densities)

import matplotlib.pyplot as plt
import re
import statistics

plt.rcParams["font.size"] = 12

fig = plt.figure(figsize = (7, 4))

tpss = []
costss = []

colors = ('tab:orange', 'tab:cyan', 'tab:blue', 'tab:purple')
lws = [(1, 1, 2, 1), (1, 1, 1, 2), (1, 1, 1, 2), (1, 1, 1, 2), (1, 2, 1, 1), (1, 1, 2, 1)]

iteration = 10

for env_id, env in enumerate(envs):

    tps = []
    costs = []
    with open(env+ '/evaluation.txt', "r") as result_file:
        result_file.readline()
        res = list(result_file.readline().split())
        for item in res:
            tps.append(float(re.sub(r"[^0-9.]", '', item)))
        res = list(result_file.readline().split())
        for item in res:
            costs.append(float(re.sub(r"[^0-9.]", '', item)))

    print(len(tps), len(costs))
    tpss.append(tps)
    costss.append(costs)
    ax = fig.add_subplot(2, 3, env_id+1)

    for index, name in enumerate(planners):
        means = []
        devs = []
        for i in range(N):
            data = tps[(N*index+i)*iteration:(N*index+i+1)*iteration]
            means.append(statistics.mean(data))
            devs.append(statistics.stdev(data))
        ax.errorbar(densities, means, devs, label = name, color =colors[index], lw = lws[env_id][index])
    ax.set_title(env_names[env_id])

plt.legend(bbox_to_anchor=(-1.0, -0.6), loc="lower center", ncol = 4)
fig.subplots_adjust(left=0.09, right=0.97, bottom=0.18, top=0.92, hspace = 0.4, wspace = 0.3)
plt.savefig('thoughrate.pdf')
plt.show()

fig = plt.figure(figsize = (7, 4))

for env_id, env in enumerate(envs):
    ax = fig.add_subplot(2, 3, env_id+1)

    for index, name in enumerate(planners):
        means = []
        devs = []
        for i in range(N):
            data = costss[env_id][(N*index+i)*iteration:(N*index+i+1)*iteration]
            means.append(statistics.mean(data))
            devs.append(statistics.stdev(data))
        ax.errorbar(densities, means, devs, label = name, color =colors[index], lw = lws[env_id][index])

    ax.set_title(env_names[env_id])
    plt.ylim([0.0, 30.0])

plt.legend(bbox_to_anchor=(-1.0, -0.6), loc="lower center", ncol = 4)
fig.subplots_adjust(left=0.09, right=0.97, bottom=0.18, top=0.92, hspace = 0.4, wspace = 0.3)
plt.savefig('delay.pdf')
plt.show()
