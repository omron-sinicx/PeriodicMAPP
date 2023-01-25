#!/usr/bin/env python3

# Copyright (c) 2023 OMRON SINIC X Corporation
# Author: Kazumi Kasaura

import sys
import copy
with open(sys.argv[1], "r") as problem_file:
    number_of_vertices, number_of_edges, number_of_agents = map(int,problem_file.readline().split())
    vertices=[]
    for i in range(number_of_vertices):
        x,y=map(float, problem_file.readline().split())
        vertices.append([x,y])
    if len(sys.argv)>2:
        D=int(sys.argv[2])
    else:
        D=10
    added= number_of_agents*(D+1)
    for p in range(D):
        d=(p+1)*(1.+1e-8)
        for i in range(number_of_agents):
            initial = copy.copy(vertices[number_of_vertices-2*number_of_agents+2*i])
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
            vertices.append(initial)
    d=D
    for i in range(number_of_agents):
        goal = copy.copy(vertices[number_of_vertices-2*number_of_agents+2*i+1])
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
        vertices.append(goal)
    edges=[]
    for i in range(number_of_edges):
        start,target=map(int, problem_file.readline().split())
        edges.append([start,target])
    for i in range(number_of_agents):
        edges.append([number_of_vertices+i, number_of_vertices-2*number_of_agents+2*i])
    for p in range(1,D):
        for i in range(number_of_agents):
            edges.append([number_of_vertices+p*number_of_agents+i, number_of_vertices+(p-1)*number_of_agents+i])
    for i in range(number_of_agents):
        edges.append([number_of_vertices-2*number_of_agents+2*i+1, number_of_vertices+D*number_of_agents+i])
    tasks = []
    for i in range(number_of_agents):
        initial, goal = map(int,problem_file.readline().split())
        tasks.append([ number_of_vertices+(D-1)*number_of_agents+i, number_of_vertices+D*number_of_agents+i])
    agent_size = float(problem_file.readline())

print(len(vertices), len(edges), number_of_agents)
for vertex in vertices:
    print(vertex[0], vertex[1])
for edge in edges:
    print(edge[0], edge[1])
for task in tasks:
    print(task[0], task[1])
print(agent_size)
