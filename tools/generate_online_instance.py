#!/usr/bin/env python3

# Copyright (c) 2023 OMRON SINIC X Corporation
# Author: Kazumi Kasaura

import sys
with open(sys.argv[1], "r") as problem_file:
    number_of_vertices, number_of_edges, number_of_agents = map(int,problem_file.readline().split())
    vertices=[]
    for i in range(number_of_vertices):
        x,y=map(float, problem_file.readline().split())
        vertices.append([x,y])
    edges=[]
    for i in range(number_of_edges):
        start,target=map(int, problem_file.readline().split())
        edges.append([start,target])
    tasks = []
    for i in range(number_of_agents):
        initial, goal = map(int,problem_file.readline().split())
        tasks.append([initial, goal])
    agent_size = float(problem_file.readline())
    lines = []
    while True:
        line = problem_file.readline()
        if line == "":
            break;
        lines.append(line)

with open(sys.argv[2], "r") as timing_file:
    number_of_arrive = int(timing_file.readline())
    agents = []
    for i in range(number_of_arrive):
        line = timing_file.readline().split()
        agent =int(line[1])
        timing = float(line[0])
        agents.append([tasks[agent][0], tasks[agent][1], timing])

print(len(vertices), len(edges), number_of_arrive)
for vertex in vertices:
    print(vertex[0], vertex[1])
for edge in edges:
    print(edge[0], edge[1])
for agent in agents:
    print(agent[0], agent[1], agent[2])
print(agent_size)
for line in lines:
    print(line)
