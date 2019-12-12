# -*- coding: utf-8 -*-
"""
Created on Mon Nov 11 04:49:28 2019

@author: Dell
"""

import numpy as np
from collections import deque

graph = 0.05 * np.eye(7)
graph[0, 1] = 3
graph[0, 2] = 2
graph[1, 3] = 1
graph[2, 3] = 3
graph[3, 4] = 3
graph[3, 5] = 1
graph[4, 6] = 2

heuristic = [0., 3., 2., 3.5, 2., 3., 4.]

def astar(graph, start, goal, heuristic):
    """Plan a path from start to goal using Dijkstra's algorithm.

    Args:
        graph (ndarray): An adjacency matrix, size (n,n)
        start (int): The index of the start node
        goal (int): The index of the goal node

    Returns:
        deque: Indices of nodes along the shortest path
    """
    # TODO
    # record the last vertex to get shortest path
    pathmark = np.zeros(len(graph[0]))  # record the last node to get shortest path (path[i]=j)
    low_cost = np.zeros(len(graph[0]))
    finalmark = np.zeros(len(graph[0]))  # Initialise array that'll record if shortest path was found (1 yes)
    shortest_dis = np.zeros(len(graph[0]))  # Initialise array containing the relatively shortest distance (start-other node)
    path = deque()  # Initialise an empty array record shortest distance from start node to others

    for i in range(len(graph[0])):
        # pathmark[i] = 0
        # finalmark[i] = 0
        low_cost[i] = np.Inf
        if graph[start][i] != 0:  # They are connected
            shortest_dis[i] = graph[start][i]  # Weights used to store the shortest path to each point

        else:
            shortest_dis[i] = np.Inf  # If not connected then value it'll be infinity

    #finalmark[start] = 1  # No need to find the shortest path from V0 to V0
    low_cost[start] = heuristic[start]
    shortest_dis[start] = 0

    i = 0
    for i in range(len(graph[0])):
        minmun = np.Inf
        minindex = start

        for j in range(len(graph[0])):  # Will find the smallest cost from the neighbours
            if finalmark[j] == 0 and low_cost[j] < minmun:
                minmun = low_cost[j]
                minindex = j

        u = minindex
        finalmark[u] = 1  # Description found
        for v in range(len(graph[0])):  # It will update the value for the shortest distance if a shorter was found.
            dis = 0
            if graph[u][v] == 0:
                dis = np.Inf
            else:
                dis = graph[u][v]
            if finalmark[v] == 0 and shortest_dis[v] >= shortest_dis[u] + dis:
                shortest_dis[v] = shortest_dis[u] + dis
                low_cost[v] = shortest_dis[v] + heuristic[v]
                pathmark[v] = u

    #-----------------------------------------------------
    """
    dead = np.zeros(len(graph[0]))
    active = np.zeros(len(graph[0]))
    cost_to = np.zeros(len(graph[0]))
    low_cost = np.zeros(len(graph[0]))
    pathmark = np.zeros(len(graph[0]))

    for i in range(len(graph[0])):
        dead[i] = 0
        active[i] = 0
        pathmark[i] = 0
        low_cost[i] = np.Inf
        if graph[start][i] != 0:
            cost_to[i] = graph[start][i]
        else:
            cost_to[i] = np.Inf

    active[start] = 1
    low_cost[start] = heuristic[start]
    i = 0
    for i in range(len(graph[0])):
        minmun = np.Inf
        minindex = -1

        for j in range(len(graph[0])):
            if active[j] == 1 and low_cost[j] < minmun:
                minmun = low_cost[j]
                minindex = j

        u = minindex
        succ = []
        for v in range(len(graph[0])):
            if graph[u][v] != 0 and u != v:
                succ.append(v)

        for n in succ:
            if active[n] != 1 and dead[n] != 2:
                active[n] = 1
                cost_to[n] = cost_to[u] + graph[u][n]
                low_cost[n] = cost_to[n] + heuristic[n]
            elif active[n] == 1:
                if cost_to[n] > cost_to[u] + graph[u][n]:
                    cost_to[n] = cost_to[u] + graph[u][n]
                    low_cost[n] = cost_to[n] + heuristic[n]
            pathmark[n] = u

        active[u] = 0
        dead[u] = 2

        if u == goal:
            break
    """

    currentnode = goal
    path = deque()
    while currentnode != start:
        try:
            currentnode = int(currentnode)
            path.appendleft(currentnode)
            currentnode = pathmark[currentnode]
        except KeyError:
            print("Path is not reachable")
            break
    path.appendleft(start)

    return path


des = astar(graph, 0, 6, heuristic)
print(des)