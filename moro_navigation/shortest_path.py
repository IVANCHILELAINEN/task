#!/usr/bin/env python
import numpy as np
from collections import deque
import probabilistic_roadmap
from collections import defaultdict
# Values for node status
VIRGIN = 0
ACTIVE = 1
DEAD = 2


def dijkstra(graph, start, goal):
    """Plan a path from start to goal using Dijkstra's algorithm.

    Args:
        graph (ndarray): An adjacency matrix, size (n,n)
        start (int): The index of the start node
        goal (int): The index of the goal node

    Returns:
        deque: Indices of nodes along the shortest path
    """
    """Plan a path from start to goal using Dijkstra's algorithm.

    Args:
        graph (ndarray): An adjacency matrix, size (n,n)
        start (int): The index of the start node
        goal (int): The index of the goal node

    Returns:
        deque: Indices of nodes along the shortest path
    """
    #TODO

    pathmark = np.zeros(len(graph[0]))  # record the last node to get shortest path (path[i]=j)

    finalmark = np.zeros(len(graph[0]))  # Initialise array that'll record if shortest path was found (1 yes)
    shortest_dis = np.zeros(len(graph[0]))  # Initialise array containing the relatively shortest distance (start-other node)
    path = deque()  #Initialise an empty array record shortest distance from start node to others

    for i in range(len(graph[0])):
        #pathmark[i] = 0
        #finalmark[i] = 0
        if graph[start][i] != 0:  # They are connected
            shortest_dis[i] = graph[start][i]  # Weights used to store the shortest path to each point

        else:
            shortest_dis[i] = np.Inf # If not connected then value it'll be infinity

    finalmark[start] = 1  # No need to find the shortest path from V0 to V0
    # shortest_dis[start] = 0

    i = 0
    for i in range(len(graph[0])):
        minmun = np.Inf
        minindex = -1

        for j in range(len(graph[0])):   # Will find the smallest cost from the neighbours
            if finalmark[j] == 0 and shortest_dis[j] < minmun:
                minmun = shortest_dis[j]
                minindex = j

        u = minindex
        finalmark[u] = 1  # Description found
        for v in range(len(graph[0])):  # It will update the value for the shortest distance if a shorter was found.
            dis = 0
            if graph[u][v] == 0:
                dis = np.Inf
            else:
                dis = graph[u][v]
            if finalmark[v] == 0 and shortest_dis[v] > shortest_dis[u] + dis:
                shortest_dis[v] = shortest_dis[u] + dis
                pathmark[v] = u

    currentnode = goal
    while currentnode != start:
        try:
            currentnode = int(currentnode)
            path.appendleft(currentnode)
            currentnode = pathmark[currentnode]
        except KeyError:
            print "Path is not reachable"
            break
    path.appendleft(start)

    return path


def astar(graph, start, goal, heuristic):
    """Plan a path from start to goal using A* algorithm.

    Args:
        graph (ndarray): An adjacency matrix, size (n,n)
        start (int): The index of the start node
        goal (int): The index of the goal node
        heuristic (ndarray): The heuristic used for expanding the search

    Returns:
        deque: Indices of nodes along the shortest path
    """
    # TODO
    # record the last vertex to get shortest path
    pathmark = np.zeros(len(graph[0]))  # record the last node to get shortest path (path[i]=j)
    low_cost = np.zeros(len(graph[0]))
    finalmark = np.zeros(len(graph[0]))  # Initialise array that'll record if shortest path was found (1 yes)
    shortest_dis = np.zeros(
        len(graph[0]))  # Initialise array containing the relatively shortest distance (start-other node)
    path = deque()  # Initialise an empty array record shortest distance from start node to others

    for i in range(len(graph[0])):
        # pathmark[i] = 0
        # finalmark[i] = 0
        low_cost[i] = np.Inf
        if graph[start][i] != 0:  # They are connected
            shortest_dis[i] = graph[start][i]  # Weights used to store the shortest path to each point

        else:
            shortest_dis[i] = np.Inf  # If not connected then value it'll be infinity

    # finalmark[start] = 1  # No need to find the shortest path from V0 to V0
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


def dynamic_programming(graph, start, goal):
    """Plan a path from start to goal using dynamic programming. The goal node
    and information about the shortest paths are saved as function attributes
    to avoid unnecessary recalculation.

    Args:
        graph (ndarray): An adjacency matrix, size (n,n)
        start (int): The index of the start node
        goal (int): The index of the goal node

    Returns:
        deque: Indices of nodes along the shortest path
    """
    #TODO

    # number of vertices
    #np.fill_diagonal(graph, 0)  #fills element diagolas with zeros
    graph[graph == 0] = np.inf #Changes zero values by infinity
    n = graph.shape[0] #first element
    # distance array
    #d=shortest_dist
    #p=previous
    shortest_dist = np.full(shape=n, fill_value=np.inf)
    # predecessor array, -1 means no predecessor
    prev = np.full(shape=n, fill_value=-1)

    # Bellman-ford
    shortest_dist[start] = 0
    for i in range(n-1):
        for u in range(len(graph[0])):  #scanns all rows
            for v in range(len(graph[0])):  #scanns all columns
                if shortest_dist[v] > shortest_dist[u] + graph[u, v]:
                    shortest_dist[v] = shortest_dist[u] + graph[u, v]
                    prev[v] = u  #corresponds to current node in the graph

    # traverse the predecessor array
    path = deque()  # creates an empty list called path
    currentnode = goal
    while currentnode != -1:  #when -1 has found the goal
        path.appendleft(currentnode)
        #path.append(cur)
        currentnode = prev[currentnode]
    #path.reverse()
    return path

    #return deque()
