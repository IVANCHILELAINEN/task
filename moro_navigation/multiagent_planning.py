#!/usr/bin/env python
import numpy as np
from itertools import permutations
from shortest_path import dijkstra
import copy


def time_expand(graph, nodes, obstruction):
    """Create a time-expanded graph taking into account the given
    dynamic obstruction.

    Args:
        graph (ndarray): The original graph adjacency matrix, size (n,n)
        nodes (ndarray): Node coordinates in the graph, size (n,2)
        obstruction (array-like): Indexes of obstructed nodes, length t

    Returns:
        tuple: The time-expanded graph, size (tn,tn), and node
            coordinates in the new graph, size(tn,2)
    """
    # TODO
    #a_graph = np.array(graph, copy=True)
    tn = len(obstruction)
    num_nodes = len(nodes)
    tot_new_nodes = (tn + 1) * num_nodes
    expanded_graph = np.zeros((tot_new_nodes, tot_new_nodes))
    expanded_nodes = np.ndarray((tot_new_nodes, 2))

    for ts in range(tn):  # Initializes the time expanded graph, populating its diagonal with "A"
        first_node = ts * num_nodes  # first node in ti(time step)
        last_node = first_node + num_nodes  # corresponds to the last node in time step
        expanded_graph[first_node:last_node, last_node:last_node + num_nodes] = graph

    # It will clear paths that are obstructed by assigning value 0 (no path)
    for ts in range(len(obstruction)):
        first_node = ts * num_nodes
        expanded_graph[:, first_node + obstruction[ts]] = 0
        try:  # Eliminates the paths to the next location if obstructed(avoid sharing path)
            expanded_graph[first_node + obstruction[ts + 1], :] = 0
        except:
            break

    expanded_graph[tot_new_nodes - num_nodes:tot_new_nodes, :] = \
        expanded_graph[tot_new_nodes - 2 * num_nodes:tot_new_nodes - num_nodes, :]  # At the end of adjacency matrix
    # #copies "At" when obstruction is static

    expanded_nodes = np.tile(nodes, (tn + 1, 1))  # Tiles nodes

    return expanded_graph, expanded_nodes



def joint_graph(graph, nodes):
    """Create a joint graph for two agents based on the given single
    agent graph

    Args:
        graph (ndarray): The single agent graph adjacency matrix, size (n,n)
        nodes (ndarray): Node coordinates in the graph, size (n,2)

    Returns:
        tuple: The joint graph, size (n^2-n,n^2-n), and node coordinates in the
            joint graph, size (n^2-n,2,2), where the second axis is the two
            agents and the third axis is coordinates
    """
    # TODO
    joint_graph = nodes = None

    return joint_graph, nodes


if __name__ == '__main__':
    print("python main function")
    graph = 0.05 * np.eye(7)
    graph[0, 1] = 3
    graph[0, 2] = 2
    graph[1, 3] = 1
    graph[2, 3] = 3
    graph[3, 4] = 3
    graph[3, 5] = 1
    graph[4, 6] = 2
    graph[0, 0] = 0.01
    graph = graph + graph.T
    nodes = np.array([[0., 0.],
                      [1., 1.],
                      [1., -1.],
                      [2., 0.],
                      [3., 1.],
                      [3., -1.],
                      [4., 0.]])
    num_nodes = 7

    obstruction = [6, 4, 3, 1, 0]
    sp = [0, 0, 1, 3, 4, 6]  # sp = [0, 0, 2, 3, 4, 6]
    graph, nodes = time_expand(graph, nodes, obstruction)
    path = dijkstra(graph, 0, nodes.shape[0] - 1)
    print [node % num_nodes for node in path]
