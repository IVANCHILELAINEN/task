#!/usr/bin/env python
from skimage.draw import line, circle
from copy import copy
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from utils import distance
import math

# The values in the occupancy grid map
OCCUPIED = 100
FREE = 0
UNKNOWN = -1


class ProbabilisticRoadmap(object):
    """Provides probabilistic roadmapping in a given map.

    Attributes:
        graph (ndarray): An adjacency matrix of size (num_nodes,num_nodes)
            consisting of edge costs between nodes.
        nodes (ndarray): Node coordinates of size (num_nodes,2)
    """

    def __init__(self, og, inflation_radius=0.25):
        """Initialize the ProbabilisticRoadmap.

        Args:
            og (nav_msgs/OccupancyGrid): The map to use for roadmapping.
            inflation_radius (float, optional): How much obstacles are inflated
                in the map in meters. Default: 0.25
        """
        super(ProbabilisticRoadmap, self).__init__()
        self.no_nodes = 46
        self.nodes = np.ndarray((self.no_nodes, 2))
        self.graph = np.ndarray((self.no_nodes, self.no_nodes))

        # Unpack the data from the occupancy grid
        self._resolution = og.info.resolution
        self._origin = np.array([og.info.origin.position.x,
                                 og.info.origin.position.y])
        self._xmin = self._origin[0]
        self._xmax = self._origin[0] + og.info.width * self._resolution
        self._ymin = self._origin[1]
        self._ymax = self._origin[1] + og.info.height * self._resolution

        self._og_map = np.array(og.data).reshape((og.info.height,
                                                  og.info.width))

        # Inflate the obstacles in the map by inflation_radius
        self._map = self._inflate_map(self._og_map, inflation_radius)

        # Create the graph. This fills out self.nodes and self.graph
        self.create_graph()

    def _figure_coordinates(self, position):
        """Get map figure coordinates for a position.

        Args:
            position (ndarray): Array of coordinates size (2,) or (n,2).
                For a single position also list or tuple of length 2.

        Returns:
            ndarray: Coordinates of position in map figure. Same size
                as position.
        """
        position = np.array(position)
        scaled = np.atleast_2d((position - self._origin) / self._resolution)
        return np.fliplr(scaled).astype(np.uint16).reshape(position.shape)

    def _is_free(self, position):
        """Check whether a position is free in the map.

        Args:
            position (ndarray): A single position to check, size (2,).
                Alternatively list or tuple of length 2.

        Returns:
            bool: Returns True if the position is free.
        """
        index = self._figure_coordinates(position)
        return self._map[tuple(index)] == FREE

    def _inflate_map(self, og, radius):
        """Inflate the obstacles in map by a given radius.

        Args:
            og (ndarray): Array representing an occupancy grid
            radius (double): Inflation radius in meters

        Returns:
            ndarray: Inflated occupancy grid. Same size as og.
        """
        # TODO
        pix_radious = radius / self._resolution
        h_ = og.shape[0]
        w_ = og.shape[1]

        for j in range(h_):
            for k in range(w_):
                if og[j, k] == OCCUPIED:
                    rr, cc = circle(j, k, pix_radious, (j + 1, k + 1))  # Pixel coordinates of the circle
                    og[rr, cc] = OCCUPIED

        og = copy(og)  # ocupancy grid
        return og

    def _draw_sample(self):
        """Draw a random sample from the configuration space

        Returns:
            ndarray: Sampled coordinates, size (2,).
        """
        # TODO

        sample = np.ndarray((self.no_nodes, 2))  # Array of the coordinates
        np.random.seed(0)
        for i in range(self.no_nodes):
            x_ = self._xmax * np.random.rand()  # x coordinate for the rand gen node
            y_ = self._ymax * np.random.rand()  # y coordinate for the rand gen node
            # while self._is_free(x_, y_) == False:
            while not self._is_free((x_, y_)):
                x_ = self._xmax * np.random.rand()
                y_ = self._ymax * np.random.rand()
            sample[i, 0], sample[i, 1] = x_, y_

        return sample

        """
        sample = np.empty((0, 2))
        sample = np.ndarray((self.no_nodes, 2))

        #sample = np.array([[1, 1]])
        i = 0
        for x_ in range(1, int(self._xmax), 2):
            for y_ in range(1, int(self._ymax), 2):
                pos = (x_, y_)
                if self._is_free(pos):
                    sample[i, 0], sample[i, 1] = x_, y_

                    i += 1


        return sample
        """
    def can_connect(self, a, b):
        """Check whether the connecting line segment between two points
        is unobstructed.

        Args:
            a (ndarray): Coordinates for first point, size (2,)
            b (ndarray): Coordinates for second point, size (2,)

        Returns:
            bool: Returns True if there are no obstacles between the points.
        """
        # TODO
        a_x_y = self._figure_coordinates(a)  # Coordinates for first point
        b_x_y = self._figure_coordinates(b)  # Coordinates for second point
        line_pix_coord = []
        x_c, y_c = line(a_x_y[0], a_x_y[1], b_x_y[0], b_x_y[1])  # Generates the coordinates of pix present on the line

        for i in range(len(x_c)):
            line_pix_coord2 = (x_c[i], y_c[i])
            line_pix_coord.append(line_pix_coord2)  # annexing pixel coordinates to list

        for pixel_coord in line_pix_coord:
            if (self._map[pixel_coord[0], pixel_coord[1]] == OCCUPIED):  # checks if that pixel is occupied
                return False
        return True

    def create_graph(self):
        """Create the nodes and connections in the graph. Fills out the class
        attributes self.nodes and self.graph with the corresponding values.
        """
        # TODO
        # Save the results to the class attributes
        self.nodes = self._draw_sample()
        i = 0
        for n1 in self.nodes:
            j = 0
            for n2 in self.nodes:
                if self.can_connect(n1, n2):
                    self.graph[i, j] = math.sqrt(((n1[0]-n2[0])**2)+((n1[1]-n2[1])**2))
                    self.graph[j, i] = math.sqrt(((n1[0]-n2[0])**2)+((n1[1]-n2[1])**2))
                else:
                    self.graph[i, j] = 0
                    self.graph[j, i] = 0
                j = j + 1
            i = i + 1

        # print(self.graph)

    def plot(self, path=None, color= 'ro-'):
        """Plot the map, nodes and connections of the ProbabilisticRoadmap

        Args:
            path (list, optional): Highlight the nodes that make up the path
        """
        ax = plt.gca()
        extent = (self._xmin, self._xmax, self._ymin, self._ymax)
        ax.imshow(self._og_map, cmap='Greys', origin='lower', extent=extent)
        ax.imshow(self._map, cmap='Reds', origin='lower',
                  extent=extent, alpha=0.3)

        ax.plot(self.nodes[:, 0], self.nodes[:, 1], 'bo')

        source, sink = np.nonzero(self.graph)
        source = self.nodes[source]
        sink = self.nodes[sink]
        lc = LineCollection(np.stack((source, sink), axis=1),
                            linewidths=[1], colors=[(0, 0.75, 1, 1)])
        ax.add_collection(lc)

        ax.set_xlim((self._xmin, self._xmax))
        ax.set_ylim((self._ymin, self._ymax))

        if path:
            path = self.nodes[path]
            ax.plot(path[:, 0], path[:, 1], color, linewidth=2)