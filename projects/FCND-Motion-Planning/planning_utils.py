from enum import Enum
from queue import PriorityQueue
import numpy as np
import copy
import logging
from bresenham import bresenham
from drawing_utils import *
import time

# For graphs
from sklearn.neighbors import KDTree
import networkx as nx
from shapely.geometry import Polygon, Point, LineString

# For vronoi
from scipy.spatial import Voronoi, voronoi_plot_2d

from tqdm import tqdm

logging.basicConfig(level=logging.DEBUG)






def create_grid(data, drone_altitude, safety_distance):
    """
    Returns a grid representation of a 2D configuration space
    based on given obstacle data, drone altitude and safety distance
    arguments.
    """

    # minimum and maximum north coordinates
    north_min = np.floor(np.min(data[:, 0] - data[:, 3]))
    north_max = np.ceil(np.max(data[:, 0] + data[:, 3]))

    # minimum and maximum east coordinates
    east_min = np.floor(np.min(data[:, 1] - data[:, 4]))
    east_max = np.ceil(np.max(data[:, 1] + data[:, 4]))

    # given the minimum and maximum coordinates we can
    # calculate the size of the grid.
    north_size = int(np.ceil(north_max - north_min))
    east_size = int(np.ceil(east_max - east_min))

    # Initialize an empty grid
    grid = np.zeros((north_size, east_size))

    # Populate the grid with obstacles
    for i in range(data.shape[0]):
        north, east, alt, d_north, d_east, d_alt = data[i, :]
        if alt + d_alt + safety_distance > drone_altitude:
            obstacle = [
                int(np.clip(north - d_north - safety_distance - north_min, 0, north_size-1)),
                int(np.clip(north + d_north + safety_distance - north_min, 0, north_size-1)),
                int(np.clip(east - d_east - safety_distance - east_min, 0, east_size-1)),
                int(np.clip(east + d_east + safety_distance - east_min, 0, east_size-1)),
            ]
            grid[obstacle[0]:obstacle[1]+1, obstacle[2]:obstacle[3]+1] = 1

    return grid, int(north_min), int(east_min)


# Assume all actions cost the same.
class Action(Enum):
    """
    An action is represented by a 3 element tuple.

    The first 2 values are the delta of the action relative
    to the current grid position. The third and final value
    is the cost of performing the action.
    """

    WEST = (0, -1, 1)
    EAST = (0, 1, 1)
    NORTH = (-1, 0, 1)
    SOUTH = (1, 0, 1)

    NORTH_EAST = (NORTH[0] + EAST[0], NORTH[1] + EAST[1], np.sqrt(2))
    SOUTH_EAST = (SOUTH[0] + EAST[0], SOUTH[1] + EAST[1], np.sqrt(2))
    SOUTH_WEST = (SOUTH[0] + WEST[0], SOUTH[1] + WEST[1], np.sqrt(2))
    NORTH_WEST = (NORTH[0] + WEST[0], NORTH[1] + WEST[1], np.sqrt(2))




    @property
    def cost(self):
        return self.value[2]

    @property
    def delta(self):
        return (self.value[0], self.value[1])


def valid_actions(grid, current_node):
    """
    Returns a list of valid actions given a grid and current node.
    """
    valid_actions = list(Action)
    n, m = grid.shape[0] - 1, grid.shape[1] - 1
    x, y = current_node

    # check if the node is off the grid or
    # it's an obstacle

    if x - 1 < 0 or grid[x - 1, y] == 1:
        valid_actions.remove(Action.NORTH)
    if x + 1 > n or grid[x + 1, y] == 1:
        valid_actions.remove(Action.SOUTH)
    if y - 1 < 0 or grid[x, y - 1] == 1:
        valid_actions.remove(Action.WEST)
    if y + 1 > m or grid[x, y + 1] == 1:
        valid_actions.remove(Action.EAST)

    # N-E
    if ( x - 1 < 0 or y + 1 > m ) or grid[x - 1, y + 1] == 1:
        valid_actions.remove(Action.NORTH_EAST)

    # S-E
    if ( x + 1 > n or y + 1 > m) or grid[x + 1, y + 1] == 1:
        valid_actions.remove(Action.SOUTH_EAST)

    # S-W
    if ( x + 1 > n or y - 1 < 0) or grid[x + 1, y - 1] == 1:
        valid_actions.remove(Action.SOUTH_WEST)

    # N-W
    if ( x - 1 < 0 or y - 1 < 0 ) or grid[x - 1, y - 1] == 1:
        valid_actions.remove(Action.NORTH_WEST)

    return valid_actions


def a_star(g, h, start, goal, representation='grid'):
    """ g can be grid or graph """

    path = []
    path_cost = 0
    queue = PriorityQueue()
    queue.put((0, start))
    visited = set(start)

    branch = {}
    found = False
    
    while not queue.empty():
        item = queue.get()
        current_node = item[1]
        if current_node == start:
            current_cost = 0.0
        else:              
            current_cost = branch[current_node][0]
            
        if current_node == goal:        
            logging.info('Found a path.')
            found = True
            break
        else:
            if representation == 'grid':
                for action in valid_actions(g, current_node):
                    # get the tuple representation
                    da = action.delta
                    next_node = (current_node[0] + da[0], current_node[1] + da[1])
                    branch_cost = current_cost + action.cost
                    queue_cost = branch_cost + h(next_node, goal)
                    
                    if next_node not in visited:                
                        visited.add(next_node)               
                        branch[next_node] = (branch_cost, current_node, action)
                        queue.put((queue_cost, next_node))
            elif representation == 'graph':

                for next_node in g[current_node]:
                    cost = g.edges[current_node, next_node]['weight']
                    branch_cost = current_cost + cost
                    queue_cost = branch_cost + h(next_node, goal)
                    if next_node not in visited:
                        visited.add(next_node)
                        branch[next_node] = (branch_cost, current_node)
                        queue.put((queue_cost, next_node))
                 
    if found:
        # retrace steps
        n = goal
        path_cost = branch[n][0]
        path.append(goal)
        while branch[n][1] != start:
            path.append(branch[n][1])
            n = branch[n][1]
        path.append(branch[n][1])
    else:
        logging.error('**********************')
        logging.error('Failed to find a path!')
        logging.error('**********************') 
    return path[::-1], path_cost



def heuristic(position, goal_position):
    return np.linalg.norm(np.array(position) - np.array(goal_position))


def collinearity_check(m, eps):
    # Returns True or False using matrix det of [p1,p2,p3]
   
    det = np.linalg.det(m)

    logging.debug('det for mat {} is {}'.format(m, det))

    return abs(det) < eps


    
def prune_path_collinearity(path, eps):
    ''' Given a path prune using collinearity'''

    pruned_path = copy.copy(path)
    i = 0
    while i < len(pruned_path) - 2:
        mat = np.matrix([
            [pruned_path[i    ][0]    ,pruned_path[i    ][1], 1.0],
            [pruned_path[i + 1][0]    ,pruned_path[i + 1][1], 1.0],
            [pruned_path[i + 2][0]    ,pruned_path[i + 2][1], 1.0]
            ])

        co_res = collinearity_check(mat, eps)

        if co_res == True:
            pruned_path.remove(pruned_path[i + 1])
            logging.debug('removing node {}'.format(pruned_path[i + 1]))
        else:
            i += 1

    logging.info('Path length before prune using collineary {}, after {}'.format(len(path), len(pruned_path)))


    return pruned_path


def prune_path_bresenham(path, grid):
    """ Prune path using Bresenham with ray tracking """

    current_node = path[0]

    pruned_path_bres = []

    # Add first node
    pruned_path_bres.append(current_node) 

    i = 1
    while i < len(path) - 1:
        n1 = current_node
        n2 = path[i + 1]

        cells = list(bresenham(int(n1[0]), int(n1[1]), int(n2[0]), int(n2[1])))

        # Assume no obstacles in cells 
        has_obstacle = False
        for c in cells:
            if grid[c[0], c[1]] == 1:
                has_obstacle = True

        # If no obstacles, move on to next node
        if has_obstacle is False:
            i += 1
        else:
            current_node = path[i]
            pruned_path_bres.append(current_node)

    # Add last node if not in already
    if path[-1] not in pruned_path_bres:
        pruned_path_bres.append(path[-1]) 
    
    logging.info('Path length before prune using Bresenham {}, after {}'.format(len(path), len(pruned_path_bres)))


    return pruned_path_bres


""" 
Graph utils 
"""

def create_voronoi_grid_and_edges(data, drone_altitude, safety_distance):
    """
    Taken from online exercise
    Returns a grid representation of a 2D configuration space
    along with Voronoi graph edges given obstacle data and the
    drone's altitude.
    """
    logging.info('Creating vironoi graph ...')

    # minimum and maximum north coordinates
    north_min = np.floor(np.min(data[:, 0] - data[:, 3]))
    north_max = np.ceil(np.max(data[:, 0] + data[:, 3]))

    # minimum and maximum east coordinates
    east_min = np.floor(np.min(data[:, 1] - data[:, 4]))
    east_max = np.ceil(np.max(data[:, 1] + data[:, 4]))

    # given the minimum and maximum coordinates we can
    # calculate the size of the grid.
    north_size = int(np.ceil((north_max - north_min)))
    east_size = int(np.ceil((east_max - east_min)))

    # Initialize an empty grid
    grid = np.zeros((north_size, east_size))
    # Center offset for grid
    north_min_center = np.min(data[:, 0])
    east_min_center = np.min(data[:, 1])
    
   
    
    # Define a list to hold Voronoi points
    points = []
    # Populate the grid with obstacles
    for i in range(data.shape[0]):
        north, east, alt, d_north, d_east, d_alt = data[i, :]

        if alt + d_alt + safety_distance > drone_altitude:
            obstacle = [
                int(north - d_north - safety_distance - north_min_center),
                int(north + d_north + safety_distance - north_min_center),
                int(east - d_east - safety_distance - east_min_center),
                int(east + d_east + safety_distance - east_min_center),
            ]
            grid[obstacle[0]:obstacle[1]+1, obstacle[2]:obstacle[3]+1] = 1
            
            # add center of obstacles to points list
            points.append([north - north_min, east - east_min])

    # TODO: create a voronoi graph based on
    # location of obstacle centres

    graph = Voronoi(points)

    # TODO: check each edge from graph.ridge_vertices for collision
    edges = []
    edges = []
    
    logging.info('Finished voronoi vertices number : {}'.format(len(graph.ridge_vertices)))
    G = nx.Graph()
    for v in graph.ridge_vertices:
        p1 = graph.vertices[v[0]]
        p2 = graph.vertices[v[1]]
        cells = list(bresenham(int(p1[0]), int(p1[1]), int(p2[0]), int(p2[1])))
        hit = False

        for c in cells:
            # First check if we're off the map
            if np.amin(c) < 0 or c[0] >= grid.shape[0] or c[1] >= grid.shape[1]:
                hit = True
                break
            # Next check if we're in collision
            if grid[c[0], c[1]] == 1:
                hit = True
                break

        # If the edge does not hit on obstacle
        # add it to the list
        if not hit:
            # array to tuple for future graph creation step)
            p1 = (p1[0], p1[1])
            p2 = (p2[0], p2[1])
            G.add_edge(p1, p2, weight=np.linalg.norm(np.array(p1)-np.array(p2)))
    

    return G, north_min_center, east_min_center

def find_closest_node(node, graph):
    # Returns closes node

    dist_min = 10000000000
    closest_node = None
    for n in graph.nodes:
        dist = np.linalg.norm(np.array(node) - np.array(n))

        if dist < dist_min:
            closest_node = n
            dist_min = dist

    return closest_node








if  __name__ == '__main__':
    data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)

   
    #G = create_probabilistic_roadmap(data, num_samples=500)
    grid, north_offset, east_offset = create_grid(data, 5, 2)

    g, a, b = create_voronoi_grid_and_edges(data, 5, 2)
    print(g)
    plot_graph(grid, g, 0, 0, start=(310.2389, 439.2315), end=(694.2389000000001, 213.23149999999998))

