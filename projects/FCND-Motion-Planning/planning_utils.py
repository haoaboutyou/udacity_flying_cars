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


from tqdm import tqdm







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
                    queue_cost = branch_cost + heuristic(next_node, goal)
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

        cells = list(bresenham(n1[0], n1[1], n2[0], n2[1]))

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


""" Graph utils """
def get_polygons(data):
        polygons = []

        for i in range(data.shape[0]):
            n, e, h, d_n, d_e, d_h = data[i, :]

            n1 = n - d_n
            n2 = n + d_n
            e1 = e - d_e
            e2 = e + d_e
            

            corners = [
                (n1, e1),
                (n2, e1),
                (n2, e2),
                (n1, e2), 
            ]


            p = Polygon(corners)

            polygons.append((p, h + d_h))
        return polygons

def collides(polygons, point):
    # Returns true if point is inside poygons
    test_point = Point(point)
    for (p, h) in polygons:

        if p.contains(test_point) and h >= point[2]:
           

            return True


    return False

def can_connect(n1, n2, polygons):
    l = LineString([n1, n2])
    for p in polygons:
        if p[0].crosses(l) and p[1] >= min(n1[2], n2[2]):
            return False

    return True
def create_probabilistic_roadmap(data, num_samples=200):                                                                                                            
    """
    Returns a 2.5 polygon configuration space given on data
    """
    

    north_min = np.min(data[:, 0] - data[:, 3])
    north_max = np.max(data[:, 0] + data[:, 3])

    east_min = np.min(data[:, 1] - data[:, 4])
    east_max = np.max(data[:, 1] + data[:, 4])


    alt_min = 0
    alt_max = 100

    print(north_min, north_max, east_min, east_max, alt_min, alt_max)

    xvals = np.random.uniform(north_min, north_max, num_samples)
    yvals = np.random.uniform(east_min, east_max, num_samples)
    zvals = np.random.uniform(alt_min, alt_max, num_samples)

    samples = list(zip(xvals, yvals, zvals))


    # Create polygons
    logging.debug('creating polygons ...')
    polygons = get_polygons(data)

    # Add nodes if not in polygons
    logging.debug('creating nodes ...')
    nodes = []
    for p in tqdm(samples):
        if not collides(polygons, p):
            nodes.append(p)



    # Create a graph

    logging.debug('creating graph ...')
    G = nx.Graph()

    tree = KDTree(nodes)

    for n1 in nodes:
        # get closest 5
        ids = tree.query([n1], 3, return_distance=False)[0]
        for i in ids:
            n2 = nodes[i]

            if can_connect(n1, n2, polygons) and n1 != n2:
                G.add_edge(n1, n2, weight=1)


    return G



if  __name__ == '__main__':
    data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)

   
    logging.info('Creating probabilistic map ...')
    G = create_probabilistic_roadmap(data, num_samples=500)
    grid, north_offset, east_offset = create_grid(data, 5, 5)
    plot_graph(grid, G)

