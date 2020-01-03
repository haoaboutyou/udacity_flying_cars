import argparse
import time
import msgpack
from enum import Enum, auto

import numpy as np

from planning_utils import *
from drawing_utils import plot_grid_path
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local

import logging

logging.basicConfig(level=logging.DEBUG)


class States(Enum):
    MANUAL = auto()
    ARMING = auto()
    TAKEOFF = auto()
    WAYPOINT = auto()
    LANDING = auto()
    DISARMING = auto()
    PLANNING = auto()


class MotionPlanning(Drone):

    def __init__(self, connection, target_lon, target_lat, target_alt, pruning_method, representation):
        super().__init__(connection)

        self.target_position = np.array([target_lon, target_lat, target_alt])
        self.waypoints = []
        self.in_mission = True
        self.check_state = {}

        # initial state
        self.flight_state = States.MANUAL

        # register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

        # prune method
        self.pruning_method = pruning_method

        # grid or graph method
        self.representation = representation

    def local_position_callback(self):
        if self.flight_state == States.TAKEOFF:
            if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < 1.0:
                if len(self.waypoints) > 0:
                    self.waypoint_transition()
                else:
                    if np.linalg.norm(self.local_velocity[0:2]) < 1.0:
                        self.landing_transition()

    def velocity_callback(self):
        if self.flight_state == States.LANDING:
            if self.global_position[2] - self.global_home[2] < 0.1:
                if abs(self.local_position[2]) < 0.01:
                    self.disarming_transition()

    def state_callback(self):
        if self.in_mission:
            if self.flight_state == States.MANUAL:
                self.arming_transition()
            elif self.flight_state == States.ARMING:
                if self.armed:
                    self.plan_path()
            elif self.flight_state == States.PLANNING:
                self.takeoff_transition()
            elif self.flight_state == States.DISARMING:
                if ~self.armed & ~self.guided:
                    self.manual_transition()

    def arming_transition(self):
        self.flight_state = States.ARMING
        logging.debug("arming transition")
        self.arm()
        self.take_control()

    def takeoff_transition(self):
        self.flight_state = States.TAKEOFF
        logging.debug("takeoff transition")
        self.takeoff(self.target_position[2])

    def waypoint_transition(self):
        self.flight_state = States.WAYPOINT
        logging.debug("waypoint transition")
        self.target_position = self.waypoints.pop(0)
        logging.debug('target position {}'.format(self.target_position))
        self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2], self.target_position[3])

    def landing_transition(self):
        self.flight_state = States.LANDING
        logging.debug("landing transition")
        self.land()

    def disarming_transition(self):
        self.flight_state = States.DISARMING
        logging.debug("disarm transition")
        self.disarm()
        self.release_control()

    def manual_transition(self):
        self.flight_state = States.MANUAL
        logging.debug("manual transition")
        self.stop()
        self.in_mission = False

    def send_waypoints(self):
        logging.debug("Sending waypoints to simulator ...")
        data = msgpack.dumps(self.waypoints)
        self.connection._master.write(data)

    def plan_path(self):
        self.flight_state = States.PLANNING
        logging.debug("Searching for a path ...")
        TARGET_ALTITUDE = 5
        SAFETY_DISTANCE = 3

        self.target_position[2] = TARGET_ALTITUDE

        # TODO: read lat0, lon0 from colliders into floating point values
        with open('colliders.csv', 'r') as f:
            line = f.readline().strip().replace(' ', ',').replace(',,',',').split(',')
            lat0 = float(line[1])
            lon0 = float(line[3])

            logging.debug(' lat0 : {}, lon0 : {}'.format(lat0, lon0))

        
        # TODO: set home position to (lon0, lat0, 0)
        self.set_home_position(lon0, lat0, 0.0)


        # TODO: retrieve current global position
        global_position = self.global_position


 
        # TODO: convert to current local position using global_to_local()
        local_position = global_to_local(global_position, (lon0, lat0, 0.0))

        logging.debug(' current local position {}'.format(local_position))
        
        logging.debug(' global home {0}, position {1}, local position {2}'.format(self.global_home, self.global_position,
                                                                         self.local_position))


        # Read in obstacle map
        data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)

        # Define a grid for a particular altitude and safety margin around obstacles
        start = time.time()         
        grid, north_offset, east_offset = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
        if self.representation == 'graph':
            logging.info('Creating vronoi graph ...')
            G, north_offset, east_offset = create_voronoi_grid_and_edges(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
        
        
        end = time.time()
        logging.info('create_grid time {}s'.format(end - start))

        logging.debug("North offset = {0}, east offset = {1}".format(north_offset, east_offset))

        # Define starting point on the grid (this is just grid center)
        grid_start = (-north_offset, -east_offset)
        # TODO: convert start position to current position rather than map center

        logging.debug(' original start {}'.format(grid_start))
        debug = False
        if debug is False: # return to home if we are debugging
            grid_start = (grid_start[0] + int(local_position[0]), grid_start[1] + int(local_position[1]))

        logging.debug(' new start {}'.format(grid_start))
        
        # Set goal as some arbitrary position on the grid
        grid_goal = (-north_offset + 50, -east_offset + 50)
        # TODO: adapt to set goal as latitude / longitude position and convert
        
        global_goal = (self.target_position[0], self.target_position[1], TARGET_ALTITUDE)



        logging.debug(' global_goal {}'.format(global_goal))
        logging.debug(' global_home {}'.format((lon0, lat0, 0.0)))

        grid_goal = global_to_local(global_goal, (lon0, lat0, 0.0))
        grid_goal = (-north_offset + int(grid_goal[0]), -east_offset + int(grid_goal[1]))



        # Run A* to find a path from start to goal
        # TODO: add diagonal motions with a cost of sqrt(2) to your A* implementation
        # or move to a different search space such as a graph (not done here)
        logging.debug('Local Start and Goal: {} {}'.format(grid_start, grid_goal))
               

        # A* for both grid and graph   
        if self.representation == 'grid':
            logging.info('Finding path using a* for grid ...')
            path, _ = a_star(grid, heuristic, grid_start, grid_goal, representation=self.representation)
        elif self.representation == 'graph':
            logging.info('Finding path using a* for graph ...')

            start_node = find_closest_node(grid_start, G)
            end_node = find_closest_node(grid_goal, G)

            logging.info('Start node : {}'.format(start_node))
            logging.info('End node : {}'.format(end_node))

            # Find start and end node in graph         

            path, _ = a_star(G, heuristic, start_node, end_node, representation=self.representation)


        # Path pruning 
        if self.pruning_method == 'collinear':
            start = time.time()
            path = prune_path_collinearity(path, eps=0.005)
            end = time.time()
            logging.info('Pruning using collinearity took {}s'.format(end - start))           
        elif self.pruning_method == 'bresenham':
            start = time.time()
            path = prune_path_bresenham(path, grid)
            end = time.time()
            logging.info('Pruning using bresenham took {}s'.format(end - start))
        else:
            logging.warning('Pruning {} not supported using default path'.format(self.pruning_method))


        # Plots for debugging    
        # plot_grid_path(grid, grid_start, grid_goal, path, 'after_col_prune')
        # plot_grid_path(grid, grid_start, grid_goal, path, 'after_bres_prune')
        # plot_graph(grid, G, 0, 0, path=path, start=(310.2389, 439.2315), end=(694.2389000000001, 213.23149999999998), plt_name='voronoi_graph')



        # Convert path to waypoints
        waypoints = [[int(p[0] + north_offset), int(p[1] + east_offset), TARGET_ALTITUDE, 0] for p in path]
        logging.debug(' waypoints {}'.format(waypoints))
        # Set self.waypoints
        self.waypoints = waypoints
        # TODO: send waypoints to sim (this is just for visualization of waypoints)
        self.send_waypoints()

    def start(self):
        self.start_log("Logs", "NavLog.txt")

        logging.debug("starting connection")
        self.connection.start()

        # Only required if they do threaded
        # while self.in_mission:
        #    pass

        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")

    parser.add_argument('--target_lon', type=float, default=-122.40000, help='Target longitude')
    parser.add_argument('--target_lat', type=float, default=37.795956, help='Target latitude')
    parser.add_argument('--target_alt', type=float, default=0.0, help='Target altitude')
    parser.add_argument('--pruning_method', type=str, default='bresenham', help='Pruning method, can be bresenham or collinear')
    parser.add_argument('--representation', type=str, default='graph', help='Representation of map, can be grid or graph')

    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=60)

    target_lon = args.target_lon
    target_lat = args.target_lat
    target_alt = args.target_alt
    pruning_method = args.pruning_method
    representation = args.representation

    drone = MotionPlanning(
        conn, 
        target_lon=target_lon, 
        target_lat=target_lat, 
        target_alt=target_alt, 
        pruning_method=pruning_method, 
        representation=representation)
    time.sleep(1)

    drone.start()
