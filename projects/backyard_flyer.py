import argparse
import time
from enum import Enum

import numpy as np

from udacidrone import Drone
from udacidrone.connection import MavlinkConnection, WebSocketConnection  # noqa: F401
from udacidrone.messaging import MsgID


class States(Enum):
    MANUAL = 0
    ARMING = 1
    TAKEOFF = 2
    WAYPOINT = 3
    LANDING = 4
    DISARMING = 5

class BackyardFlyer(Drone): 
    def __init__(self, connection):
        super().__init__(connection)
        self.target_position = np.array([0.0, 0.0, 0.0])
        self.all_waypoints = []
        self.in_mission = True
        self.check_state = {}

        # initial state
        self.flight_state = States.MANUAL

        # waypoint variables
        self.waypoint_id = 0 
        self.waypoint_counter = 0
        self.square_size = 9.0
        

        # TODO: Register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    @staticmethod
    def dist_xy(p1, p2):
        print(p1, p2)
        return np.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

    def local_position_callback(self):
        """
        TODO: Implement this method

        This triggers when `MsgID.LOCAL_POSITION` is received and self.local_position contains new data
        """
        print('[info] p_cb ,  local position {} '.format(self.local_position))
        print('[info] waypoints {}'.format(self.all_waypoints))
        if self.flight_state == States.TAKEOFF:
            altitude = -1.0 * self.local_position[2]

            print('[info] tp {}'.format(self.target_position))

            if altitude > 0.98 * self.target_position[2]:

                self.calculate_box()

                self.waypoint_transition()
        


        
    def velocity_callback(self):
        """
        TODO: Implement this method

        This triggers when `MsgID.LOCAL_VELOCITY` is received and self.local_velocity contains new data
        """

        if self.flight_state == States.LANDING:
            if abs(self.local_position[2]) < 0.01:
                self.disarming_transition()

    def state_callback(self):
        """
        TODO: Implement this method

        This triggers when `MsgID.STATE` is received and self.armed and self.guided contain new data
        """

        if not self.in_mission:
            return

        if self.flight_state == States.MANUAL:
            self.arming_transition()

        elif self.flight_state == States.ARMING:
            if self.armed:
                self.takeoff_transition()

        elif self.flight_state == States.WAYPOINT:
            if self.armed:
                self.waypoint_transition()

        elif self.flight_state == States.DISARMING:
            if not self.armed:
                self.manual_transition()

    def calculate_box(self):
        """TODO: Fill out this method
        
        1. Return waypoints to fly a box
        """

        # Set xy box
    
        self.all_waypoints = [
                (self.local_position[0]                    , self.local_position[1] ),
                (self.local_position[0]+self.square_size   , self.local_position[1] ),
                (self.local_position[0]+self.square_size   , self.local_position[1]+ self.square_size),
                (self.local_position[0]                    , self.local_position[1]+ self.square_size),
                ]
        

    def arming_transition(self):
        """TODO: Fill out this method
        
        1. Take control of the drone
        2. Pass an arming command
        3. Set the home location to current position
        4. Transition to the ARMING state
        """
        print("arming transition")

        self.take_control()
        self.arm()

        self.set_home_position(
                self.global_position[0],
                self.global_position[1],
                self.global_position[2],
                )
        
        self.flight_state = States.ARMING

    def takeoff_transition(self):
        """TODO: Fill out this method
        
        1. Set target_position altitude to 3.0m
        2. Command a takeoff to 3.0m
        3. Transition to the TAKEOFF state
        """
        print("takeoff transition")
        self.target_position[2] = 3.0
        self.takeoff(3.0)
        self.flight_state = States.TAKEOFF

    def waypoint_transition(self):
        """TODO: Fill out this method
    
        1. Command the next waypoint position
        2. Transition to WAYPOINT state
        """
        print("waypoint transition")

        # find which waypoint if sufficient move to next one

        for i, wp in enumerate(self.all_waypoints):

            # distance to i-th waypoint

            dist = self.dist_xy(wp, np.array([self.local_position[0], self.local_position[1]]))
            print('[info] wp : {}, lp :  {}'.format(wp, self.local_position[0:2]))
            print('[info] dist to waypoint {} is {}'.format(i, dist))
            
            if dist < 0.2: # if we are at waypoint fly to next one
 
                self.waypoint_id = (i + 1)%4

                print('[info] we are flying to next wp at {} '.format(self.all_waypoints[self.waypoint_id]))
                print('--------------------- {} ------------------------'.format(self.waypoint_counter))

                # If we have finished one loop set to landing state
                if self.waypoint_counter == 5: 
                    self.landing_transition()

                self.waypoint_counter += 1
                time.sleep(0.2)


                    



        if self.flight_state is not States.LANDING: 
            fly_to_pt = self.all_waypoints[self.waypoint_id%4]
            self.cmd_position(fly_to_pt[0],  fly_to_pt[1], 3.0, 0)

            print('[info] flying to {}, with id {}'.format(fly_to_pt, self.waypoint_id))
                
            self.flight_state = States.WAYPOINT 

        
        print('[state] {} '.format(self.flight_state))

        
        

    def landing_transition(self):
        """TODO: Fill out this method
        
        1. Command the drone to land
        2. Transition to the LANDING state
        """
        print("landing transition")
        self.land()
        self.flight_state = States.LANDING

    def disarming_transition(self):
        """TODO: Fill out this method
        
        1. Command the drone to disarm
        2. Transition to the DISARMING state
        """
        print("disarm transition")
        self.disarm()
        self.flight_state = States.DISARMING

    def manual_transition(self):
        """This method is provided
        
        1. Release control of the drone
        2. Stop the connection (and telemetry log)
        3. End the mission
        4. Transition to the MANUAL state
        """
        print("manual transition")

        self.release_control()
        self.stop()
        self.in_mission = False
        self.flight_state = States.MANUAL

    def start(self):
        """This method is provided
        
        1. Open a log file
        2. Start the drone connection
        3. Close the log file
        """
        print("Creating log file")
        self.start_log("Logs", "NavLog.txt")
        print("starting connection")
        self.connection.start()
        print("Closing log file")
        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), threaded=False, PX4=False)
    #conn = WebSocketConnection('ws://{0}:{1}'.format(args.host, args.port))
    drone = BackyardFlyer(conn)
    time.sleep(2)
    drone.start()
