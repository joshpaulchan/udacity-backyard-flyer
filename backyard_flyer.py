import argparse
import time
from enum import Enum

import numpy as np
from numpy.linalg import norm as distance

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

    HOVERED = 10

class BackyardFlyer(Drone):

    def __init__(self, connection):
        super().__init__(connection)
        self.target_position = np.array([0.0, 0.0, 0.0])
        self.all_waypoints = []
        self.in_mission = True
        self.check_state = {}

        # initial state
        self.flight_state = States.MANUAL
        self.next_state = None # placeholder for wait states (hover, etc.)

        # TODO: Register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        """
        @note On position-dependent transitions, use this

        This triggers when `MsgID.LOCAL_POSITION` is received and self.local_position contains new data
        """

        # if in flight state
        if self.flight_state == States.TAKEOFF:
            tgt_alt = -1.0 * self.target_position[2]
            # flexible comparison,
            if tgt_alt > 0.90 * self.local_position[2]:
                # self.landing_transition()

                # start pathing
                self.all_waypoints = self.calculate_box()
                self.waypoint_transition()
        
        elif self.flight_state == States.WAYPOINT:
            print(self.target_position)
            print(self.local_position)
            cur_pos = self.local_position
            cur_pos[2] *= -1
            # TODO: distance caculation + comparison (7.25) is arbitrary, better way to compare point accuracy in 3d space?
            dist = abs(distance(self.target_position - self.local_position))
            print("dist: {}".format(dist))
            # if it's in the area and moving slow
            if abs(dist) < 7.25 and self.local_velocity[0] < 0.5 and self.local_velocity[1] < 0.5:
                self.waypoint_transition()

    def velocity_callback(self):
        """
        TODO: Implement this method

        This triggers when `MsgID.LOCAL_VELOCITY` is received and self.local_velocity contains new data
        """
        if self.flight_state == States.LANDING:
            gps_alt = self.global_position[2]
            imu_alt = -1.0 * self.local_position[2]
            if (abs(gps_alt - self.global_home[2]) < 0.1 and imu_alt < 0.1):
                self.disarming_transition()

    def state_callback(self):
        """
        TODO: Implement this method

        This triggers when `MsgID.STATE` is received and self.armed and self.guided contain new data
        """
        if not self.in_mission: return

        
        if self.flight_state == States.MANUAL:
            self.arming_transition()
        elif self.flight_state == States.ARMING:
            self.takeoff_transition()

        # allow for hovering between wayponts
        # if self.flight_state == States.HOVERED:
        #     self.resume_next_state()

        if self.flight_state == States.DISARMING:
            self.manual_transition()

    def calculate_box(self):
        """Returns a list of local waypoints to fly a box 
        
        1. Return a list waypoints to fly a box
        """
        box_length = 10
        cn = self.local_position[0]
        ce = self.local_position[1]
        ca = -1 * self.local_position[2]
        return [
            np.array([cn + box_length, ce, ca]),
            np.array([cn + box_length, ce + box_length, ca]),
            np.array([cn, ce + box_length, ca]),
            np.array([cn, ce, ca])
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

        # set the current location to be the home position
        self.set_home_position(self.global_position[0],
                               self.global_position[1],
                               self.global_position[2])

        self.flight_state = States.ARMING

    def takeoff_transition(self):
        """Once you have control, take off 
        
        """
        print("takeoff transition")

        # 1. Set target_position altitude to 3.0m
        target_altitude = 3.0
        self.target_position[2] = target_altitude

        # 2. Command a takeoff to 3.0m
        self.takeoff(target_altitude)

        # 3. Transition to the TAKEOFF state
        self.flight_state = States.TAKEOFF

    def waypoint_transition(self):
        """Transition to the next waypoint position
    
        1. Command the next waypoint position
        2. Transition to WAYPOINT state
        """
        print("waypoint transition")

        if len(self.all_waypoints) > 0:
            # 1. Get next waypoint
            waypoint = self.all_waypoints.pop(0)

            print(waypoint)

            # 2. Set target position
            self.target_position = waypoint

            # 3. Start moving there
            self.cmd_position(*waypoint, 0)

            # 3. Transition to WAYPOINT state
            self.flight_state = States.WAYPOINT
        else:
            self.landing_transition()
    
    def landing_transition(self):
        """TODO: Fill out this method
        
        1. Command the drone to land
        2. Transition to the LANDING state
        """
        print("landing transition")
        # set target altitude
        # target_altitude = 0

        # # set target position
        # self.target_position[2] = 0

        # # move to target altitude
        # self.takeoff(target_altitude)

        # just start descending
        self.land()

        # change state
        self.flight_state = States.LANDING
    
    def resume_next_state(self):
        """Transitions from hovering to the next state
        """
        pass

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
