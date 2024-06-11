import enum
import socket
import time

import rclpy
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator

class RobotState(enum.Enum):
    WAITING = "waiting"     # waiting for the rover to arrive
    WORKING = "working"     # placing a workpiece onto the rover
    READY = "ready"         # the robot has finished its work

class RoverState(enum.Enum):
    DOCKED = "docked"       # rover is docked to the charging station
    MOVING = "moving"       # rover is navigating to the robot
    ON_ABB = "onABB"        # rover has arrived to the ABB robot
    ON_UR5 = "onUR5"        # rover has arrived to the UR5 robot

HOST = "192.168.0.100"
PORT = 3000

# Responses.
RESPONSE_OK = "ok"
RESPONSE_NOT_OK = "notOk"

# Coordinates of the robots.
COORDS_ABB = [-1.80, 2.80]
COORDS_UR5 = [-4.25, -3.30]


def send_message(message: str) -> str:
    """Send a message to the server socket. Return a response."""
    response = ""
    client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    errnum = client.connect_ex((HOST, PORT))
    if errnum == 0:
        client.sendall(message.encode())
        response = client.recv(128).decode()
        client.close()
    return response


def main(args=None):
    rclpy.init(args=args)
    navigator = TurtleBot4Navigator()

    # Start by docking the rover.
    if not navigator.getDockedStatus():
        send_message(f"setState,tb4,{RoverState.MOVING.value}")
        navigator.info("Docking before initialising pose")
        navigator.dock()
    else: 
        navigator.info("Docked already")

    # Inform the server that the rover has docked.
    send_message(f"setState,tb4,{RoverState.DOCKED.value}")

    # Set initial pose.
    initial_pose = navigator.getPoseStamped([0.0, 0.0], TurtleBot4Directions.NORTH)
    navigator.setInitialPose(initial_pose)

    # Wait for Nav2.
    navigator.waitUntilNav2Active()
    xcoord = COORDS_ABB[0]
    ycoord = COORDS_ABB[1]
    while True:
        try:
            # Set goal poses.
            # Positive X axis: on the front of the Turtlebot4
            # Positive Y axis: on the left-hand side of the Turtlebot4
            newx = input(f'Set X-coordinate (empty string to use previous value {xcoord}):')
            newy = input(f'Set Y-coordinate (empty string to use previous value {ycoord}):')
            xcoord = float(newx) if len(newx)>0 else xcoord
            ycoord = float(newy) if len(newy)>0 else ycoord 
            
            pose_abb = navigator.getPoseStamped([xcoord, ycoord], TurtleBot4Directions.SOUTH)
            pose_ur5 = navigator.getPoseStamped(COORDS_UR5, TurtleBot4Directions.NORTH)

            # Undock.
            navigator.undock()

            # Navigate to the ABB cobot.
            send_message(f"setState,tb4,{RoverState.MOVING.value}")
            navigator.startToPose(pose_abb)

            # Wait for the ABB to place the workpiece onto the rover.
            send_message(f"setState,tb4,{RoverState.ON_ABB.value}")
            while True:
                response = send_message("getState,abb")
                if response == RobotState.READY.value:
                    break
                time.sleep(1)

            # Navigate to the UR5 cobot.
            send_message(f"setState,tb4,{RoverState.MOVING.value}")
            navigator.startToPose(pose_ur5)

            # Wait for the UR5 robot to pick the workpiece from the rover.
            send_message(f"setState,tb4,{RoverState.ON_UR5.value}")
            while True:
                response = send_message("getState,ur5")
                if response == RobotState.READY.value:
                    break
                time.sleep(1)

            # Navigate to the loading station.
            send_message(f"setState,tb4,{RoverState.MOVING.value}")
            pose_end = navigator.getPoseStamped([-0.20, 0.00], TurtleBot4Directions.NORTH)
            navigator.startToPose(pose_end)
            if not navigator.getDockedStatus():
                navigator.dock()
            send_message(f"setState,tb4,{RoverState.DOCKED.value}")
        
        except KeyboardInterrupt:
            break

    rclpy.shutdown()


if __name__ == "__main__":
    main()
