import utils
from utils import SimpleRobotSimulation, Parameters, setPositionToRobot
import sys
from signal import signal, SIGINT
import traceback
import pybullet as p
from onshape_to_robot.simulation import Simulation


def main():
    robotPath = "phantomx_description/urdf/phantomx.urdf"
    sim = Simulation(robotPath, gui=True, panels=True, useUrdfInertia=False)
    
    # Try to play with the friction parameters of the simulator
    # sim.setFloorFrictions(lateral=0, spinning=0, rolling=0)
    
    # Setting the robot's initial position
    sim.setRobotPose([0, 0, 0.5], [0, 0, 0, 1])

    robot = SimpleRobotSimulation(sim)
    params = Parameters(
        freq=50,
        speed=1,
        z=-110,
        travelDistancePerStep=80,
        lateralDistance=130,
        frontDistance=87,
        frontStart=32,
        method="minJerk",
    )
    robot.params = params
    robot.centerCamera = True
    robot.init()
    robot.enable_torque()
    for m in robot.motors():
        print(m)
    # Defining the shutdown function here so it has visibility over the robot variable
    def shutdown(signal_received, frame):
        # Handle any cleanup here
        print("SIGINT or CTRL-C detected. Setting motors to compliant and exiting")
        robot.disable_torque()
        print("Done ticking. Exiting.")
        sys.exit()
        # Brutal exit
        # os._exit(1)

    # Tell Python to run the shutdown() function when SIGINT is recieved
    signal(SIGINT, shutdown)
    try:
        print("Setting initial position")
        
        setPositionToRobot(0, 0, 0, robot, params)
        robot.smooth_tick_read_and_write(3, verbose=False)
        print("Init position reached")
        # You can create a graphical user interface if you want
        # ui = display.RobotUI(robot, params)
        keep_going = True
        while keep_going:
            # keep_going = ui.tick()

            # Getting the robot position and asking the camera to follow
            robot_pose = (
                sim.getRobotPose()
            )  # (tuple(3), tuple(3)) -- (x,y,z), (roll, pitch, yaw)
            yaw = robot_pose[1][2]
            sim.lookAt(robot_pose[0])

            # Ticking the simulation
            sim.tick()
        return
    except Exception as e:
        track = traceback.format_exc()
        print(track)
    finally:
        shutdown(None, None)


main()