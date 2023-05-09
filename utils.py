from kinematics import computeIKOriented, computeIK, computeDKDetailed, computeDK, rotaton_2D
from constants import *
import time


class Parameters:
    def __init__(
        self,
        freq=50,
        speed=1,
        z=-60,
        travelDistancePerStep=80,
        lateralDistance=90,
        frontDistance=87,
        frontStart=32,
        method="constantSpeed",
        maxAccel=6000,
        maxSpeed=500,
        startFromInit=True,
        endToInit=False,
        up=False,
        down=False,
        left=False,
        right=False,
        walkMagnitudeX=0,
        walkMagnitudeY=0,
        activateWalk=False,
    ):
        self.freq = freq
        self.speed = speed
        self.z = z
        self.travelDistancePerStep = travelDistancePerStep
        self.lateralDistance = lateralDistance
        self.frontDistance = frontDistance
        self.frontStart = frontStart
        self.method = method
        self.maxAccel = maxAccel
        self.maxSpeed = maxSpeed
        self.startFromInit = startFromInit
        self.endToInit = endToInit
        self.up = up
        self.down = down
        self.left = left
        self.right = right
        self.walkMagnitudeX = walkMagnitudeX
        self.walkMagnitudeY = walkMagnitudeY
        self.activateWalk = activateWalk
        # Angle between the X axis of the leg and the X axis of the robot for each leg
        self.legAngles = LEG_ANGLES
        # Initial leg positions in the coordinates of each leg.
        self.initLeg = []  # INIT_LEG_POSITIONS

        self.initLeg.append([self.lateralDistance, 0])
        self.initLeg.append([self.lateralDistance, 0])
        self.initLeg.append([self.lateralDistance, 0])
        self.initLeg.append([self.lateralDistance, 0])
        self.initLeg.append([self.lateralDistance, 0])
        self.initLeg.append([self.lateralDistance, 0])


# Classes used to use dxl_io directly and by-pass Pypot's Robot class
class SimpleMotor:
    def __init__(self, id):
        self.id = id
        self.present_position = 0
        self.goal_position = 0
        self.smooth_start_position = 0
        self.smooth_final_position = 0

    def __repr__(self):
        return "id {}, goal_position {}, present_position {}".format(
            self.id, self.goal_position, self.present_position
        )


class SimpleRobot:
    def __init__(self, dxl_io):
        self.dxl_io = dxl_io
        self.legs = {
            1: [SimpleMotor(11), SimpleMotor(12), SimpleMotor(13)],
            2: [SimpleMotor(21), SimpleMotor(22), SimpleMotor(23)],
            3: [SimpleMotor(31), SimpleMotor(32), SimpleMotor(33)],
            4: [SimpleMotor(41), SimpleMotor(42), SimpleMotor(43)],
            5: [SimpleMotor(51), SimpleMotor(52), SimpleMotor(53)],
            6: [SimpleMotor(61), SimpleMotor(62), SimpleMotor(63)],
        }
        self.delay_after_write = 0.01
        self.params = None

    def __repr__(self):
        output = "##### Robot #####\n"
        for k, v in self.legs.items():
            output += "# Leg{}: [{:.2f}] [{:.2f}] [{:.2f}]\n".format(
                k, v[0], v[1], v[2]
            )
        return output

    def print_dk(self):
        output = "##### Robot #####\n"
        output += "/!\ The IK check only works for legs whose IDs are not in LIST_OF_INVERTED_IDS\n"

        for k, v in self.legs.items():
            p0, p1, p2, p3 = computeDKDetailed(
                v[0].present_position,
                v[1].present_position,
                v[2].present_position,
            )
            output += "# Leg{}. Angles: [{:.2f}] [{:.2f}] [{:.2f}]. DK P3: x={:.2f}, y={:.2f}, z={:.2f} DK P2: x={:.2f}, y={:.2f}, z={:.2f}".format(
                k,
                v[0].present_position,
                v[1].present_position,
                v[2].present_position,
                p3[0],
                p3[1],
                p3[2],
                p2[0],
                p2[1],
                p2[2],
            )
            # This verification doens't work well because the low level will invert the sign of some motors if we ue the new "LIST_OF_INVERTED_IDS"
            # Testing if the IK is coherent with the DK
            angles_from_ik = computeIK(p3[0], p3[1], p3[2])
            output += f"\n IK({p3[0]:.2f}, {p3[1]:.2f}, {p3[2]:.2f}) --> theta1={angles_from_ik[0]:.2f} (real:{v[0].present_position:.2f})"
            output += (
                f", theta2={angles_from_ik[1]:.2f} (real:{v[1].present_position:.2f})"
            )
            output += (
                f", theta3={angles_from_ik[2]:.2f} (real:{v[2].present_position:.2f})\n"
            )

        print(output)

    def init(self):
        """Sets the goal_position to the present_position"""
        self.tick_read(verbose=True)
        for k, v in self.legs.items():
            v[0].goal_position = v[0].present_position
            v[1].goal_position = v[1].present_position
            v[2].goal_position = v[2].present_position

    def motors(self):
        list_of_motors = []
        for k, v in self.legs.items():
            list_of_motors.append(v[0])
            list_of_motors.append(v[1])
            list_of_motors.append(v[2])
        return list_of_motors

    def enable_torque(self, list_of_ids=None):
        to_send = []
        if list_of_ids == None:
            for k, v in self.legs.items():
                to_send.append(v[0].id)
                to_send.append(v[1].id)
                to_send.append(v[2].id)
        else:
            to_send = list_of_ids
        self.dxl_io.enable_torque(to_send)
        time.sleep(self.delay_after_write)

    def disable_torque(self, list_of_ids=None):
        to_send = []
        if list_of_ids == None:
            for k, v in self.legs.items():
                to_send.append(v[0].id)
                to_send.append(v[1].id)
                to_send.append(v[2].id)
        else:
            to_send = list_of_ids
        print("Disabling torques...")
        self.dxl_io.disable_torque(to_send)
        print("waiting...")
        time.sleep(self.delay_after_write)
        print(f"Torques disabled for ids: {list_of_ids}")

    def tick_read(self, verbose=False):
        # Creating a list for a read request
        to_read = []
        for k, v in self.legs.items():
            to_read.append(v[0].id)
            to_read.append(v[1].id)
            to_read.append(v[2].id)

        if verbose:
            print("Sending read command '{}'".format(to_read))
        result = self.dxl_io.get_present_position(to_read)
        for i in range(len(to_read)):
            id = to_read[i]
            value = result[i]
            # TODO get a getter by ID instead of this loop spamming...
            for m in self.motors():
                factor = 1.0
                if m.id == id:
                    if id in LIST_OF_INVERTED_IDS:
                        factor = -1.0
                    m.present_position = value * factor
        if verbose:
            print("Read tick done")

    def tick_write(self, verbose=False):
        # Creating a dict for a write request
        to_write = {}
        for k, v in self.legs.items():
            for i in range(3):
                if MOTOR_TYPE == AX12:
                    # Not sending values out of motor range
                    if v[i].goal_position <= -150 or v[i].goal_position >= 150:
                        continue
                factor = 1.0
                if v[i].id in LIST_OF_INVERTED_IDS:
                    factor = -1.0
                to_write[v[i].id] = v[i].goal_position * factor

        if verbose:
            print("Sending write command '{}'".format(to_write))
        self.dxl_io.set_goal_position(to_write)
        if verbose:
            print("Write tick done")
        time.sleep(self.delay_after_write)

    def tick_read_and_write(self, verbose=False):
        # Creating a list for a read request and a dict for a write request
        self.tick_read()
        self.tick_write()
        if verbose:
            print("IO tick done")

    def smooth_tick_read_and_write(self, delay, verbose=False):
        # Reads the current state of the robot and applies the write positions smoothly over 'time'
        self.tick_read()
        # Setting the start and end positions
        t0 = time.time()
        for m in self.motors():
            m.smooth_start_position = m.present_position
            m.smooth_final_position = m.goal_position
            if verbose:
                print(
                    "m.smooth_start_position {}, m.smooth_final_position {}".format(
                        m.smooth_start_position,
                        m.smooth_final_position,
                    )
                )
        t = time.time() - t0
        while t < delay:
            for m in self.motors():
                m.goal_position = (t / delay) * (
                    m.smooth_final_position - m.smooth_start_position
                ) + m.smooth_start_position
            self.tick_write(verbose=verbose)
            t = time.time() - t0
        for m in self.motors():
            m.goal_position = m.smooth_final_position
        self.tick_write(verbose=verbose)
        self.tick_read()
        self.print_dk()
        if verbose:
            print("IO smooth tick done")


# Class used to simulate the robot in PyBullet's (actually onshape_to_robot) environnment
class SimpleRobotSimulation:
    def __init__(self, sim):
        self.sim = sim  # e.g onshape_to_robot.simulation.Simulation("phantomx_description/urdf/phantomx.urdf", gui=True, panels=True, useUrdfInertia=False)
        self.legs = {
            1: [
                SimpleMotor("j_c1_rf"),
                SimpleMotor("j_thigh_rf"),
                SimpleMotor("j_tibia_rf"),
            ],
            6: [
                SimpleMotor("j_c1_rm"),
                SimpleMotor("j_thigh_rm"),
                SimpleMotor("j_tibia_rm"),
            ],
            5: [
                SimpleMotor("j_c1_rr"),
                SimpleMotor("j_thigh_rr"),
                SimpleMotor("j_tibia_rr"),
            ],
            2: [
                SimpleMotor("j_c1_lf"),
                SimpleMotor("j_thigh_lf"),
                SimpleMotor("j_tibia_lf"),
            ],
            3: [
                SimpleMotor("j_c1_lm"),
                SimpleMotor("j_thigh_lm"),
                SimpleMotor("j_tibia_lm"),
            ],
            4: [
                SimpleMotor("j_c1_lr"),
                SimpleMotor("j_thigh_lr"),
                SimpleMotor("j_tibia_lr"),
            ],
        }
        self.delay_after_write = 0.00
        self.drawOn = True
        self.params = None
        self.centerCamera = False

    def __repr__(self):
        output = "##### Robot #####\n"
        for k, v in self.legs.items():
            output += "# Leg{}: [{}] [{}] [{}]\n".format(k, v[0], v[1], v[2])
        return output

    def print_dk(self):
        output = "##### Robot #####\n"

        for k, v in self.legs.items():
            p0, p1, p2, p3 = computeDKDetailed(
                v[0].present_position,
                v[1].present_position,
                v[2].present_position,
            )
            output += "# Leg{}. Angles: [{:.2f}] [{:.2f}] [{:.2f}]. DK P3: x={:.2f}, y={:.2f}, z={:.2f} DK P2: x={:.2f}, y={:.2f}, z={:.2f}\n".format(
                k,
                v[0].present_position,
                v[1].present_position,
                v[2].present_position,
                p3[0],
                p3[1],
                p3[2],
                p2[0],
                p2[1],
                p2[2],
            )
        print(output)

    def init(self):
        """Sets the goal_position to the present_position"""
        self.tick_read(verbose=True)
        for k, v in self.legs.items():
            v[0].goal_position = v[0].present_position
            v[1].goal_position = v[1].present_position
            v[2].goal_position = v[2].present_position

    def motors(self):
        list_of_motors = []
        for k, v in self.legs.items():
            list_of_motors.append(v[0])
            list_of_motors.append(v[1])
            list_of_motors.append(v[2])
        return list_of_motors

    def enable_torque(self, list_of_ids=None):
        if list_of_ids == None:
            # Enabling torque for all motors (it's weird I know)
            self.sim.maxTorques = {}
        else:
            new_torques = {}
            for k, v in self.sim.maxTorques.items():
                if not (k in list_of_ids):
                    # Copying previous IDs and removing the ones that need their torque reactivated
                    new_torques[k] = v
            self.sim.maxTorques = new_torques

    def disable_torque(self, list_of_ids=None):
        if list_of_ids == None:
            # Disabling torque for all motors
            for m in self.motors():
                self.sim.maxTorques[m.id] = 0
        else:
            new_torques = self.sim.maxTorques
            for id in list_of_ids:
                if not (id in new_torques):
                    # Adding new IDs and avoiding duplicates
                    new_torques[id] = 0
            self.sim.maxTorques = new_torques

    def tick_read(self, verbose=False):
        # Read and write are not separated for now in the simulator
        self.tick_read_and_write()

    def tick_write(self, verbose=False):
        # Read and write are not separated for now in the simulator
        self.tick_read_and_write()

    def tick_read_and_write(self, verbose=False):
        # Creating a list for a read request and a dict for a write request
        to_write = {}
        for k, v in self.legs.items():
            for i in range(3):
                to_write[v[i].id] = v[i].goal_position

        if verbose:
            print("Sending write command '{}'".format(to_write))
        state = self.sim.setJoints(to_write)
        for m in self.motors():
            id = m.id
            position = state[id][
                0
            ]  # contains [position, speed, (3 forces and 3 torques)
            m.present_position = position
        if self.drawOn:
            self.drawLegTips()

        if verbose:
            print("IO tick done")

    def smooth_tick_read_and_write(self, delay, verbose=False):
        # Reads the current state of the robot and applies the write positions smoothly over 'time'
        self.tick_read()
        # Setting the start and end positions
        t0 = time.time()
        for m in self.motors():
            m.smooth_start_position = m.present_position
            m.smooth_final_position = m.goal_position
            if verbose:
                print(
                    "m.smooth_start_position {}, m.smooth_final_position {}".format(
                        m.smooth_start_position,
                        m.smooth_final_position,
                    )
                )
        t = time.time() - t0
        while t < delay:
            for m in self.motors():
                m.goal_position = (t / delay) * (
                    m.smooth_final_position - m.smooth_start_position
                ) + m.smooth_start_position
            self.tick_write(verbose=verbose)
            t = time.time() - t0
            # Blocking call has to tick the simulation here
            self.sim.tick()
        for m in self.motors():
            m.goal_position = m.smooth_final_position
        self.tick_write(verbose=verbose)
        if verbose:
            print("IO smooth tick done")

    def drawLegTips(self, duration=2):
        # Printing the tip of each leg
        robot_pose = (
            self.sim.getRobotPose()
        )  # (tuple(3), tuple(3)) -- (x,y,z), (roll, pitch, yaw)
        yaw = robot_pose[1][2]
        for i in range(6):
            motors = self.legs[i + 1]
            pos = computeDK(
                motors[0].present_position,
                motors[1].present_position,
                motors[2].present_position,
                use_rads=True,
            )
            pos = rotaton_2D(
                pos[0],
                pos[1],
                pos[2],
                self.params.legAngles[i] + yaw,
            )
            leg_center_position = rotaton_2D(
                LEG_CENTER_POS[i][0],
                LEG_CENTER_POS[i][1],
                LEG_CENTER_POS[i][2],
                yaw,
            )
            pos[0] += robot_pose[0][0] + leg_center_position[0]
            pos[1] += robot_pose[0][1] + leg_center_position[1]
            pos[2] += robot_pose[0][2] + leg_center_position[2]
            self.sim.addDebugPosition(pos, duration=duration)

    def tickSim(self):
        if self.centerCamera:
            self.centerCameraOnRobot()
        self.sim.tick()

    def centerCameraOnRobot(self):
        robot_pose = (
            self.sim.getRobotPose()
        )  # (tuple(3), tuple(3)) -- (x,y,z), (roll, pitch, yaw)
        self.sim.lookAt(robot_pose[0])


def setAnglesToLeg(angles, leg, verbose=False):
    i = 0
    for m in leg:
        if verbose:
            print("Setting angle {} to id {}".format(angles[i], m.id))
        m.goal_position = angles[i]
        i = i + 1


def setPositionToLeg(x, y, z, leg):
    angles = computeIK(x, y, z)
    setAnglesToLeg(angles, leg)


def setPositionToRobot(x, y, z, robot, params, extra_theta=0):
    angles = []
    for i in range(1, 7):
        # For the robot to move forward, the legs have to move backwards, hence the "-"
        angles.append(computeIKOriented(-x, -y, -z, i, params, extra_theta=extra_theta))

    for i in range(1, 7):
        setAnglesToLeg(angles[i - 1], robot.legs[i], verbose=False)


def setPositionToTripod1(x, y, z, robot, params):
    angles = []

    # For the robot to move forward, the legs have to move backwards, hence the "-"
    angles.append(computeIKOriented(-x, -y, -z, 1, params))
    angles.append(computeIKOriented(-x, -y, -z, 3, params))
    angles.append(computeIKOriented(-x, -y, -z, 5, params))

    setAnglesToLeg(angles[0], robot.legs[1])
    setAnglesToLeg(angles[1], robot.legs[3])
    setAnglesToLeg(angles[2], robot.legs[5])


def setPositionToTripod2(x, y, z, robot, params):
    angles = []

    # For the robot to move forward, the legs have to move backwards, hence the "-"
    angles.append(computeIKOriented(-x, -y, -z, 4, params))
    angles.append(computeIKOriented(-x, -y, -z, 2, params))
    angles.append(computeIKOriented(-x, -y, -z, 6, params))

    setAnglesToLeg(angles[0], robot.legs[4])
    setAnglesToLeg(angles[1], robot.legs[2])
    setAnglesToLeg(angles[2], robot.legs[6])


def printAllMotors(robot):
    for m in robot.motors:
        print("ID '{}' position '{}'".format(m.id, m.present_position))


def allMotorsCompliantAndPrint(robot):
    for m in robot.motors:
        print("ID '{}' position '{}'".format(m.id, m.present_position))
        m.compliant = True


def allMotorsCompliantAndPrintForEver(robot):
    while True:
        time.sleep(0.1)
        for m in robot.motors:
            print("ID '{}' position '{}'".format(m.id, m.present_position))
            m.compliant = True


def allMotorsNotCompliant(robot):
    for m in robot.motors:
        m.compliant = False


def allMotorsCompliant(robot):
    for m in robot.motors:
        m.compliant = True


def allMotorsToPresentPosition(robot):
    for m in robot.motors:
        print("Setting motor {} to angle {}".format(m.id, m.present_position))
        m.goal_position = m.present_position