import math
from constants import *
from scipy.optimize import minimize
import numpy as np


# Given the sizes (a, b, c) of the 3 sides of a triangle, returns the angle between a and b using the alKashi theorem.
def alKashi(a, b, c, sign=1):
    if a * b == 0:
        print("WARNING a or b is null in AlKashi")
        return 0
    # Note : to get the other altenative, simply change the sign of the return :
    return sign * math.acos(min(1, max(-1, (a**2 + b**2 - c**2) / (2 * a * b))))


# Computes the direct kinematics of a leg in the leg's frame
# Given the angles (theta1, theta2, theta3) of a limb with 3 rotational axes separated by the distances (l1, l2, l3),
# returns the destination point (x, y, z)
def computeDK(
    theta1,
    theta2,
    theta3,
    l1=constL1,
    l2=constL2,
    l3=constL3,
    use_rads=USE_RADS_INPUT,
    use_mm=USE_MM_OUTPUT,
):
    angle_unit = 1
    dist_unit = 1
    if not (use_rads):
        angle_unit = math.pi / 180.0
    if use_mm:
        dist_unit = 1000
    theta1 = THETA1_MOTOR_SIGN * theta1 * angle_unit
    theta2 = (THETA2_MOTOR_SIGN * theta2 - theta2Correction) * angle_unit
    theta3 = (THETA3_MOTOR_SIGN * theta3 - theta3Correction) * angle_unit
    # print(
    #     "corrected angles={}, {}, {}".format(
    #         theta1 * (1.0 / angle_unit),
    #         theta2 * (1.0 / angle_unit),
    #         theta3 * (1.0 / angle_unit),
    #     )
    # )

    planContribution = l1 + l2 * math.cos(theta2) + l3 * math.cos(theta2 + theta3)

    x = math.cos(theta1) * planContribution * dist_unit
    y = math.sin(theta1) * planContribution * dist_unit
    z = -(l2 * math.sin(theta2) + l3 * math.sin(theta2 + theta3)) * dist_unit

    return [x, y, z]


def computeDKDetailed(
    theta1,
    theta2,
    theta3,
    l1=constL1,
    l2=constL2,
    l3=constL3,
    use_rads=USE_RADS_INPUT,
    use_mm=USE_MM_OUTPUT,
):
    theta1_verif = theta1
    theta2_verif = theta2
    theta3_verif = theta3
    angle_unit = 1
    dist_unit = 1
    if not (use_rads):
        angle_unit = math.pi / 180.0
    if use_mm:
        dist_unit = 1000
    theta1 = THETA1_MOTOR_SIGN * theta1 * angle_unit
    theta2 = (THETA2_MOTOR_SIGN * theta2 - theta2Correction) * angle_unit
    theta3 = (THETA3_MOTOR_SIGN * theta3 - theta3Correction) * angle_unit

    print(
        "corrected angles={}, {}, {}".format(
            theta1 * (1.0 / angle_unit),
            theta2 * (1.0 / angle_unit),
            theta3 * (1.0 / angle_unit),
        )
    )

    planContribution = l1 + l2 * math.cos(theta2) + l3 * math.cos(theta2 + theta3)

    x = math.cos(theta1) * planContribution
    y = math.sin(theta1) * planContribution
    z = -(l2 * math.sin(theta2) + l3 * math.sin(theta2 + theta3))

    p0 = [0, 0, 0]
    p1 = [l1 * math.cos(theta1) * dist_unit, l1 * math.sin(theta1) * dist_unit, 0]
    p2 = [
        (l1 + l2 * math.cos(theta2)) * math.cos(theta1) * dist_unit,
        (l1 + l2 * math.cos(theta2)) * math.sin(theta1) * dist_unit,
        -l2 * math.sin(theta2) * dist_unit,
    ]
    p3 = [x * dist_unit, y * dist_unit, z * dist_unit]
    p3_verif = computeDK(
        theta1_verif, theta2_verif, theta3_verif, l1, l2, l3, use_rads, use_mm
    )
    if (p3[0] != p3_verif[0]) or (p3[1] != p3_verif[1]) or (p3[2] != p3_verif[2]):
        print(
            "ERROR: the DK function is broken!!! p3 = {}, p3_verif = {}".format(
                p3, p3_verif
            )
        )

    return [p0, p1, p2, p3]


# Computes the inverse kinematics of a leg in the leg's frame
# Given the destination point (x, y, z) of a limb with 3 rotational axes separated by the distances (l1, l2, l3),
# returns the angles to apply to the 3 axes
def computeIK(
    x,
    y,
    z,
    l1=constL1,
    l2=constL2,
    l3=constL3,
    verbose=False,
    use_rads=USE_RADS_OUTPUT,
    sign=DEFAULT_COMPUTE_IK_SIGN,
    use_mm=USE_MM_INPUT,
):
    dist_unit = 1
    if use_mm:
        dist_unit = 0.001
    x = x * dist_unit
    y = y * dist_unit
    z = z * dist_unit

    # theta1 is simply the angle of the leg in the X/Y plane. We have the first angle we wanted.
    if y == 0 and x == 0:
        # Taking care of this singularity (leg right on top of the first rotational axis)
        theta1 = 0
    else:
        theta1 = math.atan2(y, x)

    # Distance between the second motor and the projection of the end of the leg on the X/Y plane
    xp = math.sqrt(x * x + y * y) - l1
    # if xp < 0:
    #     print("Destination point too close")
    #     xp = 0

    # Distance between the second motor arm and the end of the leg
    d = math.sqrt(math.pow(xp, 2) + math.pow(z, 2))
    # if d > l2 + l3:
    #     print("Destination point too far away")
    #     d = l2 + l3

    # Knowing l2, l3 and d, theta1 and theta2 can be computed using the Al Kashi law
    # There are 2 solutions for most of the points, forcing a convention here
    theta2 = alKashi(l2, d, l3, sign=sign) - Z_DIRECTION * math.atan2(z, xp)
    theta3 = math.pi + alKashi(l2, l3, d, sign=sign)

    if use_rads:
        result = [
            angleRestrict(THETA1_MOTOR_SIGN * theta1, use_rads=use_rads),
            angleRestrict(
                THETA2_MOTOR_SIGN * (theta2 + theta2Correction), use_rads=use_rads
            ),
            angleRestrict(
                THETA3_MOTOR_SIGN * (theta3 + theta3Correction), use_rads=use_rads
            ),
        ]

    else:
        result = [
            angleRestrict(THETA1_MOTOR_SIGN * math.degrees(theta1), use_rads=use_rads),
            angleRestrict(
                THETA2_MOTOR_SIGN * (math.degrees(theta2) + theta2Correction),
                use_rads=use_rads,
            ),
            angleRestrict(
                THETA3_MOTOR_SIGN * (math.degrees(theta3) + theta3Correction),
                use_rads=use_rads,
            ),
        ]
    if verbose:
        print(
            "Asked IK for x={}, y={}, z={}\n, --> theta1={}, theta2={}, theta3={}".format(
                x,
                y,
                z,
                result[0],
                result[1],
                result[2],
            )
        )

    return result


# Computes the inverse kinematics of a leg in a frame colinear to the robot's frame (x points in front of the robot, y points to its left, z towards the sky)
# but whose (0,0) point is leg dependent, ie will match the leg's initial position.
# Given the destination point (x, y, z) of a limb with 3 rotational axes separated by the distances (l1, l2, l3),
# returns the angles to apply to the 3 axes
def computeIKOriented(x, y, z, legID, params, extra_theta=0, verbose=False):
    if verbose:
        print(
            "computeIKOriented for x {}, y {}, z {}, legID {}, extra_theta {}".format(
                x, y, z, legID, extra_theta
            )
        )
    if legID < 1 or legID > 6:
        raise RuntimeError("ERROR unknown legID '{}'".format(legID))

    theta = params.legAngles[legID - 1]

    theta = theta + extra_theta
    # Applying a rotation
    new_x, new_y, _ = rotaton_2D(x, y, z, -theta)

    return computeIK(
        new_x + params.initLeg[legID - 1][0],
        new_y + params.initLeg[legID - 1][1],
        z + params.z,
        verbose=verbose,
    )


def computeIKOrientedSimple(x, y, z, legID, extra_theta=0, verbose=False):
    if verbose:
        print(
            "computeIKOriented for x {}, y {}, z {}, legID {}, extra_theta {}".format(
                x, y, z, legID, extra_theta
            )
        )
    if legID == 2 or legID == 3:
        theta = 0
    elif legID == 4:
        theta = -math.pi / 2.0
    elif legID == 5 or legID == 6:
        theta = math.pi
    elif legID == 1:
        theta = math.pi / 2.0
    else:
        raise RuntimeError("ERROR unknown legID '{}'".format(legID))
    theta = theta + extra_theta
    # Applying a rotation
    new_x, new_y, _ = rotaton_2D(x, y, z, theta)

    return computeIK(
        new_x,
        new_y,
        z,
        verbose=verbose,
    )


# Computes the inverse kinematics of a leg in a frame colinear to the leg's frame (x points in front of the leg, y points to its left, z towards the sky)
# but whose (0,0) point matches the leg's initial position.
# Given the destination point (x, y, z) of a limb with 3 rotational axes separated by the distances (l1, l2, l3),
# returns the angles to apply to the 3 axes
def computeIKNotOriented(x, y, z, legID, params, verbose=False):
    if verbose:
        print("computeIKOriented for x {}, y {}, z {}, legID {}".format(x, y, z, legID))

    return computeIK(
        x + params.initLeg[legID - 1][0],
        y + params.initLeg[legID - 1][1],
        z + params.z,
        verbose=verbose,
    )


def computeIKNotOrientedRobotCentered(x, y, z, legID, params, verbose=False):
    if verbose:
        print(
            "computeIKNotOrientedRobotCentered for x {}, y {}, z {}, legID {}".format(
                x, y, z, legID
            )
        )
    leg_center_position = [
        1000 * LEG_CENTER_POS[legID - 1][0],
        1000 * LEG_CENTER_POS[legID - 1][1],
        1000 * LEG_CENTER_POS[legID - 1][2],
    ]

    x = x - leg_center_position[0]
    y = y - leg_center_position[1]
    z = z - leg_center_position[2]

    pos = rotaton_2D(x, y, z, -params.legAngles[legID - 1])

    return computeIK(
        pos[0],
        pos[1],
        pos[2],
        verbose=verbose,
    )


def rotaton_2D(x, y, z, theta):
    # Applying a rotation around the Z axis
    new_x = x * math.cos(theta) - y * math.sin(theta)
    new_y = x * math.sin(theta) + y * math.cos(theta)

    return [new_x, new_y, z]


def angleRestrict(angle, use_rads=False):
    if use_rads:
        return modulopi(angle)
    else:
        return modulo180(angle)


# Takes an angle that's between 0 and 360 and returns an angle that is between -180 and 180
def modulo180(angle):
    if -180 < angle < 180:
        return angle

    angle = angle % 360
    if angle > 180:
        return -360 + angle

    return angle


def modulopi(angle):
    if -math.pi < angle < math.pi:
        return angle

    angle = angle % (math.pi * 2)
    if angle > math.pi:
        return -math.pi * 2 + angle

    return angle


def trianglePoints(x, z, h, w):
    """
    Takes the geometric parameters of the triangle and returns the position of the 3 points of the triagles. Format : [[x1, y1, z1], [x2, y2, z2], [x3, y3, z3]]
    """
    return [[x, 0, h + z], [x, -w / 2, z], [x, w / 2, z]]


def triangle(x, z, h, w, t, legID, params, rotation):
    """
    Takes the geometric parameters of the triangle and the current time, gives the joint angles to draw the triangle with the tip of th leg. Format : [theta1, theta2, theta3]
    """

    points = trianglePoints(x, z, h, w)

    # Sélection de deux points
    P1 = np.array(points[(int(t)) % 3])
    P2 = np.array(points[(int(t) + 1) % 3])

    # Interpolation entre les deux points
    T = math.fmod(t, 1)
    pos = P2 * T + (1 - T) * P1

    # return computeIK(pos[0], pos[1], -pos[2])
    return computeIKOriented(pos[0], pos[1], pos[2], legID, params, extra_theta=rotation)
    # return [pos[0], pos[1], -pos[2]]


def circlePoints(x, z, r, N=16):
    """
    Takes the geometric parameters of the cercle and returns N points approximating the circle. Format : [[x1, y1, z1], [x2, y2, z2], [x3, y3, z3], etc]
    """

    points = []
    delta = math.pi * 2 / N
    for segment in range(N):
        points.append(
            [x, math.cos(delta * segment) * r, z + math.sin(delta * segment) * r]
        )

    return points


"""Autre façon de faire le cercle (à présenter)
            # y_circle = r * math.cos(2 * math.pi * (1 / duration) * sim.t)
            # z_circle = r * math.sin(2 * math.pi * (1 / duration) * sim.t)
            # alphas = kinematics.computeIK(x, y_circle, z_circle + z)
"""


def circle(x, z, r, t, duration):
    """
    Takes the geometric parameters of the circle and the current time, gives the joint angles to draw the circle with the tip of th leg. Format : [theta1, theta2, theta3]
    """

    N = 16
    points = circlePoints(x, z, r, N)

    # Sélection de deux points
    part = int(math.fmod(t, duration) * N / duration)
    P1 = np.array(points[part])
    P2 = np.array(points[(part + 1) % N])

    # Interpolation entre les deux points
    T = math.fmod(t, duration / N) / (duration / N)
    pos = P2 * T + (1 - T) * P1

    return computeIK(pos[0], pos[1], pos[2])


def segment(
    segment_x1, segment_y1, segment_z1, segment_x2, segment_y2, segment_z2, t, duration
):
    half_duration = 0.5 * duration
    t_lin = math.fmod(t, half_duration)
    if math.fmod(t, duration) < half_duration:
        x = segment_x1 + (t_lin / half_duration) * (segment_x2 - segment_x1)
        y = segment_y1 + (t_lin / half_duration) * (segment_y2 - segment_y1)
        z = segment_z1 + (t_lin / half_duration) * (segment_z2 - segment_z1)
    else:
        x = segment_x2 + (t_lin / half_duration) * (segment_x1 - segment_x2)
        y = segment_y2 + (t_lin / half_duration) * (segment_y1 - segment_y2)
        z = segment_z2 + (t_lin / half_duration) * (segment_z1 - segment_z2)

    return computeIK(x, y, z)

def rotate_hexapod(rotation_angle, radius, speed):
    
    # Angle de rotation pour chaque groupe de pattes
    leg_ids = [0, 1, 2, 3, 4, 5]  # Liste des identifiants des pattes
    rotation_angle_rad = math.radians(rotation_angle)
    rotation_angle_group = rotation_angle_rad / 2
    # Distance entre le centre et les pattes
    leg_distance = radius * math.sin(rotation_angle_group)   #rajouter offset = Longueur des pattes                                                      #Lancer la fonction ComputeIk en appelant triangle()
    # Boucle principale pour le mouvement de rotation
    t = 0
  
    # Boucle principale pour le mouvement de rotation

    leg_ids = [0, 1, 2, 3, 4, 5]  # Liste des identifiants des pattes
    while True:
    # Rotation du groupe avant dans un sens
     for leg_id in leg_ids[:3]:  # Utilisation de la liste des identifiants des pattes
        # Calcul des coordonnées du point du triangle à un temps donné
        x = radius * math.cos(speed * t) + leg_id * (2 * math.pi / 3) #+offset
        y = leg_distance * math.sin(rotation_angle_group) #+offset
        # Calcul des angles des moteurs pour atteindre les coordonnées
        angles = triangle(x, 0, y, leg_distance, t , leg_id, 1)
        # Déplacer la patte en utilisant les angles calculés
        angles = computeIK(x, y, 0)
    
    # Rotation du groupe arrière dans l'autre sens
     for leg_id in leg_ids[3:]:  # Utilisation de la liste des identifiants des pattes
        # Calcul des coordonnées du point du triangle à un temps donné
        x = radius * math.cos(speed * t) + leg_id * (2 * math.pi / 3) #+offset)
        y = -leg_distance * math.sin(rotation_angle_group) # + offdet
        # Calcul des angles des moteurs pour atteindre les coordonnées
        angles = triangle(x, 0, y, leg_distance, t , leg_id, -1)
        # Déplacer la patte en utilisant les angles calculés
        angles = computeIK(x, y, 0)
    
    # Mettre à jour le temps pour le mouvement fluide
    t += 0.01
    # Mettre en pause pour ralentir le mouvement
    time.sleep(0.01)


def main():
    print(
        "0, -90, -90 --> ", computeDK(0, -90, -90, l1=constL1, l2=constL2, l3=constL3)
    )
    print("0, 0, 0 --> ", computeDK(0, 0, 0, l1=constL1, l2=constL2, l3=constL3))
    print("90, 0, 0 --> ", computeDK(90, 0, 0, l1=constL1, l2=constL2, l3=constL3))
    print(
        "180, -30.501, -67.819 --> ",
        computeDK(180, -30.501, -67.819, l1=constL1, l2=constL2, l3=constL3),
    )
    print(
        "0, -30.645, 38.501 --> ",
        computeDK(0, -30.645, 38.501, l1=constL1, l2=constL2, l3=constL3),
    )


if __name__ == "__main__":
    main()
