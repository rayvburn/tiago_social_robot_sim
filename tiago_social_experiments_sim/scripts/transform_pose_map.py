#
# Applies a transform given by --tf argument (optional, default is appropriate for `012` lab case) to the pose given
# by positional arguments.
# By default, transforms a pose from the "SLAM-generated map" to the pose expressed in the "ideal map"
#
# To transform from the "simulator" spawning pose to the "SLAM-based map" of `012`:
#
#     python3 transform_pose_map.py 1.85 4.50 -1.06 --inverse
#
# where given express the pose in the simulation world.
# To transform from the "SLAM-generated" `012` map to the simulator world:
#
#     python3 transform_pose_map.py 3.692 8.356 -1.257
#

import argparse
import json
import numpy as np
import math

from typing import List


# https://stackoverflow.com/a/56207565
def quaternion_to_euler(w, x, y, z):
    ysqr = y * y

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + ysqr)
    X = np.arctan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = np.where(t2>+1.0,+1.0,t2)
    #t2 = +1.0 if t2 > +1.0 else t2

    t2 = np.where(t2<-1.0, -1.0, t2)
    #t2 = -1.0 if t2 < -1.0 else t2
    Y = np.arcsin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (ysqr + z * z)
    Z = np.arctan2(t3, t4)

    return X, Y, Z


# https://automaticaddison.com/how-to-convert-euler-angles-to-quaternions-using-python/
def quaternion_from_euler(roll, pitch, yaw):
    """
    Convert an Euler angle to a quaternion.

    Input
    :param roll: The roll (rotation around x-axis) angle in radians.
    :param pitch: The pitch (rotation around y-axis) angle in radians.
    :param yaw: The yaw (rotation around z-axis) angle in radians.

    Output
    :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
    """
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

    return [qx, qy, qz, qw]


# Some of the functions below were copied from srpb_evaluation/scripts/rewind_experiment_utils.py
def rotz(theta):
    return np.matrix([[math.cos(theta), -math.sin(theta), 0 ],
                      [math.sin(theta),  math.cos(theta), 0 ],
                      [0           , 0            , 1 ]])


def transform_pose(p_in: np.array, R: np.matrix, t: np.array, inverse: bool) -> List[float]:
    if not inverse:
        T = [
            [R[0,0], R[0,1], R[0,2], t[0]],
            [R[1,0], R[1,1], R[1,2], t[1]],
            [R[2,0], R[2,1], R[2,2], t[2]],
            [0,      0,      0,      1]
        ]
    else:
        Rinv = np.linalg.inv(R)
        dinv = np.matmul(-Rinv, t)
        T = [
            [Rinv[0,0], Rinv[0,1], Rinv[0,2], dinv[0,0]],
            [Rinv[1,0], Rinv[1,1], Rinv[1,2], dinv[0,1]],
            [Rinv[2,0], Rinv[2,1], Rinv[2,2], dinv[0,2]],
            [        0,         0,         0,         1]
        ]

    p_inT = np.array([[p_in[0]], [p_in[1]], [p_in[2]], [1.0]])
    p_out = np.matmul(T, p_inT)
    return p_out[0:2]


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("-tf", "--tf", default=[-0.1920, -3.9164, +0.1620],
                        help="Custom transform, e.g., from the SLAM-generated map to the Gazebo world center")
    parser.add_argument("-inv", "--inverse", default=False, action='store_true',
                        help="Whether to apply the inverse transform")
    parser.add_argument('input', metavar='N', type=float, nargs='+', help='(x y yaw) or (x y qx qy qz qw)')
    args = parser.parse_args()

    # total arguments
    n = len(args.input)

    if n == 3:
        x = float(args.input[0])
        y = float(args.input[1])
        yaw = float(args.input[2])
        [qx, qy, qz, qw] = quaternion_from_euler(roll=0.0, pitch=0.0, yaw=yaw)
        print(f"Input:  X={x}, Y={y}, Yaw={yaw}, converted to Quaternion: (Qx={qx}, Qy={qy}, Qz={qz}, Qw={qw})")

    elif n == 6:
        x = args.input[0]
        y = args.input[1]
        qx = args.input[2]
        qy = args.input[3]
        qz = args.input[4]
        qw = args.input[5]
        _, _, yaw = quaternion_to_euler(x=qx, y=qy, z=qz, w=qw)
        print(f"Input:  X={x}, Y={y}, Qx={qx}, Qy={qy}, Qz={qz}, Qw={qw}, converted to Yaw={yaw}")

    else:
        print(f"Invalid number of arguments passed. Got {n-1}")
        exit(2)

    # tf is a string when command line argument is given
    if isinstance(args.tf, str):
        tf = json.loads(args.tf)
    else:
        tf = args.tf

    print(f"Tf:     X={tf[0]}, Y={tf[1]}, Yaw={tf[2]}")

    R = rotz(tf[-1])
    t = np.array([tf[0], tf[1], 0.0])
    p_in = np.array([x, y, yaw])
    p_out = transform_pose(p_in=p_in, R=R, t=t, inverse=args.inverse)
    yaw_out = tf[-1] + yaw
    # normalize angle
    yaw_out = math.atan2(math.sin(yaw_out), math.cos(yaw_out))
    [qx, qy, qz, qw] = quaternion_from_euler(roll=0.0, pitch=0.0, yaw=yaw_out)
    print(f"Output: X={p_out[0]}, Y={p_out[1]}, Yaw={yaw_out} (Qx={qx}, Qy={qy}, Qz={qz}, Qw={qw})")
