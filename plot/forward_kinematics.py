from mpl_toolkits import mplot3d
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import sys

def dh_matrix(theta_deg, d, a, alpha_deg):
    alpha = np.pi*alpha_deg/180.0
    theta = np.pi*theta_deg/180.0
    return np.array([
        [np.cos(theta), -np.sin(theta)*np.cos(alpha),  np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
        [np.sin(theta),  np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
        [            0,                np.sin(alpha),                np.cos(alpha),               d],
        [            0,                            0,                            0,               1],
    ])

def leg(constant, joint):
    link1 = dh_matrix(joint[0]-90, 0, 0, -90)
    link2 = dh_matrix(joint[1]-90, 0, 0, 90)
    link3 = dh_matrix(joint[2], 0, -constant['thigh'], 0)
    link4 = dh_matrix(joint[3], 0, -constant['calf'], 0)
    link5 = dh_matrix(joint[4]+180, 0, 0, 90)
    link6 = dh_matrix(joint[5], 0, constant['ankle'], 0)

    link12 = np.matmul(link1, link2)
    link123 = np.matmul(link12, link3)
    link1234 = np.matmul(link123, link4)
    link12345 = np.matmul(link1234, link5)
    link123456 = np.matmul(link12345, link6)

    # knee_mat = link123
    # knee_pos = [knee_mat[0,3], knee_mat[1,3], knee_mat[2,3]]

    # ankle_mat = link1234
    # ankle_pos = [ankle_mat[0,3], ankle_mat[1,3], ankle_mat[2,3]]

    end_mat = link123456
    end_pos = [end_mat[0,3], end_mat[1,3], end_mat[2,3]]

    return end_pos

constant = {
    "thigh" : 1,
    "calf" : 1,
    "ankle" : 1,
}
max_length = constant['thigh'] + constant['calf'] + constant['ankle']

joints = [
    [0,0,0,0,0,0],
    [30,30,30,30,30,30],
    [40,40,40,40,40,40],
    [60,60,60,60,60,60],
    [90,90,90,90,90,90],
]

for joint in joints:
    print(leg(constant, joint))