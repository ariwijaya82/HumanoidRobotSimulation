import numpy as np

def dh_matrix(theta_deg, d, a, alpha_deg):
    alpha = np.pi*alpha_deg/180.0
    theta = np.pi*theta_deg/180.0
    return np.array([
        [np.cos(theta), -np.sin(theta)*np.cos(alpha),  np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
        [np.sin(theta),  np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
        [            0,                np.sin(alpha),                np.cos(alpha),               d],
        [            0,                            0,                            0,               1],
    ])

def joint_rad2deg(joint):
    result = []
    for j in joint:
        result.append(j * 180.0 / np.pi)

    return result

def compute_fk(joint, dir=None):
    # joint in degree
    # joint = [hip_yaw, hip_roll, hip_pitch, knee, ankle_pitch, ankle_roll]
    THIGH_LENGTH = 199.0
    CALF_LENGTH = 195.0
    ANKLE_LENGTH = 59.7
    LEG_LENGTH = THIGH_LENGTH + CALF_LENGTH + ANKLE_LENGTH

    if dir is None:
        dir = [1,1,1,1,1,1]

    link1 = dh_matrix(dir[0]*joint[0]-90, 0, 0, -90)
    link2 = dh_matrix(dir[1]*joint[1]-90, 0, 0, 90)
    link3 = dh_matrix(dir[2]*joint[2], 0, -THIGH_LENGTH, 0)
    link4 = dh_matrix(dir[3]*joint[3], 0, -CALF_LENGTH, 0)
    link5 = dh_matrix(dir[4]*joint[4]+180, 0, 0, 90)
    link6 = dh_matrix(dir[5]*joint[5], 0, ANKLE_LENGTH, 0)

    link12 = np.matmul(link1, link2)
    link123 = np.matmul(link12, link3)
    link1234 = np.matmul(link123, link4)
    link12345 = np.matmul(link1234, link5)
    link123456 = np.matmul(link12345, link6)

    end_mat = link123456
    end_pos = [
        round(end_mat[0,3], 2), 
        round(end_mat[1,3], 2), 
        round(end_mat[2,3], 2) + LEG_LENGTH
    ]

    return end_pos

if __name__ == "__main__":
    joint = [-0.0, 1.11, -9.85, 19.9, -8.95, -1.11]
    print(compute_fk(joint))