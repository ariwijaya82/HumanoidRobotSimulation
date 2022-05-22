import numpy as np
import math
import sys

class Matrix:
    def __init__(self):
        self.m = np.array([
            [1.0,0.0,0.0,0.0],
            [0.0,1.0,0.0,0.0],
            [0.0,0.0,1.0,0.0],
            [0.0,0.0,0.0,1.0],
        ])

    def SetTransform(self, point, angle):
        Cx = math.cos(angle[0] * math.pi / 180.0)
        Cy = math.cos(angle[1] * math.pi / 180.0)
        Cz = math.cos(angle[2] * math.pi / 180.0)
        Sx = math.sin(angle[0] * math.pi / 180.0)
        Sy = math.sin(angle[1] * math.pi / 180.0)
        Sz = math.sin(angle[2] * math.pi / 180.0)

        self.m[0,0] = Cz * Cy
        self.m[0,1] = Cz * Sy * Sx - Sz * Cx
        self.m[0,2] = Cz * Sy * Cx + Sz * Sx
        self.m[0,3] = point[0]
        self.m[1,0] = Sz * Cy
        self.m[1,1] = Sz * Sy * Sx + Cz * Cx
        self.m[1,2] = Sz * Sy * Sx - Cz * Sx
        self.m[1,3] = point[1]
        self.m[2,0] = -Sy
        self.m[2,1] = Cy * Sx
        self.m[2,2] = Cy * Cx
        self.m[2,3] = point[2]

    def Inverse(self):
        try:
            self.m = np.linalg.inv(self.m)
        except:
            print("Error inverse matrix")
            sys.exit()

    def Copy(self, mat):
        for i in range(4):
            for j in range(4):
                self.m[i,j] = mat.m[i,j]

    def __mul__(self, mat):
        result = Matrix()
        result.m = np.matmul(self.m, mat.m)
        return result

def compute_ik(x,y,z):
    THIGH_LENGTH = 199.0
    CALF_LENGTH = 195.0
    ANKLE_LENGTH = 59.7
    LEG_LENGTH = THIGH_LENGTH + CALF_LENGTH + ANKLE_LENGTH

    result = [0,0,0,0,0,0]

    Tad = Matrix() 
    Tda = Matrix() 
    Tcd = Matrix() 
    Tdc = Matrix() 
    Tac = Matrix()

    Tad.SetTransform(
        [x, y, z - LEG_LENGTH],
        [0,0,0]
    )

    X = x + Tad.m[0,2] * ANKLE_LENGTH
    Y = y + Tad.m[1,2] * ANKLE_LENGTH
    Z = (z - LEG_LENGTH) + Tad.m[2,2] * ANKLE_LENGTH

    _Rac = math.sqrt(X**2 + Y**2 + Z**2)
    calc = (_Rac * _Rac - THIGH_LENGTH * THIGH_LENGTH - CALF_LENGTH * CALF_LENGTH) / (2 * THIGH_LENGTH * CALF_LENGTH)
    if calc > 1.0: 
        print("Error in calculation triangle _Rac")
        return False
    _Acos = math.acos(
        (_Rac * _Rac - THIGH_LENGTH * THIGH_LENGTH - CALF_LENGTH * CALF_LENGTH) /
        (2 * THIGH_LENGTH * CALF_LENGTH))
    result[3] = _Acos # knee

    Tda.Copy(Tad)
    Tda.Inverse()

    _k = math.sqrt(Tda.m[1,3]**2 + Tda.m[2,3]**2)
    _l = math.sqrt(Tda.m[1,3]**2 + (Tda.m[2,3]-ANKLE_LENGTH)**2)
    _m = (_k * _k - _l * _l - ANKLE_LENGTH * ANKLE_LENGTH) / (2 * _l * ANKLE_LENGTH)
    if (_m > 1.0):
        _m = 1.0
    elif (_m < -1.0):
        _m = -1.0
    _Acos = math.acos(_m)
    if (math.isnan(_Acos) == True):
        print("Error in calculation triangle Tad")
        return False
    result[5] = -_Acos if Tda.m[1,3] < 0.0 else _Acos # ankle roll

    Tcd.SetTransform(
        [0,0,-ANKLE_LENGTH],
        [result[5]*180.0/math.pi,0,0]
    )

    Tdc.Copy(Tcd)
    Tdc.Inverse()
    Tac = Tad * Tdc
    _Atan = math.atan2(-Tac.m[0,1], Tac.m[1,1])
    if (math.isinf(_Atan) == True):
        print("Error in calculation triangle Tac")
        return False
    result[0] = _Atan # hip yaw

    _Atan = math.atan2(Tac.m[2,1], -Tac.m[0,1] * math.sin(result[0]) + Tac.m[1,1] * math.cos(result[0]))
    if (math.isinf(_Atan) == True):
        print("Error in caculation hip roll")
        return False
    result[1] = _Atan # hip roll

    _theta = _Atan
    _k = math.sin(result[3]) * CALF_LENGTH
    _l = -THIGH_LENGTH - math.cos(result[3]) * CALF_LENGTH
    _m = math.cos(result[0]) * X + math.sin(result[0]) * Y
    _n = math.cos(result[1]) * Z + math.sin(result[0]) * math.sin(result[1]) * X - math.cos(result[0]) * math.sin(
        result[1]) * Y
    _s = (_k * _n + _l * _m) / (_k * _k + _l * _l)
    _c = (_n - _k * _s) / _l
    _Atan = math.atan2(_s, _c)
    if (math.isinf(_Atan) == True):
        print("Error in calculation hip pitch")
        return False
    result[2] = _Atan # hip pitch
    result[4] = _theta - result[3] - result[2] # ankle pitch

    return result

def compute_ik_degree(x,y,z):
    result = compute_ik(x,y,z)
    for i in range(len(result)):
        result[i] = round(result[i] * 180.0 / np.pi, 2)
    
    return result


if __name__ == '__main__':
    print(compute_ik_degree(0.0,7.5,6.0))