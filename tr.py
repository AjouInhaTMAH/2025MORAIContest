import numpy as np
from math import radians, cos, sin

def create_transform_matrix(x, y, z, yaw_deg):
    yaw = radians(yaw_deg)
    T = np.array([
        [cos(yaw), -sin(yaw), 0, x],
        [sin(yaw),  cos(yaw), 0, y],
        [0,         0,        1, z],
        [0,         0,        0, 1]
    ])
    return T

T_map_to_base = create_transform_matrix(
    x=4.382297992706299,
    y=5.394063949584961,
    z=0.03103354200720787,
    yaw_deg=-90  # Z축 +90도 회전
)

point_in_object =  np.array([-8.302000045776367, -4.080999851226807, 0.0, 1.0])
T_base_to_map = np.linalg.inv(T_map_to_base)
point_in_map = T_base_to_map @ point_in_object
print(T_base_to_map)
print("\nobject의 world -> map 기준 위치 (x, y, z):", point_in_map[:3])
# 실제위치
# x: 9.628307342529297
# y: -12.711262702941895
# z: 0.0