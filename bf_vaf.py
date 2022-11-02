# VAF is for variables and functions
import colorsys
import struct
from math import pi
from bf_tmi import *

current_goal = 1 # 0 Speed, 1 Nosepos, 2 Height, 3 Point
def setGoalPtr():
    global current_goal_ptr
    if current_goal == 0:
        current_goal_ptr = MainClient.speed
    if current_goal == 1:
        current_goal_ptr = MainClient.nosepos
    if current_goal == 2:
        current_goal_ptr = MainClient.height
    if current_goal == 3:
        current_goal_ptr = MainClient.point

is_registered = False
server = ""
coordinates = [0, 0, 0, 0, 0, 0]
def unpackCoordinates():
    global coordinates, minX, minY, minZ, maxX, maxY, maxZ
    (minX, maxX), (minY, maxY), (minZ, maxZ) = [
        sorted((round(coordinates[i], 2), round(coordinates[i+3], 2))) for i in range(3)
    ]

unpackCoordinates()
strategy = "any"
extra_yaw = 0
time_min = 0
time_max = time_min

point = [0, 0, 0]
min_speed_kmh = 0
min_cp = 0
must_touch_ground = False

# bf result window (yes)
current_best = -1
improvements = 0
improvement_time = round(time_min/1000, 2)
velocity = [0, 0, 0]
real_speed = 0
rotation = [0, 0, 0]



def h2r(h, s, v, a):
    # hsva values must be [0, 1] range
    out = list(colorsys.hsv_to_rgb(h*255, s*255, v*255))
    out.append(a)
    return [out[0]/255, out[1]/255, out[2]/255, out[3]]

def r2h(r, g, b, a):
    # rgba values must be [0, 1] range
    out = list(colorsys.rgb_to_hsv(r*255, g*255, b*255))
    out.append(a)
    return [out[0]/255, out[1]/255, out[2]/255, out[3]]


def to_rad(deg):
    return deg / 180 * pi

def to_deg(rad):
    return rad * 180 / pi

def get_nb_cp(state):
    return len([time for time, _ in state.cp_data.cp_times if time != -1])

WHEEL_OFFSETS = tuple([(3056 // 4) * i for i in range(4)])
def nb_wheels_on_ground(state):
    return sum([struct.unpack('i', state.simulation_wheels[o+292:o+296])[0] for o in WHEEL_OFFSETS])
