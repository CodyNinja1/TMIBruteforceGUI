import math
import numpy

def to_rad(deg):
    return deg / 180 * math.pi

def to_deg(rad):
    return rad * 180 / math.pi

class GoalSpeed:
    def is_better(self, client, g):
        client.current = numpy.linalg.norm(client.state.velocity) * 3.6

        return client.best == -1 or client.current > client.best

    def print(self, client, g):
        print(f"New best speed at {g.improvement_time} s: {client.best} km/h")

class GoalNosepos:
    def is_better(self, client, g):
        car_yaw, car_pitch, car_roll = client.state.yaw_pitch_roll

        target_yaw = math.atan2(client.state.velocity[0], client.state.velocity[2])
        target_pitch = to_rad(90)
        target_roll = to_rad(0)

        # Customize diff_yaw
        if g.strategy == "any":
            # any angle
            diff_yaw = to_deg(abs(car_yaw - target_yaw))
            if diff_yaw < 90:
                diff_yaw = 0

        else:
            # define the yaw angle you want in degrees, from -90 to -90
            target_yaw += to_rad(g.extra_yaw)
            diff_yaw = to_deg(abs(car_yaw - target_yaw))

        client.current = diff_yaw + to_deg(abs(car_pitch - target_pitch)) + to_deg(abs(car_roll - target_roll))

        return client.best == -1 or client.current < client.best

    def print(self, client, g):
        print(f"New best nosepos at {g.improvement_time} s: {client.best}°")

class GoalHeight:
    def is_better(self, client, g):
        client.current = client.state.position[1]

        return client.best == -1 or client.current > client.best

    def print(self, client, g):
        print(f"New best height at {g.improvement_time} s: {client.best} m")

class GoalPoint:
    def is_better(self, client, g):
        client.current = sum([(client.state.position[p] - g.point[p]) ** 2 for p in range(3)])

        return client.best == -1 or client.current < client.best

    def print(self, client, g):
        print(f"New best distance at {g.improvement_time} s: {math.sqrt(client.best)} m")

# TODO add specific GUI here (somehow)

# class GUI:
#     def bf_speed_gui(self): 
#         global min_cp
#         min_cp = imgui.input_int('Minimum Checkpoints', min_cp)[1]

#     def bf_height_gui(self): 
#         global min_speed_kmh, min_cp
#         min_speed_kmh = imgui.input_int('Minimum Speed (km/h)', min_speed_kmh)[1]
#         min_cp = imgui.input_int('Minimum Checkpoints', min_cp)[1]

#     def bf_nose_gui(self):
#         global min_speed_kmh, min_cp, must_touch_ground, coordinates, extra_yaw, strategy
#         pair1, pair2 = coordinates[:3], coordinates[3:]
#         min_speed_kmh = imgui.input_int('Minimum Speed (km/h)', min_speed_kmh)[1]
#         min_cp = imgui.input_int('Minimum Checkpoints', min_cp)[1]
#         must_touch_ground = imgui.checkbox("Must touch ground", must_touch_ground)[1]
#         self.enableExtraYaw = imgui.checkbox("Enable Custom Yaw Value", self.enableExtraYaw)[1]
        
#         imgui.separator()
        
#         if self.enableExtraYaw:
#             strategy = "custom"
#             extra_yaw = imgui.input_float('Yaw', extra_yaw)[1]
#             imgui.separator()
#         else:
#             strategy = "any"
        
#         input_pair = lambda s, pair: imgui.input_float3(s, *pair)[1]
#         pair1, pair2 = input_pair('Coordinate 1', pair1), input_pair('Coordinate 2', pair2)
#         coordinates = pair1 + pair2
#         unpackCoordinates()

#     def bf_point_gui(self): 
#         global point, min_cp, min_speed_kmh, must_touch_ground
#         point = imgui.input_float3('Point Coordinates', *point)[1]
#         min_speed_kmh = imgui.input_int('Minimum Speed (km/h)', min_speed_kmh)[1]
#         min_cp = imgui.input_int('Minimum Checkpoints', min_cp)[1]
#         must_touch_ground = imgui.checkbox("Must touch ground", must_touch_ground)[1]
