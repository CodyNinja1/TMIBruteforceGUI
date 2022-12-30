import math
import numpy

def to_rad(deg):
    return deg / 180 * math.pi

def to_deg(rad):
    return rad * 180 / math.pi

class GoalSpeed:
    def is_better(self, client, g):
        client.current = numpy.linalg.norm(client.state.velocity) * 3.6

        return g.current_best == -1 or client.current > g.current_best

    def print(self, g):
        print(f"New best speed at {g.improvement_time} s: {g.current_best} km/h")

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

        return g.current_best == -1 or client.current < g.current_best

    def print(self, g):
        print(f"New best nosepos at {g.improvement_time} s: {g.current_best}°")

class GoalHeight:
    def is_better(self, client, g):
        client.current = client.state.position[1]

        return g.current_best == -1 or client.current > g.current_best

    def print(self, g):
        print(f"New best height at {g.improvement_time} s: {g.current_best} m")

class GoalPoint:
    def is_better(self, client, g):
        client.current = sum([(client.state.position[p] - g.point[p]) ** 2 for p in range(3)])

        return g.current_best == -1 or client.current < g.current_best

    def print(self, g):
        print(f"New best distance at {g.improvement_time} s: {math.sqrt(g.current_best)} m")
