import numpy
import sys
from tminterface.structs import BFEvaluationDecision, BFEvaluationInfo, BFEvaluationResponse, BFPhase
from tminterface.interface import TMInterface
from tminterface.client import Client
from math import *
import time
import struct
import signal
import bf_vaf

class MainClient(Client):
    def __init__(self) -> None:
        super().__init__()
        self.time = -1
        self.finished = False

    def on_registered(self, iface: TMInterface) -> None:
        global is_registered, server
        print(f'Registered to {iface.server_name}')
        is_registered = True
        server = iface.server_name

    def on_deregistered(self, iface: TMInterface):
        global is_registered
        print(f'Deregistered from {iface.server_name}')
        is_registered = False

    def on_simulation_begin(self, iface):
        global improvements
        self.lowest_time = iface.get_event_buffer().events_duration
        self.time = -1
        self.best = -1
        improvements = 0
    
    def on_bruteforce_evaluate(self, iface, info: BFEvaluationInfo) -> BFEvaluationResponse:
        global current_best, improvements, improvement_time, velocity, rotation
        self.time = info.time
        self.phase = info.phase

        response = BFEvaluationResponse()
        response.decision = BFEvaluationDecision.DO_NOTHING
        if self.phase == BFPhase.SEARCH:
            if self.is_eval_time() and self.is_better(iface):
                self.best, current_best = self.current, self.current
                improvements += 1
                improvement_time = self.time
                real_speed = numpy.linalg.norm(self.state.velocity) * 3.6
                velocity = [
                    round(
                    numpy.sum(
                        [
                            self.state.velocity[i] * self.state.rotation_matrix[i][idx]
                            for i in range(3)
                        ]
                    ), 3)
                    for idx in range(3)
                ]
                degs = lambda angle_rad: round(bf_vaf.to_deg(angle_rad), 3)
                rotation = [degs(self.yaw_rad), degs(self.pitch_rad), degs(self.roll_rad)]

                response.decision = BFEvaluationDecision.ACCEPT
                        
            if self.is_past_eval_time():
                if response.decision != BFEvaluationDecision.ACCEPT:
                    response.decision = BFEvaluationDecision.REJECT

        return response

    def is_better(self, iface):
        self.state = iface.get_simulation_state()
        self.yaw_rad, self.pitch_rad, self.roll_rad = self.state.yaw_pitch_roll
        self.vel = numpy.linalg.norm(self.state.velocity)
        return bf_vaf.current_goal_ptr(self)

    def speed(self):
        self.current = self.vel
        if bf_vaf.min_cp > bf_vaf.get_nb_cp(self.state):
            return False

        if bf_vaf.must_touch_ground and not bf_vaf.nb_wheels_on_ground(self.state):
            return False
        
        return self.best == -1 or self.current > self.best

    def nosepos(self):
        if bf_vaf.min_speed_kmh > self.vel * 3.6:
            return False

        if bf_vaf.min_cp > bf_vaf.get_nb_cp(self.state):
            return False

        if bf_vaf.must_touch_ground and bf_vaf.nb_wheels_on_ground(self.state) == 0:
            return False

        car_yaw, car_pitch, car_roll = self.state.yaw_pitch_roll

        target_yaw = atan2(self.state.velocity[0], self.state.velocity[2])
        target_pitch = bf_vaf.to_rad(90)
        target_roll = bf_vaf.to_rad(0)

        # coordinates condition

        car_x, car_y, car_z = self.state.position
        
        if not (bf_vaf.minX < car_x < bf_vaf.maxX and bf_vaf.minY < car_y < bf_vaf.maxY and bf_vaf.minZ < car_z < bf_vaf.maxZ):
            return False

        # Customize diff_yaw

        if bf_vaf.strategy == "any":
            # any angle
            diff_yaw = bf_vaf.to_deg(abs(car_yaw - target_yaw))
            if diff_yaw < 90:
                diff_yaw = 0

        else:
            # define the yaw angle you want in degrees, from -90 to -90
            target_yaw += bf_vaf.to_rad(bf_vaf.extra_yaw)
            diff_yaw = bf_vaf.to_deg(abs(car_yaw - target_yaw))

        self.current = diff_yaw + bf_vaf.to_deg(abs(car_pitch - target_pitch)) + bf_vaf.to_deg(abs(car_roll - target_roll))

        return self.best == -1 or self.current < self.best

    def height(self):
        self.current = self.state.position[1]

        if bf_vaf.min_cp > bf_vaf.get_nb_cp(self.state):
            return False

        if bf_vaf.must_touch_ground and not bf_vaf.nb_wheels_on_ground(self.state):
            return False
        return self.best == -1 or (self.current > self.best and self.vel * 3.6 > bf_vaf.min_speed_kmh)

    def point(self):
        # Conditions
        if bf_vaf.min_speed_kmh > self.vel * 3.6:
            return False

        if bf_vaf.min_cp > bf_vaf.get_nb_cp(self.state):
            return False

        if bf_vaf.must_touch_ground and bf_vaf.nb_wheels_on_ground(self.state) == 0:
            return False
        
        # Distance evaluation
        self.current = sum([(self.state.position[p] - bf_vaf.point[p]) ** 2 for p in range(3)])
        return self.best == -1 or self.current < self.best

    def is_eval_time(self):
        return bf_vaf.time_min <= self.time <= bf_vaf.time_max

    def is_past_eval_time(self):
        return bf_vaf.time_max <= self.time

    def is_max_time(self):
        return bf_vaf.time_max == self.time

    def on_checkpoint_count_changed(self, iface: TMInterface, current: int, target: int):
        if current == target:
            self.finished = True
