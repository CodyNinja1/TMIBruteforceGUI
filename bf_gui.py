# shoutout to bestie

import colorsys
import glfw
import imgui
import math
import numpy
import OpenGL.GL as gl
import signal
import struct
import sys
import threading
import time

from imgui.integrations.glfw import GlfwRenderer

from tminterface.structs import BFEvaluationDecision, BFEvaluationInfo, BFEvaluationResponse, BFPhase
from tminterface.interface import TMInterface
from tminterface.client import Client

from bf_specific import GoalSpeed, GoalNosepos, GoalHeight, GoalPoint

class Global:
    def __init__(self):
        self.is_registered = False
        self.server = ""

        # Goal
        self.current_goal = 0 # 0 Speed, 1 Nosepos, 2 Height, 3 Point
        self.strategy = "any"
        self.extra_yaw = 0
        self.point = [0, 0, 0]

        # Conditions
        self.enablePositionCheck = False
        self.pair1 = [0, 0, 0]
        self.pair2 = [999, 999, 999]

    def unpackCoordinates(self):
        """Execute only once, on simulation start"""
        (self.minX, self.maxX), (self.minY, self.maxY), (self.minZ, self.maxZ) = [
            sorted((round(self.pair1[i], 2), round(self.pair2[i], 2))) for i in range(3)
        ]

    def isCarInTrigger(self, state):
        """Execute every tick where is_eval_time() is True"""
        car_x, car_y, car_z = state.position
        return self.minX <= car_x <= self.maxX and self.minY <= car_y <= self.maxY and self.minZ <= car_z <= self.maxZ


g = Global()

time_min = 0
time_max = time_min

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

def makeGUI():
    GUI()

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

def pushStyleColor(style, color):
    imgui.push_style_color(style, color[0], color[1], color[2], color[3])

def to_rad(deg):
    return deg / 180 * math.pi

def to_deg(rad):
    return rad * 180 / math.pi

def get_nb_cp(state):
    return len([time for time, _ in state.cp_data.cp_times if time != -1])

WHEEL_OFFSETS = tuple([(3056 // 4) * i for i in range(4)])
def nb_wheels_on_ground(state):
    return sum([struct.unpack('i', state.simulation_wheels[o+292:o+296])[0] for o in WHEEL_OFFSETS])

class MainClient(Client):
    def __init__(self) -> None:
        super().__init__()
        self.time = -1
        self.finished = False
        self.goal = GoalSpeed()

    def on_registered(self, iface: TMInterface) -> None:
        print(f'Registered to {iface.server_name}')
        g.is_registered = True
        g.server = iface.server_name

    def on_deregistered(self, iface: TMInterface):
        print(f'Deregistered from {iface.server_name}')
        g.is_registered = False

    def on_simulation_begin(self, iface):
        global improvements
        self.lowest_time = iface.get_event_buffer().events_duration
        self.time = -1
        self.best = -1
        improvements = 0

        g.unpackCoordinates()
        if g.current_goal == 0: self.goal = GoalSpeed()
        if g.current_goal == 1: self.goal = GoalNosepos()
        if g.current_goal == 2: self.goal = GoalHeight()
        if g.current_goal == 3: self.goal = GoalPoint()
    
    def on_bruteforce_evaluate(self, iface, info: BFEvaluationInfo) -> BFEvaluationResponse:
        global current_best, improvements, improvement_time, velocity, rotation
        self.time = info.time
        self.phase = info.phase

        # TODO fix 2 bugs:
        # - no base run self.best because no initial phase
        # - no checking initial phase post accept, even though search phase accepts instantly (before evaluation time stop)
        
        response = BFEvaluationResponse()
        response.decision = BFEvaluationDecision.DO_NOTHING
        if self.phase == BFPhase.SEARCH:
            if self.is_eval_time() and self.is_better(iface):
                self.best, current_best = self.current, self.current
                improvements += 1
                improvement_time = self.time
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
                degs = lambda angle_rad: round(to_deg(angle_rad), 3)
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

        # Conditions
        if min_speed_kmh > self.vel * 3.6: # Min speed
            return False

        if min_cp > get_nb_cp(self.state): # Min CP
            return False

        if must_touch_ground and nb_wheels_on_ground(self.state) == 0: # Touch ground
            return False

        if g.enablePositionCheck and not g.isCarInTrigger(self.state): # Position
            return False

        # Specific goal bruteforce
        # This line is a bit complicated, but for example, for speed it means: GoalSpeed.is_better(MainClient, g)
        return self.goal.is_better(self, g)

    def is_eval_time(self):
        return time_min <= self.time <= time_max

    def is_past_eval_time(self):
        return time_max <= self.time

    def is_max_time(self):
        return time_max == self.time

    def on_checkpoint_count_changed(self, iface: TMInterface, current: int, target: int):
        if current == target:
            self.finished = True

def impl_glfw_init(window_name="TrackMania Bruteforce", width=300, height=300):
    if not glfw.init():
        print("Could not initialize OpenGL context")
        exit(1)

    # OS X supports only forward-compatible core profiles from 3.2
    glfw.window_hint(glfw.CONTEXT_VERSION_MAJOR, 3)
    glfw.window_hint(glfw.CONTEXT_VERSION_MINOR, 3)
    glfw.window_hint(glfw.OPENGL_PROFILE, glfw.OPENGL_CORE_PROFILE)
    glfw.window_hint(glfw.OPENGL_FORWARD_COMPAT, gl.GL_TRUE)

    # Create a windowed mode window and its OpenGL context
    window = glfw.create_window(int(width), int(height), window_name, None, None)
    glfw.make_context_current(window)

    if not window:
        glfw.terminate()
        print("Could not initialize Window")
        exit(1)

    return window

class GUI:
    def __init__(self):
        self.fontPath = "" # font
        self.color = [0.25, 0.5, 0.75, 0.5] # background color
        self.bgcolor = [0.25, 0.5, 0.75, 0.5] # wtf does this do this isnt background color
        self.titlecolor = [0, 0, 0, 1] # coming soon wink wink
        self.colorChange = 0 # dont modify pls
        self.rgbScroll = False # its in the name
        self.enableExtraYaw = False
        self.goals = ["Speed", "Nosebug position", "Height", "Minimum distance from point"]
        self.backgroundColor = [0.25, 0.5, 0.75, 0.5]

        self.settings = { 
            "font": self.fontPath,
            "color": self.color,
            "rgb_speed": self.colorChange,
            "rgb_e": self.rgbScroll,
            "yaw_e": self.enableExtraYaw,
            "yaw": g.extra_yaw, 
            "coordsCheck": g.enablePositionCheck,
            "pair1": g.pair1,
            "pair2": g.pair2,
            "point": g.point,
            "bf_goal": g.current_goal
        }

        self.window = impl_glfw_init(width=700, height=500)
        gl.glClearColor(*self.backgroundColor)
        imgui.create_context()
        self.impl = GlfwRenderer(self.window)
        if self.fontPath:
            io = imgui.get_io()
            io.fonts.clear()
            io.font_global_scale = 1
            new_font = io.fonts.add_font_from_file_ttf(self.fontPath, 20, io.fonts.get_glyph_ranges_latin())
            self.impl.refresh_font_texture()

        self.loop()
    
    # def save_settings(self, file_location):
    #     with open(file_location, "r+") as s_file:
    #             json.dump(self.settings, s_file)
    
    # def load_settings(self, file_location):
    #     with open(file_location, "r") as s_file:
    #             json.dump(self.settings, s_file) 
    # currently working on this feature, don't remove

    def bf_conditions_gui(self):
        global min_speed_kmh, min_cp, must_touch_ground
        min_speed_kmh = imgui.input_int('Minimum Speed (km/h)', min_speed_kmh)[1]
        min_cp = imgui.input_int('Minimum Checkpoints', min_cp)[1]
        must_touch_ground = imgui.checkbox("Must touch ground", must_touch_ground)[1]
        
        # position coordinates
        g.enablePositionCheck = imgui.checkbox("Enable Position check (car must be inside trigger)", g.enablePositionCheck)[1]
        
        if g.enablePositionCheck:
            input_pair = lambda s, pair: imgui.input_float3(s, *pair)[1]
            g.pair1, g.pair2 = input_pair('Trigger corner 1', g.pair1), input_pair('Trigger corner 2', g.pair2)
            imgui.separator()

    def bf_speed_gui(self):
        pass

    def bf_height_gui(self):
        pass

    def bf_nose_gui(self):
        self.enableExtraYaw = imgui.checkbox("Enable Custom Yaw Value", self.enableExtraYaw)[1]
        
        imgui.separator()
        
        if self.enableExtraYaw:
            g.strategy = "custom"
            g.extra_yaw = imgui.input_float('Yaw', g.extra_yaw)[1]
            imgui.separator()
        else:
            g.strategy = "any"

    def bf_point_gui(self):
        g.point = imgui.input_float3('Point Coordinates', *g.point)[1]

    def bf_settings(self):
        global time_min, time_max
        imgui.begin("Bruteforce Settings", True)

        imgui.text("Goal and parameters")
        g.current_goal = imgui.combo("Bruteforce Goal", g.current_goal, self.goals)[1]
        if   g.current_goal == 0: self.bf_speed_gui()
        elif g.current_goal == 1: self.bf_nose_gui()
        elif g.current_goal == 2: self.bf_height_gui()
        elif g.current_goal == 3: self.bf_point_gui()
        
        imgui.separator()

        imgui.text("Evaluation time")
        timetext = lambda s, t: round(imgui.input_float(s, t/1000)[1] * 1000, 3)
        time_min = timetext("Evaluation start (s)", time_min)
        time_max = timetext("Evaluation end (s)", time_max)
        if time_min > time_max:
            time_max = time_min

        imgui.separator()
        
        imgui.text("Conditions")
        self.bf_conditions_gui()

        imgui.end()

    def bf_result(self):
        imgui.begin("Bruteforce Result", True) 
        
        imgui.text(f"Bruteforce Best: {round(current_best, 3)} ")
        imgui.text(f"Improvements: {improvements}")
        imgui.text(f"Car information at {improvement_time}:")
        imgui.text(f"Velocity (sideways, vertically, in facing direction): {velocity}")
        imgui.text(f"Rotation (yaw, pitch, roll): {rotation}")
        imgui.text("Connection Status: " + (f"Connected to {g.server}" if g.is_registered else "Not Registered"))
        
        imgui.separator()
        
        imgui.end()
    
    def customize(self):
        imgui.begin("Customize", True)

        if imgui.button("Start RGB scroll" if not self.rgbScroll else "Stop RGB Scroll"):
            self.rgbScroll = not self.rgbScroll
        
        if self.rgbScroll:
            if not any(self.color[:3]): self.color = [(i + 1) / 4 for i in range(4)]
            self.colorChange = imgui.slider_float(
                "Speed", self.colorChange,
                min_value=0, max_value=32,
                power=1
            )[1]
        
        else:
            self.color = list(imgui.color_edit4("Background", *self.color, show_alpha=False)[1])
            self.backgroundColor = self.color.copy()

        imgui.end()

    def loop(self):
        while not glfw.window_should_close(self.window):
            glfw.poll_events()
            self.impl.process_inputs()
            imgui.new_frame()

            self.bf_result()
            self.bf_settings()
            self.customize()

            imgui.render()

            if self.rgbScroll:
                self.color = r2h(*self.color) # convert to hsv
                self.color[0] = (self.color[0] + self.colorChange/1000000) % 1 # add hue
                self.color = h2r(*self.color) # convert back into rgb

            self.backgroundColor = self.color.copy() # set the self.color

            gl.glClearColor(*self.backgroundColor)
            gl.glClear(gl.GL_COLOR_BUFFER_BIT)

            self.impl.render(imgui.get_draw_data())
            glfw.swap_buffers(self.window)

        self.impl.shutdown()
        glfw.terminate()

def main():
    server_name = f'TMInterface{sys.argv[1]}' if len(sys.argv) > 1 else 'TMInterface0'
    print(f'Connecting to {server_name}...')
    client = MainClient()
    iface = TMInterface(server_name)

    def handler(signum, frame):
        iface.close()
        quit()

    signal.signal(signal.SIGBREAK, handler)
    signal.signal(signal.SIGINT, handler)
    iface.register(client)

    while not iface.registered:
        time.sleep(0)

    last_finished = False
    last_time = 0
    while iface.registered:
        if last_finished != client.finished:
            last_finished = client.finished
            if last_finished:
                print('Finished')

        if client.time != last_time:
            last_time = client.time
        time.sleep(0)

if __name__ == '__main__':
    x = threading.Thread(target=makeGUI, daemon=True)
    x.start()
    main()
