# shoutout to stuntlover

import struct
import imgui
import signal
import glfw
import OpenGL.GL as gl
from imgui.integrations.glfw import GlfwRenderer
import numpy
import sys
from tminterface.structs import BFEvaluationDecision, BFEvaluationInfo, BFEvaluationResponse, BFPhase
from tminterface.interface import TMInterface
from tminterface.client import Client
from math import *
import threading
import time
import colorsys

current_goal = 1 # 0 Speed, 1 Nosepos, 2 Height, 3 Point
extra_yaw = 0

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
time_min = 0
time_max = time_min
improvements = 0
current_best = -1
rotation = [0, 0, 0]

point = [0, 0, 0]
min_speed_kmh = 0
min_cp = 0
must_touch_ground = False

def makeGUI():
    gui = GUI()

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
    return deg / 180 * pi

def to_deg(rad):
    return rad * 180 / pi

def get_nb_cp(state):
    return len([time for time, _ in state.cp_data.cp_times if time != -1])

WHEEL_OFFSETS = tuple([(3056 // 4) * i for i in range(4)])
def nb_wheels_on_ground(state):
    return sum([struct.unpack('i', state.simulation_wheels[o+292:o+296])[0] for o in WHEEL_OFFSETS])

class MainClient(Client):
    def __init__(self) -> None:
        super().__init__()
        self.time = 0
        self.finished = False

    def on_registered(self, iface: TMInterface) -> None:
        print(f'Registered to {iface.server_name}')
        global is_registered, server
        is_registered = True
        server = iface.server_name

    def on_deregistered(self, iface: TMInterface):
        print(f'Deregistered from {iface.server_name}')
        global is_registered
        is_registered = False

    def on_simulation_begin(self, iface):
        self.lowest_time = iface.get_event_buffer().events_duration
        self.best = -1
        self.time = -1
        global improvements
        improvements = 0
    
    def on_bruteforce_evaluate(self, iface, info: BFEvaluationInfo) -> BFEvaluationResponse:
        self.current_time = info.time
        self.phase = info.phase

        response = BFEvaluationResponse()
        response.decision = BFEvaluationDecision.DO_NOTHING
        if self.phase == BFPhase.INITIAL:
            if self.is_eval_time() and self.is_better(iface):
                self.best = self.current
                self.time = self.current_time
                global current_best
                current_best = self.best

        elif self.phase == BFPhase.SEARCH:
            if self.is_eval_time() and self.is_better(iface):
                response.decision = BFEvaluationDecision.ACCEPT
                        
            if self.is_past_eval_time():
                if response.decision != BFEvaluationDecision.ACCEPT:
                    response.decision = BFEvaluationDecision.REJECT

        return response

    def is_better(self, iface):
        global improvements, rotation
        state = iface.get_simulation_state()
        yaw_rad, pitch_rad, roll_rad = state.yaw_pitch_roll
        vel = numpy.linalg.norm(state.velocity)

        if current_goal == 0: # Speed
            self.current = vel
            if (self.current > self.best): 
                # increments improvement counter if the improvement is accepted
                improvements += self.phase == BFPhase.SEARCH
                degs = lambda angle_rad: round(to_deg(angle_rad), 3)
                rotation = [degs(yaw_rad), degs(pitch_rad), degs(roll_rad)]
            
            if min_cp > get_nb_cp(state):
                return False

            if must_touch_ground and nb_wheels_on_ground(state) == 0:
                return False
            
            return self.best == -1 or (self.current > self.best)

        elif current_goal == 1: # Nosepos
            if min_speed_kmh > vel * 3.6:
                return False

            if min_cp > get_nb_cp(state):
                return False

            if must_touch_ground and nb_wheels_on_ground(state) == 0:
                return False

            car_yaw, car_pitch, car_roll = state.yaw_pitch_roll

            target_yaw = atan2(state.velocity[0], state.velocity[2])
            target_pitch = to_rad(90)
            target_roll = to_rad(0)

            # coordinates condition

            car_x, car_y, car_z = state.position
            
            if not (minX < car_x < maxX and minY < car_y < maxY and minZ < car_z < maxZ):
                return False

            # Customize diff_yaw

            if strategy == "any":
                # any angle
                diff_yaw = to_deg(abs(car_yaw - target_yaw))
                if diff_yaw < 90:
                    diff_yaw = 0

            else:
                # define the yaw angle you want in degrees, from -90 to -90
                target_yaw += to_rad(extra_yaw)
                diff_yaw = to_deg(abs(car_yaw - target_yaw))

            self.current = diff_yaw + to_deg(abs(car_pitch - target_pitch)) + to_deg(abs(car_roll - target_roll))

            return self.best == -1 or self.current < self.best

        elif current_goal == 2: # Height
            self.current = iface.get_simulation_state().position[1]

            if min_cp > get_nb_cp(state):
                return False

            if must_touch_ground and nb_wheels_on_ground(state) == 0:
                return False
            return self.best == -1 or (self.current > self.best and vel * 3.6 > min_speed_kmh)

        elif current_goal == 3: # Point
            state = iface.get_simulation_state()
            pos = state.position

            # Conditions
            if min_speed_kmh > vel * 3.6:
                return False

            if min_cp > get_nb_cp(state):
                return False

            if must_touch_ground and nb_wheels_on_ground(state) == 0:
                return False
            
            # Distance evaluation
            self.current = (pos[0]-point[0]) ** 2 + (pos[1]-point[1]) ** 2 + (pos[2]-point[2]) ** 2
            return self.best == -1 or self.current < self.best

    def is_eval_time(self):
        return time_min <= self.current_time <= time_max

    def is_past_eval_time(self):
        return time_max <= self.current_time

    def is_max_time(self):
        return time_max == self.current_time

    def on_run_step(self, iface: TMInterface, _time: int):
        self.time = _time

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
        self.fontPath = "DroidSans.ttf" # font
        self.color = [0, 0, 0, 0] # background color
        self.bgcolor = [0, 0, 0, 1] # wtf does this do this isnt background color
        self.titlecolor = [0, 0, 0, 1] # coming soon wink wink
        self.colorChange = 0 # dont modify pls
        self.rgbScroll = False # its in the name
        self.enableExtraYaw = False
        self.goals = ["Speed", "Nosebug position", "Height", "Minimum distance from point"]

        self.backgroundColor = [0., 1., 0., 0.]
        self.window = impl_glfw_init()
        gl.glClearColor(*self.backgroundColor)
        imgui.create_context()
        self.impl = GlfwRenderer(self.window)
        if self.fontPath:
            io = imgui.get_io()
            io.fonts.clear()
            io.font_global_scale = 1
            new_font = io.fonts.add_font_from_file_ttf(self.fontPath, 20.0, io.fonts.get_glyph_ranges_latin())
            self.impl.refresh_font_texture()

        self.loop()
    
    def bf_speed_gui(self): 
        global min_cp
        min_cp = imgui.input_int('Minimum Checkpoints', min_cp)[1]

    def bf_height_gui(self): 
        global min_speed_kmh, min_cp
        min_speed_kmh = imgui.input_float('Minimum Speed (km/h)', round(min_speed_kmh, 2))[1]
        min_cp = imgui.input_int('Minimum Checkpoints', min_cp)[1]

    def bf_nose_gui(self):
        global min_speed_kmh, min_cp, must_touch_ground, coordinates, extra_yaw, strategy
        pair1, pair2 = coordinates[:3], coordinates[3:]
        min_speed_kmh = imgui.input_float('Minimum Speed (km/h)', round(min_speed_kmh, 2))[1]
        min_cp = imgui.input_int('Minimum Checkpoints', min_cp)[1]
        must_touch_ground = imgui.checkbox("Must touch ground", must_touch_ground)[1]
        self.enableExtraYaw = imgui.checkbox("Enable Custom Yaw Value", self.enableExtraYaw)[1]
        
        imgui.separator()
        
        if self.enableExtraYaw:
            strategy = "custom"
            extra_yaw = imgui.input_float('Yaw', *pair1)[1]
            imgui.separator()
        else:
            strategy = "any"
        
        pair1 = imgui.input_float3('Coordinate 1', *pair1)[1]
        pair2 = imgui.input_float3('Coordinate 2', *pair2)[1]
        coordinates = pair1 + pair2
        unpackCoordinates()

    def bf_point_gui(self): 
        global point, min_cp, min_speed_kmh, must_touch_ground
        point = imgui.input_float3('Point Coordinates', *point)[1]
        min_speed_kmh = imgui.input_float('Minimum Speed (km/h)', min_speed_kmh)[1]
        min_cp = imgui.input_int('Minimum Checkpoints', min_cp)[1]
        must_touch_ground = imgui.checkbox("Must touch ground", must_touch_ground)[1]

    def bf_settings(self):
        imgui.begin("Bruteforce Settings", True)
        global current_goal, time_min, time_max
        current_goal = imgui.combo("Bruteforce Goal", current_goal, self.goals)[1]
        timetext = lambda s, t: int(float(imgui.input_text(s, str(t/1000), 256)[1])) * 1000
        time_min = timetext("Evaluation start (s)", time_min)
        if time_min > time_max:
            time_max = time_min
        time_max = timetext("Evaluation end (s)", time_max)

        imgui.separator()

        if current_goal == 0:
            self.bf_speed_gui()
        elif current_goal == 1:
            self.bf_nose_gui()
        elif current_goal == 2:
            self.bf_height_gui()
        elif current_goal == 3:
            self.bf_point_gui()

        imgui.end()

    def bf_result(self):
        imgui.begin("Bruteforce Result", True)

        global server, is_registered

        imgui.text(f"Bruteforce Best: {round(current_best, 3)} ")
        imgui.separator()
        imgui.text(f"Car information at {time_min/1000}:")
        imgui.text(f"Rotation (yaw, pitch, roll): {rotation[0]}, {rotation[1]}, {rotation[2]}")
        imgui.text(f"Improvements: {improvements}")
        imgui.text("Connection Status: " + f"Connected to {server}" if is_registered else "Not Registered")
        imgui.separator
                
        imgui.end()
    
    def customize(self):
        imgui.begin("Customize", True)
        if not self.rgbScroll:
            self.color = list(imgui.color_edit4("Background", *self.color, show_alpha=False)[1])

        if self.rgbScroll:
            self.colorChange = imgui.slider_float(
                "Speed", self.colorChange,
                min_value=0.0, max_value=100.0,
                format="%.0f",
                power=1.
            )[1]

        if imgui.button("Start RGB scroll" if not self.rgbScroll else "Stop RGB Scroll"):
            print("Start scroll" if not self.rgbScroll else "Stop scroll")
            self.rgbScroll = not self.rgbScroll
        
        if not self.rgbScroll:
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
                self.color[0] = (self.color[0] + self.colorChange/1000000) % 256 # add hue
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
