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
from tminterface.client import Client, run_client
from math import *
import threading
import time
import colorsys

fontPath = "DroidSans.ttf" # font
color = [.75, .75, 1, 0] # default color
bgcolor = [0, 0, 0, 1] # useless but useful
titlecolor = [0, 0, 0, 1] # coming soon
colorChange = 0 # dont modify pls
returnColor = [] # idk
rgbScroll = False # its in the name
isRGBing = False # rgbing flag, dont modify
currentGoal = 1
extra_yaw = 45
enableExtraYaw = False

GOALS = ["Speed", "Nosebug position", "Height", "Minimum distance from point", "Stuntpoints"]
IS_REGISTERED = False
SERVER = ""
STOP_BF = False
COORDINATES = [0, 0, 0, 0, 0, 0]
minX, maxX = sorted([COORDINATES[0], COORDINATES[3]])
minY, maxY = sorted([COORDINATES[1], COORDINATES[4]])
minZ, maxZ = sorted([COORDINATES[2], COORDINATES[5]])
strategy = "any"

TIME_MIN = 0
TIME_MAX = TIME_MIN
TIME = TIME_MIN / 1000 
IMPROVEMENTS = -.5 # TODO: improve this abomination of an improvement counter
CURRENT_BEST = -1
ROTATION = [0, 0, 0]

POINT = [0, 0, 0]
MIN_SPEED_KMH = 0
MIN_CP = 0
MUST_TOUCH_GROUND = False

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
    return len([time for (time, _) in state.cp_data.cp_times if time != -1])

def nb_wheels_on_ground(state):
    number = 0
    
    for i in range(4):
        current_offset = (3056 // 4) * i
        hasgroundcontact = struct.unpack('i', state.simulation_wheels[current_offset+292:current_offset+296])[0]
        if hasgroundcontact:
            number += 1

    return number

class MainClient(Client):
    def __init__(self) -> None:
        super(MainClient, self).__init__()
        self.time = 0
        self.finished = False

    def on_registered(self, iface: TMInterface) -> None:
        print(f'Registered to {iface.server_name}')
        global IS_REGISTERED, SERVER
        IS_REGISTERED = True
        SERVER = iface.server_name

    def on_deregistered(self, iface: TMInterface):
        print(f'Deregistered from {iface.server_name}')
        global IS_REGISTERED
        IS_REGISTERED = False

    def on_simulation_begin(self, iface):
        self.lowest_time = iface.get_event_buffer().events_duration
        self.best = -1
        self.time = -1
        global IMPROVEMENTS
        IMPROVEMENTS = 0
    
    def on_bruteforce_evaluate(self, iface, info: BFEvaluationInfo) -> BFEvaluationResponse:
        global STOP_BF
        self.current_time = info.time
        self.phase = info.phase

        response = BFEvaluationResponse()
        response.decision = BFEvaluationDecision.DO_NOTHING
        if STOP_BF:
            response.decision = BFEvaluationDecision.STOP
            STOP_BF = False
            return response
        if self.phase == BFPhase.INITIAL:
            if self.is_eval_time() and self.is_better(iface):
                self.best = self.current
                self.time = self.current_time
                global CURRENT_BEST
                CURRENT_BEST = self.best

            #if self.is_max_time():
                #print(f"base at {self.time}: {self.best=}, improvements: {IMPROVEMENTS}")

        elif self.phase == BFPhase.SEARCH:
            if self.is_eval_time() and self.is_better(iface):
                response.decision = BFEvaluationDecision.ACCEPT
                        
            if self.is_past_eval_time():
                if response.decision != BFEvaluationDecision.ACCEPT:
                    response.decision = BFEvaluationDecision.REJECT

        return response

    def is_better(self, iface):
        global CURRENT_BEST, IMPROVEMENTS, STOP_BF, ROTATION, currentGoal, strategy
        state = iface.get_simulation_state()
        yaw_rad, pitch_rad, roll_rad = state.yaw_pitch_roll
        vel = numpy.linalg.norm(state.velocity)

        # Conditions
        # if pitch_rad < 1.4 or pitch_rad > 1.7:
        #     return False
        # 
        # if not (496 < x < 502 and 85 < y < 87 and 178 < z < 182):
        #     return False
        if currentGoal == 0:
            self.current = numpy.linalg.norm(state.velocity)
            if (self.current > self.best): 
                IMPROVEMENTS += .5 # .5 because this if statement gets called twice per improvement :P
                ROTATION = [round((yaw_rad/(2*pi))*360, 3), round((pitch_rad/(2*pi))*360, 3), round((roll_rad/(2*pi))*360, 3)]
            return self.best == -1 or (self.current > self.best)
        elif currentGoal == 1:
            if MIN_SPEED_KMH > numpy.linalg.norm(state.velocity) * 3.6:
                return False

            if MIN_CP > get_nb_cp(state):
                return False

            if MUST_TOUCH_GROUND and nb_wheels_on_ground(state) == 0:
                return False

            car_yaw, car_pitch, car_roll = state.yaw_pitch_roll

            target_yaw = atan2(state.velocity[0], state.velocity[2])
            target_pitch = to_rad(90)
            target_roll = to_rad(0)

            # coordinates condition

            car_x, car_y, car_z = state.position
            
            if not minX < car_x < maxX:
                return False
            if not minY < car_y < maxY:
                return False
            if not minZ  < car_z < maxZ:
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
        elif currentGoal == 2:
            self.current = iface.get_simulation_state().position[1]
            return self.best == -1 or (self.current > self.best and vel * 3.6 > MIN_SPEED_KMH)
        elif currentGoal == 3:
            state = iface.get_simulation_state()
            pos = state.position
            speed = numpy.linalg.norm(state.velocity)

            # Conditions
            if MIN_SPEED_KMH > speed * 3.6:
                return False

            if MIN_CP > get_nb_cp(state):
                return False

            if MUST_TOUCH_GROUND and nb_wheels_on_ground(state) == 0:
                return False

            #x, y, z = state.position
            #x1, y1, z1, x2, y2, z2 = TRIGGER
            #if not (min(x1,x2) < x < max(x1,x2) and min(y1,y2) < y < max(y1,y2) and min(z1,z2) < z < max(z1,z2)):
            #    return False
            
            # Distance evaluation
            self.current = (pos[0]-POINT[0]) ** 2 + (pos[1]-POINT[1]) ** 2 + (pos[2]-POINT[2]) ** 2
            return self.best == -1 or self.current < self.best
    def is_eval_time(self):
        return TIME_MIN <= self.current_time <= TIME_MAX

    def is_past_eval_time(self):
        return TIME_MAX <= self.current_time

    def is_max_time(self):
        return TIME_MAX == self.current_time

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


class GUI(object):
    def __init__(self):
        super().__init__()
        self.backgroundColor = [0., 1., 0., 0.]
        self.window = impl_glfw_init()
        gl.glClearColor(*self.backgroundColor)
        imgui.create_context()
        self.impl = GlfwRenderer(self.window)
        if fontPath != "":
            io = imgui.get_io()
            io.fonts.clear()
            io.font_global_scale = 1
            new_font = io.fonts.add_font_from_file_ttf(fontPath, 20.0, io.fonts.get_glyph_ranges_latin())
            self.impl.refresh_font_texture()

        self.string = ""
        self.f = 0.5

        self.loop()
    
    def bf_speed_gui(self): 
        None

    def bf_height_gui(self): 
        global MIN_SPEED_KMH
        MIN_SPEED_KMH = round(MIN_SPEED_KMH, 2)
        changed, MIN_SPEED_KMH = imgui.input_float('Minimum Speed (km/h)', MIN_SPEED_KMH)

    def bf_nose_gui(self):
        global MIN_SPEED_KMH, MIN_CP, MUST_TOUCH_GROUND, COORDINATES, minX, minY, minZ, maxX, maxY, maxZ, enableExtraYaw, extra_yaw, strategy, TIME_MAX
        pair1 = [COORDINATES[0], COORDINATES[1], COORDINATES[2]].copy()
        pair2 = [COORDINATES[3], COORDINATES[4], COORDINATES[5]].copy()
        MIN_SPEED_KMH = round(MIN_SPEED_KMH, 2)
        changed, MIN_SPEED_KMH = imgui.input_float('Minimum Speed (km/h)', MIN_SPEED_KMH)
        changed, MIN_CP = imgui.input_int('Minimum Checkpoints', MIN_CP)
        changed, TIME_MAX = imgui.input_float('Maxiumum evaluation time', TIME_MAX)
        _, MUST_TOUCH_GROUND = imgui.checkbox("Must touch ground", MUST_TOUCH_GROUND)
        _, enableExtraYaw = imgui.checkbox("Enable Custom Yaw Value", enableExtraYaw)
        
        imgui.separator()
        
        if enableExtraYaw:
            strategy = "custom"
            changed, extra_yaw = imgui.input_float('Yaw', *pair1)
            imgui.separator()
        else:
            strategy = "any"
        
        changed, pair1 = imgui.input_float3('Coordinate 1', *pair1)
        changed, pair2 = imgui.input_float3('Coordinate 2', *pair2)
        COORDINATES[0] = pair1[0]
        COORDINATES[1] = pair1[1]
        COORDINATES[2] = pair1[2]

        COORDINATES[3] = pair2[0]
        COORDINATES[4] = pair2[1]
        COORDINATES[5] = pair2[2]


        minX, maxX = sorted([round(COORDINATES[0], 2), round(COORDINATES[3], 2)])
        minY, maxY = sorted([round(COORDINATES[1], 2), round(COORDINATES[4], 2)])
        minZ, maxZ = sorted([round(COORDINATES[2], 2), round(COORDINATES[5], 2)])

    def bf_point_gui(self): 
        global POINT, MIN_CP, MIN_SPEED_KMH, MUST_TOUCH_GROUND
        changed, POINT = imgui.input_float3('Point Coordinates', *POINT)
        changed, MIN_SPEED_KMH = imgui.input_float('Minimum Speed (km/h)', MIN_SPEED_KMH)
        changed, MIN_CP = imgui.input_int('Minimum Checkpoints', MIN_CP)
        _, MUST_TOUCH_GROUND = imgui.checkbox("Must touch ground", MUST_TOUCH_GROUND)

    def bf_settings(self):
        imgui.begin("Bruteforce Settings", True)
        global currentGoal, GOALS
        clicked = False
        clicked, currentGoal = imgui.combo(
            "Bruteforce Goal", currentGoal, GOALS
        )

        changed = False
        global TIME
        changed, TIME = imgui.input_float('Evaluation time', TIME)
        TIME = round(TIME, 2)

        imgui.separator()

        # match currentGoal:
        #     case 0:
        #         self.bf_speed_gui()
        #     case 1:
        #         self.bf_nose_gui()
        #     case 2:
        #         self.bf_height_gui()
        #     case 3:
        #         self.bf_point_gui()
        if currentGoal == 0:
            self.bf_speed_gui()
        elif currentGoal == 1:
            self.bf_nose_gui()
        elif currentGoal == 2:
            self.bf_height_gui()
        elif currentGoal == 3:
            self.bf_point_gui()

        imgui.end()
    def bf_result(self):
        # pushStyleColor(imgui.COLOR_WINDOW_BACKGROUND, bgcolor)
        # pushStyleColor(imgui.COLOR_TITLE_BACKGROUND, titlecolor)
        imgui.begin("Bruteforce Result", True)

        global SERVER, IS_REGISTERED, STOP_BF

        imgui.text(f"Bruteforce Best: {round(CURRENT_BEST, 3)} ")
        imgui.separator()
        imgui.text(f"Car information at {TIME}:")
        imgui.text(f"Rotation (yaw, pitch, roll): {ROTATION[0]}, {ROTATION[1]}, {ROTATION[2]}")
        imgui.text(f"Improvements: {IMPROVEMENTS}")
        connection = f"Connected to {SERVER}" if IS_REGISTERED else "Not Registered"
        imgui.text(f"Connection Status: {connection}")
        imgui.separator
        if imgui.button("Stop Bruteforce"):
            STOP_BF = True
                
        imgui.end()
        # imgui.pop_style_color(2)
    
    def customize(self):
        global color, bgcolor, titlecolor, rgbScroll
        # pushStyleColor(imgui.COLOR_WINDOW_BACKGROUND, bgcolor)
        # pushStyleColor(imgui.COLOR_BUTTON, bgcolor)
        # pushStyleColor(imgui.COLOR_TITLE_BACKGROUND, titlecolor)
        imgui.begin("Customize", True)
        if not rgbScroll:
            _, color = imgui.color_edit4("Background", *color, show_alpha=False)
            # _, bgcolor = imgui.color_edit4("Window Background", *bgcolor, show_alpha=False)
            # _, titlecolor = imgui.color_edit4("Active Titlebar Background", *titlecolor, show_alpha=False)

        if rgbScroll:
            global colorChange, brightness
            changed1, colorChange = imgui.slider_float(
                "Speed", colorChange,
                min_value=0.0, max_value=100.0,
                format="%.0f",
                power=1.
            )        

        if imgui.button("Start RGB scroll" if not rgbScroll else "Stop RGB Scroll"):
            print("Start scroll" if not rgbScroll else "Stop scroll")
            rgbScroll = not rgbScroll
        
        color = list(color)
        if not rgbScroll:
            self.backgroundColor = color.copy()

        imgui.end()
        # imgui.pop_style_color(3)

    def loop(self):
        while not glfw.window_should_close(self.window):
            glfw.poll_events()
            self.impl.process_inputs()
            imgui.new_frame()

            self.bf_result()
            self.bf_settings()
            self.customize()

            imgui.render()

            global color, rgbScroll, isRGBing, colorChange

            if rgbScroll:
                color = r2h(color[0], color[1], color[2], color[3]) # now it is hsv
                color[0] += colorChange/1000000 # add to hue

                if color[0] > 255:
                    color[0] = 0

                color = h2r(color[0], color[1], color[2], color[3]) # convert back into rgb after adding to the hue

            color = list(color)
            self.backgroundColor = color.copy() # set the color

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


# 500th line: this is the goal
