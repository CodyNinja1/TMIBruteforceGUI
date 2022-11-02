# shoutout to bestie

import imgui
import glfw
import OpenGL.GL as gl
from imgui.integrations.glfw import GlfwRenderer
import threading
from bf_vaf import *

def makeGUI():
    gui = GUI()

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
            "yaw": extra_yaw, 
            "bf_goal": current_goal,
            "coords": coordinates,
            "point": point
        }

        self.window = impl_glfw_init()
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

    def bf_speed_gui(self): 
        global min_cp
        min_cp = imgui.input_int('Minimum Checkpoints', min_cp)[1]

    def bf_height_gui(self): 
        global min_speed_kmh, min_cp
        min_speed_kmh = imgui.input_int('Minimum Speed (km/h)', min_speed_kmh)[1]
        min_cp = imgui.input_int('Minimum Checkpoints', min_cp)[1]

    def bf_nose_gui(self):
        global min_speed_kmh, min_cp, must_touch_ground, coordinates, extra_yaw, strategy
        pair1, pair2 = coordinates[:3], coordinates[3:]
        min_speed_kmh = imgui.input_int('Minimum Speed (km/h)', min_speed_kmh)[1]
        min_cp = imgui.input_int('Minimum Checkpoints', min_cp)[1]
        must_touch_ground = imgui.checkbox("Must touch ground", must_touch_ground)[1]
        self.enableExtraYaw = imgui.checkbox("Enable Custom Yaw Value", self.enableExtraYaw)[1]
        
        imgui.separator()
        
        if self.enableExtraYaw:
            strategy = "custom"
            extra_yaw = imgui.input_float('Yaw', extra_yaw)[1]
            imgui.separator()
        else:
            strategy = "any"
        
        input_pair = lambda s, pair: imgui.input_float3(s, *pair)[1]
        pair1, pair2 = input_pair('Coordinate 1', pair1), input_pair('Coordinate 2', pair2)
        coordinates = pair1 + pair2
        unpackCoordinates()

    def bf_point_gui(self): 
        global point, min_cp, min_speed_kmh, must_touch_ground
        point = imgui.input_float3('Point Coordinates', *point)[1]
        min_speed_kmh = imgui.input_int('Minimum Speed (km/h)', min_speed_kmh)[1]
        min_cp = imgui.input_int('Minimum Checkpoints', min_cp)[1]
        must_touch_ground = imgui.checkbox("Must touch ground", must_touch_ground)[1]

    def bf_settings(self):
        global current_goal, time_min, time_max
        imgui.begin("Bruteforce Settings", True)
        
        current_goal = imgui.combo("Bruteforce Goal", current_goal, self.goals)[1]
        if current_goal == 0:
            self.bf_speed_gui()
        elif current_goal == 1:
            self.bf_nose_gui()
        elif current_goal == 2:
            self.bf_height_gui()
        elif current_goal == 3:
            self.bf_point_gui()
        setGoalPtr()

        timetext = lambda s, t: round(imgui.input_float(s, t/1000)[1] * 1000, 3)
        time_min = timetext("Evaluation start (s)", time_min)
        if time_min > time_max:
            time_max = time_min
        time_max = timetext("Evaluation end (s)", time_max)

        imgui.separator()

        imgui.end()

    def bf_result(self):
        imgui.begin("Bruteforce Result", True) 
        
        imgui.text(f"Bruteforce Best: {round(current_best, 3)} ")
        imgui.text(f"Improvements: {improvements}")
        imgui.text(f"Car information at {improvement_time}:")
        imgui.text(f"Velocity (sideways, vertically, in facing direction): {velocity}")
        imgui.text(f"Rotation (yaw, pitch, roll): {rotation}")
        imgui.text("Connection Status: " + (f"Connected to {server}" if is_registered else "Not Registered"))
        
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

