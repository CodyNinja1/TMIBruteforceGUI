import os
try:
    import numpy as np
    import glfw
    import requests
    import imgui
    import numpy
    import OpenGL.GL as gl
    import win32api

    from imgui.integrations.glfw import GlfwRenderer

    from tminterface.structs import BFEvaluationDecision, BFEvaluationInfo, BFEvaluationResponse, BFPhase
    from tminterface.interface import TMInterface
    from tminterface.client import Client
    from tminterface.constants import SIMULATION_WHEELS_SIZE

except:
    print("Failed to import modules, trying to install...")
    os.system("python -m pip install -r requirements.txt")
    os.system("python -m pip install requests")
    print("Installed requirements")

import colorsys
import math
import signal
from struct import unpack
import sys
import threading
import time

try:
    import global_funcs as g
    from bf_goals import GoalSpeed, GoalNosepos, GoalHeight, GoalPoint
except: # This is kinda stupid cuz its just installing it here too :xdd: but idk
    update_file_url = 'https://raw.githubusercontent.com/CodyNinja1/TMIBruteforceGUI/main/bf_gui_version.txt'
    update_file_lines = requests.get(update_file_url).text.split("\n")
    
    if not os.path.exists("global_funcs.py"):
        with open("global_funcs.py", "x") as globf: pass
    
        with open("global_funcs.py", "wb") as globf:
            globf.write(requests.get(update_file_lines[3]).content)

        with open("bf_goals.py", "x") as goalsf: pass
    
        with open("bf_goals.py", "wb") as goalsf:
            goalsf.write(requests.get(update_file_lines[2]).content)

        import global_funcs as g
        from bf_goals import GoalSpeed, GoalNosepos, GoalHeight, GoalPoint
            

update = g.update()

if update == 0:
    print("Updated, running TMIBruteforceGUI")
    os.system("python bf_gui.py")

elif update == 1:
    print("No updates found, running TMIBruteforceGUI")

elif update == 2:
    print("Declined update")


try:
    g.load_settings("autosave.json")
except:
    if not os.path.exists("autosave.json"):
        with open("autosave.json", "x") as autosave:
            g.save_settings("autosave.json")

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
WHEELS_SIZE = tuple([(SIMULATION_WHEELS_SIZE >> 2) * i for i in range(4)])
def getWheelContact(state):
        return [
            bool(unpack('i', state.simulation_wheels[i+292:i+296]))
            for i in WHEELS_SIZE
        ]

def nb_wheels_on_ground(state):
    return sum(list(getWheelContact(state)))

class MainClient(Client):
    def __init__(self) -> None:
        super().__init__()
        self.time = -1
        self.finished = False
        self.goal = GoalSpeed()

    def on_registered(self, iface: TMInterface) -> None:
        print(f'Registered to {iface.server_name}')
        g.settings_dict["is_registered"] = True
        g.settings_dict["server"] = iface.server_name

    def on_deregistered(self, iface: TMInterface):
        print(f'Deregistered from {iface.server_name}')
        g.settings_dict["is_registered"] = False

    def on_simulation_begin(self, iface):
        iface.execute_command('set controller bruteforce')

        self.lowest_time = iface.get_event_buffer().events_duration
        self.time = -1
        self.current = -1
        self.iterations = 0
        g.settings_dict["current_best"] = -1
        g.settings_dict["improvement_time"] = -1
        g.settings_dict["improvements"] = 0
        g.settings_dict["improvements_list"] = [0.0]

        g.unpackCoordinates()
        if g.settings_dict["current_goal"] == 0: self.goal = GoalSpeed()
        if g.settings_dict["current_goal"] == 1: self.goal = GoalNosepos()
        if g.settings_dict["current_goal"] == 2: self.goal = GoalHeight()
        if g.settings_dict["current_goal"] == 3: self.goal = GoalPoint()

    def on_bruteforce_evaluate(self, iface, info: BFEvaluationInfo) -> BFEvaluationResponse:
        self.time = info.time
        self.phase = info.phase

        response = BFEvaluationResponse()
        response.decision = BFEvaluationDecision.DO_NOTHING

        if g.settings_dict["time_min"] > self.time: # early return
            return response

        self.state = iface.get_simulation_state()

        # Initial phase (base run + after every ACCEPT improvement)
        # Check the all the ticks in eval_time and print the best one when run is in last tick of eval_time
        if self.phase == BFPhase.INITIAL:
            if self.is_eval_time() and self.is_better():
                g.settings_dict["current_best"] = self.current
                g.settings_dict["improvement_time"] = round(self.time/1000, 2)
                g.settings_dict["position"] = [round(pos, 3) for pos in self.state.position]
                g.settings_dict["velocity"] = [round(vel, 3) for vel in self.state.velocity]
                g.settings_dict["rotation"] = [round(to_deg(ypr), 3) for ypr in self.state.yaw_pitch_roll]

            if self.is_max_time():
                self.goal.print(g.Global())

        # Search phase only impacts decision, logic is in initial phase
        elif self.phase == BFPhase.SEARCH:
            if self.is_eval_time() and self.is_better():
                response.decision = BFEvaluationDecision.ACCEPT
                g.settings_dict["improvements"] += 1
                g.settings_dict["improvements_list"].append(float(g.settings_dict["current_best"]))

                self.iterations += 1
                if g.settings_dict["save_inputs"]:
                    self.save_result(filename=f"improvement_{g.improvements}.txt", event_buffer=iface.get_event_buffer())

            elif self.is_past_eval_time():
                response.decision = BFEvaluationDecision.REJECT
                self.iterations += 1
                if g.settings_dict["save_inputs"] and not g.settings_dict["save_only_results"]:
                    self.save_result(filename=f"iteration_{self.iterations}.txt", event_buffer=iface.get_event_buffer())

        return response

    def is_better(self):

        # Conditions
        if g.settings_dict["min_speed_kmh"] > numpy.linalg.norm(self.state.velocity) * 3.6: # Min speed
            return False

        if g.settings_dict["min_cp"] > get_nb_cp(self.state): # Min CP
            return False

        if g.settings_dict["must_touch_ground"] and nb_wheels_on_ground(self.state) == 0: # Touch ground
            return False
        
        if g.settings_dict["min_wheels_on_ground"] > nb_wheels_on_ground(self.state):
            return False

        if g.settings_dict["enable_position_check"] and not g.isCarInTrigger(self.state): # Position
            return False
        
        if g.settings_dict["enable_yaw_check"] and not g.isCarInMinMaxYaw(): # Yaw
            return False

        # Specific goal bruteforce
        # This line is a bit complicated, but for example, for speed it means: GoalSpeed.is_better(MainClient, g)
        return self.goal.is_better(self, g.Global())

    def is_eval_time(self):
        return g.settings_dict["time_min"] <= self.time <= g.settings_dict["time_max"]

    def is_past_eval_time(self):
        return self.time > g.settings_dict["time_max"]

    def is_max_time(self):
        return self.time == g.settings_dict["time_max"]

    def on_checkpoint_count_changed(self, iface: TMInterface, current: int, target: int):
        if current == target:
            self.finished = True

    def save_result(self, filename: str, event_buffer):
        # event_buffer is type EventBufferData

        # Find TMInterface/Scripts/
        scripts_dir = os.path.join(os.path.expanduser('~'), "Documents", "TMInterface", "Scripts")
        if not os.path.isdir(scripts_dir):
            # try OneDrive path
            scripts_dir = os.path.join(os.path.expanduser('~'), "OneDrive", "Documents", "TMInterface", "Scripts")
            if not os.path.isdir(scripts_dir):
                print("ERROR: path to Scripts/ not found.")
                return

        # Find or create directory to save inputs in this bruteforce session
        session_dir = os.path.join(scripts_dir, g.save_folder)
        if not os.path.isdir(session_dir):
            os.mkdir(session_dir)

        # Write inputs to a file
        filepath = os.path.join(session_dir, filename)
        with open(filepath, "w") as f:
            f.write(event_buffer.to_commands_str())

class GUI:
    def __init__(self):
        self.window = self.impl_glfw_init(width=700, height=500)
        gl.glClearColor(*g.settings_dict["background_color"])
        imgui.create_context()
        self.impl = GlfwRenderer(self.window)

        self.loop()

    def impl_glfw_init(self, window_name=f"TrackMania Bruteforce GUI {g.current_version}", width=300, height=300):
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

        # glfw.set_window_opacity(window, 0.5)
        # glfw.set_window_attrib(window, glfw.DECORATED, False)

        if not window:
            glfw.terminate()
            print("Could not initialize Window")
            exit(1)

        return window

    def bf_speed_gui(self):
        pass

    def bf_height_gui(self):
        pass

    def bf_nose_gui(self):
        g.settings_dict["enable_custom_yaw"] = imgui.checkbox("Enable Custom Yaw Value", g.settings_dict["enable_custom_yaw"])[1]

        if g.settings_dict["enable_custom_yaw"]:
            g.settings_dict["strategy"] = "custom"
            g.settings_dict["extra_yaw"] = imgui.input_float('Yaw', g.settings_dict["extra_yaw"])[1]
        else:
            g.settings_dict["strategy"] = "any"

    def bf_point_gui(self):
        g.settings_dict["point"] = imgui.input_float3('Point Coordinates', *g.settings_dict["point"])[1]

    def bf_conditions_gui(self):
        g.settings_dict["min_speed_kmh"] = imgui.input_int('Minimum Speed (km/h)', g.settings_dict["min_speed_kmh"])[1]
        g.settings_dict["min_cp"] = imgui.input_int('Minimum Checkpoints', g.settings_dict["min_cp"])[1]
        g.settings_dict["must_touch_ground"] = imgui.checkbox("Must touch ground (1 or more wheels must be touching any surface)", g.settings_dict["must_touch_ground"])[1]
        if g.settings_dict["must_touch_ground"]:
            g.settings_dict["min_wheels_on_ground"] = imgui.input_int("Minimum amount of wheels touching any surface", g.settings_dict["min_wheels_on_ground"])[1]

        # Position Check
        g.settings_dict["enable_position_check"] = imgui.checkbox("Enable Position check (Car must be inside Trigger)", g.settings_dict["enable_position_check"])[1]

        if g.settings_dict["enable_position_check"]:
            input_pair = lambda s, pair: imgui.input_float3(s, *pair)[1]
            g.settings_dict["trigger_corner_1"], g.settings_dict["trigger_corner_2"] = input_pair('Trigger Corner 1', g.settings_dict["trigger_corner_1"]), input_pair('Trigger Corner 2', g.settings_dict["trigger_corner_2"])
        
        # Yaw Check
        g.settings_dict["enable_yaw_check"] = imgui.checkbox("Enable Yaw check (Car must be between 2 Yaw values)", g.settings_dict["enable_yaw_check"])[1]
        
        if g.settings_dict["enable_yaw_check"]:
            g.settings_dict["min_yaw"] = imgui.input_float('Minimum Yaw', g.settings_dict["min_yaw"])[1]
            g.settings_dict["max_yaw"] = imgui.input_float('Maximum Yaw', g.settings_dict["max_yaw"])[1]

    def bf_other_gui(self):
        g.settings_dict["save_inputs"] = imgui.checkbox("Save inputs of every iteration and/or improvements separately in a folder", g.settings_dict["save_inputs"])[1]
        if g.settings_dict["save_inputs"]:
            g.settings_dict["save_folder"] = imgui.input_text('Folder Name', g.settings_dict["save_folder"], 256)[1]
            g.settings_dict["save_only_results"] = imgui.checkbox("Save only improvements separately", g.settings_dict["save_only_results"])[1]

    def save_settings_gui(self):
        save = imgui.button("Save Settings")
        if save:
            g.save_settings(g.settings_dict["settings_file_name"])

    def load_settings_gui(self):
        load = imgui.button("Load Settings")
        if load:
            g.load_settings(g.settings_dict["settings_file_name"])

    def settings_file_name_gui(self):
        g.settings_dict["settings_file_name"] = imgui.input_text("Settings File Name", g.settings_dict["settings_file_name"], 256)[1] 

    def bf_settings(self):
        imgui.begin("Evaluation Settings", True)

        imgui.text("Goal and parameters")
        g.settings_dict["current_goal"] = imgui.combo("Bruteforce Goal", g.settings_dict["current_goal"], g.settings_dict["goals"])[1]
        if g.settings_dict["current_goal"] == 0: self.bf_speed_gui()
        elif g.settings_dict["current_goal"] == 1: self.bf_nose_gui()
        elif g.settings_dict["current_goal"] == 2: self.bf_height_gui()
        elif g.settings_dict["current_goal"] == 3: self.bf_point_gui()

        imgui.separator()

        imgui.text("Evaluation time (in seconds)")
        timetext = lambda s, t: round(imgui.input_float(s, t/1000)[1] * 1000, 3)
        g.settings_dict["time_min"] = timetext("Evaluation start (s)", g.settings_dict["time_min"])
        g.settings_dict["time_max"] = timetext("Evaluation end (s)", g.settings_dict["time_max"])
        if g.settings_dict["time_min"] > g.settings_dict["time_max"]:
            g.settings_dict["time_max"] = g.settings_dict["time_min"]

        imgui.separator()

        imgui.text("Conditions")
        self.bf_conditions_gui()

        imgui.separator()

        imgui.text("Other")
        self.bf_other_gui()

        imgui.separator()

        imgui.text("Settings")
        self.save_settings_gui()
        self.load_settings_gui()
        self.settings_file_name_gui()

        imgui.end()

    def bf_result(self):
        # thanks shweetz
        # this is to clarify what unit of measurement is currently used
        if g.settings_dict["current_goal"] == 0: unit = "(km/h)" # Speed
        elif g.settings_dict["current_goal"] == 1: unit = "(degrees)" # Nosepos
        else: unit = "(m)" # Point/Height

        imgui.begin("Bruteforce Info", True)

        imgui.text("Connection Status: " + (f"Connected to {g.settings_dict['server']}" if g.settings_dict['is_registered'] else "Not Registered"))
        imgui.separator()

        best = g.settings_dict["current_best"]
        if g.settings_dict["current_goal"] == 3 and best > 0: # Point
            best = math.sqrt(best)

        imgui.text(f"Bruteforce Best: {round(best, 3)} {unit}")

        imgui.text(f"Improvements: {g.settings_dict['improvements']}")
        imgui.text(f"Car information at {g.settings_dict['improvement_time']}:")

        imgui.separator()

        imgui.text(f"Position (x, y, z): {g.settings_dict['position']}")
        imgui.text(f"Velocity (x, y, z): {g.settings_dict['velocity']}")
        imgui.text(f"Rotation (yaw, pitch, roll): {g.settings_dict['rotation']}")

        imgui.separator()
        
        g.settings_dict["improvement_graph"] = imgui.checkbox("Enable improvement graph", g.settings_dict["improvement_graph"])[1]

        imgui.end()

    def bf_improvement_graph(self):
        if not g.settings_dict["improvement_graph"]:
            pass
        else:
            imgui.begin("Improvement Graph", True)
            # The "##" is to make the name disappear, it is used to clarify what this plot is for in code.    
            improvement = g.settings_dict["improvements_list"][len(g.settings_dict["improvements_list"])-1]
            if improvement > g.settings_dict["improvement_graph_scale"]: g.settings_dict["improvement_graph_scale"] = improvement

            imgui.plot_lines(
                "##Improvement Graph", 
                np.array(g.settings_dict["improvements_list"], np.float32), 
                graph_size=(700, 400),
                scale_max=g.settings_dict["improvement_graph_scale"] 
            )
            
            imgui.end()


    def customize(self):
        imgui.begin("Customize", True)

        if imgui.button("Start RGB scroll" if not g.settings_dict["rgb_scroll"] else "Stop RGB Scroll"):
            g.settings_dict["rgb_scroll"] = not g.settings_dict["rgb_scroll"]

        if g.settings_dict["rgb_scroll"]:
            if not any(g.settings_dict["color"][:3]): g.settings_dict["color"] = [(i + 1) / 4 for i in range(4)]
            g.settings_dict["color_change"] = imgui.slider_float(
                "Speed", g.settings_dict["color_change"],
                min_value=0, max_value=32,
                power=1
            )[1]

        else:
            g.settings_dict["color"] = list(imgui.color_edit4("Background", *g.settings_dict["color"], show_alpha=False)[1])
            g.settings_dict["background_color"] = g.settings_dict["color"].copy()

        imgui.end()

    def loop(self):
        while not glfw.window_should_close(self.window):
            glfw.poll_events()
            self.impl.process_inputs()
            imgui.new_frame()

            self.bf_result()
            self.bf_settings()
            self.bf_improvement_graph()
            self.customize()

            imgui.render()

            if g.settings_dict["rgb_scroll"]:
                g.settings_dict["color"] = r2h(*g.settings_dict["color"]) # convert to hsv
                g.settings_dict["color"][0] = (g.settings_dict["color"][0] + g.settings_dict["color_change"]/1000000) % 1 # add hue (division is because of very high FPS)
                g.settings_dict["color"] = h2r(*g.settings_dict["color"]) # convert back into rgb

            g.settings_dict["background_color"] = g.settings_dict["color"].copy() # set the self.color

            gl.glClearColor(*g.settings_dict["background_color"])
            gl.glClear(gl.GL_COLOR_BUFFER_BIT)

            self.impl.render(imgui.get_draw_data())
            glfw.swap_buffers(self.window)

        self.impl.shutdown()
        glfw.terminate()  

        g.save_settings("autosave.json")
        exit()

def main():
    if len(sys.argv) > 2:
        for id in range(int(sys.argv[1]), int(sys.argv[2]) + 1):
                server_name = f'TMInterface{id}'
                client = MainClient()
                iface = TMInterface(server_name)
                iface.register(client)
                g.registered_ids.append(id)

    else:
        server_name = f'TMInterface{sys.argv[1]}' if len(sys.argv) > 1 else 'TMInterface0'
        print(f'Connecting to {server_name}...')
        client = MainClient()
        iface = TMInterface(server_name)
        iface.register(client)

    def handler(signum, frame):
        iface.close()
        quit()

    def on_exit(signal_type):
        g.save_settings("autosave.json")
    
    win32api.SetConsoleCtrlHandler(on_exit, True)

    signal.signal(signal.SIGBREAK, handler)
    signal.signal(signal.SIGINT, handler)

    while not iface.registered:
        time.sleep(0)

    while iface.registered:
        time.sleep(0)

if __name__ == '__main__':
    GUI_thread = threading.Thread(target=makeGUI, daemon=True)
    GUI_thread.start()

    main()
