import requests
import ctypes
import os
import json

settings_dict = {
    # Server
    "registered_ids": [],
    "is_registered": False,
    "server": "",

    # Goal
    "current_goal": 0, # 0 Speed, 1 Nosepos, 2 Height, 3 Point
    "strategy": "any",
    "extra_yaw": 0,
    "point": [0, 0, 0],
    "time_min": 0,
    "time_max": 0,

    # Conditions
    "enable_position_check": False,
    "enable_yaw_check": False,
    "enable_custom_yaw": False,
    "trigger_corner_1": [0, 0, 0],
    "trigger_corner_2": [999, 999, 999],
    "min_yaw": 0.000,
    "max_yaw": 999.000,
    "min_speed_kmh": 0,
    "min_cp": 0,
    "must_touch_ground": False,
    "min_wheels_on_ground": 0,

    # Result
    "current_best": -1,
    "improvement_time": 0,
    "improvements": 0,
    "position": [0, 0, 0],
    "velocity": [0, 0, 0],
    "rotation": [0, 0, 0],

    # Saving Inputs

    "save_inputs": False,
    "save_folder": "current",
    "save_only_results": False,
    # Color

    "color": [0.25, 0.5, 0.75, 0.5],
    "bgcolor": [0.25, 0.5, 0.75, 0.5], # this is from glfw (don't ask ok)
    "color_change": 0, # you can change this if you want
    "rgb_scroll": False, # chroma background flag
    "goals": ["Speed", "Nosebug position", "Height", "Minimum distance from point"],
    "background_color": [0.25, 0.5, 0.75, 0.5],

    # Improvement Graph
    "improvements_list": [0.0],
    "improvement_graph": False,
    "improvement_graph_scale": 0,

    # Settings
    "settings_file_name": "settings.json"
}

update_file_url = 'https://raw.githubusercontent.com/CodyNinja1/TMIBruteforceGUI/main/bf_gui_version.txt' # This should always stay the same
update_file_lines = requests.get(update_file_url).text.split("\n")
version = (update_file_lines[0][:16] + "...") if len(update_file_lines[0]) > 16 else update_file_lines[0]
current_version = "v1.0.0"


def save_settings(filename):
    """Save bruteforce settings, takes filename as argument"""
    with open(filename, "w") as settings_file:
        json.dump(settings_dict, settings_file, sort_keys=True, indent=4)

def load_settings(filename):
    """Load bruteforce settings, takes filename as argument"""
    try:
        with open(filename, "r") as settings_file:
            settings = json.load(settings_file)

            settings_dict["current_goal"] = settings["current_goal"]
            settings_dict["extra_yaw"] = settings["extra_yaw"]
            settings_dict["point"] = settings["point"]

            settings_dict["enable_position_check"] = settings["enable_position_check"]
            settings_dict["enable_yaw_check"] = settings["enable_yaw_check"]
            settings_dict["enable_custom_yaw"] = settings["enable_custom_yaw"]
            settings_dict["trigger_corner_1"] = settings["trigger_corner_1"]

            settings_dict["trigger_corner_2"] = settings["trigger_corner_2"]

            settings_dict["min_yaw"] = settings["min_yaw"]
            settings_dict["max_yaw"] = settings["max_yaw"]

            settings_dict["save_inputs"] = settings["save_inputs"]
            settings_dict["save_folder"] = settings["save_folder"]
            settings_dict["save_only_results"] = settings["save_only_results"]

            settings_dict["color"] = settings["background_color"]

            settings_dict["improvement_graph"] = settings["improvement_graph"]

            settings_dict["time_min"] = settings["time_min"]
            settings_dict["time_max"] = settings["time_max"]

            settings_dict["min_speed_kmh"] = settings["min_speed_kmh"]
            settings_dict["min_cp"] = settings["min_cp"]
            settings_dict["must_touch_ground"] = settings["must_touch_ground"]
            settings_dict["min_wheels_on_ground"] = settings["min_wheels_on_ground"]
            settings_dict["settings_file_name"] = settings["settings_file_name"]

    except:
        if not os.path.exists(filename):
            with open(filename, "x") as settings_file: pass
            save_settings(filename)

def unpackCoordinates():
    """Execute only once, on simulation start"""
    global minX, maxX, minY, maxY, minZ, maxZ
    (minX, maxX), (minY, maxY), (minZ, maxZ) = [
        sorted((round(settings_dict["trigger_corner_1"][i], 2), round(settings_dict["trigger_corner_2"][i], 2))) for i in range(3)
    ]

def isCarInTrigger(state):
    """Execute every tick where is_eval_time() is True"""
    car_x, car_y, car_z = state.position
    return minX <= car_x <= maxX and minY <= car_y <= maxY and minZ <= car_z <= maxZ

def isCarInMinMaxYaw():
    yaw = settings_dict["rotation"][0]
    return settings_dict["min_yaw"] <= yaw <= settings_dict["max_yaw"]

def update():
    """
    Prompts user to update if they are on an out of date version, automatically replaces old files
    Returns 0 if it was updated, 1 if not, 2 if there was a new update but the user declined it
    """
    accepted_update = None 

    ICON_INFO = 0x40
    ICON_WARNING = 0x30
    MB_OK = 0x0
    MB_YESNO = 0x4

    print("Checking for updates...")
    if version != current_version:
        print(f"Found new update, new version: {version}, current version: {current_version}")
        accepted_update = ctypes.windll.user32.MessageBoxW(0, f"New update available! Would you like to install the newest version?\n(Warning: This will replace any code you have changed)", f"{version} Version Available!", ICON_WARNING | MB_YESNO)
    

    if accepted_update == 6:
        download = lambda file_name, file_url : open(file_name, 'wb').write(file_url.content)

        download("bf_gui.py", requests.get(update_file_lines[1]))
        download("bf_goals.py", requests.get(update_file_lines[2]))
        download("global_funcs.py", requests.get(update_file_lines[3]))
        download("requirements.txt", requests.get(update_file_lines[4]))

        ctypes.windll.user32.MessageBoxW(0, "Done updating, all necessary files have been replaced\nProgram should automatically restart", "Update Complete", MB_OK | ICON_INFO)

        return 0

    elif accepted_update == 7: return 2

    else: return 1
