import serial
import json
import threading
import tkinter as tk
from tkinter import ttk
from tkintermapview import TkinterMapView
from PIL import Image, ImageTk

# ───────────────────────────────────────────────────────────
# SERIAL (Ground Station) SETTINGS
SERIAL_PORT = "COM14"    # ← change to your port
BAUD_RATE   = 115200
# ───────────────────────────────────────────────────────────
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)

# Global state
gps_data_lock = threading.Lock()
drone_coords = {"Master": (None, None), "Slave 1": (None, None), "Slave 2": (None, None)}
drone_markers = {k: None for k in drone_coords}
drone_paths = {k: [] for k in drone_coords}
drone_path_lines = {k: None for k in drone_coords}
drone_visibility = {k: True for k in drone_coords}
paths_visible = {"enabled": True}

# ───────────────────────────────────────────────────────────
# GUI setup
root = tk.Tk()
root.title("Swarm Drone Tracker")
root.geometry("950x680")

map_widget = TkinterMapView(root, width=950, height=580, corner_radius=0)
map_widget.pack(fill="both", expand=True)

# Re-register the OpenStreetMap server but tell it “hey, max zoom is 50”
map_widget.set_tile_server(
    "https://a.tile.openstreetmap.org/{z}/{x}/{y}.png",
    max_zoom=50
)

# Then bump your zoom
map_widget.set_zoom(50)


# Load icons
master_icon = ImageTk.PhotoImage(
    Image.open("master_drone.png").resize((100, 100), Image.Resampling.LANCZOS)
)
slave_icon = ImageTk.PhotoImage(
    Image.open("slave_drone.png").resize((60, 60), Image.Resampling.LANCZOS)
)

# Live master location label
location_label = ttk.Label(root, text="Master Drone Location: --, --", font=("Arial", 10))
location_label.pack(side="right", padx=10, pady=5)

# Button frame
button_frame = ttk.Frame(root)
button_frame.pack(pady=5)

# Toggle drone visibility
def toggle_drone_visibility(drone_label):
    with gps_data_lock:
        drone_visibility[drone_label] = not drone_visibility[drone_label]
        if drone_markers[drone_label]:
            drone_markers[drone_label].delete()
            drone_markers[drone_label] = None
        if drone_path_lines[drone_label]:
            drone_path_lines[drone_label].delete()
            drone_path_lines[drone_label] = None

for i, drone_label in enumerate(drone_coords.keys()):
    ttk.Button(
        button_frame,
        text=f"Toggle {drone_label}",
        command=lambda l=drone_label: toggle_drone_visibility(l)
    ).grid(row=0, column=i, padx=5)

# Toggle all paths
def toggle_all_paths():
    with gps_data_lock:
        paths_visible["enabled"] = not paths_visible["enabled"]
        for drone_label in drone_path_lines:
            if drone_path_lines[drone_label]:
                drone_path_lines[drone_label].delete()
                drone_path_lines[drone_label] = None

ttk.Button(button_frame, text="Toggle Paths", command=toggle_all_paths).grid(row=0, column=len(drone_coords), padx=5)

# ───────────────────────────────────────────────────────────
# Serial listener (replaces UDP)
def data_listener():
    while True:
        raw = ser.readline()
        if not raw:
            continue
        line = raw.decode('utf-8', errors='ignore').strip()
        try:
            parsed = json.loads(line)
        except json.JSONDecodeError:
            continue
        with gps_data_lock:
            if "drone1" in parsed:
                print(parsed["drone1"]["lat"])
                drone_coords["Master"] = (parsed["drone1"]["lat"], parsed["drone1"]["lng"])
                if drone_visibility["Master"]:
                    drone_paths["Master"].append(drone_coords["Master"])
            if "drone2" in parsed:
                drone_coords["Slave 1"] = (parsed["drone2"]["lat"], parsed["drone2"]["lng"])
                if drone_visibility["Slave 1"]:
                    drone_paths["Slave 1"].append(drone_coords["Slave 1"])
            if "drone3" in parsed:
                drone_coords["Slave 2"] = (parsed["drone3"]["lat"], parsed["drone3"]["lng"])
                if drone_visibility["Slave 2"]:
                    drone_paths["Slave 2"].append(drone_coords["Slave 2"])

# ───────────────────────────────────────────────────────────
# Map update loop
def update_map():
    with gps_data_lock:
        for drone_label, (lat, lng) in drone_coords.items():
            if lat is not None and lng is not None and drone_visibility[drone_label]:
                icon = master_icon if drone_label == "Master" else slave_icon
                if drone_markers[drone_label]:
                    drone_markers[drone_label].set_position(lat, lng)
                else:
                    drone_markers[drone_label] = map_widget.set_marker(lat, lng, text=drone_label, icon=icon)

                if paths_visible["enabled"] and len(drone_paths[drone_label]) >= 2:
                    if drone_path_lines[drone_label]:
                        drone_path_lines[drone_label].set_position_list(drone_paths[drone_label])
                    else:
                        drone_path_lines[drone_label] = map_widget.set_path(drone_paths[drone_label])
                elif not paths_visible["enabled"] and drone_path_lines[drone_label]:
                    drone_path_lines[drone_label].delete()
                    drone_path_lines[drone_label] = None

        mlat, mlng = drone_coords["Master"]
        if mlat is not None and mlng is not None:
            location_label.config(text=f"Master Drone Location: {mlat:.6f}, {mlng:.6f}")
            map_widget.set_position(mlat, mlng)

    root.after(1000, update_map)

# Start listener and GUI loop
threading.Thread(target=data_listener, daemon=True).start()
update_map()
root.mainloop()
