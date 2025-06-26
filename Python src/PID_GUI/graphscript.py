import serial
import struct
import threading
import tkinter as tk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import time
import re

# ───────────────────────────────────────────────────────────
# SERIAL SETTINGS
SERIAL_PORT = "COM14"        # ← change to your port
BAUD_RATE   = 115200
# ───────────────────────────────────────────────────────────
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)

# Data storage for telemetry
time_stamps    = []
desired_roll   = []
actual_roll    = []
desired_pitch  = []
actual_pitch   = []
desired_yaw    = []
actual_yaw     = []

# Lock for thread safety
data_lock = threading.Lock()

# Default view range (seconds)
VIEW_WINDOW = 10
paused      = False

# Default PID and Offset values
pid_values = {
    "Roll_KP": 7.0,  "Roll_KI": 0.1,  "Roll_KD": 1.5,
    "Pitch_KP":7.0, "Pitch_KI":0.1,  "Pitch_KD":1.5,
    "Yaw_KP": 7.0,  "Yaw_KI": 0.5,  "Yaw_KD":0.15,
    # Offsets
    "Roll_offset":0.0, "Pitch_offset":0.0, "Yaw_offset":0.0
}

# Regexes to distinguish incoming lines
telemetry_re = re.compile(
    r'^\s*-?\d+(?:\.\d+)?,-?\d+(?:\.\d+)?,-?\d+(?:\.\d+)?,'
    r'-?\d+(?:\.\d+)?,-?\d+(?:\.\d+)?,-?\d+(?:\.\d+)?\s*$'
)
pid_ack_re = re.compile(r'^PID cmd idx=\d+ val=[0-9.+\-eE]+ → (OK|ERR)$')

# ───────────────────────────────────────────────────────────
def receive_serial():
    """Read lines from serial, handle telemetry and PID-ACK logs."""
    while True:
        raw = ser.readline()
        try:
            line = raw.decode('utf-8', errors='ignore').strip()
        except:
            continue
        if not line:
            continue

        # Telemetry?
        if telemetry_re.match(line):
            parts = line.split(',')
            d_roll, d_pitch, d_yaw, a_roll, a_pitch, a_yaw = map(float, parts)
            with data_lock:
                now = time.time()
                time_stamps.append(now)
                desired_roll.append(d_roll)
                actual_roll.append(a_roll)
                desired_pitch.append(d_pitch)
                actual_pitch.append(a_pitch)
                desired_yaw.append(d_yaw)
                actual_yaw.append(a_yaw)

        # PID-ACK log?
        elif pid_ack_re.match(line):
            log_text.insert(tk.END, line + '\n')
            log_text.see(tk.END)

# ───────────────────────────────────────────────────────────
# GUI SETUP
root = tk.Tk()
root.title("PID Controller & Real-Time Graph (Serial+ACK)")
root.state('zoomed')
root.attributes('-fullscreen', True)
root.bind("<Escape>", lambda e: root.attributes('-fullscreen', False))

main_frame = tk.Frame(root)
main_frame.pack(fill=tk.BOTH, expand=True)

# Left panel: PID & Offset inputs and ACK log
left_frame = tk.Frame(main_frame, padx=20, pady=20, bg="lightgray")
left_frame.pack(side=tk.LEFT, fill=tk.Y)

# Status label\status_label = tk.Label(left_frame, text="", fg="black", bg="lightgray")
status_label.pack(pady=(0,10))

def send_pid_value(param_name, entry_widget):
    try:
        new_value = float(entry_widget.get())
        pid_values[param_name] = new_value
        param_index = list(pid_values.keys()).index(param_name)
        packet = struct.pack('if', param_index, new_value)
        ser.write(packet)
        status_label.config(text=f"Sent {param_name}={new_value:.3f}", fg="green")
    except ValueError:
        status_label.config(text="Invalid number!", fg="red")

# PID and Offset UI
tk.Label(
    left_frame,
    text="Controller Parameters",
    font=("Arial", 16, "bold"),
    bg="lightgray"
).pack(pady=(0, 10))

entry_widgets = {}
for param in pid_values:
    frame = tk.Frame(left_frame, bg="lightgray")
    frame.pack(fill=tk.X, pady=2)
    tk.Label(
        frame, text=f"{param}:", width=12, anchor='w', bg="lightgray"
    ).pack(side=tk.LEFT)
    entry = tk.Entry(frame, width=8)
    entry.insert(0, str(pid_values[param]))
    entry.pack(side=tk.LEFT, padx=(0, 10))
    entry_widgets[param] = entry
    tk.Button(
        frame, text="Update",
        command=lambda p=param, e=entry: send_pid_value(p, e)
    ).pack(side=tk.LEFT)

# ACK log below inputs
tk.Label(left_frame, text="Ground-Station ACK Log",
         font=("Arial",12,"bold"), bg="lightgray").pack(pady=(10,0))
log_text = tk.Text(left_frame, height=8, width=30)
log_text.pack(fill=tk.X, pady=(0,10))

# Pause button

def toggle_pause():
    global paused
    paused = not paused

tk.Button(
    left_frame,
    text="Pause/Resume",
    command=toggle_pause,
    bg="lightblue"
).pack(pady=10)

# Right panel: real-time plots
right_frame = tk.Frame(main_frame)
right_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 8))
canvas = FigureCanvasTkAgg(fig, master=right_frame)
canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

scrollbar = tk.Scale(
    root,
    from_=0, to=100,
    orient=tk.HORIZONTAL,
    length=500,
    command=lambda v: update_xlim(float(v))
)
scrollbar.pack(side=tk.BOTTOM, fill=tk.X)


def update_graph(frame):
    if paused:
        return
    with data_lock:
        if not time_stamps:
            return
        start = max(time_stamps[0], time_stamps[-1] - VIEW_WINDOW)
        end   = time_stamps[-1]

        ax1.clear(); ax2.clear(); ax3.clear()
        ax1.plot(time_stamps, desired_roll, '--', label="Desired Roll")
        ax1.plot(time_stamps, actual_roll,        label="Actual Roll")
        ax1.set_xlim(start,end); ax1.set_title("Roll (°)"); ax1.legend()

        ax2.plot(time_stamps, desired_pitch, '--', label="Desired Pitch")
        ax2.plot(time_stamps, actual_pitch,       label="Actual Pitch")
        ax2.set_xlim(start,end); ax2.set_title("Pitch (°)"); ax2.legend()

        ax3.plot(time_stamps, desired_yaw, '--',  label="Desired Yaw")
        ax3.plot(time_stamps, actual_yaw,         label="Actual Yaw")
        ax3.set_xlim(start,end); ax3.set_title("Yaw (°)"); ax3.legend()

        scrollbar.config(
            from_=time_stamps[0],
            to=time_stamps[-1] - VIEW_WINDOW
        )
        canvas.draw()


def update_xlim(val):
    with data_lock:
        if not time_stamps:
            return
        new_start = val
        new_end   = new_start + VIEW_WINDOW
        for ax in (ax1,ax2,ax3):
            ax.set_xlim(new_start,new_end)
        canvas.draw()

ani = animation.FuncAnimation(fig, update_graph, interval=100, cache_frame_data=False)
threading.Thread(target=receive_serial, daemon=True).start()
root.mainloop()
