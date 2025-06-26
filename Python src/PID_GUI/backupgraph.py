import socket
import struct
import threading
import tkinter as tk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import time

# TCP Server settings
PC_IP = "0.0.0.0"
PC_PORT = 1250

# Create TCP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.bind((PC_IP, PC_PORT))
sock.listen(1)
print("Waiting for ESP32 connection...")

conn, addr = sock.accept()
print(f"Connected to ESP32: {addr}")

# Data storage
time_stamps = []
desired_roll, actual_roll = [], []
desired_pitch, actual_pitch = [], []
desired_yaw, actual_yaw = [], []

# Lock for thread safety
data_lock = threading.Lock()

# Default view range (seconds)
VIEW_WINDOW = 10  # Show only the last 10 seconds in the graph
paused = False  # Pause flag

# Default PID values
pid_values = {
    "Roll_KP": 2.5, "Roll_KI": 0.0, "Roll_KD": 0.0,
    "Pitch_KP": 0.0, "Pitch_KI": 0.0, "Pitch_KD": 0.0,
    "Yaw_KP": 0.0, "Yaw_KI": 0.0, "Yaw_KD": 0.0
}


def receive_data():
    while True:
        try:
            data = conn.recv(24)  # Expecting 6 float values (4 bytes each)
            if len(data) == 24:
                d_roll, d_pitch, d_yaw, a_roll, a_pitch, a_yaw = struct.unpack('ffffff', data)

                with data_lock:
                    current_time = time.time()
                    time_stamps.append(current_time)
                    desired_roll.append(d_roll)
                    actual_roll.append(a_roll)
                    desired_pitch.append(d_pitch)
                    actual_pitch.append(a_pitch)
                    desired_yaw.append(d_yaw)
                    actual_yaw.append(a_yaw)
        except:
            print("Connection lost, waiting for reconnection...")
            break

        # GUI Setup


root = tk.Tk()
root.title("PID Controller & Real-Time Graph")
root.state('zoomed')
root.attributes('-fullscreen', True)


def exit_fullscreen(event):
    root.attributes('-fullscreen', False)


root.bind("<Escape>", exit_fullscreen)

main_frame = tk.Frame(root)
main_frame.pack(fill=tk.BOTH, expand=True)

left_frame = tk.Frame(main_frame, padx=20, pady=20, bg="lightgray")
left_frame.pack(side=tk.LEFT, fill=tk.Y)

right_frame = tk.Frame(main_frame)
right_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 8))
canvas = FigureCanvasTkAgg(fig, master=right_frame)
canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

scrollbar = tk.Scale(root, from_=0, to=100, orient=tk.HORIZONTAL, length=500,
                     command=lambda val: update_xlim(float(val)))
scrollbar.pack(side=tk.BOTTOM, fill=tk.X)


def send_pid_value(param_name, entry_widget):
    try:
        new_value = float(entry_widget.get())
        pid_values[param_name] = new_value

        param_index = list(pid_values.keys()).index(param_name)
        packet = struct.pack('if', param_index, new_value)
        conn.sendall(packet)

        status_label.config(text=f"Updated {param_name} to {new_value}", fg="green")
        print(f"Sent {param_name}: {new_value}")
    except ValueError:
        status_label.config(text="Invalid input! Enter a valid number.", fg="red")


entry_widgets = {}
tk.Label(left_frame, text="PID Controller Adjustment", font=("Arial", 16, "bold"), bg="lightgray").pack()

for param in pid_values:
    frame = tk.Frame(left_frame, bg="lightgray")
    frame.pack()
    tk.Label(frame, text=f"{param}:", bg="lightgray").pack(side=tk.LEFT)
    entry = tk.Entry(frame)
    entry.insert(0, str(pid_values[param]))
    entry.pack(side=tk.LEFT)
    entry_widgets[param] = entry
    tk.Button(frame, text="Update", command=lambda p=param, e=entry: send_pid_value(p, e)).pack(side=tk.LEFT)


def update_graph(frame):
    if paused:
        return

    with data_lock:
        if not time_stamps:
            return

        start_time = max(time_stamps[0], time_stamps[-1] - VIEW_WINDOW)
        end_time = time_stamps[-1]

        ax1.clear()
        ax2.clear()
        ax3.clear()

        ax1.plot(time_stamps, desired_roll, label="Desired Roll", linestyle='dashed')
        ax1.plot(time_stamps, actual_roll, label="Actual Roll")
        ax1.set_xlim(start_time, end_time)
        ax1.set_title("Roll (Degrees)")
        ax1.legend()

        ax2.plot(time_stamps, desired_pitch, label="Desired Pitch", linestyle='dashed')
        ax2.plot(time_stamps, actual_pitch, label="Actual Pitch")
        ax2.set_xlim(start_time, end_time)
        ax2.set_title("Pitch (Degrees)")
        ax2.legend()

        ax3.plot(time_stamps, desired_yaw, label="Desired Yaw", linestyle='dashed')
        ax3.plot(time_stamps, actual_yaw, label="Actual Yaw")
        ax3.set_xlim(start_time, end_time)
        ax3.set_title("Yaw (Degrees)")
        ax3.legend()

        scrollbar.config(from_=time_stamps[0], to=time_stamps[-1] - VIEW_WINDOW)

        canvas.draw()


def update_xlim(value):
    with data_lock:
        if not time_stamps:
            return
        new_start = value
        new_end = new_start + VIEW_WINDOW
        ax1.set_xlim(new_start, new_end)
        ax2.set_xlim(new_start, new_end)
        ax3.set_xlim(new_start, new_end)
        canvas.draw()


def toggle_pause():
    global paused
    paused = not paused
    tk.Button(left_frame, text="Pause", command=toggle_pause, bg="lightblue").pack(pady=10)


tk.Button(left_frame, text="Pause", command=toggle_pause, bg="lightblue").pack(pady=10)
status_label = tk.Label(left_frame, text="", fg="black", bg="lightgray")
status_label.pack()

ani = animation.FuncAnimation(fig, update_graph, interval=100)
receiver_thread = threading.Thread(target=receive_data, daemon=True)
receiver_thread.start()

root.mainloop()
