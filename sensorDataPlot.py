#!/usr/bin/python3

import re
import sys
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

time = [0]
acc_x, acc_y, acc_z = [], [], []
gyr_x, gyr_y, gyr_z = [], [], []
acc_mag, gyr_mag = [], []
data_file = None
axes = []
fall_timestamps = []
plot_lines = {}


INPUT_REGEX = re.compile(r"Accelerometer Data: \((?P<acc_x>-?\d+), (?P<acc_y>-?\d+), (?P<acc_z>-?\d+)\), "
                         r"Gyroscope Data: \((?P<gyr_x>-?\d+), (?P<gyr_y>-?\d+), (?P<gyr_z>-?\d+)\), "
                         r"Acc_mag = (?P<acc_mag>\d+\.\d+), Gyr_mag = (?P<gyr_mag>\d+\.\d+)")

def update(frame):
    global data_file, axes, plot_lines, fall_timestamps
    while True:
        pos = data_file.tell()
        line = data_file.readline()
        if not line:
            data_file.seek(pos)
            break
        if parse_line(line):
            time.append(time[-1] + 0.04)
        elif "Fall Detected!" in line:
            t_fall = time[-1]
            if t_fall not in fall_timestamps:
                fall_timestamps.append(t_fall)
                for ax in axes:
                    ax.axvline(x=t_fall, color='red', linestyle='--', linewidth=1)
            continue
        if len(time) > 1:
            plot_lines['acc_x'].set_data(time, acc_x)
            plot_lines['acc_y'].set_data(time, acc_y)
            plot_lines['acc_z'].set_data(time, acc_z)

            plot_lines['gyr_x'].set_data(time, gyr_x)
            plot_lines['gyr_y'].set_data(time, gyr_y)
            plot_lines['gyr_z'].set_data(time, gyr_z)

            plot_lines['acc_mag'].set_data(time, acc_mag)
            plot_lines['gyr_mag'].set_data(time, gyr_mag)

        for ax in axes:
                ax.relim()
                ax.autoscale_view()

def main():
    global data_file, axes, plot_lines
    try:
        data_file = open("./tmp.txt", "r")
        while True:
            line = data_file.readline()
            if "Recording Sensor Data" in line:
                break
        line = data_file.readline() 
        parse_line(line) 
        #line = data_file.readline()
        #parse_line(line)
        #time.append(time[-1] + 0.04)

        fig, axes1 = plt.subplots(4, 1, sharex=True, figsize = (14, 10))
        axes = axes1
        
        plot_lines['acc_x'], = axes[0].plot([], [], label="Acc X")
        plot_lines['acc_y'], = axes[0].plot([], [], label="Acc Y")
        plot_lines['acc_z'], = axes[0].plot([], [], label="Acc Z")
        axes[0].set_ylabel("Accerometer Components")
        axes[0].legend()
 
        plot_lines['gyr_x'], = axes[1].plot([], [], label="Gyr X")
        plot_lines['gyr_y'], = axes[1].plot([], [], label="Gyr Y")
        plot_lines['gyr_z'], = axes[1].plot([], [], label="Gyr Z")
        axes[1].set_ylabel("Gyroscope Components")
        axes[1].legend()
        
        plot_lines['acc_mag'], = axes[2].plot([], [])
        axes[2].set_ylabel("Accelerometer Magnitude")
        
        plot_lines['gyr_mag'], = axes[3].plot([], []) 
        axes[3].set_ylabel("Gyroscope Magnitude")
       

        ani = FuncAnimation(fig, update, interval = 250)
        plt.show()

        #print("Program exited successfully!")
        #sys.exit(0)
    except Exception as e:
        print(f"An error occured: {e}")
        sys.exit(1)

def parse_line(line):
    match = INPUT_REGEX.match(line)
    if match:
        parsed_data = match.groupdict()
        acc_x.append(int(parsed_data['acc_x']))
        acc_y.append(int(parsed_data['acc_y']))
        acc_z.append(int(parsed_data['acc_z']))
        gyr_x.append(int(parsed_data['gyr_x']))
        gyr_y.append(int(parsed_data['gyr_y']))
        gyr_z.append(int(parsed_data['gyr_z']))
        acc_mag.append(float(parsed_data['acc_mag']))
        gyr_mag.append(float(parsed_data['gyr_mag']))
        return True
    else:
        return False

if __name__ == "__main__":
    main() 
