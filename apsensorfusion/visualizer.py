import os
import serial
import serial.tools.list_ports
import threading
import matplotlib.pyplot as plt
from matplotlib import animation, text
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.widgets import RadioButtons

uart = None

def set_serial(port):
    global uart
    if uart is not None:
        uart.close()
    uart = serial.Serial(port, 115200)

fig = plt.figure()
radio_ax = plt.axes([0, 0, 0.2, 0.2])
ports = [x.device for x in serial.tools.list_ports.comports()]
radios = RadioButtons(radio_ax, ports)
radios.on_clicked(set_serial)

def handle_close(unused):
    if uart is not None:
        uart.close()
    os._exit(0)


class VectorGraph:
    def __init__(self, fig, cols, rows, pos, title):
        self.lock = threading.Lock()
        self.x = 0
        self.y = 0
        self.z = 0
        self.plot = fig.add_subplot(cols, rows, pos, projection="3d")
        self.plot.set_title(title)
        self.plot.set_xlim([-1, 1])
        self.plot.set_ylim([-1, 1])
        self.plot.set_zlim([-1, 1])
        self.quiver = self.plot.quiver(0, 0, 0, 0, 0, 0)
        fig.canvas.mpl_connect('close_event', handle_close)

accel = VectorGraph(fig, 1, 3, 1, "Accelerometer Only")
gyro = VectorGraph(fig, 1, 3, 2, "Gyro Only")
filtered = VectorGraph(fig, 1, 3, 3, "Accel/Gyro Complementary Filter")

def read_values():
    while True:
        global accel, gyro, filtered
        s = ""
        try:
            if uart is not None:
                s = uart.readline()
        except:
            pass
        readings = s.split()
        try:
            with accel.lock:
                accel.x = float(readings[0])
                accel.y = float(readings[1])
                accel.z = float(readings[2])
        except:
            pass
        try:
            with gyro.lock:
                gyro.x = float(readings[3])
                gyro.y = float(readings[4])
                gyro.z = float(readings[5])
        except:
            pass
        try:
            with gyro.lock:
                filtered.x = float(readings[6])
                filtered.y = float(readings[7])
                filtered.z = float(readings[8])
        except:
            pass

def update(unused, graph):
    graph.quiver.remove()
    with graph.lock:
        graph.quiver = graph.plot.quiver(0, 0, 0, graph.x, graph.y, graph.z)
    return graph.quiver,


accel_anim = animation.FuncAnimation(fig, update, fargs=(accel,), interval=5, blit=False)
gyro_anim = animation.FuncAnimation(fig, update, fargs=(gyro,), interval=5, blit=False)
filtered_anim = animation.FuncAnimation(fig, update, fargs=(filtered,), interval=5, blit=False)
threading.Thread(target=read_values).start()
plt.show()
