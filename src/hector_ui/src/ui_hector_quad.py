#!/usr/bin/env python3
#  -*- coding: utf-8 -*-
from tkinter import ttk
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from hector_uav_msgs.msg import Altimeter
import tkinter as tk
import math
import rospy

root = tk.Tk(className="Hector Quadrotor UI")
root.title("Hector Quadrotor UI")

z_p = tk.StringVar()
z_o = tk.StringVar()

# Track currently pressed keys
key_states = {"w": 0, "s": 0, "a": 0, "d": 0, "i": 0, "k": 0, "j": 0, "l": 0}

def quaternion_to_yaw(orientation):
    """
    Converts quaternion orientation to yaw.
    :param orientation: A quaternion object from Imu topic
    """
    x = orientation.x
    y = orientation.y
    z = orientation.z
    w = orientation.w
    t3 = 2.0 * (w * z + x * y)
    t4 = 1.0 - 2.0 * (y * y + z * z)
    yawZActual = math.atan2(t3, t4)
    if yawZActual < 0:
        yawZActual = 2 * math.pi + yawZActual
    return yawZActual

# Callback for altitude from altimeter
def altimeter_callback(data):
    z_p.set("{0:.2f}".format(data.altitude))

# Callback for yaw orientation from IMU
def imu_callback(data):
    yaw = quaternion_to_yaw(data.orientation)
    z_o.set("{0:.2f}".format(math.degrees(yaw)))

# ROS initialization
rospy.init_node('HectorQ_GUI', anonymous=False)

# Subscribers
altimeter_sub = rospy.Subscriber("/quad/altimeter", Altimeter, altimeter_callback)
imu_sub = rospy.Subscriber("/quad/raw_imu", Imu, imu_callback)

# Publishers
vel_pub = rospy.Publisher('/quad/cmd_vel', Twist, queue_size=1)

# Send velocity based on key states
def update_velocity():
    vx = (key_states["i"] - key_states["k"]) * 0.5  # Forward/Backward
    vy = (key_states["j"] - key_states["l"]) * 0.5  # Left/Right (corrected)
    vz = (key_states["w"] - key_states["s"]) * 0.5  # Up/Down
    vaz = (key_states["a"] - key_states["d"]) * 1.0  # Rotate left/right

    vel_msg = Twist()
    vel_msg.linear.x = vx
    vel_msg.linear.y = vy
    vel_msg.linear.z = vz
    vel_msg.angular.z = vaz
    vel_pub.publish(vel_msg)


# Handle key press events
def key_press(event):
    if event.keysym in key_states:
        key_states[event.keysym] = 1
        update_velocity()

# Handle key release events
def key_release(event):
    if event.keysym in key_states:
        key_states[event.keysym] = 0
        update_velocity()

# Bind key press and release events
root.bind('<KeyPress>', key_press)
root.bind('<KeyRelease>', key_release)

# GUI display setup
mainframe = ttk.Frame(root, padding="3 3 12 12")
mainframe.grid(column=0, row=0, sticky=(tk.N, tk.W, tk.E, tk.S))
mainframe.columnconfigure(0, weight=1)

ttk.Label(mainframe, textvariable=z_p).grid(column=1, row=2, sticky=(tk.W, tk.E))
ttk.Label(mainframe, textvariable=z_o).grid(column=2, row=2, sticky=(tk.W, tk.E))

ttk.Label(mainframe, text="Altitude (m)").grid(column=1, row=3, sticky=tk.W)
ttk.Label(mainframe, text="Yaw (°)").grid(column=2, row=3, sticky=tk.W)

for child in mainframe.winfo_children():
    child.grid_configure(padx=5, pady=5)

root.mainloop()
