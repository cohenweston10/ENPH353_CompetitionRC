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

iconImg = """iVBORw0KGgoAAAANSUhEUgAAADAAAAAwCAYAAABXAvmHAAAABmJLR0QA/wD/AP+gvaeTAAAE4klEQVRoge2YW2wUVRjHf+fMXqHGECVB5Vq8EJW
        LAsG0knBpJZCQlmJIKKWSWEuMMSwPvvggMfpiTGTRNECiwbRbYxq6pfGhGisRuQQKNOXyYDS0QA0gGMQIvezuzOfDttDt7nZnZxtRsv992DPf9f+d
        b86cmQN55JFHHnnk8aCjtLKmo3TT68f/i/lcBw4elUxGdZ/vA1HYsS1fUazG0o93Pp3J4IFAaWWNlFbWZJyN+5FvzHYDBEPhDmBx/EpOBKrWv5QTu3HOZ
        +cWWnxvqJY4p2YbWeVL24EDB4+sAbUXmDpaV7ajGIDW9486IZiEDPF+E5HadStfbkulHKMDqcnfB0xVSu1Np3Q5ifjnE8+Czrh8/pV4ab1afjiyeqjyaU
        6JjRN6lVK1ZcuLvk2ltFV207Fjfne/PC5aHlOWnqaRhShKBOaPB0MFZxDaLdRp0VavstTVqF9d2VBU1G/D1zkOHDy8BPSXwByHIX4Ga0v5iqUnnHLI+UZ
        ubj9caGh9wYmvaVmz15cs7c4lf06vEh990fpQ7+9/VDr1v3ztxsa6pqaCXDg4LiDYuH+V12v+gsgHTmMo+DAacf0abNy/ymkMRwUEG8PbEd0GTAF+ROR6
        tjHE4roIh4ApiG7b1dCyzQmXrAsIhsJvIHwCmCi2BaoqlgtsAXqzCNOrDfXa9s0Vy5SoAGCKkuDOhuaabPlktYg/bQjPtxTHAY8oqrdvqmjMNmEqBEMtV
        SD1wCDaWhKofPWsXd+sOiCKzwAfqJ3jRR4gULUuhBAEfFhqt4jYnljbBexsbC4RWArcjFoTdjghOhbc3th7wE1QRcGvwivt+tl+F1Ki3gRQsPud6lV30t
        mVP//1K0rRAkwYpfpLi5Q0n994KpXfWxs23A42hvcgvDuUq90OL1sd+Lj+u4nAGsCyROrS2a1d+M0ErdiTgjzAw5ZSDVtm7vOl8ze0UQcIsHooZ0bY6oB
        X9xVb4FNwLrB5/dVhedWi1lKt2AvMikss8PnHCjXH8vn7qye3Dl/3gNTWnyxvB3h7Y9mVXaHweYG5bnWnCPg+EzdbHRDkRQBRcijBOYH8PcQsk77IILcH
        Brg9MEBfZJCYaaYKPUtIetf/CUAp9YIdbrY6IDAv/q9PjyYw2nYwGiVixhJkpiX0WxE84sLrcifoFBSOvLaUOqVEEGSBHW72FrFSTyGClrE3q5hlEjFjG
        IbBogXzKJwxHUHouXSZU13niMRiGErjMoy0MbTQO3Qc8eT4FSDyKIAg7cFQ+K64c1eiWSQan/lFC+ZRvOxpAG5dg+fmPIMIdHR2ETFjSQUEQ+G7RyjC8D
        CeMxPs7gOP2DGyJJ68cMZ0UDDyYGf2rBkAmJZlM6WyldPuPuAH8N9xT9y6dW3fsLB6cWvC4ZOMoHzrKgmQoeKUSt5kA1UVd4V1TU0F0YjrbyDt43Yk7Hb
        AAJg0qX9wTCMdD3fh4qUkXXdPXGakKGAkbkyePDA0tDW5OX2Rje5AzDTpj0YwDIOF8+dSOHM6AN0XL3P6zFlM08Lv8eDSiWug/mSZYx6OjlXSBjMMPOIi
        EovR0dlFR2dXgt5juJLI54pcT6d7Rgu8Ljd+tweX1qihn0tr/B4PXrc7KYDA/fsmBqlNRcBlGPg9Xgp8Pgp8Pvweb8qZF+jW6NrcOOSRRx7/a/wDNeu3t
        iRlt6UAAAAASUVORK5CYII="""
img = tk.PhotoImage(data=iconImg)
root.tk.call('wm','iconphoto',root._w,img)

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

linMult = 0.2
angMult = 1.0

# Send velocity based on key states
def update_velocity():
    vx = (key_states["i"] - key_states["k"]) * linMult # Forward/Backward
    vy = (key_states["j"] - key_states["l"]) * linMult # Left/Right (corrected)
    vz = (key_states["w"] - key_states["s"]) * linMult # Up/Down
    vaz = (key_states["a"] - key_states["d"]) * angMult  # Rotate left/right

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
