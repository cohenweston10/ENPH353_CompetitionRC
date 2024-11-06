#!/usr/bin/env python3
#  -*- coding: utf-8 -*-
from tkinter import ttk
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
import tkinter as tk
import math
import rospy

root = tk.Tk(className="Hector Quadrotor UI")
root.title("Hector Quadrotor UI")

x_p = tk.StringVar()
y_p = tk.StringVar()
z_p = tk.StringVar()
z_o = tk.StringVar()

def quaterionToRads(data):
    x = data.pose.orientation.x
    y = data.pose.orientation.y
    z = data.pose.orientation.z
    w = data.pose.orientation.w
    t3 = 2.0 * (w * z + x * y)
    t4 = 1.0 - 2.0 * (y * y + z * z)
    yawZActual = math.atan2(t3, t4)
    if yawZActual < 0:
        yawZActual = 2*math.pi + yawZActual
    return yawZActual

# Callback for simulator position and orientation
def pose_callback(data):
    x_p.set("{0:.2f}".format(data.pose.pose.position.x))
    y_p.set("{0:.2f}".format(data.pose.pose.position.y))
    z_p.set("{0:.2f}".format(data.pose.pose.position.z))

def rot_callback(data):
    z_o.set("{0:.2f}".format(math.degrees(quaterionToRads(data))))

# ROS initialization
rospy.init_node('HectorQ_GUI', anonymous=False)

# Subscribers
posicionLider_sub = rospy.Subscriber("/quad/ground_truth/state", Odometry , pose_callback)
orientaLider_sub = rospy.Subscriber("/quad/ground_truth_to_tf/pose", PoseStamped , rot_callback)

# Publishers
takeoff_pub = rospy.Publisher('/ardrone/takeoff', Empty, queue_size=1)
land_pub = rospy.Publisher('/ardrone/land', Empty, queue_size=1)
vel_pub = rospy.Publisher('/quad/cmd_vel', Twist, queue_size=1)

# Movement functions
def enviar_velocidad(vx,vy,vz,vaz):
    vel_msg = Twist()
    vel_msg.linear.x = float(vx)
    vel_msg.linear.y = float(vy)
    vel_msg.linear.z = float(vz)
    vel_msg.angular.z = float(vaz)
    vel_pub.publish(vel_msg)

def hover_pub():
    enviar_velocidad(0.0,0.0,0.0,0.0)

def takeoff_fun():
    takeoff_pub.publish(Empty())

def land_fun():
    land_pub.publish(Empty())

def up_fun(): enviar_velocidad(0.0,0.0,0.5,0.0)
def down_fun(): enviar_velocidad(0.0,0.0,-0.5,0.0)
def forward_fun(): enviar_velocidad(0.5,0.0,0.0,0.0)
def backward_fun(): enviar_velocidad(-0.5,0.0,0.0,0.0)
def right_fun(): enviar_velocidad(0.0,-0.5,0.0,0.0)
def left_fun(): enviar_velocidad(0.0,0.5,0.0,0.0)
def cw_fun(): enviar_velocidad(0.0,0.0,0.0,-0.5)
def ccw_fun(): enviar_velocidad(0.0,0.0,0.0,0.5)

# Bind keys for keyboard control
root.bind('<KeyPress-i>', lambda e: forward_fun())
root.bind('<KeyPress-k>', lambda e: backward_fun())
root.bind('<KeyPress-j>', lambda e: left_fun())
root.bind('<KeyPress-l>', lambda e: right_fun())
root.bind('<KeyPress-space>', lambda e: hover_pub())
root.bind('<KeyPress-w>', lambda e: up_fun())
root.bind('<KeyPress-s>', lambda e: down_fun())
root.bind('<KeyPress-a>', lambda e: ccw_fun())
root.bind('<KeyPress-d>', lambda e: cw_fun())

# GUI display setup
mainframe = ttk.Frame(root, padding="3 3 12 12")
mainframe.grid(column=0, row=0, sticky=(tk.N, tk.W, tk.E, tk.S))
mainframe.columnconfigure(0, weight=1)

ttk.Label(mainframe, textvariable=x_p).grid(column=1, row=2, sticky=(tk.W, tk.E))
ttk.Label(mainframe, textvariable=y_p).grid(column=2, row=2, sticky=(tk.W, tk.E))
ttk.Label(mainframe, textvariable=z_p).grid(column=3, row=2, sticky=(tk.W, tk.E))
ttk.Label(mainframe, textvariable=z_o).grid(column=4, row=2, sticky=(tk.W, tk.E))

ttk.Label(mainframe, text="X (m)").grid(column=1, row=3, sticky=tk.W)
ttk.Label(mainframe, text="Y (m)").grid(column=2, row=3, sticky=tk.W)
ttk.Label(mainframe, text="Z (m)").grid(column=3, row=3, sticky=tk.W)
ttk.Label(mainframe, text="Yaw (°)").grid(column=4, row=3, sticky=tk.W)

ttk.Button(mainframe, text="Take Off", command=takeoff_fun).grid(column=3, row=4, sticky=tk.W)
ttk.Button(mainframe, text="Land", command=land_fun).grid(column=2, row=4, sticky=tk.W)

for child in mainframe.winfo_children(): 
    child.grid_configure(padx=5, pady=5)

root.mainloop()
