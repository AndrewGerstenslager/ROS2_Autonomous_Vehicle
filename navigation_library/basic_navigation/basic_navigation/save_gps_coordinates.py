import rclpy
from sensor_msgs.msg import NavSatFix
import tkinter as tk
import json
import threading

# Initialize the ROS client library
rclpy.init()
node = rclpy.create_node('gps_gui')

# Create a list to store the waypoints
waypoints = []

# Initialize the current location
current_location = (0.0, 0.0)

def gps_callback(msg: NavSatFix):
    # Store the current GPS location
    global current_location
    current_location = (msg.latitude, msg.longitude)

def add_location():
    # Add the current location to the waypoints list
    waypoints.append(current_location)
    # Update the label
    label.config(text=f"Number of coordinates: {len(waypoints)}")
    print(waypoints)

def save_locations():
    # Save the waypoints to a JSON file
    with open('waypoints.json', 'w') as f:
        json.dump({'waypoints': waypoints}, f)

# Create the subscription
subscription = node.create_subscription(NavSatFix, '/gps/fix', gps_callback, 10)

# Create the GUI
root = tk.Tk()
add_button = tk.Button(root, text="Add location", command=add_location)
add_button.pack()
save_button = tk.Button(root, text="Save locations", command=save_locations)
save_button.pack()
label = tk.Label(root, text=f"Number of coordinates: {len(waypoints)}")
label.pack()

# Start the ROS spin in a separate thread
def ros_spin():
    rclpy.spin(node)

ros_thread = threading.Thread(target=ros_spin)
ros_thread.start()

# Start the GUI
root.mainloop()