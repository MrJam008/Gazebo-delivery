#!/usr/bin/env python3

import rospy
from clover import srv
from std_srvs.srv import Trigger
import math
import time

rospy.init_node('delivery')

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
land = rospy.ServiceProxy('land', Trigger)
set_leds = rospy.ServiceProxy('/led/set_effect', srv.SetLEDEffect)

# Known victim positions (modify these coordinates as needed)
VICTIMS = [
    {"name": "blue", "position": (2.0, 3.0), "led_color": (0, 0, 255)},
    {"name": "purple", "position": (6.0, 7.0), "led_color": (255, 0, 255)},
    {"name": "green", "position": (8.0, 2.0), "led_color": (0, 255, 0)}
]

# Hardcoded control flight path
CONTROL_PATH = [
    (0.0, 0.0),
    (0.0, 10.0),
    (2.0, 10.0),
    (2.0, 0.0),
    (4.0, 0.0),
    (4.0, 9.0),
    (6.0, 9.0),
    (6.0, 0.0),
    (8.0, 0.0),
    (8.0, 9.0),
    (9.0, 9.0),
    (9.0, 0.0),
    (0.0, 0.0)
]

def navigate_wait(x=0, y=0, z=0, yaw=float('nan'), speed=0.5, frame_id='body', auto_arm=False):
    """Blocking navigation with position confirmation"""
    navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)
    
    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < 0.2:
            return
        rospy.sleep(0.2)

def blink_green():
    """Blink green LEDs three times"""
    for _ in range(3):
        set_leds(effect='fill', r=0, g=255, b=0)
        rospy.sleep(0.5)
        set_leds(effect='fill', r=0, g=0, b=0)
        rospy.sleep(0.5)

def set_led_color(r, g, b):
    """Set LED color"""
    set_leds(effect='fill', r=r, g=g, b=b)

def deliver_to_victim(name, x, y, r, g, b):
    """Deliver cargo to a victim at specified coordinates"""
    rospy.loginfo(f"Delivering to {name} victim at ({x:.1f}, {y:.1f})")
    
    # Approach at 1.5m height
    navigate_wait(x=x, y=y, z=1.5, frame_id='aruco_map', speed=1)
    
    # Precise positioning at 1.0m
    navigate_wait(x=x, y=y, z=1.0, frame_id='aruco_map', speed=0.3)
    
    # Show victim color
    set_led_color(r, g, b)
    rospy.loginfo(f"Showing {name} color on LEDs")
    rospy.sleep(2.0)
    
    # Drop cargo (descend to 0.3m)
    navigate_wait(x=x, y=y, z=0.3, frame_id='aruco_map', speed=0.2)
    rospy.loginfo(f"Dropping cargo for {name} victim")
    rospy.sleep(1.5)
    
    # Ascend back to 1.0m
    navigate_wait(x=x, y=y, z=1.0, frame_id='aruco_map', speed=0.3)
    
    # Turn off LEDs
    set_led_color(0, 0, 0)
    rospy.loginfo(f"Cargo delivered to {name} victim")

def perform_control_flight():
    """Perform control flight along hardcoded path"""
    rospy.loginfo("Starting control flight at 1.5m height")
    
    for i, (x, y) in enumerate(CONTROL_PATH):
        rospy.loginfo(f"Control flight waypoint {i+1}/{len(CONTROL_PATH)}: ({x:.1f}, {y:.1f})")
        navigate_wait(x=x, y=y, z=1.5, frame_id='aruco_map', speed=1)
        rospy.sleep(0.3)

def main():
    rospy.loginfo("Starting delivery mission")
    
    # Take off from warehouse (marker id=0)
    rospy.loginfo("Taking off from warehouse")
    navigate_wait(z=1.5, frame_id='body', auto_arm=True, speed=0.5)
    rospy.sleep(1.0)
    
    # Blink green to indicate readiness
    rospy.loginfo("Blinking green - ready for delivery")
    blink_green()
    
    # Deliver to all victims
    for victim in VICTIMS:
        deliver_to_victim(
            victim["name"],
            victim["position"][0],
            victim["position"][1],
            victim["led_color"][0],
            victim["led_color"][1],
            victim["led_color"][2]
        )
        rospy.sleep(1.0)
    
    # Control flight
    rospy.loginfo("Performing control flight")
    perform_control_flight()
    
    # Return to warehouse
    rospy.loginfo("Returning to warehouse")
    navigate_wait(x=0.0, y=0.0, z=1.5, frame_id='aruco_map', speed=0.7)
    
    # Land
    rospy.loginfo("Landing on warehouse marker")
    navigate_wait(x=0.0, y=9.0, z=0.5, frame_id='aruco_map', speed=0.7)
    land()
    
    rospy.loginfo("Mission completed successfully")

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        rospy.logerr(f"Critical error: {e}")
        rospy.loginfo("Emergency landing")
        try:
            land()
        except:
            pass
    finally:
        # Ensure LEDs are turned off
        try:
            set_leds(effect='fill', r=0, g=0, b=0)
        except:
            pass
        rospy.loginfo("Delivery mission terminated")