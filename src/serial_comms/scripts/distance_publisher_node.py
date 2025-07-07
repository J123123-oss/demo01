#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import serial
from serial_comms.msg import Distances

def main():
    """
    Main function to initialize the ROS node, open the serial port,
    and start publishing distance data.
    """
    rospy.init_node('distance_publisher_node', anonymous=True)

    # Get parameters from the ROS parameter server
    # These can be set in a launch file or via the command line
    port = rospy.get_param('~port', '/dev/ttyACM3')
    baudrate = rospy.get_param('~baudrate', 115200) # Common baudrate, adjust if needed

    # Create a publisher for the custom Distances message
    pub = rospy.Publisher('distance_data', Distances, queue_size=10)

    rospy.loginfo("Starting distance publisher on port %s at %d baud.", port, baudrate)

    try:
        # Configure and open the serial port
        ser = serial.Serial(port=port, baudrate=baudrate, timeout=1)
    except serial.SerialException as e:
        rospy.logerr("Error opening serial port %s: %s", port, e)
        return

    # Main loop to read from serial and publish to ROS
    while not rospy.is_shutdown():
        try:
            # 1. Wait for the header "Distance Data"
            #    Decode bytes to string, ignoring any invalid characters
            line = ser.readline().decode('utf-8', 'ignore').strip()
            if line != "Distance Data":
                continue

            # 2. Read the next three lines which contain the data
            #    Decode each line, ignoring invalid characters
            data_lines = [ser.readline().decode('utf-8', 'ignore').strip() for _ in range(3)]
            
            # 3. Parse the data lines
            parsed_data = {}
            for data_line in data_lines:
                parts = data_line.split('\t') # Split by tab
                for part in parts:
                    if ':' in part:
                        key, value = part.split(':', 1)
                        try:
                            parsed_data[key] = int(value)
                        except (ValueError, TypeError):
                            rospy.logwarn("Could not parse value for key '%s': %s", key, value)

            # 4. Check if we have all 6 required values
            required_keys = ['A', 'B', 'C', 'D', 'E', 'F']
            if all(key in parsed_data for key in required_keys):
                # 5. Create and publish the message
                msg = Distances()
                msg.distance_a = parsed_data['A']
                msg.distance_b = parsed_data['B']
                msg.distance_c = parsed_data['C']
                msg.distance_d = parsed_data['D']
                msg.distance_e = parsed_data['E']
                msg.distance_f = parsed_data['F']
                # print("distance_a:", msg.distance_a)
                
                pub.publish(msg)
                rospy.logdebug("Published distances: %s", parsed_data)
            else:
                rospy.logwarn("Incomplete data packet received. Missing keys.")

        except serial.SerialException as e:
            rospy.logerr("Serial error: %s", e)
            break # Exit on serial error
        except Exception as e:
            rospy.logerr("An unexpected error occurred: %s", e)

    # Clean up: close the serial port
    if ser.is_open:
        ser.close()
        rospy.loginfo("Serial port %s closed.", port)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
