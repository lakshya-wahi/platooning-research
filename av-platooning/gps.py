########## DEPENDENCIES ##########


from dronekit import connect, VehicleMode, LocationGlobalRelative

import time

import math

import argparse

import can

import struct



######### FUNCTIONS #################


def establish_rover_connection():

    parser = argparse.ArgumentParser(description='Rover connection parameters')

    parser.add_argument('--connect')

    args = parser.parse_args()


    conn_str = args.connect

    rover = connect(conn_str, baud=57600, wait_ready=True)


    return rover



def compute_compass_heading(dest_coords, current_coords):

    delta_lat = dest_coords.lat - current_coords.lat

    delta_lon = dest_coords.lon - current_coords.lon


    # Calculate angle in radians and convert to degrees

    bearing_deg = math.atan2(delta_lon, delta_lat) * (180 / math.pi)

    if bearing_deg < 0:

        bearing_deg += 360  # Normalize to 0-360


    # Convert bearing into compass direction

    if 337.5 <= bearing_deg < 360 or 0 <= bearing_deg < 22.5:

        compass = "N"

    elif 22.5 <= bearing_deg < 67.5:

        compass = "NE"

    elif 67.5 <= bearing_deg < 112.5:

        compass = "E"

    elif 112.5 <= bearing_deg < 157.5:

        compass = "SE"

    elif 157.5 <= bearing_deg < 202.5:

        compass = "S"

    elif 202.5 <= bearing_deg < 247.5:

        compass = "SW"

    elif 247.5 <= bearing_deg < 292.5:

        compass = "W"

    elif 292.5 <= bearing_deg < 337.5:

        compass = "NW"


    return f"{compass} ({bearing_deg:.1f}°)"



def calculate_haversine_distance(lat_a, lon_a, lat_b, lon_b):

    earth_radius = 6371000  # meters

    phi_a, phi_b = math.radians(lat_a), math.radians(lat_b)

    delta_phi = math.radians(lat_b - lat_a)

    delta_lambda = math.radians(lon_b - lon_a)


    a = math.sin(delta_phi / 2) ** 2 + math.cos(phi_a) * math.cos(phi_b) * math.sin(delta_lambda / 2) ** 2

    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))


    return earth_radius * c



def send_position_velocity_via_can(lat, velocity, bus):

    """

    Sends current latitude and groundspeed over CAN bus (8 bytes: 2 floats).

    Matches the structure expected by Arduino in Code 1.

    """

    try:

        data = struct.pack('<ff', lat, velocity)

        msg = can.Message(arbitration_id=0x100, data=data, is_extended_id=False)

        bus.send(msg)

        print(f"Sent via CAN → lat: {lat:.6f}, velocity: {velocity:.2f} m/s")

    except can.CanError:

        print("CAN transmission failed.")



def navigate_to_waypoint(target_waypoint, active_rover, can_bus):

    while True:

        current_coords = active_rover.location.global_relative_frame

        groundspeed = active_rover.groundspeed  # m/s


        if current_coords and current_coords.lat and current_coords.lon:

            heading_info = compute_compass_heading(target_waypoint, current_coords)

            remaining_distance = calculate_haversine_distance(

                current_coords.lat, current_coords.lon,

                target_waypoint.lat, target_waypoint.lon

            )


            print(f"Rover Position: ({current_coords.lat}, {current_coords.lon})")

            print(f"Waypoint: ({target_waypoint.lat}, {target_waypoint.lon})")

            print(f"Heading: {heading_info}")

            print(f"Remaining Distance: {remaining_distance:.2f} meters")

            print(f"Ground Speed: {groundspeed:.2f} m/s\n")


            # Send position & velocity to Arduino via CAN

            send_position_velocity_via_can(current_coords.lat, groundspeed, can_bus)


            if remaining_distance < 5:

                print("Destination reached successfully!")

                break


            time.sleep(1)

        else:

            print("Awaiting valid GPS data...")

            time.sleep(1)



########## MAIN EXECUTION ###########


if __name__ == '__main__':

    # Example waypoint

    final_waypoint = LocationGlobalRelative(47.63195438872842, -122.0527076386417, 2)


    print("Connecting to rover and initializing CAN interface...")

    can_bus = can.interface.Bus(channel='can0', bustype='socketcan') 


    rover_connection = establish_rover_connection()


    try:

        print("Beginning navigation to final waypoint...")

        navigate_to_waypoint(final_waypoint, rover_connection, can_bus)

    finally:

        rover_connection.close()
