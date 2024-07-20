import csv
import math
import time
import numpy as np
import matplotlib.pyplot as plt
from motor_control import move_forward, turn_left, turn_right, stop
from sensor_fusion import shared_state

def load_path_from_csv():
    path = []
    with open('gps_coordinates.csv', 'r') as csvfile:
        reader = csv.reader(csvfile)
        next(reader)  
        for row in reader:
            path.append((float(row[1]), float(row[2])))  # Append (latitude, longitude) tuples
    return path

path = load_path_from_csv()

# Calculate distance between two GPS coordinates using Haversine formula
def haversine(lat1, lon1, lat2, lon2):
    R = 6371e3  # Earth radius in meters
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lon2 - lon1)
    a = math.sin(delta_phi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    distance = R * c
    return distance

# Find the next carrot point on the path
def find_carrot_index(current_lat, current_lon, path, lookahead_distance):
    closest_distance = float('inf')
    carrot_index = 0
    for i, (lat, lon) in enumerate(path):
        distance = haversine(current_lat, current_lon, lat, lon)
        if distance < closest_distance and distance > lookahead_distance:
            closest_distance = distance
            carrot_index = i
    return carrot_index

# Move towards the carrot point
def move_to_carrot(current_lat, current_lon, carrot_lat, carrot_lon):
    angle_to_carrot = math.atan2(carrot_lon - current_lon, carrot_lat - current_lat)
    distance_to_carrot = haversine(current_lat, current_lon, carrot_lat, carrot_lon)

    if distance_to_carrot < 1:  # If within 1 meter of the carrot point, no need to move
        return

    if -math.pi / 4 < angle_to_carrot < math.pi / 4:
        move_forward(50)
    elif angle_to_carrot >= math.pi / 4:
        turn_right(50)
    elif angle_to_carrot <= -math.pi / 4:
        turn_left(50)

    time.sleep(distance_to_carrot / 50)  

# Follow the path to the destination
def follow_path():
    fig, ax, robot_position = visualize_path()
    try:
        while True:
            current_state = shared_state.get_state()
            if current_state is not None:
                current_lat, current_lon = current_state[0], current_state[1]

                carrot_index = find_carrot_index(current_lat, current_lon, path, 5)
                carrot_lat, carrot_lon = path[carrot_index]
                move_to_carrot(current_lat, current_lon, carrot_lat, carrot_lon)

                # Update plot
                robot_position.set_data(current_lat, current_lon)
                plt.draw()
                plt.pause(0.01)

            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        stop()
        plt.ioff()
        plt.show()

# Visualization of the path
def visualize_path():
    plt.ion()
    fig, ax = plt.subplots()
    path_array = np.array(path)
    ax.scatter(path_array[:, 0], path_array[:, 1], c='blue', label='Path')
    robot_position, = ax.plot([], [], 'ro', label='Robot Position')
    ax.legend()
    ax.set_title('Robot Path Following')
    ax.set_xlabel('Latitude')
    ax.set_ylabel('Longitude')
    return fig, ax, robot_position

if __name__ == "__main__":
    follow_path()
