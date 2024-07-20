import cv2
import numpy as np
import time
from motor_control import stop, move_forward, turn_left, turn_right, cleanup
from sensor_fusion import shared_state
from heapq import heappush, heappop

calibration_data = np.load('camera_calibration_data.npz')
camera_matrix = calibration_data['camera_matrix']
dist_coeffs = calibration_data['dist_coeffs']

camera = cv2.VideoCapture(0)

def undistort_frame(frame):
    return cv2.undistort(frame, camera_matrix, dist_coeffs, None, camera_matrix)

def preprocess_frame(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    edged = cv2.Canny(blurred, 50, 150)
    return edged

def detect_obstacles(frame):
    contours, _ = cv2.findContours(frame, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    obstacle_mask = np.zeros_like(frame)
    for contour in contours:
        if cv2.contourArea(contour) > 500:  
            cv2.drawContours(obstacle_mask, [contour], -1, 255, -1)
    return obstacle_mask

def create_occupancy_grid(obstacle_mask):
    grid = np.zeros((GRID_SIZE, GRID_SIZE))
    scale_x = obstacle_mask.shape[1] / GRID_SIZE
    scale_y = obstacle_mask.shape[0] / GRID_SIZE
    for y in range(GRID_SIZE):
        for x in range(GRID_SIZE):
            if np.any(obstacle_mask[int(y * scale_y):int((y + 1) * scale_y), int(x * scale_x):int((x + 1) * scale_x)]):
                grid[y, x] = 1
    return grid

def a_star(start, goal, grid):
    open_list = []
    heappush(open_list, (0, start))
    came_from = {}
    g_score = {start: 0}
    f_score = {start: heuristic(start, goal)}

    while open_list:
        current = heappop(open_list)[1]
        if current == goal:
            return reconstruct_path(came_from, current)
        
        for neighbor in get_neighbors(current, grid):
            tentative_g_score = g_score[current] + 1
            if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = g_score[neighbor] + heuristic(neighbor, goal)
                if neighbor not in open_list:
                    heappush(open_list, (f_score[neighbor], neighbor))
    
    return None

def heuristic(node, goal):
    return np.sqrt((node[0] - goal[0]) ** 2 + (node[1] - goal[1]) ** 2)

def get_neighbors(node, grid):
    neighbors = []
    for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
        neighbor = (node[0] + dx, node[1] + dy)
        if 0 <= neighbor[0] < GRID_SIZE and 0 <= neighbor[1] < GRID_SIZE and grid[neighbor[0], neighbor[1]] == 0:
            neighbors.append(neighbor)
    return neighbors

def reconstruct_path(came_from, current):
    path = []
    while current in came_from:
        path.append(current)
        current = came_from[current]
    path.reverse()
    return path

def handle_obstacle():
    stop()
    print("Robot stopped. Waiting for 25 seconds...")
    time.sleep(25)
    ret, frame = camera.read()
    if ret:
        frame = undistort_frame(frame)
        preprocessed_frame = preprocess_frame(frame)
        obstacle_mask = detect_obstacles(preprocessed_frame)
        if np.any(obstacle_mask):
            print("Obstacle still present. Calculating detour path...")
            calculate_detour_path(obstacle_mask)
        else:
            print("Obstacle cleared. Resuming navigation...")

def calculate_detour_path(obstacle_mask):
    current_state = shared_state.get_state()
    if current_state is not None:
        current_lat, current_lon = current_state[0], current_state[1]
        goal_lat, goal_lon = get_goal_gps_position()
        start = (int(current_lat * GRID_RESOLUTION), int(current_lon * GRID_RESOLUTION))
        goal = (int(goal_lat * GRID_RESOLUTION), int(goal_lon * GRID_RESOLUTION))
        occupancy_grid = create_occupancy_grid(obstacle_mask)
        path = a_star(start, goal, occupancy_grid)
        if path:
            print("Detour path found:", path)
            follow_path(path)
        else:
            print("No detour path found. Stopping robot.")
            stop()
    else:
        print("Failed to get current state for detour calculation.")

def follow_path(path):
    for node in path:
        x, y = node
        real_x = x / GRID_RESOLUTION
        real_y = y / GRID_RESOLUTION
        move_to_position(real_x, real_y)

def move_to_position(x, y):
    current_state = shared_state.get_state()
    if current_state is not None:
        direction = calculate_direction(current_state, (x, y))
        distance = calculate_distance(current_state, (x, y))
        if direction == "forward":
            move_forward(distance)
        elif direction == "backward":
            move_backward(distance)
        elif direction == "left":
            turn_left(50)
            move_forward(distance)
        elif direction == "right":
            turn_right(50)
            move_forward(distance)

def calculate_direction(current, target):
    if current[0] < target[0]:
        return "forward"
    elif current[0] > target[0]:
        return "backward"
    elif current[1] < target[1]:
        return "right"
    elif current[1] > target[1]:
        return "left"

def calculate_distance(current, target):
    return np.sqrt((target[0] - current[0]) ** 2 + (target[1] - current[1]) ** 2)

def get_goal_gps_position():
    goal_lat = float(input("Enter destination latitude: "))
    goal_lon = float(input("Enter destination longitude: "))
    return goal_lat, goal_lon

if __name__ == "__main__":
    try:
        while True:
            ret, frame = camera.read()
            if ret:
                frame = undistort_frame(frame)
                preprocessed_frame = preprocess_frame(frame)
                obstacle_mask = detect_obstacles(preprocessed_frame)
                if np.any(obstacle_mask):
                    handle_obstacle()
            time.sleep(0.5)  
    except KeyboardInterrupt:
        print("Stopping camera...")
    finally:
        camera.release()
        cv2.destroyAllWindows()
        cleanup()
