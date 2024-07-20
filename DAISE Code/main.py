import threading
from sensor_fusion import start_sensor_fusion, shared_state
from path_following import follow_path, load_path_from_csv
from lidar_object_detection import initialize_lidar, read_lidar_data, handle_obstacle as lidar_handle_obstacle
from camera_path_planning import handle_obstacle as camera_handle_obstacle
from motor_control import stop, cleanup
import matplotlib.pyplot as plt
import cv2

def get_goal_gps_position():
    goal_lat = float(input("Enter destination latitude: "))
    goal_lon = float(input("Enter destination longitude: "))
    return goal_lat, goal_lon

def main():
   
    start_sensor_fusion()
    
    initialize_lidar()
    
    path = load_path_from_csv()
    
    fig, ax, robot_position = visualize_path(path)
    
    try:
        while True:
            current_state = shared_state.get_state()
            if current_state is not None:
                current_lat, current_lon = current_state[0], current_state[1]
                update_plot(current_lat, current_lon, robot_position)
                
                scan_data = read_lidar_data()
                if scan_data:
                    lidar_handle_obstacle()
                
                ret, frame = camera.read()
                if ret:
                    frame = undistort_frame(frame)
                    preprocessed_frame = preprocess_frame(frame)
                    obstacle_mask = detect_obstacles(preprocessed_frame)
                    if np.any(obstacle_mask):
                        camera_handle_obstacle()
                
                carrot_index = find_carrot_index(current_lat, current_lon, path, 5)
                carrot_lat, carrot_lon = path[carrot_index]
                move_to_carrot(current_lat, current_lon, carrot_lat, carrot_lon)
                
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        stop()
        cleanup()
        lidar.stop()
        lidar.stop_motor()
        lidar.disconnect()
        camera.release()
        cv2.destroyAllWindows()
        plt.ioff()
        plt.show()

def visualize_path(path):
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

def update_plot(current_lat, current_lon, robot_position):
    robot_position.set_data(current_lat, current_lon)
    plt.draw()
    plt.pause(0.01)

if __name__ == "__main__":
    main()
