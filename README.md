![Daise Logo](https://github.com/user-attachments/assets/79a0a723-003d-4940-9f4d-7455bb1a33df)


# D.A.I.S.E - Delivery Autonomous Intelligent Sidewalk Explorer

##
## Introduction
Welcome to the D.A.I.S.E project! This repository contains the complete source code and documentation for building and operating a low-cost delivery robot within a $400 budget. This project aims to demonstrate that effective robotic delivery systems can be developed economically without relying on prebuilt libraries or software.

D.A.I.S.E (Delivery Autonomous Intelligent Sidewalk Explorer) is designed to navigate sidewalks, avoid obstacles, and deliver goods to specified locations using a combination of GPS, LiDAR, camera, and IMU sensors. The project leverages sensor fusion and custom algorithms for path planning, obstacle avoidance, and re-routing.

##
## Objectives

- **Low-Cost Design**: Develop a functional delivery robot within a $400 budget.
- **Autonomous Navigation**: Enable the robot to autonomously navigate from a starting point to a destination using GPS coordinates.
- **Obstacle Avoidance**: Implement robust obstacle detection and avoidance using LiDAR and camera sensors.
- **Sensor Fusion**: Combine data from multiple sensors (GPS, IMU, LiDAR, Camera) to improve navigation accuracy.
- **Real-Time Visualization**: Real-time visualization of the robot's path and current position.

##  
## Key Features

- **GPS-Based Navigation**: The robot uses GPS coordinates for global path planning and navigation.
- **Carrot-chasing Algorithm**: Implements a carrot-chasing algorithm to guide the robot along the predefined path.
- **LiDAR Obstacle Detection**: A 2D LiDAR sensor detects and avoids obstacles in the robot's path.
- **Camera-Based Path Planning**: A single Raspberry Pi camera is used for local path planning and obstacle detection.
- **IMU Integration**: Incorporates an IMU sensor to monitor and adjust the robot's orientation.
- **Sensor Fusion**: Combines data from GPS, IMU, LiDAR, and camera for improved accuracy and reliability.
- **Real-Time Visualization**: Displays the robot's path and current position using Matplotlib for real-time monitoring.

##
## Hardware Components

- **Raspberry Pi 4 Computer Model B**: The main processing unit for running the robot's software.
- **Four DC Motors with Hall Effect Sensors**: Provide mobility and speed control feedback.
- **RPLIDAR A1**: A 2D LiDAR sensor for obstacle detection.
- **Adafruit Ultimate GPS Breakout Board**: For GPS-based navigation.
- **MPU-6050**: A 3-axis Accelerometer and Gyro Sensor for orientation.
- **Arducam 5MP Camera (OV5647)**: For local path planning and obstacle detection.
- **Power System**: Includes a LiPo battery, buck converter, and L298N motor drivers.

##
## Mechanical Design

![Mechanical Design](path_to_mechanical_design_image)

##
## Detailed Electronic Connections for D.A.I.S.E.

### Components and Their Connections

#### 1. Raspberry Pi 4 Computer Model B
- **Power Supply**: Connect the Raspberry Pi to a 5V power source using a USB-C power adapter.
- **GPIO Pins**: Used to connect various sensors and modules.

#### 2. Four DC Motors with Hall Effect Sensors
- **Motor Driver (L298N)**:
  - **Back Left Motor (motor1)**:
    - IN1 (Motor Driver) to GPIO 17 (Raspberry Pi)
    - IN2 (Motor Driver) to GPIO 18 (Raspberry Pi)
    - ENA (Motor Driver) to GPIO 22 (Raspberry Pi)
    - ENCODER A (Motor) to GPIO 27 (Raspberry Pi)
    - ENCODER B (Motor) to GPIO 4 (Raspberry Pi)
  - **Back Right Motor (motor2)**:
    - IN1 (Motor Driver) to GPIO 23 (Raspberry Pi)
    - IN2 (Motor Driver) to GPIO 24 (Raspberry Pi)
    - ENA (Motor Driver) to GPIO 25 (Raspberry Pi)
    - ENCODER A (Motor) to GPIO 5 (Raspberry Pi)
    - ENCODER B (Motor) to GPIO 6 (Raspberry Pi)
  - **Front Left Motor (motor3)**:
    - IN1 (Motor Driver) to GPIO 5 (Raspberry Pi)
    - IN2 (Motor Driver) to GPIO 6 (Raspberry Pi)
    - ENA (Motor Driver) to GPIO 13 (Raspberry Pi)
    - ENCODER A (Motor) to GPIO 12 (Raspberry Pi)
    - ENCODER B (Motor) to GPIO 16 (Raspberry Pi)
  - **Front Right Motor (motor4)**:
    - IN1 (Motor Driver) to GPIO 19 (Raspberry Pi)
    - IN2 (Motor Driver) to GPIO 26 (Raspberry Pi)
    - ENA (Motor Driver) to GPIO 21 (Raspberry Pi)
    - ENCODER A (Motor) to GPIO 20 (Raspberry Pi)
    - ENCODER B (Motor) to GPIO 21 (Raspberry Pi)

#### 3. RPLIDAR A1 (2D LiDAR Sensor)
- **Connection to Raspberry Pi**:
  - **Power Supply**: Connect the LiDAR to a 5V power source.
  - **Data Connection**: Use a USB to serial adapter to connect the LiDAR to the Raspberry Pi.
    - USB Port on Raspberry Pi to USB connector on LiDAR.

#### 4. Adafruit Ultimate GPS Breakout Board
- **Connection to Raspberry Pi**:
  - VCC (GPS) to 5V (Raspberry Pi)
  - GND (GPS) to GND (Raspberry Pi)
  - TX (GPS) to GPIO 15 (Raspberry Pi)
  - RX (GPS) to GPIO 14 (Raspberry Pi)

#### 5. MPU-6050 (3-Axis Accelerometer and Gyro Sensor)
- **Connection to Raspberry Pi**:
  - VCC (MPU-6050) to 3.3V (Raspberry Pi)
  - GND (MPU-6050) to GND (Raspberry Pi)
  - SCL (MPU-6050) to GPIO 3 (SCL) (Raspberry Pi)
  - SDA (MPU-6050) to GPIO 2 (SDA) (Raspberry Pi)

#### 6. Arducam 5MP Camera (OV5647)
- **Connection to Raspberry Pi**:
  - Connect the camera module to the Raspberry Pi's camera interface (CSI) port. Ensure the ribbon cable is correctly oriented and securely inserted.

#### 7. Power System
- **LiPo Battery**:
  - Connect the positive terminal of the LiPo battery to the 12V power rail on the breadboard.
  - Connect the negative terminal of the LiPo battery to the common ground rail on the breadboard.
- **Buck Converter**:
  - Input: Connect to the 12V power rail on the breadboard.
  - Output: Adjust to 5V and connect to the 5V power rail on the breadboard.
- **Raspberry Pi Power**:
  - 5V from the buck converter to Pin 2 (5V) on the Raspberry Pi.
  - Ground to Pin 6 (GND) on the Raspberry Pi.

![Daise Electronics](https://github.com/user-attachments/assets/0046365f-de82-46ab-b9c2-d106c93564ef)

##
## Major Concepts of Robotics Involved in the D.A.I.S.E. Project

### Kinematics and Dynamics

- **4WD Differential Drive Kinematics**: Controls movement using a four-wheel-drive system with differential drive kinematics.
- **PID Control**: Maintains precise control over motor speeds using PID controllers.

### Sensor Integration and Fusion

- **Sensor Fusion**: Combines data from multiple sensors to improve navigation and obstacle detection.
- **Kalman Filtering**: Estimates the robot’s state from noisy sensor measurements.

### Localization and Mapping

- **GPS Navigation**: Uses a GPS module to record paths and navigate to predefined coordinates.
- **IMU Data**: Provides accelerometer and gyroscope data for orientation and stability.
- **LiDAR Mapping**: Creates occupancy grids to identify obstacles.

### Path Planning and Following

- **Carrot Chasing Algorithm**: Follows a look-ahead point on the predefined path.
- **A Star Pathfinding**: Finds the shortest path around obstacles.

### Obstacle Detection and Avoidance

- **LiDAR Obstacle Detection**: Detects obstacles based on distance measurements.
- **Camera-Based Obstacle Detection**: Image processing is used to detect obstacles in the field of view.

### Real-Time Control and Decision Making

- **Control Loops**: Adjusts motor speeds and directions based on sensor inputs.
- **Dynamic Re-Routing**: Modifies the path in real-time based on obstacle detection.

### Visualization and Debugging

- **Real-Time Visualization**: Displays the robot's path and position for monitoring and debugging.
- **Data Logging**: Records sensor data and robot states for analysis and tuning.

### Software Development Practices

- **Modular Design**: Structures code into modules for readability, maintainability, and scalability.
- **Threading**: Uses threads to handle different tasks concurrently.

##
## Detailed Description of Each Script in the D.A.I.S.E. Project

### `main.py`
**Purpose**: Orchestrates the overall operation of the robot.

**Functions and Flow**:

- **Initialization**:
  - Start sensor fusion (`start_sensor_fusion()`).
  - Initialize the LiDAR sensor (`initialize_lidar()`).
  - Load the predefined path (`load_path_from_csv()`).
  - Setup real-time visualization (`visualize_path(path)`).
  - Prompt the user to input destination coordinates.

- **Main Control Loop**:
  - Fetch the current state from `shared_state`.
  - Update the visualization plot.
  - Read LiDAR data and handle obstacles (`lidar_handle_obstacle()`).
  - Capture and process camera frames, handle obstacles (`camera_handle_obstacle()`).
  - Find the next carrot point and move towards it.
  - Wait for a short interval before repeating the loop.

- **Finalization**:
  - Stop the robot and clean up sensors and GPIO pins on interruption.

### `motor_control.py`
**Purpose**: Manages the control of the robot's four DC motors with PID control.

**Functions and Flow**:

- **Initialization**:
  - Setup GPIO pins and PWM for motor control.

- **Motor Control Functions**:
  - `set_motor_speed(motor, speed)`: Sets motor speed.
  - `set_motor_direction(motor, direction)`: Sets motor direction.
  - Movement functions: `move_forward()`, `move_backward()`, `turn_left()`, `turn_right()`, `stop()`.

- **PID Controllers**:
  - Define `PIDController` class.
  - Use encoder pulse counts to update actual speeds.
  - Adjust motor speeds based on PID outputs.

### `sensor_fusion.py`
**Purpose**: Integrates data from GPS, IMU, LiDAR, and Camera using a Kalman filter.

**Functions and Flow**:

- **Shared State Object**:
  - Define `SharedState` class for storing and updating the robot's state.

- **Sensor Measurement Functions**:
  - `get_gps_measurement()`, `get_imu_measurement()`, `get_lidar_measurement()`, `get_camera_measurement()`.

- **State Prediction and Update**:
  - `predict_state(state)`: Predicts the next state.
  - `update_state(predicted_state)`: Updates state with sensor data using a Kalman filter.

- **Kalman Filter Calculation**:
  - `calculate_kalman_gain()`: Calculates Kalman gain.

- **Sensor Fusion Loop**:
  - Continuously predict and update state based on sensor data.

### `path_following.py`
**Purpose**: Guides the robot along a predefined path using the carrot chasing algorithm.

**Functions and Flow**:

- **Path Loading**:
  - `load_path_from_csv()`: Loads path from `gps_coordinates.csv`.

- **Carrot Chasing Algorithm**:
  - `haversine()`: Calculates distance between GPS coordinates.
  - `find_carrot_index()`: Finds the next carrot point.
  - `move_to_carrot()`: Moves the robot towards the carrot point.

- **Path Following Loop**:
  - Fetch current state, find carrot point, move towards it.
  - Update visualization plot.

### `lidar_object_detection.py`
**Purpose**: Detects obstacles using LiDAR and reroutes the robot.

**Functions and Flow**:

- **LiDAR Initialization**:
  - `initialize_lidar()`: Starts the LiDAR sensor.

- **LiDAR Data Reading**:
  - `read_lidar_data()`: Reads and processes LiDAR scan data.

- **Obstacle Handling**:
  - `handle_obstacle()`: Stops robot, waits, checks for obstacle, calculates detour path if needed.
  - `calculate_detour_path()`: Uses A* algorithm to find a detour path.
  - `follow_path()`: Follows the detour path.
  - `return_to_path()`: Returns to the predefined path.

### `camera_path_planning.py`
**Purpose**: Detects obstacles using camera data and reroutes the robot.

**Functions and Flow**:

- **Camera Initialization**:
  - Initialize camera and load calibration parameters.

- **Obstacle Detection**:
  - Capture, undistort, preprocess frames.
  - Detect obstacles.

- **Obstacle Handling**:
  - `handle_obstacle()`: Stops robot, waits, checks for obstacle, calculates detour path if needed.
  - `calculate_detour_path()`: Uses A* algorithm to find a detour path.
  - `follow_path()`: Follows the detour path.
  - `return_to_path()`: Returns to the predefined path.

### `record_coordinates.py`
**Purpose**: Records GPS coordinates along the desired path.

**Functions and Flow**:

- **GPS Data Reading**:
  - Initialize serial connection to GPS module.
  - `read_gps()`: Reads GPS coordinates.

- **Recording Coordinates**:
  - `record_coordinates()`: Records GPS coordinates and saves to `gps_coordinates.csv`.

## 
## Step-by-Step Guide to Running the D.A.I.S.E. Project

### 1. Setting Up the VNC Viewer

**Install VNC Viewer on Your Computer:**
- Download and install VNC Viewer from the official website.

**Connect to the Raspberry Pi:**
- Ensure the Raspberry Pi and your computer are on the same network.
- Open VNC Viewer and enter the IP address of the Raspberry Pi.
- Enter the VNC password when prompted to access the Raspberry Pi desktop.

### 2. Prepare the Raspberry Pi

**Update the Raspberry Pi:**

- Open a terminal window on the Raspberry Pi.
- Run the following commands to update and upgrade the system:
  ```bash
  sudo apt-get update
  sudo apt-get upgrade
- Install Python 3 and pip if they are not already installed:
  ```bash
  sudo apt-get install python3 python3-pip
- Install necessary Python libraries:
  ```bash
  pip3 install numpy matplotlib RPi.GPIO opencv-python serial rplidar

### 3. Camera Calibration

**Run the Camera Calibration Script:**

- Ensure the camera module is connected and enabled on the Raspberry Pi.
- In the terminal, navigate to the directory containing `camera_calibration.py`.
- Run the script to capture images of the chessboard pattern and calculate the calibration parameters:
  ```bash
  python3 camera_calibration.py
- The script saves the camera matrix and distortion coefficients in `camera_calibration_data.npz`.

### 4. Recording GPS Coordinates

**Run the GPS Recording Script:**

- Connect the GPS module to the Raspberry Pi.
- In the terminal, navigate to the directory containing `record_coordinates.py`.
- Run the script to record GPS coordinates along the desired path:
```bash
python3 record_coordinates.py 
```
- Move the robot manually along the path to record accurate coordinates. The coordinates are saved in `gps_coordinates.csv`.

**Convert and Correct Coordinates:**

- Use external software to convert `gps_coordinates.cs`v to KML format and visualize the path in Google Earth.
- Manually correct any errors in the CSV file and reconvert it to KML to ensure accuracy.

### 5. Running the Main Script

**Ensure All Components Are Connected:**

- Connect the DC motors, LiDAR sensor, GPS module, IMU, and camera to the Raspberry Pi as per the project wiring diagram.
  
**Start the Main Script:**

- In the terminal, navigate to the directory containing main.py.
- Run the script to start the robot's autonomous operation:
```
python3 main.py
```

##
## Flowchart

![Daise flowchart 2](https://github.com/user-attachments/assets/ce58a0d2-4acc-49fe-a863-7bfec431cd85)

### Start:
- The process begins by initializing all system components. This includes setting up the Raspberry Pi, initializing the sensor fusion thread, starting the LiDAR motor, and checking the sensor’s health status.
- The sensor fusion process is initiated in a separate thread to continuously integrate data from multiple sensors (GPS, IMU, LiDAR, Camera).

### User Input:
- After initializing the system components, the user is prompted to input the destination coordinates (latitude and longitude). These coordinates define the robot's final destination.
- The predefined path, loaded from `gps_coordinates.csv`, is used in conjunction with the user-inputted destination to guide the robot.

### Main Control Loop:
- The robot enters the main control loop, which operates continuously until the destination is reached or the process is interrupted.
- In each iteration of the loop:
  - **Fetch Current State**: The robot’s current state (position, velocity, orientation) is fetched from the shared state object, continuously updated by the sensor fusion thread.
  - **Update Visualization**: The real-time visualization plot is updated with the robot's current position to provide visual feedback on its progress.
  - **Read LiDAR Data**: LiDAR data is read to detect obstacles in the robot’s path. If an obstacle is detected, the `handle_obstacle()` function is called to manage the obstacle.

### Obstacle Detection and Handling:

#### LiDAR Obstacle Detection:
- If the LiDAR sensor detects an obstacle:
  - The robot stops immediately.
  - The robot waits for 25 seconds to see if the obstacle clears on its own.
  - If the obstacle persists after 25 seconds, the `calculate_detour_path(scan_data)` function is called to find a path around the obstacle using the A* algorithm.
  - The robot follows the detour path calculated by the A* algorithm.
  - After navigating the obstacle, the robot returns to the predefined path and resumes its journey.

#### Camera Obstacle Detection:
- If an obstacle is detected by the camera:
  - The robot stops immediately.
  - The robot waits for 25 seconds to see if the obstacle clears on its own.
  - If the obstacle persists after 25 seconds, the `calculate_detour_path(obstacle_mask)` function is called to find a path around the obstacle using the A* algorithm.
  - The robot follows the detour path calculated by the A* algorithm.
  - After navigating the obstacle, the robot returns to the predefined path and resumes its journey.

### Path Following:
- The robot uses the carrot-chasing algorithm to follow the predefined path. This involves continuously adjusting its movement based on the lookahead point (carrot) on the path.
- The `find_carrot_index(current_lat, current_lon, path, 5)` function finds the next carrot point on the path.
- The `move_to_carrot(current_lat, current_lon, carrot_lat, carrot_lon)` function calculates the direction and distance to the carrot point and moves the robot accordingly.

### Dynamic Re-Routing:
- If obstacles are encountered, the robot dynamically recalculates the path and follows a detour using the A* algorithm.
- The robot temporarily leaves the predefined path to navigate around the obstacle.
- Once the obstacle is cleared, the robot recalculates its position and re-joins the predefined path to continue toward the destination.

### Completion:
- The process continues iteratively, with the robot fetching its current state, updating the visualization, reading sensor data, and adjusting its path as needed.
- The loop repeats until the robot reaches the final destination as defined by the user-inputted coordinates.
- Once the destination is reached, the flowchart ends, marking the completion of the robot's navigation task.

By following these detailed steps, the D.A.I.S.E. project ensures that the robot can autonomously navigate from its initial position to the final destination, avoiding obstacles and dynamically adjusting its path as needed. This comprehensive process leverages advanced robotics concepts, sensor fusion, and real-time decision-making to achieve efficient and reliable autonomous navigation.
