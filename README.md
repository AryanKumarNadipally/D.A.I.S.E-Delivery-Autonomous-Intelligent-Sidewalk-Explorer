![Daise Logo](https://github.com/user-attachments/assets/79a0a723-003d-4940-9f4d-7455bb1a33df)


# D.A.I.S.E - Delivery Autonomous Intelligent Sidewalk Explorer

## Introduction
Welcome to the D.A.I.S.E project! This repository contains the complete source code and documentation for building and operating a low-cost delivery robot within a $400 budget. This project aims to demonstrate that effective robotic delivery systems can be developed economically without relying on prebuilt libraries or software.

D.A.I.S.E (Delivery Autonomous Intelligent Sidewalk Explorer) is designed to navigate sidewalks, avoid obstacles, and deliver goods to specified locations using a combination of GPS, LiDAR, camera, and IMU sensors. The project leverages sensor fusion and custom algorithms for path planning, obstacle avoidance, and re-routing.

This project is the first prototype of D.A.I.S.E, built using materials and tools available within our budget constraints. As a prototype, the robot does not carry any payload because our initial focus is testing and validating the software and navigation systems.


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

### CAD model

![Daise Cad 1](https://github.com/user-attachments/assets/fba48c3a-4e46-4f30-a954-b9f4530a71fc)

![1](https://github.com/user-attachments/assets/b6c09015-8b60-48e4-a60f-0cec64677b67)

![2](https://github.com/user-attachments/assets/0d81e27a-5235-48e2-bed2-acbad210dda5)

As this is a prototype, the outer body was designed using a storage box from a dollar store, and 3D-printed parts were used to attach and support certain components. All the components were arranged and placed with consideration to maintain the center of gravity and ensure stability. This approach allowed us to focus on the functionality and testing of the software while staying within our budget constraints.

##
## Detailed Electronic Connections for D.A.I.S.E.

### Components and Their Connections

#### 1. Raspberry Pi 4 Computer Model B
- **Power Supply**: Connect the Raspberry Pi to a 5V power source using the 5V rail from the buck converter.
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
  - **Data and Power Connection**: Connect the LiDAR via USB to the Raspberry Pi.

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
## Assembly of D.A.I.S.E
https://github.com/user-attachments/assets/a9027fc0-b29c-4042-94c8-281aaddbccc7

https://github.com/user-attachments/assets/8f086992-f5dc-48b8-a38d-5520f3ea8299


##
## Major Concepts of Robotics Involved in the D.A.I.S.E. Project

### Kinematics and Dynamics

#### 4WD Differential Drive Kinematics
To control its movement, the D.A.I.S.E. robot uses a four-wheel-drive (4WD) system with differential drive kinematics. This involves controlling the speeds and directions of the four DC motors to achieve forward, backward, and turning motions.

**Mathematical Formulation:**

The following equations can describe the differential drive kinematics:

$$v = \frac{r}{2} (\omega_r + \omega_l)$$

$$\omega = \frac{r}{d} (\omega_r - \omega_l)$$

Where:
- `v`  is the linear velocity of the robot.
- `omega` is the angular velocity of the robot.
- `r` is the radius of the wheels.
- `d` is the distance between the wheels.
- `omega_r` and `omega_l` are the angular velocities of the right and left wheels, respectively.

**Implementation:**
In the motor control script, the speeds of the motors are controlled using PWM signals. The direction of each motor is controlled using GPIO pins to achieve the desired motion. The motor encoders provide feedback to adjust the motor speeds and directions accurately.

#### PID Control
Proportional-Integral-Derivative (PID) controllers are used to maintain precise control over the motor speeds. Each motor's speed is regulated using a PID controller that adjusts the motor's power based on the difference between the desired and actual speeds.

**PID Control Equations:**

$$u(t) = K_p e(t) + K_i \int e(t) dt + K_d \frac{de(t)}{dt}$$

where:
- `u(t)` is the control input.
- `K_p`, `K_i`, and `K_d` are the proportional, integral, and derivative gains, respectively.
- `e(t)` is the error between the desired and actual speeds.

**Implementation:**
In the motor control script, PID controllers are implemented for each motor. The encoder readings provide feedback for the actual motor speeds, and the PID controller adjusts the PWM signals to maintain the desired speeds.

### Sensor Integration and Fusion

#### Sensor Fusion
Sensor fusion combines data from multiple sensors (GPS, IMU, LiDAR, Camera) to improve the accuracy and robustness of the robot's navigation and obstacle detection.

**Implementation:**
The sensor fusion script continuously collects data from the GPS, IMU, LiDAR, and Camera sensors. The data is integrated to estimate the robot's state (position, velocity, orientation) using a Kalman filter.

#### Kalman Filtering
A Kalman filter is used to estimate the robot’s state from noisy sensor measurements. The filter integrates data from the GPS, IMU, and other sensors to provide a more accurate and consistent estimate of the robot's position and movement.

**Kalman Filter Equations:**

![Screenshot 2024-07-20 090501](https://github.com/user-attachments/assets/a1c650a5-4e41-4cce-9981-f05b83b098e6)

Where:

- `hat{x}_{k|k-1}` is the predicted state estimate.
- `P_{k|k-1}` is the predicted covariance estimate.
- `K_k` is the Kalman gain.
- `z_k` is the measurement.
- `F`, `B`, `H`, `Q`, and `R` are the system matrices.

### Localization and Mapping

#### GPS Navigation
The robot uses a GPS module to record its path and navigate to predefined coordinates. GPS coordinates are used for global path planning and to track the robot's position on a larger scale.

**Implementation:**
The GPS module provides latitude and longitude data, which is recorded and used to define the robot's path. The path-following script uses this data to guide the robot to its destination.

#### IMU Data
An IMU (Inertial Measurement Unit) provides accelerometer and gyroscope data to estimate the robot's orientation and motion. This data is crucial for correcting the robot's course and maintaining stability.

**Implementation:**
The IMU module provides orientation (roll, pitch, yaw) and acceleration data. This data is integrated with the GPS data in the sensor fusion script to improve the accuracy of the robot's state estimation.

#### LiDAR Mapping
A 2D LiDAR sensor creates occupancy grids to identify obstacles in the robot's path. The LiDAR data is processed to map the environment, helping the robot to detect and avoid obstacles dynamically.

**Implementation:**
The LiDAR object detection script processes the LiDAR data to detect obstacles. The data is used to create an occupancy grid, which is used for obstacle detection and avoidance.

### Path Planning and Following

#### Carrot Chasing Algorithm
The carrot-chasing algorithm is used for path following, where the robot follows a look-ahead point (carrot) on the predefined path. The robot continually adjusts its direction to chase the carrot point, ensuring it stays on course.

**Algorithm:**

1. Determine the current position of the robot.
2. Find the path's next look-ahead point (carrot) within a specified look-ahead distance.
3. Calculate the direction and distance to the carrot point.
4. Adjust the robot's movement to follow the carrot point.

**Implementation:**
The path-following script implements the carrot-chasing algorithm. The current position is fetched from the sensor fusion script, and the next carrot point is determined. The robot's movement is adjusted accordingly.

#### A* Pathfinding
When obstacles are detected, the A* algorithm finds the shortest path around them. This algorithm calculates an optimal detour, allowing the robot to navigate around obstacles and return to the predefined path efficiently.

**A* Algorithm:**

1. Initialize the start and goal nodes.
2. Add the start node to the open list.
3. While the open list is not empty:
   - Select the node with the lowest f-cost (g-cost + h-cost).
   - If the goal node is reached, reconstruct the path.
   - Otherwise, expand the node and add its neighbors to the open list.
4. Use the reconstructed path as the detour.

**Implementation:**
When an obstacle is detected, the detour path is calculated using the A* algorithm. The robot follows the detour path and then returns to the predefined path.

### Obstacle Detection and Avoidance

#### LiDAR Obstacle Detection
The LiDAR sensor detects obstacles based on distance measurements. When an obstacle is detected within a threshold distance, the robot stops and initiates rerouting to avoid the obstacle.

**Implementation:**
The LiDAR object detection script reads the LiDAR data and checks for obstacles within a threshold distance. If an obstacle is detected, the robot stops and calculates a detour path using the A* algorithm.

#### Camera-Based Obstacle Detection
The camera detects obstacles in the robot's field of view using image processing techniques. This helps in identifying obstacles that may not be detected by the LiDAR, particularly those that are below the LiDAR's detection height.

**Implementation:**
The camera path planning script captures frames from the camera and processes them to detect obstacles. If an obstacle is detected, the robot stops and calculates a detour path using the A* algorithm.

### Real-Time Control and Decision Making

#### Control Loops
Real-time control loops adjust motor speeds and directions based on sensor inputs. These loops ensure the robot responds quickly to changes in its environment, maintaining smooth and accurate movement.

**Implementation:**
Control loops are implemented in the motor control script. The PID controllers adjust the motor speeds based on encoder feedback to maintain the desired speeds.

#### Dynamic Re-Routing

Dynamic re-routing enables the robot to adapt to obstacles in real-time by calculating detour paths around obstacles and then returning to the original predefined path. This process involves obstacle detection, path planning, and seamlessly transitioning between paths to ensure the robot reaches its final destination efficiently and safely.

**Detailed Steps**

1. **Obstacle Detection**:
   - **LiDAR Obstacle Detection**: The LiDAR sensor continuously scans the environment and measures distances to nearby objects. If an obstacle is detected within a threshold distance (e.g., 1 meter), the robot immediately stops.
     - **Implementation**:
       - The `read_lidar_data()` function in `lidar_object_detection.py` reads the LiDAR data.
       - If any distance measurement is below the threshold, the `handle_obstacle()` function is triggered.
   - **Camera-Based Obstacle Detection**: The camera captures frames, and image processing techniques are used to identify obstacles within the robot's field of view. If an obstacle is detected, the robot stops.
     - **Implementation**:
       - The `get_camera_data()` function in `camera_path_planning.py` captures and processes camera frames.
       - If an obstacle is detected, the `camera_handle_obstacle()` function is triggered.

2. **Handling Obstacle**:
   - **Stop the Robot**: As soon as an obstacle is detected, the robot's motors are stopped to prevent a collision.
     - **Implementation**:
       - The `stop()` function in `motor_control.py` stops all motors.
   - **Wait for 25 Seconds**: The robot waits for 25 seconds to check if the obstacle clears on its own.
     - **Implementation**:
       - The `time.sleep(25)` statement in the `handle_obstacle()` function introduces a 25-second delay.

3. **Re-Check for Obstacle**:
   - **Persistent Obstacle**: If the obstacle is still present after 25 seconds, the robot needs to calculate a detour path around it.
   - **Obstacle Cleared**: If the obstacle has cleared, the robot resumes its predefined path.

4. **Calculate Detour Path**:
   - **Occupancy Grid Creation**: The LiDAR or camera data is used to create an occupancy grid, which maps the obstacles in the robot's environment.
     - **Implementation**:
       - The `calculate_detour_path(scan_data)` function processes the sensor data to create the occupancy grid.
   - **A* Pathfinding Algorithm**: The A* algorithm is used to calculate the shortest path around the detected obstacles.
     - **A* Algorithm Steps**:
       1. **Initialize**: Set the start node (current position) and goal node (next point on the predefined path).
       2. **Open List**: Add the start node to the open list (nodes to be evaluated).
       3. **Node Evaluation**: Continuously evaluate nodes by selecting the one with the lowest f-cost (g-cost + h-cost).
       4. **Path Reconstruction**: If the goal node is reached, reconstruct the path by tracing back from the goal node to the start node.
       5. **Neighbor Evaluation**: For each evaluated node, add its walkable neighbors to the open list and repeat the evaluation process.
     - **Implementation**:
       - The `a_star_algorithm()` function in `lidar_object_detection.py` and `camera_path_planning.py` implements the A* pathfinding algorithm.

5. **Follow Detour Path**:
   - **Move Along Detour Path**: The robot follows the detour path calculated by the A* algorithm to navigate around the obstacle.
     - **Implementation**:
       - The `follow_path(path)` function in `lidar_object_detection.py` and `camera_path_planning.py` moves the robot along the detour path by sequentially following the points on the path.
   - **Real-Time Adjustments**: As the robot follows the detour path, it continuously monitors for new obstacles and makes real-time adjustments if necessary.

6. **Return to Predefined Path**:
   - **Recalculate Position**: Once the robot has navigated around the obstacle, it recalculates its position relative to the predefined path.
     - **Implementation**:
       - The `find_nearest_point_on_path(current_position, path)` function in `path_following.py` determines the nearest point on the predefined path.
   - **Re-Join Predefined Path**: The robot adjusts its movement to return to the nearest point on the predefined path and resumes following the original route towards the final destination.
     - **Implementation**:
       - The `return_to_path()` function in `lidar_object_detection.py` and `camera_path_planning.py` calculates the direction and distance to the nearest point on the predefined path and adjusts the robot's movement accordingly.

By following these detailed steps, the D.A.I.S.E. project ensures that the robot can dynamically adapt to obstacles, calculate detour paths using the A* algorithm, and seamlessly return to the predefined path, all while maintaining accurate and efficient navigation towards the final destination.

### Visualization and Debugging

#### Real-Time Visualization
Real-time visualization displays the robot's path, position, and detected obstacles for monitoring and debugging purposes.

**Implementation:**
The visualization script uses Matplotlib to create real-time plots of the robot's path and position. The plots are updated continuously with the robot's current state and detected obstacles.

#### Data Logging
Data logging records sensor data and robot states for analysis and tuning. This data is essential for improving the robot's performance and making necessary adjustments to the algorithms.

**Implementation:**
Sensor data and robot states are logged to files in CSV format. These logs can be analyzed to understand the robot's behavior and identify areas for improvement.

### Software Development Practices

#### Modular Design
The code is structured into modules that handle specific tasks, such as motor control, path following, obstacle detection, and sensor fusion. This modular design improves readability, maintainability, and scalability of the software.

**Implementation:**
Each script in the project is designed to perform a specific function. For example, `motor_control.py` handles motor control, `path_following.py` handles path following, and `sensor_fusion.py` handles sensor fusion. This separation of concerns makes the code easier to understand and maintain.

#### Threading
Threads are used to handle different tasks concurrently, such as running the sensor fusion loop, reading sensor data, and controlling motors. Threading ensures that the robot can perform multiple operations simultaneously, improving efficiency and responsiveness.

**Implementation:**
The sensor fusion process runs in a separate thread, allowing it to continuously integrate data from multiple sensors while the main control loop handles navigation and obstacle detection. The `threading` module in Python is used to create and manage threads.


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

## Video Demo
To see the D.A.I.S.E. robot in action, check out our video demonstration:

https://sites.google.com/view/cse-598-group-12/home

---

## Improvements or Future Work
The D.A.I.S.E. project represents a successful prototype of a low-cost autonomous delivery robot. However, several improvements and future developments can enhance its performance and capabilities:

1. **Buck Converter**: In future iterations, I will use the DFrobot Buck converter, known for its accuracy and reliability.
2. **Better Motors**: Upgrading to higher quality motors will improve the robot's movement precision and durability.
3. **Depth Camera or Stereo Camera**: Incorporating a depth camera or a stereo camera system will enhance the robot's obstacle detection and path planning abilities.
4. **RTK GPS Module**: The current GPS module lacks the necessary accuracy for long-distance navigation. An RTK GPS module will significantly improve the robot's positional accuracy.
5. **Deep Learning or VSLAM**: Integrating deep learning techniques or Visual Simultaneous Localization and Mapping (VSLAM) will enable more advanced navigation and environment understanding.
6. **Custom PCB**: Developing a custom PCB will streamline the electronic setup, reduce wiring complexity, and improve reliability.
7. **Data Sharing System**: Enhancing the data sharing system between the robot and the main computer is crucial. Implementing WebRTC or WebSocket for data transmission and developing a web interface for data visualization and control using WiFi will facilitate better communication and control.
8. **Delivery System and Payload**: To make the robot suitable for real-world delivery applications, adding a payload carrying system and developing a customer-oriented delivery system will be essential.
9. **Improved Battery Management**: Implementing a more efficient battery management system will extend the operational time of the robot.
10. **Enhanced Software Architecture**: Refactoring the software to make it more modular and scalable will facilitate easier maintenance and future upgrades.

---

## Acknowledgment
This project was undertaken as part of the course CSE 598: Perception in Robotics during Spring 2024.

**Professor:**
Dr. Nakul Gopalan

**Team Members:**
- Sai Shivani Lingampally
- Anish Sai Racha
- Jhanvi Shailesh Shah

I would like to express my gratitude to Dr. Nakul Gopalan for his guidance and support throughout the project. Special thanks to my teammates Sai Shivani Lingampally, Anish Sai Racha, and Jhanvi Shailesh Shah for their collaboration and hard work.

Thank you to everyone who contributed to the success of this project.

---

