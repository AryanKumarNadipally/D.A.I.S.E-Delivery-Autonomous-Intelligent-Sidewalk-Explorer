![Daise Logo](https://github.com/user-attachments/assets/79a0a723-003d-4940-9f4d-7455bb1a33df)

---

# D.A.I.S.E - Delivery Autonomous Intelligent Sidewalk Explorer

## Introduction
Welcome to the D.A.I.S.E project! This repository contains the complete source code and documentation for building and operating a low-cost delivery robot within a $400 budget. This project aims to demonstrate that effective robotic delivery systems can be developed economically without relying on prebuilt libraries or software.

D.A.I.S.E (Delivery Autonomous Intelligent Sidewalk Explorer) is designed to navigate sidewalks, avoid obstacles, and deliver goods to specified locations using a combination of GPS, LiDAR, camera, and IMU sensors. The project leverages sensor fusion and custom algorithms for path planning, obstacle avoidance, and re-routing.

This project is the first prototype of D.A.I.S.E, built using materials and tools available within our budget constraints. As a prototype, the robot does not carry any payload because our initial focus is testing and validating the software and navigation systems.


---

## Objectives

- **Low-Cost Design**: Develop a functional delivery robot within a $400 budget.
- **Autonomous Navigation**: Enable the robot to autonomously navigate from a starting point to a destination using GPS coordinates.
- **Obstacle Avoidance**: Implement robust obstacle detection and avoidance using LiDAR and camera sensors.
- **Sensor Fusion**: Combine data from multiple sensors (GPS, IMU, LiDAR, Camera) to improve navigation accuracy.
- **Real-Time Visualization**: Real-time visualization of the robot's path and current position.

---
 
## Key Features

- **GPS-Based Navigation**: The robot uses GPS coordinates for global path planning and navigation.
- **Carrot-chasing Algorithm**: Implements a carrot-chasing algorithm to guide the robot along the predefined path.
- **LiDAR Obstacle Detection**: A 2D LiDAR sensor detects and avoids obstacles in the robot's path.
- **Camera-Based Path Planning**: A single Raspberry Pi camera is used for local path planning and obstacle detection.
- **IMU Integration**: Incorporates an IMU sensor to monitor and adjust the robot's orientation.
- **Sensor Fusion**: Combines data from GPS, IMU, LiDAR, and camera for improved accuracy and reliability.
- **Real-Time Visualization**: Displays the robot's path and current position using Matplotlib for real-time monitoring.

---

## Hardware Components

- **Raspberry Pi 4 Computer Model B**: The main processing unit for running the robot's software.
- **Four DC Motors with Hall Effect Sensors**: Provide mobility and speed control feedback.
- **RPLIDAR A1**: A 2D LiDAR sensor for obstacle detection.
- **Adafruit Ultimate GPS Breakout Board**: For GPS-based navigation.
- **MPU-6050**: A 3-axis Accelerometer and Gyro Sensor for orientation.
- **Arducam 5MP Camera (OV5647)**: For local path planning and obstacle detection.
- **Power System**: Includes a LiPo battery, buck converter, and L298N motor drivers.

---

## Mechanical Design

### CAD model

![Daise Cad 1](https://github.com/user-attachments/assets/fba48c3a-4e46-4f30-a954-b9f4530a71fc)

![1](https://github.com/user-attachments/assets/b6c09015-8b60-48e4-a60f-0cec64677b67)

![2](https://github.com/user-attachments/assets/0d81e27a-5235-48e2-bed2-acbad210dda5)

As this is a prototype, the outer body was designed using a storage box from a dollar store, and 3D-printed parts were used to attach and support certain components. All the components were arranged and placed with consideration to maintain the center of gravity and ensure stability. This approach allowed us to focus on the functionality and testing of the software while staying within our budget constraints.

---

## Electronic Connections.

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

---

## Assembly of D.A.I.S.E
https://github.com/user-attachments/assets/a9027fc0-b29c-4042-94c8-281aaddbccc7

https://github.com/user-attachments/assets/8f086992-f5dc-48b8-a38d-5520f3ea8299

---

## Control system of the D.A.I.S.E

The control system of the D.A.I.S.E. robot involves managing the four-wheel-drive (4WD) system using differential drive kinematics and Proportional-Integral-Derivative (PID) controllers. This section details how the motors are controlled to achieve precise movement and speed regulation.

![Daise Control Flowchart3](https://github.com/user-attachments/assets/c6ef8c7a-f5a8-4bc9-852c-f7f78a239119)

1. **Start**

   **Initialize System Components:**
   - Set up the Raspberry Pi, motor driver (L298N), and DC motors.
   - Configure GPIO pins for motor control and encoder feedback.
   - Initialize PWM signals for motor speed control.

2. **Motor Initialization**

   **Initialize Motors:**
   - Define motor control functions to set speed and direction for each motor.
   - Use GPIO pins to control the direction and PWM signals to control the speed.
   - Ensure that each motor's encoder is correctly connected for feedback.

3. **Motor Speed Control**

   **Set Desired Motor Speeds:**
   - Use path-following and navigation algorithms to determine the desired speeds for each motor.
   - Set target speeds for the motors based on the desired movement direction (forward, backward, left, right).

4. **PID Control Loop**

   **Initialize PID Controllers:**
   - Initialize PID controllers for each motor with predefined gains (\( K_p \), \( K_i \), \( K_d \)).
   - Define PID control equations to adjust motor speeds based on the error between desired and actual speeds:
     
     $$u(t) = K_p e(t) + K_i \int e(t) dt + K_d \frac{de(t)}{dt}$$
     
     Where \( u(t) \) is the control input, \( K_p \), \( K_i \), and \( K_d \) are the gains, and \( e(t) \) is the error.

   **Real-Time Motor Control:**
   - **Read Encoder Feedback:**
     - Continuously read encoder values to get the actual speeds of each motor.
     - Calculate the error between the desired and actual motor speeds.
   - **Adjust Motor Speeds:**
     - Use the PID controllers to adjust the PWM signals for each motor.
     - Update the motor speeds in real-time to minimize the error and achieve smooth and accurate movement.

5. **4WD Differential Drive Kinematics**

   **Calculate Robot Velocity:**
   - Use differential drive kinematics equations to calculate the robot’s linear and angular velocities:
     
     $$v = \frac{r}{2} (\omega_r + \omega_l)$$
     
     $$\omega = \frac{r}{d} (\omega_r - \omega_l)$$

     Where \( v \) is the linear velocity, \( \omega \) is the angular velocity, \( r \) is the wheel radius, \( d \) is the distance between wheels, and \( \omega_r \) and \( \omega_l \) are the wheel angular velocities.

   **Control Robot Movement:**
   - Adjust motor speeds to control the robot's linear and angular velocities.
   - Use kinematic equations to ensure coordinated movement of all four wheels.

6. **Dynamic Adjustments**

   **Handle Real-Time Adjustments:**
   - Continuously monitor the robot’s movement and make dynamic adjustments as needed.
   - Ensure the robot follows the predefined path and responds to obstacles effectively.

7. **Integration with Other Processes**

   **Integrate with Sensor Fusion:**
   - Use sensor fusion data to adjust motor speeds and directions based on the robot’s current position and orientation.
   - Ensure the robot stays on course and navigates accurately.

   **Respond to Obstacle Detection:**
   - Adjust motor speeds and directions in real-time to avoid obstacles detected by the LiDAR and camera sensors.
   - Use the dynamic re-routing process to navigate around obstacles and return to the original path.

8. **End**

   **Complete Motor Control Process:**
   - Ensure the robot reaches its destination accurately and efficiently.
   - Review the logged data to assess performance and identify areas for improvement.
     
---

## Sensor Integration and Fusion 

Sensor integration and fusion involve combining data from multiple sensors to improve the accuracy and robustness of the robot's navigation and obstacle detection. This section details how the D.A.I.S.E. robot integrates data from GPS, IMU, LiDAR, and Camera sensors to estimate its state using a Kalman filter.

![Daise Sensor Fusion Flowchart](https://github.com/user-attachments/assets/23bb7a04-4d95-4a37-a396-93b89daf3616)

1. **Start:**
   - Set up the Raspberry Pi and initialize the necessary libraries for sensor data collection and processing.
   - Configure GPIO pins for sensor integration and ensure all sensors (GPS, IMU, LiDAR, Camera) are correctly connected and initialized.

2. **Start Sensor Fusion Thread:**
   - Initiate Sensor Fusion Process:
     - Start a separate thread for the sensor fusion process to ensure continuous and real-time data integration from multiple sensors.
     - The sensor fusion script begins collecting data from the GPS, IMU, LiDAR, and Camera sensors.

3. **Read Sensor Data:**
   - **Read GPS Data:**
     - Continuously read latitude and longitude data from the GPS module.
     - Parse the GPS data to obtain the current coordinates of the robot.
     - This data provides the global position of the robot for path planning.
   
   - **Read IMU Data:**
     - Continuously read accelerometer and gyroscope data from the IMU (MPU-6050).
     - Parse the IMU data to obtain orientation (roll, pitch, yaw) and acceleration information.
     - This data helps in estimating the robot’s orientation and motion dynamics.
   
   - **Read LiDAR Data:**
     - Continuously read distance measurements from the LiDAR sensor.
     - Parse the LiDAR data to detect obstacles and map the environment.
     - This data helps in creating an occupancy grid for obstacle detection and avoidance.
   
   - **Read Camera Data:**
     - Continuously capture frames from the camera.
     - Process the frames to detect obstacles using image processing techniques.
     - This data helps in identifying obstacles that may not be detected by the LiDAR, particularly those that are below the LiDAR's detection height.

4. **Estimate Robot State with Kalman Filter:**
   - **Integrate Sensor Data:**
     - Combine data from the GPS, IMU, LiDAR, and Camera sensors using a Kalman filter.
     - The Kalman filter predicts and updates the robot’s state (position, velocity, orientation) based on sensor measurements.

   - **Kalman Filter Equations:**
     - **Prediction:**
       ```math
       \hat{x}_{k|k-1} = F \hat{x}_{k-1|k-1} + B u_{k-1}
       ```
       ```math
       P_{k|k-1} = F P_{k-1|k-1} F^T + Q
       ```
     - **Update:**
       ```math
       K_k = P_{k|k-1} H^T (H P_{k|k-1} H^T + R)^{-1}
       ```
       ```math
       \hat{x}_{k|k} = \hat{x}_{k|k-1} + K_k (z_k - H \hat{x}_{k|k-1})
       ```
       ```math
       P_{k|k} = (I - K_k H) P_{k|k-1}
       ```

     Where:
     - \( \hat{x}_{k|k-1} \) is the predicted state estimate.
     - \( P_{k|k-1} \) is the predicted covariance estimate.
     - \( K_k \) is the Kalman gain.
     - \( z_k \) is the measurement.
     - \( F \), \( B \), \( H \), \( Q \), and \( R \) are the system matrices.

   - **Predict Robot State:**
     - Use the Kalman filter prediction step to estimate the robot's state based on previous state estimates and control inputs.
     - This prediction accounts for the robot's motion dynamics and provides an initial estimate for the current state.

   - **Update Robot State:**
     - Use the Kalman filter update step to refine the robot's state estimate based on new sensor measurements.
     - Calculate the Kalman gain to determine the weighting of the prediction and measurement.
     - Update the state estimate and covariance matrix to improve accuracy.

5. **Output Estimated State:**
   - **Continuous State Output:**
     - The estimated state (position, velocity, orientation) is continuously outputted for use in path planning and navigation.
     - This state information is critical for accurately guiding the robot along its path and avoiding obstacles.

6. **Log Sensor Data:**
   - **Data Logging:**
     - Record the raw sensor data (GPS, IMU, LiDAR, Camera) and the estimated robot state for analysis.
     - Store the data in CSV files for post-operation review and tuning.
     - This data is essential for improving the robot’s localization and mapping accuracy.

7. **End:**
   - **Complete Sensor Fusion Process:**
     - Ensure the continuous and accurate integration of sensor data throughout the robot's operation.
     - Review the logged data to assess performance and identify areas for improvement.
---

## Localization and Mapping

Localization and mapping involve using sensor data to determine the robot's position and create a map of its environment. This section details how the D.A.I.S.E. robot uses GPS, IMU, and LiDAR data to navigate and avoid obstacles.

![Daise Localisation and Mapping Flochart](https://github.com/user-attachments/assets/516d8caa-9a5e-4b67-9e55-e4127fa0eba3)

1. **Start:**
   - Initialize System Components:
     - Set up the GPS module, IMU (MPU-6050), and LiDAR sensor on the Raspberry Pi.
     - Establish serial communication with the GPS module.
     - Initialize the I2C communication for the IMU.
     - Establish communication with the LiDAR sensor.

2. **Record GPS Coordinates:**
   - Record GPS Coordinates:
     - Record GPS coordinates along the desired path using the `record_coordinates.py` script.
     - Save the recorded GPS data in a CSV file for future use.

3. **Convert and Visualize GPS Data:**
   - Convert GPS Data to KML for Visualization:
     - Convert the recorded GPS data to a KML file for visualization in Google Earth.
     - Manually correct any errors in the recorded coordinates to ensure an accurate path.

4. **Read Sensor Data:**
   - **Read GPS Data:**
     - Continuously read latitude and longitude data from the GPS module.
     - Parse the GPS data to obtain the current coordinates of the robot.
     - Store the GPS data for real-time navigation and path recording.
   
   - **Read IMU Data:**
     - Continuously read accelerometer and gyroscope data from the IMU.
     - Parse the IMU data to obtain orientation (roll, pitch, yaw) and acceleration information.
     - Store the IMU data for real-time orientation and motion dynamics.

   - **Read LiDAR Data:**
     - Continuously read distance measurements from the LiDAR sensor.
     - Parse the LiDAR data to detect obstacles and map the environment.
     - Store the LiDAR data for real-time mapping and obstacle detection.

5. **Estimate Robot State with Sensor Fusion:**
   - **Predict Robot State:**
     - Use the Kalman filter to integrate data from the GPS and IMU sensors.
     - Predict the robot’s state (position, velocity, orientation).
       
   - **Update Robot’s Position:**
     - Continuously update the robot’s position based on the sensor fusion output.
     - Use the updated position to guide the robot along the predefined path.

6. **Create Occupancy Grid:**
   - **Generate Occupancy Grid Map:**
     - Use the LiDAR data to create an occupancy grid map of the environment.
     - Mark occupied cells where obstacles are detected and free cells for navigable space.

   - **Integrate Mapping with Localization:**
     - Combine the occupancy grid with the robot’s current position and orientation from the sensor fusion.
     - Update the map in real-time as the robot moves and new LiDAR data is obtained.

7. **Use the Map for Path Planning and Obstacle Avoidance:**
   - **Utilize Occupancy Grid for Path Planning:**
     - Use the occupancy grid for path planning algorithms to navigate around obstacles.
     - Continuously update the path based on the latest map information to ensure safe and efficient navigation.

   - **Execute Path Planning and Obstacle Avoidance:**
     - Implement dynamic re-routing to adapt to obstacles in real-time.
     - Calculate detour paths around obstacles using the A* algorithm.
     - Follow the detour path and then return to the original predefined path.

8. **Log Sensor Data:**
   - **Record and Analyze Sensor Data:**
     - Log the GPS, IMU, and LiDAR data along with the estimated robot state for analysis.
     - Store the data in CSV files for post-operation review and tuning.
     - Use the recorded data to improve the robot’s localization and mapping accuracy.

9. **End:**
   - **Complete Localization and Mapping Process:**
     - Ensure the robot has reached its final destination accurately.
     - Review the logged data to assess performance and identify areas for improvement.
---

## Path Planning and Following

Path planning and following are crucial aspects of the D.A.I.S.E. robot's functionality. These processes involve determining the optimal path for the robot to reach its destination and dynamically adjusting the path to avoid obstacles. This section details the carrot-chasing algorithm for path following and the A* algorithm for obstacle avoidance.

![Daise Path Planning and Following](https://github.com/user-attachments/assets/e1433161-5156-4348-8e16-623ab92105c4)

1. **Start:**
   - Initialize System Components:
     - Set up the Raspberry Pi, motor driver (L298N), and DC motors.
     - Configure GPIO pins for motor control and encoder feedback.
     - Initialize the PWM signals for motor speed control.

2. **Record GPS Coordinates:**
   - Run Record Coordinates Script:
     - Use the `record_coordinates.py` script to record GPS coordinates along the desired path.
     - Save the recorded GPS data in a CSV file.

3. **Load Path Data:**
   - Load Path from CSV:
     - Load the saved GPS data from the CSV file.
     - Convert coordinates for path following.

4. **Initialize Carrot Chasing Algorithm:**
   - Define Lookahead Distance:
     - Set the lookahead distance for the carrot-chasing algorithm.
   - Initialize Variables for Current Position and Carrot:
     - Set initial values for the robot's current position and the carrot point.

5. **Path Following Using Carrot Chasing Algorithm:**
   - Find Carrot Point:
     - Determine the next lookahead point (carrot) on the path.
   - Calculate Direction and Distance to Carrot:
     - Compute the direction and distance from the robot's current position to the carrot point.
   - Move to Carrot Point:
     - Adjust the robot's movement to follow the carrot point.
   - Integrate with Sensor Fusion:
     - Use sensor fusion data to refine the robot's position and orientation.
   - Respond to Control Commands:
     - Adjust motor speeds and directions based on control commands.

6. **Obstacle Detection and Avoidance:**
   - Detect Obstacles:
     - Continuously monitor the environment using LiDAR and camera sensors.
   - Calculate Detour Path using A*:
     - Create Occupancy Grid from Sensor Data:
       - Generate an occupancy grid map using LiDAR data.
     - Initialize Start and Goal Nodes:
       - Set the current position as the start node and the carrot point as the goal node.
     - Evaluate Nodes by Lowest f-cost:
       - Continuously evaluate nodes to find the path with the lowest f-cost (g-cost + h-cost).
     - Reconstruct Path from Goal to Start:
       - Reconstruct the path from the goal node back to the start node.

7. **Follow Detour Path:**
   - Handle Real-Time Adjustments:
     - Adjust the robot's movement in real-time to follow the detour path.
   - Clear Obstacle and Recalculate Position:
     - Once the obstacle is cleared, recalculate the robot's position.
   - Find Nearest Point on Predefined Path:
     - Determine the nearest point on the original path.
   - Adjust Movement to Return to Predefined Path:
     - Guide the robot back to the predefined path.

8. **End:**
   - Complete Path Planning and Following Process:
     - Ensure the robot reaches its destination accurately and efficiently.
     - Review the logged data to assess performance and identify areas for improvement.

### Carrot Chasing Algorithm
The carrot-chasing algorithm is used for path following, where the robot follows a look-ahead point (carrot) on the predefined path. The robot continually adjusts its direction to chase the carrot point, ensuring it stays on course.

**Algorithm:**
1. Determine the current position of the robot.
2. Find the path's next look-ahead point (carrot) within a specified look-ahead distance.
3. Calculate the direction and distance to the carrot point.
4. Adjust the robot's movement to follow the carrot point.

**Implementation:**
The path-following script implements the carrot-chasing algorithm. The current position is fetched from the sensor fusion script, and the next carrot point is determined. The robot's movement is adjusted accordingly.

### A* Pathfinding
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

---

## Obstacle Detection and Avoidance

The obstacle detection and avoidance system of the D.A.I.S.E. robot ensures safe navigation by detecting obstacles using LiDAR and camera sensors and calculating detour paths when necessary. This section details how the robot detects obstacles and dynamically adjusts its path to avoid them.

![Daise Obstacle Detection and Avoidance](https://github.com/user-attachments/assets/53678c0f-bb45-43db-ba9c-cc4555217b04)

1. **Start**

2. **Initialize System Components:**
   - Set up the Raspberry Pi, LiDAR sensor, and camera.
   - Initialize the necessary libraries and configure the GPIO pins for sensor integration.

3. **Sensor Data Collection:**

   **Initialize LiDAR Sensor:**
   - Start the LiDAR sensor and check its health.
   - Establish a connection to continuously read LiDAR data.

   **Initialize Camera:**
   - Set up the camera module and load the calibration parameters.
   - Ensure the camera is ready to capture frames for obstacle detection.

4. **Obstacle Detection:**

   **Read LiDAR Data:**
   - Continuously read data from the LiDAR sensor using the `read_lidar_data()` function in the `lidar_object_detection.py` script.
   - Process the LiDAR data to detect obstacles within a specified threshold distance (e.g., 1 meter).

   **Read Camera Data:**
   - Continuously capture frames from the camera using the `get_camera_data()` function in the `camera_path_planning.py` script.
   - Preprocess the camera frames (grayscale conversion, blurring, edge detection) to detect obstacles in the robot's field of view.

   **Detect Obstacles:**
   - Use LiDAR data to detect obstacles based on distance measurements.
   - Use camera data to identify obstacles that may not be detected by the LiDAR, particularly those below the LiDAR's detection height.

5. **Handling Obstacles:**

   **Stop the Robot:**
   - As soon as an obstacle is detected, stop the robot's motors to prevent a collision.
   - Use the `stop()` function in the `motor_control.py` script to stop all motors.

   **Wait for 25 Seconds:**
   - Introduce a 25-second delay to check if the obstacle clears on its own.
   - Use the `time.sleep(25)` statement in the `handle_obstacle()` function to wait.

   **Re-Check for Obstacle:**
   - After 25 seconds, re-check the sensor data to see if the obstacle is still present.
   - If the obstacle has cleared, resume following the predefined path.
   - If the obstacle persists, proceed to calculate a detour path.

6. **Calculate Detour Path:**

   **Create Occupancy Grid:**
   - Use the LiDAR or camera data to create an occupancy grid, mapping the obstacles in the robot's environment.
   - Use the `calculate_detour_path(scan_data)` function in the `lidar_object_detection.py` or `camera_path_planning.py` script to create the occupancy grid.

   **A* Pathfinding Algorithm:**
   - Use the A* algorithm to calculate the shortest path around the detected obstacles.
   - Initialize the start node (current position) and goal node (next point on the predefined path).
   - Add the start node to the open list (nodes to be evaluated).
   - Continuously evaluate nodes by selecting the one with the lowest f-cost (g-cost + h-cost).
   - If the goal node is reached, reconstruct the path by tracing back from the goal node to the start node.
   - Add walkable neighbors of each evaluated node to the open list and repeat the evaluation process.

7. **Follow Detour Path:**

   **Move Along Detour Path:**
   - Use the `follow_path(path)` function in the `lidar_object_detection.py` or `camera_path_planning.py` script to navigate along the calculated detour path.
   - Move the robot along the detour path, following each point sequentially.

   **Real-Time Adjustments:**
   - Continuously monitor for new obstacles and make real-time adjustments to the path.
   - Ensure the robot dynamically adapts to changes in the environment.

8. **Return to Predefined Path:**

   **Recalculate Position:**
   - Once the robot has navigated around the obstacle, recalculate its position relative to the predefined path.
   - Use the `find_nearest_point_on_path(current_position, path)` function in the `path_following.py` script to determine the nearest point on the predefined path.

   **Re-Join Predefined Path:**
   - Adjust the robot's movement to return to the nearest point on the predefined path and resume following the original route towards the final destination.
   - Use the `return_to_path()` function in the `lidar_object_detection.py` or `camera_path_planning.py` script to guide the robot back to the predefined path.

9. **Integration with Other Processes:**

   **Integrate with Sensor Fusion:**
   - Use sensor fusion data to update the robot's current position and orientation.
   - Ensure accurate navigation by integrating data from GPS, IMU, LiDAR, and camera sensors.

   **Respond to Control Commands:**
   - Adjust motor speeds and directions based on obstacle detection and avoidance requirements.
   - Ensure coordinated movement and accurate navigation towards the destination.

10. **End:**

   **Complete Obstacle Detection and Avoidance Process:**
   - Ensure the robot reaches its destination accurately and efficiently.
   - Review the logged data to assess performance and identify areas for improvement.
---

## Real-Time Control and Decision Making


The real-time control and decision-making system of the D.A.I.S.E. robot involves adjusting motor speeds and directions based on sensor inputs, ensuring the robot responds quickly to changes in its environment. This includes real-time obstacle detection, dynamic re-routing, and seamless transition between paths to reach the final destination efficiently and safely.

![Daise Real-Time Control and Decision Making](https://github.com/user-attachments/assets/af95c0a9-3d25-4d56-833b-48bff42ef082)

1. **Start**

2. **Initialize System Components:**
   - Set up the Raspberry Pi, motors, and sensors.
   - Initialize necessary libraries and configure GPIO pins for motor control and sensor integration.

3. **Control Loops:**

   **Initialize Motor Control:**
   - Set up GPIO pins for motor control and encoders.
   - Initialize PWM signals for motor speed control.
   - Ensure motors are ready to receive control commands.

   **Initialize Sensors:**
   - Initialize sensors (GPS, IMU, LiDAR, Camera) for data collection.
   - Establish communication protocols (e.g., I2C for IMU, serial for GPS, USB for LiDAR, and CSI for Camera).

4. **Real-Time Data Collection:**

   **Read Sensor Data:**
   - Continuously read data from GPS, IMU, LiDAR, and Camera sensors.
   - Use the sensor fusion script to integrate data and estimate the robot's state (position, velocity, orientation).

5. **PID Control:**

   **Compute PID Control Inputs:**
   - Calculate the control input for each motor using PID controllers.
   - Use encoder feedback to determine the actual speed of each motor.
   - Calculate the error between desired and actual speeds.

   **Adjust Motor Speeds:**
   - Adjust the PWM signals to the motors based on PID control inputs.
   - Use the `set_motor_speed()` function in the `motor_control.py` script to control motor speeds.
   - Ensure smooth and accurate motor control to achieve desired robot movement.

6. **Dynamic Re-Routing:**

   **Monitor for Obstacles:**
   - Continuously monitor LiDAR and camera data for obstacle detection.
   - If an obstacle is detected, initiate the obstacle handling process.

   **Handle Obstacles:**
   - **Stop the Robot:** Immediately stop the robot's motors to prevent a collision using the `stop()` function in the `motor_control.py` script.
   - **Wait for 25 Seconds:** Introduce a 25-second delay to check if the obstacle clears on its own using the `time.sleep(25)` statement in the `handle_obstacle()` function.
   - **Re-Check for Obstacle:** After 25 seconds, re-check the sensor data. If the obstacle persists, proceed to calculate a detour path.

7. **Calculate Detour Path:**

   **Create Occupancy Grid:**
   - Use the LiDAR or camera data to create an occupancy grid, mapping the obstacles in the robot's environment.
   - Use the `calculate_detour_path(scan_data)` function in the `lidar_object_detection.py` or `camera_path_planning.py` script to create the occupancy grid.

   **A* Pathfinding Algorithm:**
   - Use the A* algorithm to calculate the shortest path around the detected obstacles.
   - **A* Algorithm Steps:**
     1. **Initialize:** Set the start node (current position) and goal node (next point on the predefined path).
     2. **Open List:** Add the start node to the open list (nodes to be evaluated).
     3. **Node Evaluation:** Continuously evaluate nodes by selecting the one with the lowest f-cost (g-cost + h-cost).
     4. **Path Reconstruction:** If the goal node is reached, reconstruct the path by tracing back from the goal node to the start node.
     5. **Neighbor Evaluation:** For each evaluated node, add its walkable neighbors to the open list and repeat the evaluation process.
   - Use the `a_star_algorithm()` function in `lidar_object_detection.py` and `camera_path_planning.py` to implement the A* pathfinding algorithm.

8. **Follow Detour Path:**

   **Move Along Detour Path:**
   - Use the `follow_path(path)` function in the `lidar_object_detection.py` and `camera_path_planning.py` scripts to navigate along the calculated detour path.
   - Move the robot along the detour path, following each point sequentially.

   **Real-Time Adjustments:**
   - Continuously monitor for new obstacles and make real-time adjustments to the path.
   - Ensure the robot dynamically adapts to changes in the environment.

9. **Return to Predefined Path:**

   **Recalculate Position:**
   - Once the robot has navigated around the obstacle, recalculate its position relative to the predefined path.
   - Use the `find_nearest_point_on_path(current_position, path)` function in the `path_following.py` script to determine the nearest point on the predefined path.

   **Re-Join Predefined Path:**
   - Adjust the robot's movement to return to the nearest point on the predefined path and resume following the original route towards the final destination.
   - Use the `return_to_path()` function in the `lidar_object_detection.py` and `camera_path_planning.py` scripts to guide the robot back to the predefined path.

10. **Integration with Other Processes:**

    **Integrate with Sensor Fusion:**
    - Use sensor fusion data to update the robot's current position and orientation.
    - Ensure accurate navigation by integrating data from GPS, IMU, LiDAR, and camera sensors.

    **Respond to Control Commands:**
    - Adjust motor speeds and directions based on obstacle detection and avoidance requirements.
    - Ensure coordinated movement and accurate navigation towards the destination.

11. **End:**

    **Complete Real-Time Control and Decision Making Process:**
    - Ensure the robot reaches its destination accurately and efficiently.
    - Review the logged data to assess performance and identify areas for improvement.


---
## Code

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

---

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

- In the terminal, navigate to the directory containing `main.py`.
- Run the script to start the robot's autonomous operation:
```
python3 main.py
```

---

## Flowchart

![Daise Flowchart 3](https://github.com/user-attachments/assets/3593fe45-a591-4f58-8656-42ac5e367650)

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


---

## Video Demo
To see the D.A.I.S.E. robot in action, check out our video demonstration:

https://sites.google.com/view/cse-598-group-12/home

---

## Improvements or Future Work
The D.A.I.S.E. project represents a successful prototype of a low-cost autonomous delivery robot. However, several improvements and future developments can enhance its performance and capabilities:

- **Buck Converter**: In future iterations, I will use the DFrobot Buck converter, known for its accuracy and reliability.
- **Better Motors**: Upgrading to higher quality motors will improve the robot's movement precision and durability.
- **Depth Camera or Stereo Camera**: Incorporating a depth camera or a stereo camera system will enhance the robot's obstacle detection and path planning abilities.
- **RTK GPS Module**: The current GPS module lacks the necessary accuracy for long-distance navigation. An RTK GPS module will significantly improve the robot's positional accuracy.
- **Deep Learning or VSLAM**: Integrating deep learning techniques or Visual Simultaneous Localization and Mapping (VSLAM) will enable more advanced navigation and environment understanding.
- **Custom PCB**: Developing a custom PCB will streamline the electronic setup, reduce wiring complexity, and improve reliability.
- **Data Sharing System**: Enhancing the data sharing system between the robot and the main computer is crucial. Implementing WebRTC or WebSocket for data transmission and developing a web interface for data 
     visualization and control using WiFi will facilitate better communication and control.
- **Delivery System and Payload**: To make the robot suitable for real-world delivery applications, adding a payload carrying system and developing a customer-oriented delivery system will be essential.
- **Improved Battery Management**: Implementing a more efficient battery management system will extend the operational time of the robot.
- **Enhanced Software Architecture**: Refactoring the software to make it more modular and scalable will facilitate easier maintenance and future upgrades.

---

## Acknowledgment
This project was undertaken as part of the course CSE 598: Perception in Robotics during Spring 2024.

**Professor:**
Dr. Nakul Gopalan [(Website)](https://nakulgopalan.github.io/)

**Team Members:**
- Aryan Kumar Nadipally (Me) [(Linkedin)](https://www.linkedin.com/in/aryan-kumar-nadipally/)
- Sai Shivani Lingampally [(Linkedin)](https://www.linkedin.com/in/sai-shivani-lingampally-b00986206/)
- Anish Sai Racha [(Linkedin)](https://in.linkedin.com/in/anish-sai-racha-190ba51b7)
- Jhanvi Shailesh Shah [(Linkedin)](https://www.linkedin.com/in/jhanvi2001/)

I would like to express my gratitude to Dr. Nakul Gopalan for his guidance and support throughout the project. Special thanks to my teammates Sai Shivani Lingampally, Anish Sai Racha, and Jhanvi Shailesh Shah for their collaboration and hard work.

Thank you to everyone who contributed to the success of this project.

---

