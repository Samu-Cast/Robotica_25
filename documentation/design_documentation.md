Design Document – ISRLAB Virtual Robotics Project

# 1. General Project Description

This project introduces Charlie, an autonomous mobile robot designed to operate inside hazardous indoor environments affected by accidental toxic substance release. The scenario simulates a chemical laboratory where a contaminant leak has compromised air quality and visibility, making human intervention dangerous. 

Charlie’s primary mission is to reach and simulate the activation of the emergency ventilation system (represented with a red target), located in one of several possible locations inside the building. To accomplish this, the robot autonomously explores the environment, navigates toward predefined candidate points, and reacts to unexpected obstacles along the way.

During its mission, Charlie identifies chemical debris, laboratory containers, and potential victims, using onboard perception based on object detection. The robot reports detected victims to rescue teams and ignores or bypasses non-hazardous obstacles. Its operation enhances safety by avoiding the need for personnel to enter contaminated zones.

# 2. Simulated Robot model technical description

## 2.1 Robot Model

The robotic platform selected for this simulated mission is the **iRobot Create 3**, a widely standardized mobile robot in academic research for autonomous navigation. This platform is characterized by a **differential drive kinematics configuration**, which imposes non-holonomic constraints on the robot's motion—specifically, the inability to move instantaneously in the lateral direction. Despite this constraint, the differential drive system allows for a **zero turning radius**, granting the robot exceptional maneuverability in tight, clutter-filled environments such as the simulated hazardous laboratory. The robot features a compact, circular footprint with compact dimensions, ensuring minimal inertial impact during dynamic maneuvers. Its native integration with **ROS 2** (Robot Operating System) provides a robust middleware layer for high-level control, facilitating the implementation of complex navigation and perception stacks.

## 2.2 Sensors Set

The robot's perception system is architected around a multi-modal sensor suite, categorized into **Environmental** and **Internal** sensory channels to ensure robust environmental awareness and precise state estimation.

### Environmental Perception (External Sensing)
Primary semantic understanding is driven by a front-facing **RGB Camera**, which feeds visual data into a **YOLO-based neural network** for the detection of critical entities such as victims and hazard source control mechanisms. Complementing the visual system is an array of **three ultrasonic sensors** (front, front-left, front-right). This configuration enables a multi-layered obstacle avoidance strategy, providing mid-range spatial awareness for maneuvering and wall-following behaviors.

### Internal State Monitoring (Proprioception)
For localization and odometry, the platform relies on an **Inertial Measurement Unit (IMU)** and high-resolution **wheel encoders**. The fusion of inertial data (linear acceleration and angular velocity) with wheel odometry allows for accurate dead-reckoning navigation, which is indispensable for maintaining an estimate of the robot's pose map-frame.

## 2.3 Actuators Set

Actuation is governed by two independent DC motors driving the main wheels, implementing a classic differential drive topology. Control is achieved by varying the velocity difference between the left and right wheels; synchronous rotation produces linear motion, while opposing rotation generates in-place angular displacement. This kinematic arrangement is particularly advantageous for the project's operational domain, allowing the agent to perform re-orientation maneuvers without translational displacement—a critical capability when navigating narrow corridors obstructed by debris.

## 2.4 Body Shape °°°°°inserire foto°°°°°

The morphological design of the robot features a **cylindrical chassis** with a low vertical profile. This axially symmetric shape offers significant algorithmic advantages for path planning and collision checking. Since the robot's collision footprint is invariant under rotation, the motion planner can treat the robot as a simple circle in the configuration space, drastically simplifying the computational complexity of obstacle avoidance and path generation. Furthermore, the absence of protruding corners significantly reduces the risk of entanglement with environmental clutter, enhancing the robust execution of autonomous exploration tasks.

## 2.5 Detail Analysis: Localization and State Estimation

**Localization Strategy:**
The robot's positioning relies on **Odometry**, integrating kinematic data from wheel encoders to estimate displacement. The system architecture includes an **odometry correction mechanism** within the Behavior Tree's Blackboard (`odom_correction`), designed to offset accumulative drift by anchoring the robot's pose to known semantic landmarks. In the current implementation, the robot primarily trusts the short-term accuracy of the encoders and IMU (Inertial Measurement Unit) heading for local maneuvering.

**Simulation-Realism Gap (Latency & Position Uncertainty):**
A critical design constraint in this simulated environment is the **asynchronous latency** between the ROS 2 control loop and the Gazebo engine. This communication delay introduces a temporal offset between the *perceived* position (sensor data) and the *actual* state of the robot. Consequently, the robot's position is never absolute but rather a probabilistic estimate. The navigation logic incorporates tolerance thresholds to robustly handle this inherent uncertainty, prioritizing safe navigation over pixel-perfect positioning.

## 2.6 Capabilities & Degrees of Freedom (DOF)

The system displays a high degree of autonomy, integrating robust indoor navigation with real-time semantic analysis. Its core capabilities include **autonomous exploration** of unknown environments, **proactive obstacle avoidance** using fused sensor data, and **semantic detection** of victims via deep learning models. These behaviors are orchestrated by the Executive Layer, which dynamically switches between exploration and goal-seeking modes based on mission status.

In terms of kinematics, the platform operates with **2 Active Degrees of Freedom (DOF)**. While the robot exists in a 3D world, its motion is constrained to the 2D ground plane, controlled via longitudinal translation and angular rotation. Efficient path planning algorithms leverage these DOFs to generate smooth trajectories.

# 3. Simulator Environment Choice

The simulation is performed inside a virtual chemical laboratory affected by a toxic leak, modeled in Gazebo Harmonic. The environment includes:

- A single room with some obstacles and human-like targets to simulate victims.
- Broken chemical vials, containers, and scattered debris placed as navigation obstacles.
- Dedicated control zones where the emergency ventilation system may be located.

Gazebo Harmonic is paired with ROS 2 (Jazzy) to control the robot, manage sensor data, execute custom Behavior Tree-based navigation, and run the perception stack (YOLO).

# 4. Robot Goal Definition

The **primary operational objective** is to deploy an autonomous robotic agent capable of navigating a contaminated laboratory environment to identify and reach the emergency ventilation control unit, thereby imulating the activation of the extraction system.

## 4.1 Main Goal

Charlie’s core mission is to autonomously traverse the hazardous operational domain, avoiding obstacles and entangled structures, to **localize and reach** the specific activation interface for the ventilation system.

## 4.2 Sub-goals

To achieve the main objective, the system must satisfy the following functional sub-goals:

- **Hazard Avoidance:** reliably detect and bypass chemical debris, broken vials, and static obstacles to ensure navigation safety.

- **Victim Identification:** detect human presence using semantic perception and forward critical location data to rescue teams.

- **Systematic Exploration:** execute a sequential inspection of predefined candidate locations (Knowledge-Based Search) until the correct activation point is identified.

- **Operational Autonomy:** maintain system integrity and decision-making capabilities in a hazardous environment without direct human teleoperation.

# 5. Design Methodology


# 6. Testing Protocol

The system verification strategy is divided into three distinct layers to ensure robust performance across logic, perception, and actuation.

## 6.1 Unit Testing (Decision Logic & Sensor Math)
The robot's decision-making core (Plan Module) is validated through **Automated Unit Tests** using a dedicated test suite (`test_behaviors.py`). This suite mocks the sensor inputs (Blackboard variables) and verifies that the Behavior Tree transitions to the correct state.

Additionally, the **Perception Logic** is tested via:
- `test_sense_node.py`: Validates geometric helper functions used for detection processing:
  - **Bounding Box Area Calculation**: Tests both `xywh` (standard) and `xyxy` (YOLO output) formats
  - **Center Point Extraction**: Computes detection center for zone classification
  - **Horizontal Zone Detection**: Classifies detections as `left`, `center`, or `right` based on image thirds
  - **Distance Estimation**: Validates the inverse-square-root model used to estimate object distance from bbox area (`sqrt(SCALE/area)`)
  - **Ultrasonic Zone Mapping**: Verifies correct sensor selection based on detection zone (`left` → `front_left`, `center` → `front`, `right` → `front_right`)
  - **Min Area Filtering**: Tests the 2000px² threshold used to filter distant/noisy detections
- `test_color_detector.py`: A specific **"Smart Test"** suite that uses synthetic images (e.g., pure green squares) to verify the HSV thresholds and color segmentation logic. 

**Note on Human Detection**: We deliberately exclude Unit Tests for the YOLO model (`human_detector.py`) as it relies on a pre-trained neural network. Validating the network's internal weights via unit tests is redundant; instead, its performance is verified during **System Integration** (Section 6.3) and visual debugging.

## 6.2 Perception & Sensor Validation
Perception modules are tested via **Visual Debugging**:
- **Sensors:** Ultrasonic readings are cross-referenced with Gazebo ground-truth measurements to verify linearity and range configuration.
- **YOLO/Camera:** The detection pipeline is validated by monitoring the specific debug topic `/sense/debug_image`. This topic provides an **annotated video stream** where the `SenseNode` actively overlays Bounding Boxes and Confidence Scores onto the raw camera feed. Operators use tools like `rqt_image_view` to visually confirm that victims and hazards are correctly classified before the data reaches the planning layer.

## 6.3 System Integration & Navigation
The final validation phase involves **Full Mission Simulations** in the Gazebo environment:
- **Navigation Calibration:** Empirical tests are conducted to measure and minimize odometry drift during rotation-heavy maneuvers.
- **End-to-End Scenarios:** The robot is deployed in a defined starting position to verify its ability to autonomously complete the mission (find target -> simulate activation) without human intervention.


# 7. Experimental results

Da inserire alla fine:

calibrazioni

grafici

statistiche

video

log

# 8. UML Diagrams

The following UML diagrams illustrate the system design, agent behaviors, and interactions between modules:

## 8.1 Use Case Diagram

Illustrates the main functionalities of Charlie, including navigation, hazard detection, victim reporting, and simulated fire suppression triggering.

## 8.2 Activity Diagram

Shows the step-by-step activities during a mission, from environment exploration to the simulated activation of the fire suppression system.

## 8.3 State Machine Diagram

Represents the internal states of Charlie, including Idle, Exploring, Detecting, Navigating, Reporting, and Actuating.

8.4 Architecture Diagram

Visualizes the hierarchical control layers: Reactive, Executive, Deliberative, and their interactions with sensors, actuators, and the communication module.

Diagrams to be created in the appropriate UML tool; placeholders are present in this document for illustration.

Use case UML

Activity diagram

State machine del robot

Architecture diagram

9. Non-functional Requirements

Charlie’s software and hardware system must comply with the following non-functional requirements to ensure robustness, reliability, and maintainability:

All project files must be stored in the private MS Teams group channel.

The system must support Dockerized multi-process deployment.

A message broker system must be adopted to manage inter-process communication and ensure asynchronous, reliable messaging.

A centralized logging system must be implemented to record sensor readings, commands, and system events.

A GUI must allow real-time monitoring of sensor data and robot perceptions (simulated time is acceptable).

A shared software repository must be used to coordinate development and version control.

A ReadMe.md file must provide installation instructions and testing procedures.

10. Final Presentation

The final presentation should include slides and videos lasting approximately 15 minutes, followed by a Q&A session.

Every group member must describe their role and contributions during the Q&A session.
