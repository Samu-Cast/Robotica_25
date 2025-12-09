Design Document – ISRLAB Virtual Robotics Project

1. General Project Description (Abstract)

This project introduces Charlie, an autonomous mobile robot designed to operate inside hazardous indoor environments affected by accidental toxic substance release. The scenario simulates a chemical laboratory where a contaminant leak has compromised air quality and visibility, making human intervention dangerous.
Charlie’s primary mission is to reach and activate the emergency ventilation system, located in one of several possible locations inside the building. To accomplish this, the robot autonomously explores the environment, navigates toward predefined candidate points, and reacts to unexpected obstacles along the way.
During its mission, Charlie identifies chemical debris, laboratory containers, and potential victims, using onboard perception based on object detection. The robot reports detected victims to rescue teams and ignores or bypasses non-hazardous obstacles. Its operation enhances safety by avoiding the need for personnel to enter contaminated zones.

2. Robot Model Description

2.1 Robot Model

Charlie is based on a compact mobile rover platform selected from the Gazebo Harmonic standard model library. The chosen platform ensures compatibility with default sensors, differential-drive control, and ROS 2 navigation. Its chassis is optimized for indoor exploration and cluttered laboratory layouts.

2.2 Sensors Set

Charlie integrates a multimodal perception suite suited for chemical emergency scenarios:

RGB Camera – used for real-time object detection via YOLO (victims, buttons).

Ultrasonic Sensors – assist in mapping, localization, and obstacle avoidance.

2.3 Actuators Set

Charlie is equipped with:

Differential drive motors, enabling precise motion in tight interior spaces.

Optional micro-arm or virtual activation module, used only if physical interaction with the ventilation switch is required.
In simulations where physical contact is unnecessary, reaching a designated activation zone is sufficient to trigger the system.

2.4 Body Shape

Charlie is designed as a compact, lightweight rover with a circular base. Its low-height frame allows movement under partially collapsed structures, while its slim profile ensures maneuverability in tight indoor passageways. The robot's shape is optimized for stability, mobility, and safe operation in hazardous environments.

2.5 Capabilities & Degrees of Freedom

Charlie features the following capabilities:

Autonomous navigation in cluttered indoor environments.

Real‑time obstacle avoidance.

Victim detection through visual sensing.

Communication of detected victims to rescuers.

Optional physical actuation if equipped with a micro‑arm.

Degrees of Freedom (DOF):

2 DOF for differential drive (forward/backward and rotation).

+1 DOF if an optional activation arm is installed.

3. Simulator Environment Choice

The simulation is performed inside a virtual chemical laboratory affected by a toxic leak, modeled in Gazebo Harmonic. The environment includes:

- Interconnected corridors and rooms
- Broken chemical vials, containers, and scattered debris placed as navigation obstacles.
- Potential human-like targets to simulate victims.
- Dedicated activation zones where the emergency ventilation system may be located.

Gazebo Harmonic is paired with ROS 2 (Jazzy) to control the robot, manage sensor data, handle navigation (Nav2), and run the perception stack (YOLO).

4. Robot Goal Definition

4.1 Main Goal

Charlie’s primary objective is to navigate autonomously through the contaminated laboratory and reach the emergency ventilation control to activate the extraction system.

4.2 Sub-goals

- Detect and avoid chemical debris (broken vials, containers, equipment).

- Detect and report human presence, forwarding victim information to rescuers.

- Safely explore the environment, moving through predefined candidate locations until the correct activation point is found.

- Maintain safe operation in a hazardous environment without human assistance.

5. Design Methodology

5.1 Robot Agent Architecture (AI-FCA – Hierarchical Model)

Charlie’s control system is structured according to the Hierarchical Architecture described in Chapter 2 of the AI-FCA textbook. The agent is organized into three coordinated layers, each responsible for a different level of abstraction in perception, decision-making, and action execution.

![alt text](<architecture/overral architecture.drawio.png>)

Reactive Layer (Low Level)

Manages real‑time, safety‑critical behaviors:

Obstacle avoidance using proximity and depth sensors.

Immediate reaction to detected fire hotspots or dangerous heat zones.

Basic motion control for differential drive.

Deliberative Layer (High Level)

Responsible for planning and reasoning:

Environment mapping (topological or grid‑based).

Global path planning toward the fire‑suppression activation point.

High‑level goal management.

Executive / Coordination Layer (Middle Level)

Coordinates actions between planning and execution:

Converts plans into sequences of behaviors.

Supervises task execution (exploration, detection, navigation).

Error recovery in case of blocked path or inconsistent sensor data.

5.2 Layer Interactions

Bottom‑up flow: sensor data → reactive responses → structured information for planning.

Top‑down flow: high‑level goals → task decomposition → motor commands.

Continuous feedback loop for robust navigation in hazardous environments.

System Design Approach

Modular decomposition into sensing, perception, mapping, navigation, decision‑making, actuation, and communication modules.

Iterative workflow: specification → prototyping → testing → refinement.

Risk‑aware design prioritizing safety and robustness.

Clear separation of concerns to simplify debugging and testing.

Development Workflow

Initial prototyping of sensors and basic movement.

Implementation of perception pipelines (fire detection, victim detection).

Development of navigation and path‑planning algorithms.

Integration through the executive layer.

Final optimization and stress testing.

Design Criteria

Reliability: the robot must operate under uncertainty and noisy sensor data.

Robustness: ability to adapt to smoke, obstacles, dynamic hazards.

Modularity: each subsystem can be updated or replaced independently.

Scalability: architecture supports additional sensors or behaviors.

Real‑time reactivity: low‑latency responses to sudden hazards.

6. Testing protocol. Testing protocol

come testate sensori

come testate navigazione

come testate rilevamento fuoco/vittime

casi di test principali

simulazioni ripetute

7. Experimental results

Da inserire alla fine:

calibrazioni

grafici

statistiche

video

log

8. UML Diagrams

The following UML diagrams illustrate the system design, agent behaviors, and interactions between modules:

8.1 Use Case Diagram

Illustrates the main functionalities of Charlie, including navigation, hazard detection, victim reporting, and fire suppression activation.

8.2 Activity Diagram

Shows the step-by-step activities during a mission, from environment exploration to activation of the fire suppression system.

8.3 State Machine Diagram

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
