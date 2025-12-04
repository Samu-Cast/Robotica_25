Design Document – ISRLAB Virtual Robotics Project

1. General Project Description (Abstract)

This project introduces Charlie, an autonomous mobile robot designed to operate in hazardous indoor environments during emergency situations such as building fires. Charlie's primary mission is to autonomously explore the environment, locate the fire‑suppression activation point (lever or button), and enable the building’s safety system. During its exploration, Charlie detects and reports critical hazards such as fire sources and potential victims using its onboard perception system. The robot communicates these findings to rescuers to support safe and efficient intervention. Its operation focuses on navigating confined and smoke‑filled spaces, avoiding obstacles, identifying dangers, and reaching the activation area without requiring human presence in unsafe zones.

2. Robot Model Description

2.1 Robot Model

Charlie is based on a four-wheeled mobile rover platform equipped with a low-profile chassis suitable for navigating narrow corridors and cluttered indoor environments. The model is selected from the simulator’s standard robot library to ensure compatibility with built‑in sensors and actuators.

2.2 Sensors Set

Charlie integrates a multimodal sensing suite designed for emergency environment perception:

RGB Camera – captures visual information for obstacle and fire detection.

Depth Sensor / RGB-D Camera – provides 3D structure of the environment for mapping and navigation.

Thermal Sensor – detects hotspots and active fire sources.

Proximity / Lidar Sensor – assists with obstacle avoidance and safe path planning.

Environmental Temperature Sensor – monitors ambient heat to assess dangerous areas.

These sensors enable robust perception even under low visibility due to smoke or structural damage.

2.3 Actuators Set

Charlie uses the following actuators:

Differential drive motors for movement and steering.

Optional micro‑arm actuator if physical interaction with the fire‑suppression lever is required.

Depending on the simulation setup, activation of the fire‑suppression system may be achieved either through physical interaction or by reaching a designated activation zone.

2.4 Body Shape

Charlie is designed as a compact, lightweight rover with a rectangular base and four wheels. Its low-height frame allows movement under partially collapsed structures, while its slim profile ensures maneuverability in tight indoor passageways. The robot's shape is optimized for stability, mobility, and safe operation in hazardous environments.

2.5 Capabilities & Degrees of Freedom

Charlie features the following capabilities:

Autonomous navigation in cluttered indoor environments.

Real‑time mapping and obstacle avoidance.

Fire and victim detection through multimodal sensing.

Communication of detected hazards to rescuers.

Optional physical actuation if equipped with a micro‑arm.

Degrees of Freedom (DOF):

2 DOF for differential drive (left/right wheels).

+1 DOF if an optional activation arm is installed.

3. Simulator Environment Choice

The simulation takes place in a virtual indoor emergency scenario, modeled as a building affected by a localized or widespread fire. The environment includes:

Corridors, rooms, and obstacles.

Fire hotspots of varying intensity.

Areas with reduced visibility.

Potential human-like targets to simulate victims.

The chosen environment supports realistic sensing, navigation, and hazard detection tasks.

4. Robot Goal Definition

4.1 Main Goal

Charlie’s primary objective is to reach and activate the building’s fire‑suppression system by navigating autonomously through hazardous indoor environments.

4.2 Sub-goals

Detect and report fire hotspots.

Detect and report human presence.

Avoid obstacles, debris, and dangerous heat zones.

Explore the environment efficiently and safely.

5. Design Methodology

5.1 Robot Agent Architecture (AI-FCA – Hierarchical Model)

Charlie’s control system is structured according to the Hierarchical Architecture described in Chapter 2 of the AI-FCA textbook. The agent is organized into three coordinated layers, each responsible for a different level of abstraction in perception, decision-making, and action execution.

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
