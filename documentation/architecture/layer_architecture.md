```mermaid
graph TD
    %% High Level Layer
    subgraph HIGH_LEVEL["HIGH LEVEL: DECISION (Main CPU)"]
        direction TB
        BT[("Behavior Tree<br/>Executor")]
        BB[("Blackboard<br/>(Knowledge Base)")]
        TM["Task Manager"]
        
        BT <--> BB
        TM --> BB
    end

    %% Mid Level Layer
    subgraph MID_LEVEL["MID LEVEL: NAVIGATION & PERCEPTION (PC/GPU)"]
        direction TB
        GP["Global Planner<br/>(A*)"]
        LP["Local Planner<br/>(DWA/TEB)"]
        CM["Costmap<br/>(Global & Local)"]
        VIS["Object Recognition AI<br/>(YOLO/SSD)"]
        
        GP -- Path --> LP
        CM --> GP
        CM --> LP
        VIS -- "Persona/Valvola" --> BB
        VIS -- "Detrito/Ostacolo" --> CM
    end

    %% Low Level Layer
    subgraph LOW_LEVEL["LOW LEVEL: HARDWARE (Microcontroller)"]
        direction TB
        MC["Motor Controller<br/>(PID)"]
        ODOM["Odometry Calc"]
        DRV["Sensor Drivers"]
        MOT["Motori Fisici"]
        SENS["Sensori (Lidar/Cam)"]
        
        MC -- PWM --> MOT
        SENS --> DRV
        DRV -- Raw Data --> VIS
        
        %% Connessioni Encoder
        MOT -.-> ODOM
        ODOM -- Pose Estimate --> CM
        ODOM -- Pose Estimate --> BT
    end

    %% Connessioni tra Layer principali
    BT -- "Goal Pose (x,y)" --> GP
    LP -- "cmd_vel (v, w)" --> MC
    DRV -- Raw Data --> CM

    %% Stile (Opzionale per renderlo più leggibile)
    classDef high fill:#f9f,stroke:#333,stroke-width:2px;
    classDef mid fill:#bbf,stroke:#333,stroke-width:2px;
    classDef low fill:#bfb,stroke:#333,stroke-width:2px;
    
    class BT,BB,TM high;
    class GP,LP,CM,VIS mid;
    class MC,ODOM,DRV low;
    ```