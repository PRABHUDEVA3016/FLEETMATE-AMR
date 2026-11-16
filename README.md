# FLEETMATE AMR
### Industrial AMR Smart Delivery Warehouse Simulation with Centralized Fleet Management

FLEETMATE AMR is an autonomous mobile robot (AMR) warehouse simulation designed to demonstrate intelligent material handling, multi-robot coordination, autonomous navigation, and centralized fleet management in an industrial warehouse environment.

The project features two autonomous mobile robots performing independent warehouse material-handling missions while sharing the same operating environment.

## Project Overview

The system simulates an industrial warehouse where two AMRs perform coordinated material-handling operations:

- **AMR-01:** Collects packages from six pickup points and transports them to the loading zone.
- **AMR-02:** Collects boxes from the unloading zone and delivers them to six warehouse racks.

Both robots execute their missions autonomously while monitoring their operating status and maintaining safe separation.

## Key Features

### 1. Autonomous Navigation
- A* pathfinding algorithm.
- Grid-based warehouse navigation.
- Automatic route planning and replanning.
- Obstacle detection and avoidance.

### 2. Multi-AMR Coordination
- Two autonomous robots operating simultaneously.
- Independent mission execution.
- Robot priority and route-conflict handling.
- Dynamic safety-distance monitoring.

### 3. Warehouse Simulation
- Industrial warehouse layout.
- Six warehouse racks.
- Loading and unloading zones.
- Moving forklift obstacle.
- Package pickup and delivery operations.

### 4. Battery Management
- Individual battery monitoring.
- Battery consumption during movement.
- Automatic charging station management.
- Separate docking bays for both AMRs.

### 5. Centralized Fleet Management
- Live robot status monitoring.
- Task allocation and completion tracking.
- Individual battery status.
- Real-time robot coordinates.
- Current task monitoring.
- Emergency and failure alerts.

### 6. Operator Controls
- Autonomous mission start.
- Simulation reset.
- Centralized fleet dashboard.
- Manual navigation controls.

## Technology Stack

| Technology | Purpose |
|---|---|
| Python | Core application development |
| Pygame | Warehouse simulation and visualization |
| A* Algorithm | Autonomous path planning |
| Git | Version control |
| GitHub | Source code hosting |

## Project Architecture

The simulation consists of the following main components:

1. Warehouse Environment
2. AMR-01 Autonomous Navigation
3. AMR-02 Autonomous Navigation
4. Path Planning and Obstacle Avoidance
5. Battery and Charging Management
6. Multi-Robot Safety Monitoring
7. Centralized Fleet Management Dashboard

## Installation and Execution

### Prerequisites
- Python 3
- Git
- Pygame

### 1. Clone the repository

```bash
git clone https://github.com/PRABHUDEVA3016/FLEETMATE-AMR.git
```

### 2. Open the project directory

```bash
cd FLEETMATE-AMR
```

### 3. Create a virtual environment

```bash
python -m venv .venv
```

### 4. Activate the virtual environment

Windows:

```bash
.venv\Scripts\activate
```

Git Bash:

```bash
source .venv/Scripts/activate
```

### 5. Install dependencies

```bash
python -m pip install -r requirements.txt
```

### 6. Run the simulation

```bash
python main.py
```

## Simulation Controls

| Key | Function |
|---|---|
| T | Start both AMR missions |
| F | Open or close fleet management dashboard |
| R | Reset simulation |
| ESC | Exit simulation |

Additional manual navigation and obstacle-placement controls are available in the simulation.

## Project Objective

The objective of FLEETMATE AMR is to demonstrate how autonomous mobile robots can improve industrial warehouse operations through intelligent navigation, coordinated material handling, battery management, safety monitoring, and centralized fleet supervision.

## Project Information

**Project:** FLEETMATE AMR  
**Category:** Industrial Robotics / Autonomous Mobile Robots  
**Domain:** Robotics and Artificial Intelligence  
**Platform:** Python-based Warehouse Simulation

## Repository

GitHub: https://github.com/PRABHUDEVA3016/FLEETMATE-AMR

---

Developed as an Industrial AMR Smart Delivery Warehouse Simulation project.
