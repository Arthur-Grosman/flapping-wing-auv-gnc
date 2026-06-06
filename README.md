# Bio-Inspired Underwater Vehicle GNC Simulator
This repository contains a modular, control-oriented Python simulation framework designed to model, test, and validate Guidance, Navigation, and Control (GNC) architectures for a flapping-wing biomimetic Autonomous Underwater Vehicle (AUV).

Rather than relying on computationally heavy fluid dynamics, this codebase provides a lightweight numerical testing environment. It acts as a software-in-the-loop simulator to evaluate how closed-loop control algorithms and reactive path-planning logic handle high-clutter, narrow-passage marine environments like aquaculture pens and dense kelp forests.

## Core Capabilities of the Simulator

This software package is built to evaluate the unique physics of flapping-wing aquatic propulsion using classical control theory. It simulates a complete autonomous mission lifecycle:
- **Dynamic Physics Emulation:** Models a 4-DOF system tracking forward acceleration under quadratic fluid drag, state kinematics, and vertical heave dynamics.
- **Closed-Loop Depth Tracking:** Employs a parallel PID controller to handle automated diving profiles and vertical stabilization.
- **Reactive Spatial Navigation:** Combines Line-of-Sight (LOS) target pursuit with real-time Artificial Potential Field (APF) calculations to safely deflect the asset away from mapped hazards.
- **Automated Mission Lifecycle:** Handles multi-point waypoint sequencing and automates a controlled "coasting recovery" shutdown phase once the final objective coordinate is achieved.

## Software Architecture

`config.yaml` Centralized physics, control, and mission parameters

`main.py` Mission orchestrator & numerical simulation loop

`src/environment.py` Spatial grid generator (obstacle and waypoint arrays)

`src/dynamics.py` 4-DOF state-space equations of motion

`src/controls.py` PID depth stabilization & surge velocity mapping

`src/guidance.py` Path planning & APF collision-avoidance vector math

`src/visualization.py` Dual-mode telemetry exporter & matplotlib animator

## Mathematical & Algorithmic Framework

### 1. Control-Oriented Vehicle Dynamics (`dynamics.py`)

The continuous system state vector integrated over time is defined as:

$$\mathbf{x} = [x, y, \theta, v, z, \dot{z}]^T$$

Where $(x,y)$ is horizontal position, $\theta$ is heading, $v$ is forward surge velocity, $z$ is depth, and $\dot{z}$ is vertical velocity.

- **Thrust Mapping:** Thrust generation is modeled as a function of wing flapping frequency $f$:
$$F_{\text{thrust}} = k_f \cdot f^2$$

- **Fluid Resistance:** Quadratic hydrodynamic drag bounds the forward terminal velocity:
$$F_{\text{drag}} = C_d * v * |v|$$

- **Surge Acceleration:** Governed by Newton's second law:
$$\dot{v} = \frac{1}{m} (F_{\text{thrust}} - F_{\text{drag}})$$

### 2. Guidance & Obstacle Avoidance (`guidance.py`)

The vehicle dynamically alters its trajectory using an Artificial Potential Field (APF) framework:

- **Attractive Vector:** Generates a unit vector pulling the vehicle directly toward the active waypoint.

- **Repulsive Vector:** Generates an exponentially scaling outward force vector from obstacles whenever the vehicle steps inside an influence boundary $\rho_0$:
$$U_{\text{rep}} = \frac{1}{2} \eta \left( \frac{1}{\text{dist}} - \frac{1}{\rho_0} \right)^2$$
The guidance engine takes the resultant vector $V_{\text{res}} = V_{\text{att}} + V_{\text{rep}}$ to solve for the target instantaneous heading $\theta_{\text{des}}$.

### 3. Control Loops (`controls.py`)

- **Depth Loop:** A parallel Proportional-Integral-Derivative (PID) controller minimizes vertical position error, executing depth transitions and counteracting buoyancy by commanding vertical force $u_z$.

- **Surge Loop:** A Proportional + Feedforward (P+FF) velocity controller. It utilizes a feedback loop on velocity error combined with a hydrodynamic drag feedforward term to determine the required thrust demand, which is then mapped directly to the wing-flapping frequency $f$.

- **Heading Loop:** A proportional tracker translates steering error into an immediate command rate-of-turn $u_\omega$.

## Expected Output Deliverables

Upon completion of a simulation run, the framework generates two analytical assets:

### 1. Telemetry Dashboard (`telemetry_dashboard.png`)

A 4-panel diagnostic plot proving the structural health of the GNC loops:

- **Depth Tracking:** Captures the initial dive profile from the surface, showing zero steady-state error settling at the target depth.

- **Forward Velocity:** Illustrates acceleration characteristics and stabilization around the cruise speed target.

- **Flapping Frequency:** Logs the real-time frequency demands (Hz) placed on the actuators.

- **Control Inputs:** Records the raw controller outputs ($u_z$ and $u_\omega$), highlighting precise reactive spikes directly adjacent to obstacles.

### 2. Mission Playback (`trajectory_animation.mp4` / `.gif`)

A top-down 2D animation tracking the vehicle's path through space. It visualizes the vehicle tracking waypoints (red crosshairs), deflecting around hazardous obstacle zones (green circles) via APF repulsion, and coasting to a complete stop as the engine shutdown logic triggers upon final mission completion.

## Prerequisites

Ensure you have a clean Python environment and the required packages installed:
```
pip install numpy matplotlib pyyaml
```
Note: Exporting the `.mp4` video format requires `ffmpeg` installed on your local system path. If it is missing, the visualization engine will automatically default to exporting an animated `.gif` using the `pillow` library.

## Running the Simulation

Execute the primary orchestration script from your terminal:
```
python main.py
```

## Configuration and Tuning

All physical properties and controller gains are exposed inside `config.yaml`. To adjust the vehicle's mass, modify PID aggressiveness, or change target velocities, update the YAML declarations without rewriting any core source code:
```
vehicle:
  mass: 5.0             # Vehicle mass in kg
  max_f: 5.0            # Actuator flapping limit in Hz
control:
  v_target: 1.5         # Target cruise velocity (m/s)
  z_target: -5.0        # Target operating depth (m)
```

## Example Simulation Output

![Telemetry Dashboard](assets/telemetry_dashboard.png)

![Mission Animation](assets/trajectory_animation.gif)
