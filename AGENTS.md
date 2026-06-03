# AGENTS.md - Project Overview & Harness Implementation

This project is a simulation environment for the **Bosch Physics-Driven Real-Time CPS Challenge (RTAS 2026)**. It focuses on the co-design of real-time scheduling and control performance for a lateral motion control system of a vehicle.

## 🏎 Project Architecture

The system is split into two primary layers:
1.  **Functional Layer (FMU):** A Functional Mockup Unit (`LateralMotionControl.so`) that encapsulates the vehicle physics (single-track model) and the control logic (feedback/feedforward controllers).
2.  **Timing Layer (Harness):** A C++ application (`fmu_harness`) that acts as the FMI Master. It manages the simulation clock and triggers specific task-chain events (Sensor, Estimator, Controller, etc.) based on defined periods.

### The Task Chain
The controller functionality is implemented as a Directed Acyclic Graph (DAG) of tasks:
- **Vehicle Edge:** Sensor, Actuator.
- **Cloud Platform:** Estimator, Feedforward, Controller, Merger.
- **Network:** Modeled as communication blocks (`Nin`, `Nout`) with potential delays.

## 🛠 Harness Implementation

The `fmu_harness` implementation is a high-performance FMI 2.0 Co-Simulation wrapper designed for research and testing.

### Key Features:
- **Dynamic FMU Loading:** Uses `dlopen` to load the FMU shared library at runtime.
- **Trigger-Based Scheduling:** Instead of simple periodic stepping, the harness explicitly manages the `activated` and `finished` triggers for every task in the control chain, allowing for the simulation of complex real-time effects like jitter and preemptive scheduling.
- **Data Integration:** Dynamically loads track geometry, feedforward references, and velocity profiles from CSV datasets (e.g., `examples/example_v_10`).
- **Path Calculation:** Implements path heading estimation and coordinate transformation to map the FMU's internal lateral error (`e_y`) back to global $(X, Y)$ coordinates for visualization.

### Loop Execution Logic:
1.  **Update Inputs:** Read current reference and velocity from datasets and set them in the FMU.
2.  **Schedule Triggers:** Determine which tasks are active based on the current simulation time.
3.  **Step FMU:** Call `fmi2DoStep` to advance the physics and process triggered task logic.
4.  **Extract State:** Retrieve the physical state and performance metrics.
5.  **Output Log:** Generate a CSV formatted log for the visualizer.

## 📊 Performance & Visualization
A companion SFML visualizer (`track_view`) is provided to animate the simulation results, showing the vehicle's deviation from the track and the real-time activity of the task chain.
