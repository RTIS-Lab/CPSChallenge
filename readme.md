# Lateral Motion Control FMU

## 1. Overview

This Functional Mock-up Unit (FMU) implements a *LateralMotionControl* system for a vehicle. It simulates the vehicle's lateral dynamics using a discrete-time, velocity-dependent state-space model. The FMU encapsulates a (simplified) complete control loop, including sensing, estimation, control, and actuation.

This code complements the paper: **Invited Paper: Physics-Driven Real-Time CPS Challenge**, Paolo Pazzaglia, Kevin Schmidt, Laura Beermann, Dirk Ziegenbein and Arne Hamann. *32nd IEEE Real-Time and Embedded Technology and Applications Symposium (RTAS 2026)*, 2026

This self-contained model for co-simulation is provided as a realistic and challenging use-case, to test the interaction of timing effects driven by scheduling and communication, together with physical performance metrics.

The control system architecture allows for flexible triggering of task instances. The functional communication of input/output data between tasks (e.g., sensor, controller, actuator) is driven by incoming boolean trigger signals for both reading (with execution of the task) and writing. The boolean triggers need to be provided as input to the FMU from, e.g., an external scheduling simulator.
This allows for flexible and realistic simulation of distributed real-time systems, where the scheduling policy of the tasks on different ECUs and/or in the cloud can be flexibly adapted at runtime.

A key feature of the FMU is the triplicate calculation of performance metrics: one set is calculated "in local platform" (with access to the network-delayed physical state and communicated feedforward reference), another is calculated "in remote platform" (based on estimated states from sensor data), and an ideal "real" set (using the ground-truth physical state) is also provided. This allows for direct comparison and analysis of control performance under realistic data transmission and estimation scenarios.

The performance metrics output can be used at runtime to take online decisions, e.g., adapting the scheduling policy.

**FMI Standard**: FMI 2.0 for Co-Simulation

**Copyright**: (c) 2026 Robert Bosch GmbH

**License**: AGPL-3.0

**Authors**: Paolo Pazzaglia, Kevin Schmidt, Laura Beermann, Dirk Ziegenbein and Arne Hamann

For any questions and feedback, especially if you think could be useful also for other users, please consider opening an issue 

---

## 2. How to Use the FMU

The *LateralMotionControl* is provided as a .fmu file, precompiled for Windows 64 platform using gcc.
The .fmu file is essentially a .zip archive: e.g., by renaming it to .zip, the source files and the model description can be easily inspected.

This FMU is designed for FMI 2.0 Co-Simulation. It expects to be driven by a master algorithm that controls time and provides input values at each communication step.
More information is available in the official website https://fmi-standard.org/


### General Description

The file `modelDescription.xml` contains the parameters, inputs and outputs of the FMU.
A short list is presented here, while a more detailed presentation is in Section 4.

Parameters:
*   Controller and feedforward gains (`K_trc_fb_*`, `K_trc_ff_*`, etc.).
*   Initial state vector (`x0_*`).
*   Initial vehicle velocity (`init_velocity`).
*   Noise parameters (`noise_mean_*`, `noise_std_dev_*`).

Inputs:
1.  Trigger inputs (`*_trigger_*_in`).
2.  Command and status inputs:
    *   `ff_ref_0`, `ff_ref_1`: The feedforward trajectory reference.
    *   `velocity`: The current vehicle velocity. The FMU's internal `Ad`, `Bd`, and `Fd` matrices will be recalculated if this value changes.

Outputs:
1. Physical system state.
2. Intermediate labels of tasks.
3. Performance metrics ("in local platform", "in remote platform", and "real").

The file `LateralMotionControl.c` contains the functional logic of the controlled system, including the task chain communication and the physical simulation.
A task's logic is executed when its `_trigger_activated_in` trigger is true. The corresponding output is latched when its `_trigger_finished_in` (or equivalent) trigger is true.
These boolean inputs should be provided by, e.g., an external scheduling simulator that can be executed in parallel with the FMU logic.

Different examples of feedforward reference and the corresponding velocity values are provided in the `example` repository.

### Simulink

Open Simulink Model: Start MATLAB, open Simulink, and create a new, blank model or open an existing one.

1. Add the FMU Block:
    * Open the Simulink Library Browser.
    * Find the FMU block (usually under `Simulink Extras`) and drag it into your model.

2. Configure the FMU Block:
    * Double-click the FMU block to open its configuration dialog.
    * In the FMU file field, click the *Browse...* button and locate your .fmu file.
    * Once loaded, the block will automatically parse the FMU and display information about it, such as the Model name, FMU standard version, and FMU type (Co-Simulation or Model Exchange).

3. Solver Settings:
    * A fixed-step solver of 0.1 ms (equal to the FMU `communicationStepSize`) is suggested.

4. Connect Inputs and Outputs:
    * After loading the FMU, the block will automatically update to show the necessary input and output ports.
    * Connect Simulink sources to the input ports.
    * Connect Simulink sinks (e.g., Scope, Display, To Workspace blocks) to the output ports to visualize the results.

5. Run the Simulation:
    * Set your simulation stop time.
    * Click the Run button in the Simulink toolbar.
    * Observe the outputs on your Scopes or other connected blocks.


### C code

Running an FMU in C is not like linking a simple library. You must act as the simulation master or importer.

Preliminary steps:

1.  **Unzipping the FMU**: Extract the FMU archive to a temporary directory. This reveals the modelDescription.xml file and the shared library (.dll on Windows, .so on Linux, .dylib on macOS).
    **Parsing modelDescription.xml**: Read this XML file to get crucial information:
       *  The name of the shared library.
       *  Model variable names and their value references (handles used to identify variables).
       *  GUID, FMI version, and whether it's Model Exchange or Co-Simulation.
    **Loading the Shared Library**: Use your OS's functions (LoadLibrary on Windows, dlopen on Linux/macOS) to load the shared library into memory.
    **Getting Function Pointers**: Use GetProcAddress (Windows) or dlsym (Linux/macOS) to get pointers to the FMI functions defined in the shared library (e.g., fmi2Instantiate, fmi2DoStep, fmi2GetReal).

Initialization:

1.  Instantiate the FMU with `LateralMotionControl_fmi2Instantiate` followed by `LateralMotionControl_fmi2SetupExperiment`
2.  **Set Parameters (Optional)**: Before exiting initialization mode, set the parameter variables using `LateralMotionControl_fmi2SetReal`, using the values specified in `modelDescription.xml`.

3.  **Initialization Mode**: Execute `LateralMotionControl_fmi2EnterInitializationMode` followed by `LateralMotionControl_fmi2ExitInitializationMode`: The FMU finalizes its setup, copying initial parameters to the internal state.

The FMU advances its state based on the `communicationStepSize` provided in the `LateralMotionControl_fmi2DoStep` call. The `communicationStepSize` must be either 0 or a multiple of the base step (0.1 ms).
The master should follow this sequence for each step:

1.  **Set Inputs**:
    *   Set all boolean trigger inputs (`*_trigger_*_in`) via `LateralMotionControl_fmi2SetBoolean`.
    *   Set the real inputs via `LateralMotionControl_fmi2SetReal`.
2.  **Step the FMU**: Call `LateralMotionControl_fmi2DoStep(currentTime, stepSize)`. The FMU will:
    *   Evolve the internal physical state for the duration of `stepSize`.
    *   Execute logic for any activated triggers.
    *   Update internal performance metrics.
3.  **Get Outputs**: Read the output variables of interest using `LateralMotionControl_fmi2GetReal`, `LateralMotionControl_fmi2GetInteger`, and `LateralMotionControl_fmi2GetBoolean`.

Termination and cleanup:


1. **To terminate**: Execute `LateralMotionControl_fmi2Terminate`.
2. **To cleanup**: Execute `LateralMotionControl_fmi2FreeInstance`.
3. **To reset**: Execute `LateralMotionControl_fmi2Reset`.


### Python

You'll need a Python library to handle the FMI standard. The most common and recommended library is [FMpy](https://fmpy.readthedocs.io/en/latest/)


---

## 3. Model Description

The FMU models a complete vehicle lateral control system based on the following equation of motion:
```
x(k+1) = Ad(v) * x(k) + Bd(v) * u(k) + Fd(v) * r(k)
```


Where:

*   **x(k)**: The 6-element state vector at step k.
*   **u(k)**: The control input (steering angle) at step k.
*   **r(k)**: The feedforward reference input (related to path curvature) at step k.
*   **Ad(v), Bd(v), Fd(v)**: State-space matrices that are parametrically dependent on the vehicle's longitudinal velocity in inertial coordinate frame v. The FMU recalculates these matrices whenever the input velocity changes.

The FMU expects and operates with a fixed time step having granularity of `0.1 ms`. The matrices presented above are discretized with the same time step.

### State Vector (x)

The physical state of the system is described by a 6-element vector:

| State                | Description                                  |
| -------------------- | -------------------------------------------- |
| **Yaw Rate**         | `phi_dot`                                    |
| **Slip Angle**       | `beta` - the angle between the vehicle velocity vector at the center of gravity and the vehicle longitudinal axis                                     |
| **Steering Angle**   | `delta`                                      |
| **Steering Rate**    | `delta_dot` - derivative of Steering Angle                                 |
| **Lateral Error**    | `e_y` - vehicle-perpendicular distance to target path         |
| **Lateral Error Rate** | `e_y_dot` - derivative of Lateral Error                                   |

### Control Loop and Task Chain


The FMU models a chain of tasks that represent a modern, distributed, control architecture. 
The exchange mechanism of data (data read and data write) across each task instance is governed by boolean `_trigger_activated_in` and `_trigger_finished_in` input signals.


1.  **Sensor**: Reads the physical state (`current_phys_state`) and produces a sensed output (`sens_out`), corresponding to  [`phi_dot`,`delta`,`delta_dot`,`e_y`, `v`], which includes also the longitudinal velocity.
2.  **Network (Sensor -> Remote)**: Represents the transmission of sensor data over the network from the local to the remote platform. Triggered by `network_sc_trigger_sent_in` and `network_sc_trigger_received_in`.
3.  **Estimator**: Receives sensor data and reconstructs the full 6-dimensional state vector (`est_states_out`). It also calculates remote platform-side performance metrics.
4.  **Feedforward**: Calculates a feedforward steering command (`ff_psi_dot_out`) based on the reference trajectory input (`ff_ref`).
5.  **Controller**: Calculates a feedback steering command (`fb_psi_dot_out`) based on the estimated state.
6.  **Merger**: Combines the feedback and feedforward commands into a final desired steering angle (`agg_delta_out`).
7.  **Network (Remote -> Actuator)**: Represents the transmission of the final command to the actuator over the network. Triggered by `network_ca_trigger_sent_in` and `network_ca_trigger_received_in`.
8.  **Actuator**: Receives the final command and produces the actual steering input (`act_out`) applied to the vehicle dynamics.


In this setup, Sensor and Actuator are in-vehicle tasks, while Estimator, Controller, Feedforward and Merger are mapped in a shared remote platform (e.g., in cloud).

The basic parameters of the tasks are included in the example folder.

### Performance Metrics

The FMU calculates and outputs several key performance metrics to evaluate the controller's effectiveness. These are calculated for three different perspectives:
*   **In Local Platform**: Based on network-delayed data available locally (in-vehicle).
*   **In Remote Platform**: Based on the estimated state calculated on the remote platform.
*   **Real**: An ideal metric based on ground-truth data, for benchmarking.

The metrics are:
*   **Rolling Performance**: A rolling average of the quadratic cost function `x'Qx` over a 1-second window. The Q matrix penalizes lateral error.
*   **Average Performance**: The cumulative average of `x'Qx` over the entire simulation duration.
*   **Constraint Violation**: A boolean flag (`violated_constraint`) and counter (`threshold_error_cntr`) that track when the lateral error exceeds a predefined `ERROR_THRESHOLD` of 0.2 meters.
*   **Critical Section**: A boolean flag indicating if the vehicle is in a critical maneuver (e.g., turning), based on the feedforward reference.

The performance constraints are included in the example folder.

---

## 4. Inputs, Outputs, and Parameters

### Parameters (Settable at Initialization)

*   `K_trc_fb_*` (Real): Feedback controller gain matrix elements.
*   `K_trc_ff_*` (Real): Feedforward controller gain matrix elements.
*   `K_yrc_x_*`, `K_yrc_psi_*` (Real): Yaw rate controller gain matrix elements.
*   `x0_*` (Real): Initial values for the 6 system states.
*   `init_velocity` (Real): Initial vehicle velocity used to calculate matrices at the start.
*   `noise_mean_*`, `noise_std_dev_*` (Real): Mean and standard deviation for optional Gaussian noise on states.

### Inputs (Settable at Each Step)

*   `*_*_trigger_*_*_in` (Boolean): A set of 16 trigger signals to control the execution flow of the internal tasks (Sensor, Estimator, Controller, etc.). **To be obtained with a scheduling simulator**
*   `ff_ref_0`, `ff_ref_1` (Real): Feedforward reference inputs, related to desired path curvature. **Given as input in the examples**
*   `velocity` (Real): Current vehicle velocity in m/s. **Given as input in the examples**


The feedforward reference and vehicle velocity can be found in the example folder for different speed profiles.

### Outputs (Readable at Each Step)

*   `current_phys_state_*` (Real): The 6 true physical states of the vehicle model.
*   `sens_out_*` (Real): The output of the sensor task.
*   `est_states_out_*` (Real): The 6 estimated states from the estimator task.
*   `fb_psi_dot_out_0`, `ff_psi_dot_out_0` (Real): Intermediate outputs from the controller and feedforward blocks.
*   `act_out_0` (Real): The final steering angle command applied to the vehicle model.
*   `current_time_out` (Real): The current simulation time of the FMU.
*   `current_step_out` (Integer): The internal step counter.
*   **Performance Metrics** (Real/Integer/Boolean): A comprehensive set of outputs for "in local platform", "in remote platform" and "real" performance, including:
    *   `in_local_platform_rolling_performance` / `in_remote_platform_rolling_performance` / `real_rolling_performance`
    *   `in_local_platform_average_performance` / `in_remote_platform_average_performance` / `real_average_performance`
    *   `in_local_platform_threshold_error_cntr` / `in_remote_platform_threshold_error_cntr`  / `real_threshold_error_cntr`
    *   `in_local_platform_violated_constraint` / `in_remote_platform_violated_constraint`  / `real_violated_constraint`
    *   `in_local_platform_critical_section` / `in_remote_platform_critical_section`  / `real_critical_section`

> Refer to the `modelDescription.xml` for a complete list of all variables, their value references, and detailed descriptions.

---

## 5. The Challenge Questions

* **Q1: Choose a standard (non-context-aware) scheduling policy for the cloud platform and network communication, such that the highest number of vehicles `N_vehicles` can be accommodated while satisfying the given performance requirements.**

As first step, the users should identify the limit of classic hard-deadline models and classic schedulers, where no functional information of the physical system is used at runtime.
This result should serve as baseline comparison, against the more advanced solutions of the next questions.

* **Q2: Design a *context-aware* and *adaptive* resource and/or scheduling policy in-cloud that uses the control-related information of the vehicles available at runtime, such that the highest number `N_vehicles` of vehicle controllers can be executed in parallel on the available `N_cores` cloud cores while satisfying the given performance requirements.**

In this second step, we ask the users to identify adaptive strategies that make use of the performance and criticality metrics to adapt their policy online by, e.g., dynamic adjustment of task priorities or resource allocation, such that a larger number `N_vehicles` with respect to Q1 can be scheduled, while still satisfying the same control performance requirements.

* **Q3: Given a fixed number of vehicles `N_vehicles`, find the minimum amount of computational resources in cloud that satisfy the given performance requirements.**

In this question, we target the problem of efficient usage of resources, with the goal of minimizing the overall cost of operation in the cloud, while guaranteeing the required control performance requirements.
This aspect is of particular importance to obtain cost-effective applications, and free space for additional parallel tasks.

* **Q4: Identify new timing abstractions that are able to better capture the relationship between variable sensitivity towards control performance requirements to the corresponding adaptive timing constraints.**

Classic timing abstractions focus on worst-case scenarios only, hiding the dependencies to dynamic conditions of Cyber-Physical Systems (CPS).
Here, we ask the users to design more nuanced abstractions, which may, e.g., enforce stricter constraints in specific operating conditions while allowing skipped jobs or missed deadlines in other ones.

* **Q5: Analyze the resulting end-to-end latency metrics of the control chains, under different scheduling and communication strategies.**

Since control software increasingly relies on combining multiple steps in functional chains, understanding how uncertain timing on one or multiple tasks of the chain may affect the whole functionality is of critical importance.
From a control perspective, particularly interesting is providing models in terms of *data age* of the applied control commands.

* **Q6: Relax the assumption of periodic tasks in the in-cloud vehicle task chain and propose instead a data-driven, context-aware and adaptive strategy that solves the previous questions.**

As mentioned in the network section, latency metrics can be potentially reduced by allowing data-driven triggers for the in-cloud tasks.
Freeing this design direction for the previous questions can also potentially improve the overall results.

