# Task Execution Model

The task chain is composed of the following tasks, whose behavior is abstracted in the FMU:
1.  **Sensor**: Reads the physical state (`current_phys_state`) and produces a sensed output (`sens_out`), corresponding to  [`phi_dot`,`delta`,`delta_dot`,`e_y`, `v`], which includes also the longitudinal velocity.
2.  **Network (Sensor -> Cloud)**: Represents the transmission of sensor data over the network.
3.  **Estimator**: Receives sensor data and reconstructs the full 6-dimensional state vector (`est_states_out`). It also calculates cloud-side performance metrics.  In this example, its execution time considers also the execution of a sensor fusion algorithm for state estimation, which is abstracted away in the FMU.
4.  **Feedforward**: Calculates a feedforward steering command (`ff_psi_dot_out`) based on the reference trajectory input (`ff_ref`). In this example, its execution time considers also the execution of an online path-planner, which is abstracted away in the FMU.
5.  **Controller**: Calculates a feedback steering command (`fb_psi_dot_out`) based on the estimated state.
6.  **Merger**: Combines the feedback and feedforward commands into a final desired steering angle (`agg_delta_out`).
7.  **Network (Cloud -> Actuator)**: Represents the transmission of the final command to the actuator over the network.
8.  **Actuator**: Receives the final command and produces the actual steering input (`act_out`) applied to the vehicle dynamics.

In this setup, Sensor and Actuator are in-vehicle tasks, while Estimator, Controller, Feedforward and Merger are mapped in a shared remote platform (e.g., in cloud).
An example of execution times and network delays to be used for the Challenge is presented below:

| Task | Period (ms) | BCET (ms) | AvET (ms) | WCET (ms) |
| :--- | :--- | :--- | :--- | :--- |
| Sensor | 5 | 0.4 | 0.6 | 1.0 |
| Estimator | 10 | 0.7 | 0.9 | 1.1 |
| Feedforward | 20 | 0.2 | 1.2 | 2.5 |
| Controller | 20  | 0.2 | 0.3 | 0.5 |
| Merger | 20  | 0.2 | 0.3 | 0.5 |
| Actuator | 30 | 0.2 | 0.5 | 0.7 |

| Communication (Private 5G) | BC delay (ms) | Av delay (ms) | WC delay (ms) |
| :--- | :--- | :--- | :--- |
| Network (Sensor -> Cloud) |  1 | 8 | 16 |
| Network (Cloud -> Actuator)| 1 | 8 | 16 |

**Notes:**
* One task set must be created for each vehicle, with the general goal of adding the highest number of vehicles `N_vehicles` that satisfy the control-related performance constraints.
* Offsets, deadlines and scheduling policy needs to be defined by the user.
* The remote plaform comprises `N_c = 3` processing cores.
* A probabilistic execution time modelization for each task can be realized by mapping the best case, average and worst case execution times as a PERT distribution.
* More complex variant: considering the Controller Task as an abstract implementation of an MPC --> multiply its execution time x10
* All the values are given as examples: different values can be chosen and additional interfering tasks and network load can be added, with proper motivation.


