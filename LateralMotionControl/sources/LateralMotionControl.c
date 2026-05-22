/*
  Bosch RTAS Challenge
  Copyright (c) 2026 Robert Bosch GmbH
  SPDX-License-Identifier: AGPL-3.0
*/

#define FMI2_FUNCTION_PREFIX LateralMotionControl_
#include "fmi2Functions.h"
#include "fmi2FunctionTypes.h"
#include "fmi2TypesPlatform.h"
#define _USE_MATH_DEFINES
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>

// --- FMI Specific Definitions ---
#define MODEL_IDENTIFIER "LateralMotionControl"
#define MODEL_GUID "{ec101913-52ec-40d8-afe6-5fbb52430f74}" // Generate a unique GUID for your FMU

// Define matrix dimensions
#define NUM_STATES 6
#define NUM_SENSED_STATES 5 // [phi_dot, delta, delta_dot, e_y, v]
#define NUM_INPUTS 1 // B is 6x1
#define NUM_OUTPUTS 6
#define NUM_FEEDFORWARD_INPUTS 2 // F is 6x2
#define NETWORK_CA_PACKET_SIZE (NUM_INPUTS + NUM_FEEDFORWARD_INPUTS) // Combined packet size for controller->actuator network

// Define relevant vector indices
#define ERROR_STATE_INDEX 4 // Lateral error is x[4]
#define ERROR_DOT_STATE_INDEX 5 // Derivative of lateral error is x[5]
#define VELOCITY_SENSED_STATE_INDEX 4 // Velocity in sensed state is s[4]
// Define simulation step
#define SIM_STEP 0.0001 // Simulation granularity of .1ms
#define EST_STEP 0.01 // Estimator period of 10ms
// Define prediction length (in meters)
#define ELL_P 0.2
// Define TRC parameters
#define d_TRC 1
#define omega_TRC 10
// Define rolling window for performance calculation
#define PERF_WINDOW_SIZE 100 // Number of performance samples for rolling average --> 1 second
// Define lateral error threshold and failure probability threshold
#define ERROR_THRESHOLD 0.2 // 20 cm
#define PROB_THRESHOLD 0.05
// Define tolerance
#define TOL 0.000001
// Define Network Buffer (FIFO) size
#define NETWORK_BUFFER_SIZE 100 // Maximum number of packets the network can hold

// Value References (VRs) for inputs, outputs, parameters
enum {
    // Initial parameters
    VR_K_TRC_FB_START = 1, // K_TRC_FB[0][0]
    VR_K_TRC_FB_END = VR_K_TRC_FB_START + NUM_INPUTS * NUM_STATES - 1,

    VR_K_TRC_FF_START = VR_K_TRC_FB_END + 1, // K_TRC_FF[0][0]
    VR_K_TRC_FF_END = VR_K_TRC_FF_START + NUM_INPUTS * NUM_FEEDFORWARD_INPUTS - 1,

    VR_K_YRC_STATE_START = VR_K_TRC_FF_END + 1, // K_YRC_X[0][0]
    VR_K_YRC_STATE_END = VR_K_YRC_STATE_START + NUM_INPUTS * NUM_STATES - 1,

    VR_K_YRC_PSI_START = VR_K_YRC_STATE_END + 1,  // K_YRC_PSI[0][0]
    VR_K_YRC_PSI_END = VR_K_YRC_PSI_START + NUM_INPUTS * NUM_INPUTS - 1,

    VR_X0_START = VR_K_YRC_PSI_END + 1, // Initial state
    VR_X0_END = VR_X0_START + NUM_STATES - 1,

    VR_NOISE_MEAN_START = VR_X0_END + 1, // Mean noise of states
    VR_NOISE_MEAN_END = VR_NOISE_MEAN_START + NUM_STATES - 1,

    VR_NOISE_STD_DEV_START = VR_NOISE_MEAN_END + 1, // Standard deviation of noise of states
    VR_NOISE_STD_DEV_END = VR_NOISE_STD_DEV_START + NUM_STATES - 1,

    VR_INIT_VELOCITY = VR_NOISE_STD_DEV_END + 1, // Vehicle initial velocity
    
    // Inputs for triggers and feedforward reference
    // These are BOOLEAN inputs
    VR_SENSOR_TRIGGER_ACTIVATED_INPUT = 100, 
    VR_SENSOR_TRIGGER_FINISHED_INPUT,
    VR_NETWORK_SC_TRIGGER_SENT_INPUT,
    VR_NETWORK_SC_TRIGGER_RECEIVED_INPUT,
    VR_ESTIMATOR_TRIGGER_ACTIVATED_INPUT,
    VR_ESTIMATOR_TRIGGER_FINISHED_INPUT,
    VR_CONTROLLER_TRIGGER_ACTIVATED_INPUT,
    VR_CONTROLLER_TRIGGER_FINISHED_INPUT,
    VR_FEEDFORWARD_TRIGGER_ACTIVATED_INPUT,
    VR_FEEDFORWARD_TRIGGER_FINISHED_INPUT,
    VR_MERGER_TRIGGER_ACTIVATED_INPUT,
    VR_MERGER_TRIGGER_FINISHED_INPUT,
    VR_NETWORK_CA_TRIGGER_SENT_INPUT,
    VR_NETWORK_CA_TRIGGER_RECEIVED_INPUT,
    VR_ACTUATOR_TRIGGER_ACTIVATED_INPUT,
    VR_ACTUATOR_TRIGGER_FINISHED_INPUT,

    // These are REAL inputs
    VR_FF_REF_START, // ff_ref[0] 
    VR_FF_REF_END = VR_FF_REF_START + NUM_FEEDFORWARD_INPUTS - 1,

    VR_VELOCITY = VR_FF_REF_END + 1, // Vehicle velocity

    // Outputs
    // Intermediate labels
    VR_CURRENT_PHYS_STATE_START = 1000, // current_phys_state[0]
    VR_CURRENT_PHYS_STATE_END = VR_CURRENT_PHYS_STATE_START + NUM_STATES - 1,

    VR_SENS_OUT_START,
    VR_SENS_OUT_END = VR_SENS_OUT_START + NUM_SENSED_STATES - 1,

    VR_EST_STATES_OUT_START,
    VR_EST_STATES_OUT_END =  VR_EST_STATES_OUT_START + NUM_STATES - 1,

    VR_CNTRL_OUT_START,
    VR_CNTRL_OUT_END = VR_CNTRL_OUT_START + NUM_INPUTS - 1,

    VR_FFOUT_START,
    VR_FFOUT_END = VR_FFOUT_START + NUM_FEEDFORWARD_INPUTS - 1,

    VR_FFOUT_PSI_DOT_START,
    VR_FFOUT_PSI_DOT_END = VR_FFOUT_PSI_DOT_START + NUM_INPUTS - 1,

    VR_AGG_OUT_START,
    VR_AGG_OUT_END = VR_AGG_OUT_START + NUM_INPUTS - 1,

    VR_ACT_OUT_START,
    VR_ACT_OUT_END = VR_ACT_OUT_START + NUM_INPUTS - 1,

    VR_CURRENT_TIME_OUT,
    VR_CURRENT_STEP_OUT,

    // Critical section
    VR_IN_LOCAL_PLATFORM_CRITICAL_SECTION,
    VR_IN_REMOTE_PLATFORM_CRITICAL_SECTION,
    VR_REAL_CRITICAL_SECTION,

    // Rolling Performance Output
    VR_IN_LOCAL_PLATFORM_ROLLING_PERFORMANCE,
    VR_IN_REMOTE_PLATFORM_ROLLING_PERFORMANCE,
    VR_REAL_ROLLING_PERFORMANCE,

    // Average Performance Output
    VR_IN_LOCAL_PLATFORM_AVERAGE_PERFORMANCE,
    VR_IN_REMOTE_PLATFORM_AVERAGE_PERFORMANCE,
    VR_REAL_AVERAGE_PERFORMANCE,

    // Threshold error
    VR_IN_LOCAL_PLATFORM_THRESHOLD_ERROR_CNTR,
    VR_IN_REMOTE_PLATFORM_THRESHOLD_ERROR_CNTR,
    VR_REAL_THRESHOLD_ERROR_CNTR,

    VR_IN_LOCAL_PLATFORM_VIOLATED_CONSTRAINT,
    VR_IN_REMOTE_PLATFORM_VIOLATED_CONSTRAINT,
    VR_REAL_VIOLATED_CONSTRAINT
};

// --- Hardcoded Matrices ---
// Sensor task matrix
const double HARDCODED_S[NUM_SENSED_STATES][NUM_STATES] = {
    {1.0, 0.0, 0.0, 0.0, 0.0, 0.0},
    {0.0, 0.0, 1.0, 0.0, 0.0, 0.0},
    {0.0, 0.0, 0.0, 1.0, 0.0, 0.0},
    {0.0, 0.0, 0.0, 0.0, 1.0, 0.0},
    {0.0, 0.0, 0.0, 0.0, 0.0, 0.0} // This line will contain the velocity
};

// Estimator task matrix 
const double HARDCODED_EST[NUM_STATES][NUM_SENSED_STATES] = {
    {1.0, 0.0, 0.0, 0.0, 0.0},
    {0.0, 0.0, 0.0, 0.0, 0.0},
    {0.0, 1.0, 0.0, 0.0, 0.0},
    {0.0, 0.0, 1.0, 0.0, 0.0},
    {0.0, 0.0, 0.0, 1.0, 0.0},
    {0.0, 0.0, 0.0, 0.0, 0.0}
};


// Hardcoded Q matrix for performance calculation (NUM_STATES x NUM_STATES)
// Q penalizes lateral error and its rate.
const double HARDCODED_Q[NUM_STATES][NUM_STATES] = {
    {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
    {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
    {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
    {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
    {0.0, 0.0, 0.0, 0.0, 10.0, 0.0}, // Penalize lateral error more
    {0.0, 0.0, 0.0, 0.0, 0.0, 5.0}  // Penalize lateral error rate
};


// --- Helper Functions ---
// Helper function for matrix-vector multiplication (MxN * Nx1)
static void mat_vec_mult(const double* mat, int rows, int cols, const double* vec, double* result) {
    for (int i = 0; i < rows; ++i) {
        result[i] = 0.0;
        for (int j = 0; j < cols; ++j) {
            result[i] += mat[i * cols + j] * vec[j];
        }
    }
}

// Helper function for vector addition
static void vec_add(const double* vec1, const double* vec2, int size, double* result) {
    for (int i = 0; i < size; ++i) {
        result[i] = vec1[i] + vec2[i];
    }
}

// Helper function for vector copy
static void vec_copy(const double* src, double* dest, int size) {
    for (int i = 0; i < size; ++i) {
        dest[i] = src[i];
    }
}

// Helper function to calculate x' * Q * x
// x_in: NUM_STATES x 1 vector
// Q_mat: NUM_STATES x NUM_STATES matrix
// Returns the scalar value x'Qx
static double vec_transpose_mat_vec_mult(const double* x_in, const double* Q_mat) {
    double Q_x[NUM_STATES]; // Stores Q * x
    mat_vec_mult(Q_mat, NUM_STATES, NUM_STATES, x_in, Q_x);

    double result = 0.0;
    for (int i = 0; i < NUM_STATES; ++i) {
        result += x_in[i] * Q_x[i]; // Dot product of x' and (Qx)
    }
    return result;
}

// --- Structures ---
// Structure for internal network buffer (FIFO)
typedef struct {
    double** data;      // Array of pointers to packet data
    int head;           // Index to pop from
    int tail;           // Index to push to
    int count;          // Number of elements in the buffer
    int capacity;       // Buffer capacity
    int packet_size;    // Size of each packet vector
} NetworkBuffer;

// Structure for variables used by the tasks in vehicle and remote platform
typedef struct {
    // Sensor values
    double* sens_out; // NUM_SENSED_STATES
    double* sens_comp;  // NUM_SENSED_STATES
    // Network sensor to controller values (received data)
    double* network_sc_rec; // NUM_SENSED_STATES
    // Estimator values
    double* est_states_out; // NUM_STATES
    double* est_states_comp;  // NUM_STATES
    double* est_vel_out;
    double* est_vel_in;
    // Feedback control values
    double* fb_psi_dot_out; // NUM_INPUTS
    double* fb_psi_dot_comp; // NUM_INPUTS
    // Feedforward control values
    double* ff_out;   // NUM_FEEDFORWARD_INPUTS
    double* ff_comp;  // NUM_FEEDFORWARD_INPUTS
    double* ff_psi_dot_out; // NUM_INPUTS
    double* ff_psi_dot_comp; // NUM_INPUTS
    // merger values
    double* agg_delta_out;  // NUM_INPUTS
    double* agg_delta_comp; // NUM_INPUTS
    // Network controller to actuator values (received data)
    double* network_ca_fb_rec;  // NUM_INPUTS
    double* network_ca_ff_rec;  // NUM_FEEDFORWARD_INPUTS
    // Actuator values
    double* act_out; // NUM_INPUTS
    double* act_in;  // NUM_INPUTS
} TaskChainsVariables;

// Structure for physical simulation of vehicle
typedef struct
{
    // Current system state
    double current_phys_state[NUM_STATES];
    // Initial state
    double x0_param[NUM_STATES];
    // Initial velocity
    double v0_param;
    // Velocity
    double velocity_param;
    // Simulation step progress
    double current_time;
    int step;

    // Physical noise
    bool physical_noise;
    double mean_noise[NUM_STATES];
    double st_dev_noise[NUM_STATES];

} SimPhysicalVariables;

// Structure for variables used to manage trigger states (direct inputs)
typedef struct {
    fmi2Boolean sensor_trigger_activated;
    fmi2Boolean sensor_trigger_finished;
    fmi2Boolean network_sc_trigger_sent;
    fmi2Boolean network_sc_trigger_rec;
    fmi2Boolean estimator_trigger_activated;
    fmi2Boolean estimator_trigger_finished;
    fmi2Boolean controller_trigger_activated;
    fmi2Boolean controller_trigger_finished;
    fmi2Boolean feedforward_trigger_activated;
    fmi2Boolean feedforward_trigger_finished;
    fmi2Boolean merger_trigger_activated;
    fmi2Boolean merger_trigger_finished;
    fmi2Boolean network_ca_trigger_sent;
    fmi2Boolean network_ca_trigger_rec;
    fmi2Boolean actuator_trigger_activated;
    fmi2Boolean actuator_trigger_finished;
} SimTriggerVariables;

// Structure for Performance variables
typedef struct {
    // Rolling window performance
    double window_buffer[PERF_WINDOW_SIZE]; // Circular buffer for x'Qx values
    int head_index;                         // Current insertion point (oldest sample if full)
    int buffer_count;                       // Number of valid samples in the buffer
    double current_rolling_sum;             // Running sum of values in the buffer
    double rolling_average;                 // The calculated rolling average (output)

    // Total performance
    int total_count;                        // Number of total samples
    double current_total_sum;               // Running total sum of values
    double total_average;                   // The calculated average (output)

    // Critical section
    bool critical_section;                  // Checks if vehicle in a curve or not

    // Threshold performance
    bool violated_threshold;                // Threshold currently violated
    int num_violations;                     // Number of violations of chance constraint
} PerformanceMetrics;

// --- Initialization Functions for internal structs ---
// Initialization for network buffer
static void initializeNetworkBuffer(NetworkBuffer* buffer, int capacity, int packet_size, fmi2CallbackAllocateMemory allocateMemory) {
    buffer->capacity = capacity;
    buffer->packet_size = packet_size;
    buffer->head = 0;
    buffer->tail = 0;
    buffer->count = 0;
    buffer->data = (double**)allocateMemory(capacity, sizeof(double*));
    for (int i = 0; i < capacity; i++) {
        buffer->data[i] = (double*)allocateMemory(packet_size, sizeof(double));
        memset(buffer->data[i], 0, packet_size * sizeof(double));
    }
}

// Initialization for task chains variables
static void allocateTaskChainsVariables(TaskChainsVariables* s_tasks_ptr, fmi2CallbackAllocateMemory allocateMemory) {
    s_tasks_ptr->sens_comp = (double*)allocateMemory(NUM_SENSED_STATES, sizeof(double));
    s_tasks_ptr->sens_out = (double*)allocateMemory(NUM_SENSED_STATES, sizeof(double));
    s_tasks_ptr->network_sc_rec = (double*)allocateMemory(NUM_SENSED_STATES, sizeof(double));
    s_tasks_ptr->est_states_comp = (double*)allocateMemory(NUM_STATES, sizeof(double));
    s_tasks_ptr->est_states_out = (double*)allocateMemory(NUM_STATES, sizeof(double));
    s_tasks_ptr->est_vel_in = (double*)allocateMemory(1, sizeof(double));
    s_tasks_ptr->est_vel_out = (double*)allocateMemory(1, sizeof(double));
    s_tasks_ptr->fb_psi_dot_comp = (double*)allocateMemory(NUM_INPUTS, sizeof(double));
    s_tasks_ptr->fb_psi_dot_out = (double*)allocateMemory(NUM_INPUTS, sizeof(double));
    s_tasks_ptr->ff_comp = (double*)allocateMemory(NUM_FEEDFORWARD_INPUTS, sizeof(double));
    s_tasks_ptr->ff_out = (double*)allocateMemory(NUM_FEEDFORWARD_INPUTS, sizeof(double));
    s_tasks_ptr->ff_psi_dot_comp = (double*)allocateMemory(NUM_INPUTS, sizeof(double));
    s_tasks_ptr->ff_psi_dot_out = (double*)allocateMemory(NUM_INPUTS, sizeof(double));
    s_tasks_ptr->agg_delta_comp = (double*)allocateMemory(NUM_INPUTS, sizeof(double));
    s_tasks_ptr->agg_delta_out = (double*)allocateMemory(NUM_INPUTS, sizeof(double));
    s_tasks_ptr->network_ca_fb_rec = (double*)allocateMemory(NUM_INPUTS, sizeof(double));
    s_tasks_ptr->network_ca_ff_rec = (double*)allocateMemory(NUM_FEEDFORWARD_INPUTS, sizeof(double));
    s_tasks_ptr->act_in = (double*)allocateMemory(NUM_INPUTS, sizeof(double));
    s_tasks_ptr->act_out = (double*)allocateMemory(NUM_INPUTS, sizeof(double));
}

static void resetTaskChainsVariables(TaskChainsVariables* s_tasks_ptr, const double* v0_value) {
    // Initialize all values to their starting state (zero)
    memset(s_tasks_ptr->sens_out, 0, NUM_SENSED_STATES * sizeof(double));
    memset(s_tasks_ptr->sens_comp, 0, NUM_SENSED_STATES * sizeof(double));
    memset(s_tasks_ptr->network_sc_rec, 0, NUM_SENSED_STATES * sizeof(double));
    memset(s_tasks_ptr->est_states_out, 0, NUM_STATES * sizeof(double));
    memset(s_tasks_ptr->est_states_comp, 0, NUM_STATES * sizeof(double));

    memcpy(s_tasks_ptr->est_vel_in, v0_value, sizeof(double));
    memcpy(s_tasks_ptr->est_vel_out, v0_value, sizeof(double));
    memset(s_tasks_ptr->fb_psi_dot_comp, 0, NUM_INPUTS * sizeof(double));
    memset(s_tasks_ptr->fb_psi_dot_out, 0, NUM_INPUTS * sizeof(double));
    memset(s_tasks_ptr->ff_comp, 0, NUM_FEEDFORWARD_INPUTS * sizeof(double));
    memset(s_tasks_ptr->ff_out, 0, NUM_FEEDFORWARD_INPUTS * sizeof(double));
    memset(s_tasks_ptr->ff_psi_dot_comp, 0, NUM_INPUTS * sizeof(double));
    memset(s_tasks_ptr->ff_psi_dot_out, 0, NUM_INPUTS * sizeof(double));
    memset(s_tasks_ptr->agg_delta_comp, 0, NUM_INPUTS * sizeof(double));
    memset(s_tasks_ptr->agg_delta_out, 0, NUM_INPUTS * sizeof(double));
    memset(s_tasks_ptr->network_ca_fb_rec, 0, NUM_INPUTS * sizeof(double));
    memset(s_tasks_ptr->network_ca_ff_rec, 0, NUM_FEEDFORWARD_INPUTS * sizeof(double));
    memset(s_tasks_ptr->act_in, 0, NUM_INPUTS * sizeof(double));
    memset(s_tasks_ptr->act_out, 0, NUM_INPUTS * sizeof(double));
}

// Initialization for physical simulation parameters
static void initializeSimPhysicalVariables(SimPhysicalVariables* s_phys_ptr, const double* x0_init, const double v0_init, const double* st_dev_noise_init, const double* mean_noise_init) { 
    
    // Current state
    vec_copy(x0_init, s_phys_ptr->current_phys_state, NUM_STATES);
    // Initial state
    vec_copy(x0_init, s_phys_ptr->x0_param, NUM_STATES);
    // Initial velocity
    s_phys_ptr->v0_param = v0_init;
    // Velocity
    s_phys_ptr->velocity_param = v0_init;
    // Simulation step progress
    s_phys_ptr->current_time = 0;
    s_phys_ptr->step = 0;

    // Physical noise
    s_phys_ptr->physical_noise = false;
    vec_copy(mean_noise_init, s_phys_ptr->mean_noise, NUM_STATES);
    vec_copy(st_dev_noise_init, s_phys_ptr->st_dev_noise, NUM_STATES);
}

// Initialization for SimTriggerVariables (default to fmi2False)
static void initializeSimTriggerVariables(SimTriggerVariables* s_trig_ptr) {
    s_trig_ptr->sensor_trigger_activated = fmi2False;
    s_trig_ptr->sensor_trigger_finished = fmi2False;
    s_trig_ptr->network_sc_trigger_sent = fmi2False;
    s_trig_ptr->network_sc_trigger_rec = fmi2False;
    s_trig_ptr->estimator_trigger_activated = fmi2False;
    s_trig_ptr->estimator_trigger_finished = fmi2False;
    s_trig_ptr->controller_trigger_activated = fmi2False;
    s_trig_ptr->controller_trigger_finished = fmi2False;
    s_trig_ptr->feedforward_trigger_activated = fmi2False;
    s_trig_ptr->feedforward_trigger_finished = fmi2False;
    s_trig_ptr->merger_trigger_activated = fmi2False;
    s_trig_ptr->merger_trigger_finished = fmi2False;
    s_trig_ptr->network_ca_trigger_sent = fmi2False;
    s_trig_ptr->network_ca_trigger_rec = fmi2False;
    s_trig_ptr->actuator_trigger_activated = fmi2False;
    s_trig_ptr->actuator_trigger_finished = fmi2False;
}

// Initialization for PerformanceMetrics
static void initializePerformanceMetrics(PerformanceMetrics* perf_metric_ptr) {
    memset(perf_metric_ptr->window_buffer, 0, PERF_WINDOW_SIZE * sizeof(double));
    perf_metric_ptr->head_index = 0;
    perf_metric_ptr->buffer_count = 0;
    perf_metric_ptr->current_rolling_sum = 0.0;
    perf_metric_ptr->rolling_average = 0.0;

    perf_metric_ptr->current_total_sum = 0.0;
    perf_metric_ptr->total_count = 0;
    perf_metric_ptr->total_average = 0.0;

    perf_metric_ptr->critical_section = false;

    perf_metric_ptr->violated_threshold = false;
    perf_metric_ptr->num_violations = 0;
}

// --- Freeing Functions ---
static void freeNetworkBuffer(NetworkBuffer* buffer, fmi2CallbackFreeMemory freeMemory) {
    if (buffer && buffer->data) {
        for (int i = 0; i < buffer->capacity; i++) {
            if (buffer->data[i]) {
                freeMemory(buffer->data[i]);
            }
        }
        freeMemory(buffer->data);
    }
}

static void freeSimControllerVariables(TaskChainsVariables* s_tasks_ptr, fmi2CallbackFreeMemory freeMemory) {
    freeMemory(s_tasks_ptr->sens_out);
    freeMemory(s_tasks_ptr->sens_comp);
    freeMemory(s_tasks_ptr->network_sc_rec);
    freeMemory(s_tasks_ptr->est_states_out);
    freeMemory(s_tasks_ptr->est_states_comp);
    freeMemory(s_tasks_ptr->est_vel_in);
    freeMemory(s_tasks_ptr->est_vel_out);
    freeMemory(s_tasks_ptr->fb_psi_dot_out);
    freeMemory(s_tasks_ptr->fb_psi_dot_comp);
    freeMemory(s_tasks_ptr->ff_out);
    freeMemory(s_tasks_ptr->ff_comp);
    freeMemory(s_tasks_ptr->ff_psi_dot_out);
    freeMemory(s_tasks_ptr->ff_psi_dot_comp);
    freeMemory(s_tasks_ptr->agg_delta_out);
    freeMemory(s_tasks_ptr->agg_delta_comp);
    freeMemory(s_tasks_ptr->network_ca_fb_rec);
    freeMemory(s_tasks_ptr->network_ca_ff_rec);
    freeMemory(s_tasks_ptr->act_in);
    freeMemory(s_tasks_ptr->act_out);
}

// --- Network Buffer Helper Functions ---
// Push data into the network buffer
static bool networkBufferPush(NetworkBuffer* buffer, const double* packet_data) {
    if (buffer->count >= buffer->capacity) {
        return false; // Buffer is full
    }
    vec_copy(packet_data, buffer->data[buffer->tail], buffer->packet_size);
    buffer->tail = (buffer->tail + 1) % buffer->capacity;
    buffer->count++;
    return true;
}

// Pop data from the network buffer
static bool networkBufferPop(NetworkBuffer* buffer, double* output_packet) {
    if (buffer->count <= 0) {
        return false; // Buffer is empty
    }
    vec_copy(buffer->data[buffer->head], output_packet, buffer->packet_size);
    buffer->head = (buffer->head + 1) % buffer->capacity;
    buffer->count--;
    return true;
}

// Update rolling window metric
static void updatePerformanceWindow(PerformanceMetrics* perf_metric_ptr, double new_perf_value) {
    // 1. If window is full, remove the oldest sample from the sum
    if (perf_metric_ptr->buffer_count == PERF_WINDOW_SIZE) {
        perf_metric_ptr->current_rolling_sum -= perf_metric_ptr->window_buffer[perf_metric_ptr->head_index];
    }

    // 2. Add the new sample to the window and the sum
    perf_metric_ptr->window_buffer[perf_metric_ptr->head_index] = new_perf_value;
    perf_metric_ptr->current_rolling_sum += new_perf_value;

    // 3. Update buffer_count (if window is not yet full)
    if (perf_metric_ptr->buffer_count < PERF_WINDOW_SIZE) {
        perf_metric_ptr->buffer_count++;
    }

    // 4. Move the head index
    perf_metric_ptr->head_index = (perf_metric_ptr->head_index + 1) % PERF_WINDOW_SIZE;

    // 5. Calculate the new average
    if (perf_metric_ptr->buffer_count > 0) {
        perf_metric_ptr->rolling_average = perf_metric_ptr->current_rolling_sum / perf_metric_ptr->buffer_count;
    } else {
        perf_metric_ptr->rolling_average = 0.0; // Should not happen after first sample
    }
}

// Update all performance metrics
static void computePerformance(
    PerformanceMetrics* perf_metric_ptr,
    const double* current_phys_state,
    const double ff_ref_input[NUM_FEEDFORWARD_INPUTS]
) {
    // Calculate x'Qx for the current state
    double current_x_transpose_Q_x = vec_transpose_mat_vec_mult(current_phys_state, (const double*)HARDCODED_Q);
    // Update performance window
    updatePerformanceWindow(perf_metric_ptr, current_x_transpose_Q_x);

    // Update total performance value
    perf_metric_ptr->current_total_sum += current_x_transpose_Q_x;
    perf_metric_ptr->total_count++;

    if (perf_metric_ptr->total_count > 0) {
        perf_metric_ptr->total_average =  perf_metric_ptr->current_total_sum / perf_metric_ptr->total_count;
    } else {
        perf_metric_ptr->total_average = 0.0; // Should not happen after first sample
    }

    // Update chance constraint threshold check
    double lateral_error = fabs(current_phys_state[ERROR_STATE_INDEX]);
    perf_metric_ptr->violated_threshold = (lateral_error > ERROR_THRESHOLD);
    if (perf_metric_ptr->violated_threshold) {
        perf_metric_ptr->num_violations++;
    }

    // Critical section check
    perf_metric_ptr->critical_section = (fabs(ff_ref_input[0]) > TOL);
}


// Simulate physical model of vehicle
// x(k+1) = Ad x(k) + Bd u(k) + Fd r(k)
static void simulate_lateral_motion_control_step(
    const double Ad[NUM_STATES][NUM_STATES],
    const double Bd[NUM_STATES][NUM_INPUTS],
    const double Fd[NUM_STATES][NUM_FEEDFORWARD_INPUTS],
    TaskChainsVariables* s_tasks_ptr,
    SimPhysicalVariables* s_phys_ptr,
    PerformanceMetrics* perf_metric_int_ptr,
    PerformanceMetrics* perf_metric_real_ptr,
    const double ff_ref_input[NUM_FEEDFORWARD_INPUTS]
) {
    // Initialize intermediate variables for algebraic computations
    double Ad_x[NUM_STATES];
    double Bd_u_ctrl[NUM_STATES];
    double ay_des[NUM_STATES];
    double temp_state[NUM_STATES];

    // Update the state using the discrete-time system equations
    mat_vec_mult((const double*)Ad, NUM_STATES, NUM_STATES, s_phys_ptr->current_phys_state, Ad_x);

    mat_vec_mult((const double*)Bd, NUM_STATES, NUM_INPUTS, s_tasks_ptr->act_out, Bd_u_ctrl);
    //mat_vec_mult((const double*)Fd, NUM_STATES, NUM_FEEDFORWARD_INPUTS, s_tasks_ptr->network_ca_ff_rec, ay_des);
    mat_vec_mult((const double*)Fd, NUM_STATES, NUM_FEEDFORWARD_INPUTS, ff_ref_input, ay_des);

    vec_add(Ad_x, Bd_u_ctrl, NUM_STATES, temp_state);
    vec_add(temp_state, ay_des, NUM_STATES, s_phys_ptr->current_phys_state);

    // Add Gaussian noise to the state
    if (s_phys_ptr->physical_noise) {
        for (int j = 0; j < NUM_STATES; j++) {
            // Box-Muller transform
            double u1 = (double)rand() / (RAND_MAX + 1.0);
            double u2 = (double)rand() / (RAND_MAX + 1.0);
            double z0 = sqrt(-2.0 * log(u1)) * cos(2.0 * M_PI * u2);

            // Scale by the standard deviation
            s_phys_ptr->current_phys_state[j] += z0 * s_phys_ptr->st_dev_noise[j] + s_phys_ptr->mean_noise[j];
        }
    }

    if (s_phys_ptr->step % 100 == 0) { // as frequent as estimator step
        // Update in-local-platform calculated performance
        computePerformance(perf_metric_int_ptr, s_phys_ptr->current_phys_state, s_tasks_ptr->network_ca_ff_rec);
        // Update real performance
        computePerformance(perf_metric_real_ptr, s_phys_ptr->current_phys_state, ff_ref_input);
    }

    // Increment time and step
    s_phys_ptr->current_time += SIM_STEP;
    s_phys_ptr->step += 1; // Increment by 1 for each internal simulation step
}

// Process all activation and completion triggers of all tasks
static void process_trigger_events(
    const double K_TRC_FB[NUM_INPUTS][NUM_STATES],
    const double K_TRC_FF[NUM_INPUTS][NUM_FEEDFORWARD_INPUTS],
    const double K_YRC_X[NUM_INPUTS][NUM_STATES],
    const double K_YRC_PSI[NUM_INPUTS][NUM_INPUTS],
    const double E[NUM_STATES][NUM_SENSED_STATES],
    const double S[NUM_SENSED_STATES][NUM_STATES],
    TaskChainsVariables* s_tasks_ptr,
    const SimPhysicalVariables* s_phys_ptr,
    const SimTriggerVariables* s_trig_ptr, // Current input trigger states
    NetworkBuffer* network_sc_buffer,      // Network buffer for sensor->controller
    NetworkBuffer* network_ca_buffer,      // Network buffer for controller->actuator
    const double ff_ref_input[NUM_FEEDFORWARD_INPUTS], // Direct input for feedforward reference
    PerformanceMetrics* perf_metric_remote_platform_ptr
) {
    // --- Process Triggered Events using flags from s_trig_ptr ---
    // First all concluded jobs: performing copy out of the computed/input values
    // Sensor
    if (s_trig_ptr->sensor_trigger_finished) {
        vec_copy(s_tasks_ptr->sens_comp, s_tasks_ptr->sens_out, NUM_SENSED_STATES);
    }
    // Network sensor to remote platform (Receive)
    if (s_trig_ptr->network_sc_trigger_rec) {
        networkBufferPop(network_sc_buffer, s_tasks_ptr->network_sc_rec);
    }
    // Estimator
    if (s_trig_ptr->estimator_trigger_finished) {
        vec_copy(s_tasks_ptr->est_states_comp, s_tasks_ptr->est_states_out, NUM_STATES);
        vec_copy(s_tasks_ptr->est_vel_in, s_tasks_ptr->est_vel_out, 1); // velocity
    }
    // Controller
    if (s_trig_ptr->controller_trigger_finished) {
        vec_copy(s_tasks_ptr->fb_psi_dot_comp, s_tasks_ptr->fb_psi_dot_out, NUM_INPUTS);
    }
    // Feedforward
    if (s_trig_ptr->feedforward_trigger_finished) {
        vec_copy(s_tasks_ptr->ff_comp, s_tasks_ptr->ff_out, NUM_FEEDFORWARD_INPUTS);
        vec_copy(s_tasks_ptr->ff_psi_dot_comp, s_tasks_ptr->ff_psi_dot_out, NUM_INPUTS);
    }
    // merger
    if (s_trig_ptr->merger_trigger_finished) {
        vec_copy(s_tasks_ptr->agg_delta_comp, s_tasks_ptr->agg_delta_out, NUM_INPUTS);
    }
    // Network remote platform to actuator (Receive)
    if (s_trig_ptr->network_ca_trigger_rec) {
        double temp_packet[NETWORK_CA_PACKET_SIZE];
        if (networkBufferPop(network_ca_buffer, temp_packet)) {
            // De-serialize the packet
            vec_copy(temp_packet, s_tasks_ptr->network_ca_fb_rec, NUM_INPUTS);
            vec_copy(temp_packet + NUM_INPUTS, s_tasks_ptr->network_ca_ff_rec, NUM_FEEDFORWARD_INPUTS);
        }
    }
    // Actuator
    if (s_trig_ptr->actuator_trigger_finished) {
        vec_copy(s_tasks_ptr->act_in, s_tasks_ptr->act_out, NUM_INPUTS);
    }

    // Then all activated jobs: performing all computations
    // Sensor
    if (s_trig_ptr->sensor_trigger_activated) {
        double temp_state[NUM_SENSED_STATES];
        double vector_velocity[NUM_SENSED_STATES] = {0.0, 0.0, 0.0, 0.0, s_phys_ptr->velocity_param};
        mat_vec_mult((const double*)S, NUM_SENSED_STATES, NUM_STATES, s_phys_ptr->current_phys_state, temp_state);
        vec_add(temp_state, vector_velocity, NUM_SENSED_STATES, s_tasks_ptr->sens_comp); // Creates vector [phi_dot, delta, delta_dot, e_y, v]
    }
    // Network sensor to remote platform (Send)
    if (s_trig_ptr->network_sc_trigger_sent) { 
        networkBufferPush(network_sc_buffer, s_tasks_ptr->sens_out);
    }
    // Estimator plus remote platform-based performance measurement
    if (s_trig_ptr->estimator_trigger_activated) {
        double temp_est_state[NUM_STATES];
        mat_vec_mult((const double*)E, NUM_STATES, NUM_SENSED_STATES, s_tasks_ptr->network_sc_rec, temp_est_state); // Creates vector [phi_dot, beta, delta, delta_dot, e_y, e_y_dot]
        const double velocity = s_tasks_ptr->network_sc_rec[VELOCITY_SENSED_STATE_INDEX];
        vec_copy(&velocity, s_tasks_ptr->est_vel_in, 1);
        
        // Estimate derivative of lateral error
        const double phi_dot_veh = temp_est_state[0];
        const double error_y_curr = temp_est_state[ERROR_STATE_INDEX];

        const double error_y_prec = s_tasks_ptr->est_states_comp[ERROR_STATE_INDEX];
        const double error_dot_y_est_prec = s_tasks_ptr->est_states_comp[ERROR_DOT_STATE_INDEX];

        const double error_ddot_y_est = (s_tasks_ptr->ff_out[0] + s_tasks_ptr->ff_out[1]*ELL_P)*velocity*velocity - phi_dot_veh*velocity;
        double error_dot_y_est = 0.98*(error_dot_y_est_prec + error_ddot_y_est*EST_STEP) + 0.02*(error_y_curr-error_y_prec)/EST_STEP;

        error_dot_y_est = 0.95*error_dot_y_est + 0.05*error_dot_y_est_prec;
        
        // Build full state (except beta -- not used)
        double error_y_est_vec[NUM_STATES] = {0.0, 0.0, 0.0, 0.0, 0.0, error_dot_y_est};
        vec_add(temp_est_state, error_y_est_vec, NUM_STATES, s_tasks_ptr->est_states_comp);

        // Update performance
        computePerformance(perf_metric_remote_platform_ptr, s_tasks_ptr->est_states_comp, s_tasks_ptr->ff_out);
    }
    // Controller (Feedback part)
    if (s_trig_ptr->controller_trigger_activated) {
        // Adapt controller to measured velocity
        const double velocity = s_tasks_ptr->est_vel_out[0];
        double K_TRC_FB_vel[NUM_INPUTS][NUM_STATES];
        memcpy((void*)K_TRC_FB_vel, K_TRC_FB, NUM_INPUTS * NUM_STATES * sizeof(double));

        K_TRC_FB_vel[0][ERROR_STATE_INDEX] = K_TRC_FB[0][ERROR_STATE_INDEX]/velocity;
        K_TRC_FB_vel[0][ERROR_DOT_STATE_INDEX] = K_TRC_FB[0][ERROR_DOT_STATE_INDEX]/velocity;

        mat_vec_mult((const double*)K_TRC_FB_vel, NUM_INPUTS, NUM_STATES, s_tasks_ptr->est_states_out, s_tasks_ptr->fb_psi_dot_comp);
    }
    // Feedforward
    if (s_trig_ptr->feedforward_trigger_activated) {
        vec_copy(ff_ref_input, s_tasks_ptr->ff_comp, NUM_FEEDFORWARD_INPUTS); // This is a proxy for an online trajectory planner function
        
        // Adapt controller to measured velocity
        const double velocity = s_tasks_ptr->est_vel_out[0];
        double K_TRC_FF_vel[NUM_INPUTS][NUM_FEEDFORWARD_INPUTS];
        memcpy((void*)K_TRC_FF_vel, K_TRC_FF, NUM_INPUTS * NUM_FEEDFORWARD_INPUTS * sizeof(double));

        K_TRC_FF_vel[0][0] = K_TRC_FF[0][0]*velocity;
        K_TRC_FF_vel[0][1] = K_TRC_FF[0][1]*velocity + 2*d_TRC/omega_TRC*velocity*velocity;

        mat_vec_mult((const double*)K_TRC_FF_vel, NUM_INPUTS, NUM_FEEDFORWARD_INPUTS, s_tasks_ptr->ff_comp, s_tasks_ptr->ff_psi_dot_comp);
    }
    // merger 
    if (s_trig_ptr->merger_trigger_activated) {
        double delta_state[NUM_INPUTS];
        double delta_psi[NUM_INPUTS];
        double psi_dot[NUM_INPUTS];

        // Combine feedback and feedforward yaw rate commands
        vec_add(s_tasks_ptr->ff_psi_dot_out, s_tasks_ptr->fb_psi_dot_out, NUM_INPUTS, psi_dot);

        // Adapt control matrices to current speed
        const double velocity = s_tasks_ptr->est_vel_out[0];
        #define L_TOT 2.63
        #define V_CH 32.71
        double K_FF_YRC = (L_TOT)*(1+velocity*velocity/(V_CH*V_CH))/velocity;

        double K_YRC_PSI_vel[NUM_INPUTS][NUM_INPUTS];
        memcpy((void*)K_YRC_PSI_vel, K_YRC_PSI, NUM_INPUTS * NUM_INPUTS * sizeof(double));
        K_YRC_PSI_vel[0][0] = K_FF_YRC;

        double K_YRC_X_vel[NUM_INPUTS][NUM_STATES];
        memcpy((void*)K_YRC_X_vel, K_YRC_X, NUM_INPUTS * NUM_STATES * sizeof(double));
        K_YRC_X_vel[0][0] = -K_FF_YRC;
        
        // Convert total yaw rate command to steering angle delta using yaw rate controller gains
        mat_vec_mult((const double*)K_YRC_PSI_vel, NUM_INPUTS, NUM_INPUTS, psi_dot, delta_psi);
        mat_vec_mult((const double*)K_YRC_X_vel, NUM_INPUTS, NUM_STATES, s_tasks_ptr->est_states_out, delta_state);

        vec_add(delta_psi, delta_state, NUM_INPUTS, s_tasks_ptr->agg_delta_comp);
    }
    // Network remote platform to actuator (Send)
    if (s_trig_ptr->network_ca_trigger_sent) {
        // Serialize the packet before pushing
        double temp_packet[NETWORK_CA_PACKET_SIZE];
        vec_copy(s_tasks_ptr->agg_delta_out, temp_packet, NUM_INPUTS);
        vec_copy(s_tasks_ptr->ff_out, temp_packet + NUM_INPUTS, NUM_FEEDFORWARD_INPUTS);
        networkBufferPush(network_ca_buffer, temp_packet);
    }
    // Actuator
    if (s_trig_ptr->actuator_trigger_activated) {
        vec_copy(s_tasks_ptr->network_ca_fb_rec, s_tasks_ptr->act_in, NUM_INPUTS);
    }
}


static fmi2Status calculate_matrices_from_velocity(double v,
                                                   double Ad_out[NUM_STATES][NUM_STATES],
                                                   double Bd_out[NUM_STATES][NUM_INPUTS],
                                                   double Fd_out[NUM_STATES][NUM_FEEDFORWARD_INPUTS]) {

    if (v < 0.1) { //If velocity is very low, return an error or use a default matrix.
        // For simplicity, let's use a safe value.
        v = 0.1;
    }

    double v_2 = v * v;
    double v_3 = v_2 * v;
    double v_4 = v_3 * v;

    // Matrices discretized with sim_step = 0.1 ms

    // --- Calculate Ad matrix elements ---
    Ad_out[0][0] =  (1.0e-39*(9.9999992441078955351940749096684e+38*v_3 - 8.6768465022281584351278202058998e+36*v_2 + 4.2124606929881317331568202355641e+34*v - 164182153467125542443784064573640.0))/v_3;
    Ad_out[0][1] = (5.0e-39*(3.0235683429071821652633911270414e+35*v_2 - 4.2861764753450482293931772526108e+33*v + 32355623984465959638276476642571.0))/v_2;
    Ad_out[0][2] = -(1.0e-40*(- 2.5432883195577070042925349468987e+37*v_2 + 5.3015196214781872653199271128166e+34*v + 184623856390639293241490289862460.0))/v_2;
    Ad_out[0][3] = 0.00000012703796317370639392241424194679 - 0.00000000017671732071593959522513550294105/v;
    Ad_out[0][4] = 0.0;
    Ad_out[0][5] = 0.0;

    Ad_out[1][0] = (1.0e-39*(- 9.9999997480359650758140411931185e+34*v_4 + 1.4175887167900519739149878262197e+33*v_3 + 5.9277746027191436000576660869639e+36*v_2 - 8.4031617740317372385689576648245e+34*v + 634340522853345862857055293912660.0))/v_4;
    Ad_out[1][1] = (5.0e-38*(1.9999998488215791070388149819337e+37*v_3 - 3.9349851380969314496870614105301e+35*v_2 + 3.960642809890990213329831082234e+33*v - 26822037073326284723020598414963.0))/v_3;
    Ad_out[1][2] = -(2.0e-39*(63582235738255002825961481518341.0*v_3 - 3.7917752618272791612083441804878e+36*v_2 + 3.3532442397341284527102767132334e+34*v - 214678405839896777788328425575140.0))/v_3;
    Ad_out[1][3] = -(1.0e-44*(423881571588366777573335346883740.0*v_2 - 3.788004105039474223633882976614e+37*v + 2.2354961598227519865958923852818e+35))/v_2;
    Ad_out[1][4] = 0.0;
    Ad_out[1][5] = 0.0;

    Ad_out[2][0] = 0.0;
    Ad_out[2][1] = 0.0;
    Ad_out[2][2] =  0.99999876752102034860314461184316;
    Ad_out[2][3] = 0.00009985088133244983822200263601232;
    Ad_out[2][4] = 0.0;
    Ad_out[2][5] = 0.0;

    Ad_out[3][0] = 0.0;
    Ad_out[3][1] = 0.0;
    Ad_out[3][2] = -0.024637332671482679857799524825168;
    Ad_out[3][3] = 0.99701869799747722122873483385774;
    Ad_out[3][4] = 0.0;
    Ad_out[3][5] = 0.0;

    Ad_out[4][0] = 0.000000000014461412392027089250753046900418 - 0.0000000050000000000000001046128041506424*v;
    Ad_out[4][1] = -0.0000000000025196403492418600449568564397402*v;
    Ad_out[4][2] = -0.0000000000042388157158836677757333534688374*v;
    Ad_out[4][3] = 0.0;
    Ad_out[4][4] = 1.0;
    Ad_out[4][5] = 0.0001;

    Ad_out[5][0] = -(2.0e-41*(4.9999998740179825379070205965592e+36*v_2 - 2.1692118588040635977685314118163e+34*v + 70207678216468862219280337259401.0))/v;
    Ad_out[5][1] = 0.00000000071436274589084149591834829590696 - 0.000000075589210477255787642064449247115*v;
    Ad_out[5][2] = 0.00000000017671732071593959522513550294105 - 0.00000012716447147651000565192296303668*v;
    Ad_out[5][3] = -0.0000000000042388157158836677757333534688374*v;
    Ad_out[5][4] = 0.0;
    Ad_out[5][5] = 1.0; 

    // --- Calculate Bd matrix elements --- 
    Bd_out[0][0] = 0.000000001045110377136694002085510438356 - 0.000000000001090086377103607923645469856602/v;
    Bd_out[1][0] = -(2.0e-46*(130736343450025143032735776971930.0*v_2 - 1.5581493515446014309924985743344e+37*v + 6.894864351772649721929665510057e+34))/v_2;
    Bd_out[2][0] = 0.0000012324796416882199907643843928007;
    Bd_out[3][0] = 0.0246373326592935516787807870287;
    Bd_out[4][0] = 0.0;
    Bd_out[5][0] = -0.000000000000026147268690005028606547155394386*v;

    // --- Calculate Fd matrix elements ---
    Fd_out[0][0] = 0.0;
    Fd_out[0][1] = 0.0;

    Fd_out[1][0] = 0.0;
    Fd_out[1][1] = 0.0;

    Fd_out[2][0] = 0.0;
    Fd_out[2][1] = 0.0;

    Fd_out[3][0] = 0.0;
    Fd_out[3][1] = 0.0;

    Fd_out[4][0] = 0.000000005*v_2;
    Fd_out[4][1] = 0.000000001*v_2;

    Fd_out[5][0] = 0.0001*v_2;
    Fd_out[5][1] = 0.00002*v_2;

    return fmi2OK;
}

// FMI Model Instance structure
typedef struct {
    fmi2Component comp;
    fmi2String instanceName;
    fmi2Type type;
    fmi2String fmuResourceLocation;
    const fmi2CallbackFunctions* callbacks;
    fmi2Boolean loggingOn;

    fmi2Real startTime;
    fmi2Real stopTime;
    fmi2Real currentTime;
    fmi2Boolean isCoSimulation;

    // Internal data structures
    TaskChainsVariables s_taskchains;
    SimTriggerVariables s_triggers;
    SimPhysicalVariables s_phys;
    NetworkBuffer network_sc_buffer;
    NetworkBuffer network_ca_buffer;

    // Model parameters (matrices and scalar sim_step)
    double Ad_param[NUM_STATES][NUM_STATES];
    double Bd_param[NUM_STATES][NUM_INPUTS];
    double Fd_param[NUM_STATES][NUM_FEEDFORWARD_INPUTS];
    double K_trc_fb_param[NUM_INPUTS][NUM_STATES];
    double K_trc_ff_param[NUM_INPUTS][NUM_FEEDFORWARD_INPUTS];
    double K_yrc_x_param[NUM_INPUTS][NUM_STATES];
    double K_yrc_psi_param[NUM_INPUTS][NUM_INPUTS];
    double S_param[NUM_SENSED_STATES][NUM_STATES];
    double E_param[NUM_STATES][NUM_SENSED_STATES];

    // Current inputs for feedforward reference (set via fmi2SetReal)
    double ff_ref_current_input[NUM_FEEDFORWARD_INPUTS]; // Holds the current ff_ref input value

    // Performance metrics data
    PerformanceMetrics perf_metrics_internal;
    PerformanceMetrics perf_metrics_remote_platform;
    PerformanceMetrics perf_metrics_real;

} ModelInstance;


// --- FMI Functions Implementation ---

// Get an FMI ModelInstance from fmi2Component
static ModelInstance* getModelInstance(fmi2Component c) {
    if (c == NULL) {
        return NULL;
    }
    return (ModelInstance*)c;
}

// Logging helper (optional but good practice)
static void logMessage(fmi2ComponentEnvironment componentEnvironment, fmi2String instanceName, fmi2Status status, fmi2String category, fmi2String message) {
    if (componentEnvironment && ((ModelInstance*)componentEnvironment)->callbacks && ((ModelInstance*)componentEnvironment)->callbacks->logger) {
        ((ModelInstance*)componentEnvironment)->callbacks->logger(componentEnvironment, instanceName, status, category, message);
    } else {
        // Fallback if callbacks are not set up or logger is null
        printf("[%s] %s: %s\n", category, instanceName, message);
    }
}

// Wrapper function to call calculate_matrices_from_velocity and handle logging
static fmi2Status update_parametric_matrices(ModelInstance* comp, bool is_init) {
    fmi2Status status;
    double velocity = is_init ? comp->s_phys.v0_param : comp->s_phys.velocity_param;
    status = calculate_matrices_from_velocity(velocity, comp->Ad_param, comp->Bd_param, comp->Fd_param);

    if (status == fmi2OK) {
        logMessage(comp->comp, comp->instanceName, fmi2OK, "log", "Parametric matrices (Ad, Bd, Fd) updated based on velocity.");
        if (velocity < 0.1 - TOL) { // Check against small tolerance
            char msg[128];
            sprintf(msg, "Velocity too low for matrix calculation, using v=0.1. (Actual input was %f)", velocity);
            logMessage(comp->comp, comp->instanceName, fmi2Warning, "warn", msg);
        }
    } else {
        logMessage(comp->comp, comp->instanceName, status, "error", "Failed to calculate parametric matrices based on velocity.");
    }
    return status;
}

fmi2Component LateralMotionControl_fmi2Instantiate(fmi2String instanceName,
                                                     fmi2Type fmuType,
                                                     fmi2String fmuGUID,
                                                     fmi2String fmuResourceLocation,
                                                     const fmi2CallbackFunctions* functions,
                                                     fmi2Boolean visible,
                                                     fmi2Boolean loggingOn) {

    // Check FMI version compatibility and GUID
    if (fmuType != fmi2CoSimulation) {
        if(functions->logger) functions->logger(NULL, instanceName, fmi2Error, "error", "Only Co-Simulation is supported.");
        return NULL;
    }
    if (strcmp(fmuGUID, MODEL_GUID) != 0) {
        if(functions->logger) functions->logger(NULL, instanceName, fmi2Error, "error", "Incorrect GUID.");
        return NULL;
    }


    // Check callback functions
    if (!functions->logger || !functions->allocateMemory || !functions->freeMemory) {
        return NULL; // Essential callbacks missing
    }

    ModelInstance* comp = (ModelInstance*)functions->allocateMemory(1, sizeof(ModelInstance));
    if (!comp) return NULL;

    memset(comp, 0, sizeof(ModelInstance)); // Initialize all members to zero

    comp->comp = (fmi2Component)comp; // Self-reference
    comp->instanceName = instanceName;
    comp->type = fmuType;
    comp->fmuResourceLocation = fmuResourceLocation;
    comp->callbacks = functions;
    comp->loggingOn = loggingOn;
    comp->currentTime = 0.0;

    // Hardcoded initial conditions
    const double x0_init[NUM_STATES] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    const double st_dev_noise_init[NUM_STATES] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    const double mean_noise_init[NUM_STATES] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    const double velocity_init = 10.0;

    // Initialize physical parameters
    initializeSimPhysicalVariables(&comp->s_phys, x0_init, velocity_init, st_dev_noise_init, mean_noise_init);

    // Initialize internal structures, initially with zeros or defaults
    allocateTaskChainsVariables(&comp->s_taskchains, comp->callbacks->allocateMemory);
    resetTaskChainsVariables(&comp->s_taskchains, &comp->s_phys.v0_param);
    initializeSimTriggerVariables(&comp->s_triggers);
    initializeNetworkBuffer(&comp->network_sc_buffer, NETWORK_BUFFER_SIZE, NUM_SENSED_STATES, comp->callbacks->allocateMemory);
    initializeNetworkBuffer(&comp->network_ca_buffer, NETWORK_BUFFER_SIZE, NETWORK_CA_PACKET_SIZE, comp->callbacks->allocateMemory);

    // Initialize performance variables
    initializePerformanceMetrics(&comp->perf_metrics_internal);
    initializePerformanceMetrics(&comp->perf_metrics_remote_platform);
    initializePerformanceMetrics(&comp->perf_metrics_real);

    // Initial call to update matrices based on default velocity
    update_parametric_matrices(comp, true);

    // Hardcode S and E matrix
    memcpy((void*)comp->S_param, HARDCODED_S, NUM_SENSED_STATES * NUM_STATES * sizeof(double));
    memcpy((void*)comp->E_param, HARDCODED_EST, NUM_STATES * NUM_SENSED_STATES * sizeof(double));

    // Initialize current input values
    memset(comp->ff_ref_current_input, 0, NUM_FEEDFORWARD_INPUTS * sizeof(double));

    logMessage(comp->comp, instanceName, fmi2OK, "log", "fmi2Instantiate successful.");
    return comp->comp;
}

void LateralMotionControl_fmi2FreeInstance(fmi2Component c) {
    ModelInstance* comp = getModelInstance(c);
    if (!comp) return;

    freeSimControllerVariables(&comp->s_taskchains, comp->callbacks->freeMemory);
    freeNetworkBuffer(&comp->network_sc_buffer, comp->callbacks->freeMemory);
    freeNetworkBuffer(&comp->network_ca_buffer, comp->callbacks->freeMemory);
    
    logMessage(c, comp->instanceName, fmi2OK, "log", "fmi2FreeInstance successful.");
    comp->callbacks->freeMemory(comp);
}

fmi2Status LateralMotionControl_fmi2SetupExperiment(fmi2Component c,
                                                    fmi2Boolean toleranceDefined,
                                                    fmi2Real tolerance,
                                                    fmi2Real startTime,
                                                    fmi2Boolean stopTimeDefined,
                                                    fmi2Real stopTime) {
    ModelInstance* comp = getModelInstance(c);
    if (!comp) return fmi2Error;

    comp->startTime = startTime;
    comp->stopTime = stopTimeDefined ? stopTime : 0.0; // Default if not defined
    comp->currentTime = startTime;
    comp->s_phys.current_time = startTime; // Sync internal time
    comp->s_phys.step = 0; // Reset step counter

    logMessage(c, comp->instanceName, fmi2OK, "log", "fmi2SetupExperiment successful.");
    return fmi2OK;
}

fmi2Status LateralMotionControl_fmi2EnterInitializationMode(fmi2Component c) {
    ModelInstance* comp = getModelInstance(c);
    if (!comp) return fmi2Error;

    logMessage(c, comp->instanceName, fmi2OK, "log", "fmi2EnterInitializationMode successful.");
    return fmi2OK;
}

fmi2Status LateralMotionControl_fmi2ExitInitializationMode(fmi2Component c) {
    ModelInstance* comp = getModelInstance(c);
    if (!comp) return fmi2Error;

    // Matrices might need updating if init_velocity parameter was changed
    update_parametric_matrices(comp, true);

    // Reset task variables to a clean state before applying final initial values.
    resetTaskChainsVariables(&comp->s_taskchains, &comp->s_phys.v0_param);

    // Update initial state with x0_param which would have been set via fmi2SetReal
    vec_copy(comp->s_phys.x0_param, comp->s_phys.current_phys_state, NUM_STATES);
    
    // Calculate the initial sensed state based on the actual initial state
    double initial_sensed_state[NUM_SENSED_STATES] = {0.0};
    mat_vec_mult((const double*)comp->S_param, NUM_SENSED_STATES, NUM_STATES, comp->s_phys.current_phys_state, initial_sensed_state);
    initial_sensed_state[VELOCITY_SENSED_STATE_INDEX] = comp->s_phys.v0_param;

    // Propagate this initial state through the chain
    vec_copy(initial_sensed_state, comp->s_taskchains.sens_comp, NUM_SENSED_STATES);
    vec_copy(initial_sensed_state, comp->s_taskchains.sens_out, NUM_SENSED_STATES);
    vec_copy(initial_sensed_state, comp->s_taskchains.network_sc_rec, NUM_SENSED_STATES);
    vec_copy(comp->s_phys.x0_param, comp->s_taskchains.est_states_comp, NUM_STATES);
    vec_copy(comp->s_phys.x0_param, comp->s_taskchains.est_states_out, NUM_STATES);

    // Re-initialize s_triggers to default (all fmi2False) before simulation starts
    initializeSimTriggerVariables(&comp->s_triggers);
    // Re-initialize network buffers
    initializeNetworkBuffer(&comp->network_sc_buffer, NETWORK_BUFFER_SIZE, NUM_SENSED_STATES, comp->callbacks->allocateMemory);
    initializeNetworkBuffer(&comp->network_ca_buffer, NETWORK_BUFFER_SIZE, NETWORK_CA_PACKET_SIZE, comp->callbacks->allocateMemory);

    // Initialize current ff_ref input to zero
    memset(comp->ff_ref_current_input, 0, NUM_FEEDFORWARD_INPUTS * sizeof(double));

    // Initialize performance windows on exit initialization
    initializePerformanceMetrics(&comp->perf_metrics_internal);
    initializePerformanceMetrics(&comp->perf_metrics_remote_platform);
    initializePerformanceMetrics(&comp->perf_metrics_real);

    logMessage(c, comp->instanceName, fmi2OK, "log", "fmi2ExitInitializationMode successful.");
    return fmi2OK;
}

fmi2Status LateralMotionControl_fmi2DoStep(fmi2Component c,
                                            fmi2Real      currentCommunicationPoint,
                                            fmi2Real      communicationStepSize,
                                            fmi2Boolean   noSetFMUStatePriorToCurrentPoint) {
    ModelInstance* comp = getModelInstance(c);
    if (!comp) return fmi2Error;

    // Check if the FMU's internal time matches the communication point
    if (fabs(currentCommunicationPoint - comp->currentTime) > TOL) { 
        logMessage(c, comp->instanceName, fmi2Error, "error", "fmi2DoStep: currentCommunicationPoint does not match internal currentTime.");
        return fmi2Error;
    }

    // Ensure that communicationStepSize is either zero or positive (not going back in time)
    if (communicationStepSize < 0.0) {
        logMessage(c, comp->instanceName, fmi2Error, "error", "fmi2DoStep: communicationStepSize cannot be a negative value.");
        return fmi2Error;
    }

    // Convert communicationStepSize from 0.1ms granularity to steps
    double communicationStepSizeNormalized = 10000.0*communicationStepSize;

    if ((fabs(communicationStepSizeNormalized - floor(communicationStepSizeNormalized)) > TOL) && (fabs(communicationStepSizeNormalized - ceil(communicationStepSizeNormalized)) > TOL)) {
        logMessage(c, comp->instanceName, fmi2Error, "error", "fmi2DoStep: communicationStepSize must have 1 ms granularity.");
        return fmi2Error;
    }

    // Assign number of steps
    int num_steps = (int) round(communicationStepSizeNormalized);

    // The s_triggers fields (fmi2Boolean) and comp->ff_ref_current_input (fmi2Real)
    // are assumed to be set via fmi2SetBoolean and fmi2SetReal respectively,
    // before fmi2DoStep is called for this communication point.

    // Compute num_steps steps of the system dynamics if 
    if(num_steps > 0) {
        // Progress the system state for num_steps and update internal and real performance parameters
        // supposed that no triggers are occurring in this interval of time
        update_parametric_matrices(comp, false);

            for (int i = 0; i < num_steps; i++) {
        simulate_lateral_motion_control_step(
            comp->Ad_param, comp->Bd_param, comp->Fd_param,
            &comp->s_taskchains,
            &comp->s_phys,
            &comp->perf_metrics_internal,
            &comp->perf_metrics_real,
            comp->ff_ref_current_input // Pass the current ff_ref input
        );
        logMessage(c, comp->instanceName, fmi2OK, "log", "simulate_lateral_motion_control_step completed");
        }
    }
    
    // Process all triggered events that occurred during the step
    process_trigger_events(
        (const double (*)[NUM_STATES])comp->K_trc_fb_param, 
        (const double (*)[NUM_FEEDFORWARD_INPUTS])comp->K_trc_ff_param,
        (const double (*)[NUM_STATES])comp->K_yrc_x_param, 
        (const double (*)[NUM_INPUTS])comp->K_yrc_psi_param,
        (const double (*)[NUM_SENSED_STATES])comp->E_param, 
        (const double (*)[NUM_STATES])comp->S_param,
        &comp->s_taskchains,
        &comp->s_phys,
        &comp->s_triggers,
        &comp->network_sc_buffer,
        &comp->network_ca_buffer,
        comp->ff_ref_current_input,
        &comp->perf_metrics_remote_platform
    );

    // Update FMU's internal time
    comp->currentTime += communicationStepSize;

    logMessage(c, comp->instanceName, fmi2OK, "log", "fmi2DoStep successful.");
    return fmi2OK;
}

fmi2Status LateralMotionControl_fmi2GetReal(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, fmi2Real value[]) {
    ModelInstance* comp = getModelInstance(c);
    if (!comp) return fmi2Error;

    for (size_t i = 0; i < nvr; ++i) {
        fmi2ValueReference current_vr = vr[i];

        // Current state outputs
        if (current_vr >= VR_CURRENT_PHYS_STATE_START && current_vr <= VR_CURRENT_PHYS_STATE_END) {
            value[i] = comp->s_phys.current_phys_state[current_vr - VR_CURRENT_PHYS_STATE_START];
        } else if (current_vr >= VR_SENS_OUT_START && current_vr <= VR_SENS_OUT_END) {
            value[i] = comp->s_taskchains.sens_out[current_vr - VR_SENS_OUT_START];
        } else if (current_vr >= VR_EST_STATES_OUT_START && current_vr <= VR_EST_STATES_OUT_END) {
            value[i] = comp->s_taskchains.est_states_out[current_vr - VR_EST_STATES_OUT_START];
        } else if (current_vr >= VR_CNTRL_OUT_START && current_vr <= VR_CNTRL_OUT_END) {
            value[i] = comp->s_taskchains.fb_psi_dot_out[current_vr - VR_CNTRL_OUT_START];
        } else if (current_vr >= VR_FFOUT_START && current_vr <= VR_FFOUT_END) {
            value[i] = comp->s_taskchains.ff_out[current_vr - VR_FFOUT_START];
        } else if (current_vr >= VR_FFOUT_PSI_DOT_START && current_vr <= VR_FFOUT_PSI_DOT_END) {
            value[i] = comp->s_taskchains.ff_psi_dot_out[current_vr - VR_FFOUT_PSI_DOT_START];
        } else if (current_vr >= VR_AGG_OUT_START && current_vr <= VR_AGG_OUT_END) {
            value[i] = comp->s_taskchains.agg_delta_out[current_vr - VR_AGG_OUT_START];
        } else if (current_vr >= VR_ACT_OUT_START && current_vr <= VR_ACT_OUT_END) {
            value[i] = comp->s_taskchains.act_out[current_vr - VR_ACT_OUT_START];
        }
        // Time outputs
        else if (current_vr == VR_CURRENT_TIME_OUT) {
            value[i] = comp->currentTime;
        }
        // Parameters (can also be read back)
        else if (current_vr >= VR_K_TRC_FB_START && current_vr <= VR_K_TRC_FB_END) {
            int row = (current_vr - VR_K_TRC_FB_START) / NUM_STATES;
            int col = (current_vr - VR_K_TRC_FB_START) % NUM_STATES;
            value[i] = comp->K_trc_fb_param[row][col];
        } else if (current_vr >= VR_K_TRC_FF_START && current_vr <= VR_K_TRC_FF_END) {
            int row = (current_vr - VR_K_TRC_FF_START) / NUM_FEEDFORWARD_INPUTS;
            int col = (current_vr - VR_K_TRC_FF_START) % NUM_FEEDFORWARD_INPUTS;
            value[i] = comp->K_trc_ff_param[row][col];
        } else if (current_vr >= VR_K_YRC_STATE_START && current_vr <= VR_K_YRC_STATE_END) {
            int row = (current_vr - VR_K_YRC_STATE_START) / NUM_STATES;
            int col = (current_vr - VR_K_YRC_STATE_START) % NUM_STATES;
            value[i] = comp->K_yrc_x_param[row][col];
        } else if (current_vr >= VR_K_YRC_PSI_START && current_vr <= VR_K_YRC_PSI_END) {
            int row = (current_vr - VR_K_YRC_PSI_START) / NUM_INPUTS;
            int col = (current_vr - VR_K_YRC_PSI_START) % NUM_INPUTS;
            value[i] = comp->K_yrc_psi_param[row][col];
        }
        // Read ff_ref inputs
        else if (current_vr >= VR_FF_REF_START && current_vr <= VR_FF_REF_END) {
            value[i] = comp->ff_ref_current_input[current_vr - VR_FF_REF_START];
        }
        // Read initial state
        else if (current_vr >= VR_X0_START && current_vr <= VR_X0_END) {
            value[i] = comp->s_phys.x0_param[current_vr - VR_X0_START];
        }
        // Read initial velocity
        else if (current_vr == VR_INIT_VELOCITY) {
            value[i] = comp->s_phys.v0_param;
        }
        // Read current velocity
        else if (current_vr == VR_VELOCITY) {
            value[i] = comp->s_phys.velocity_param;
        }
        // Get rolling performance output
        else if (current_vr == VR_IN_LOCAL_PLATFORM_ROLLING_PERFORMANCE) {
            value[i] = comp->perf_metrics_internal.rolling_average;
        }
        else if (current_vr == VR_IN_REMOTE_PLATFORM_ROLLING_PERFORMANCE) {
            value[i] = comp->perf_metrics_remote_platform.rolling_average;
        }
        else if (current_vr == VR_REAL_ROLLING_PERFORMANCE) {
            value[i] = comp->perf_metrics_real.rolling_average;
        }
        // Get average performance output
        else if (current_vr == VR_IN_LOCAL_PLATFORM_AVERAGE_PERFORMANCE) {
            value[i] = comp->perf_metrics_internal.total_average;
        }
        else if (current_vr == VR_IN_REMOTE_PLATFORM_AVERAGE_PERFORMANCE) {
            value[i] = comp->perf_metrics_remote_platform.total_average;
        }
        else if (current_vr == VR_REAL_AVERAGE_PERFORMANCE) {
            value[i] = comp->perf_metrics_real.total_average;
        }
        else {
            logMessage(c, comp->instanceName, fmi2Error, "error", "fmi2GetReal: Unknown value reference.");
            return fmi2Error;
        }
    }
    return fmi2OK;
}

fmi2Status LateralMotionControl_fmi2SetReal(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, const fmi2Real value[]) {
    ModelInstance* comp = getModelInstance(c);
    if (!comp) return fmi2Error;

    for (size_t i = 0; i < nvr; ++i) {
        fmi2ValueReference current_vr = vr[i];

        // Parameters
        if (current_vr >= VR_K_TRC_FB_START && current_vr <= VR_K_TRC_FB_END) {
            int row = (current_vr - VR_K_TRC_FB_START) / NUM_STATES;
            int col = (current_vr - VR_K_TRC_FB_START) % NUM_STATES;
            comp->K_trc_fb_param[row][col] = value[i];
        } else if (current_vr >= VR_K_TRC_FF_START && current_vr <= VR_K_TRC_FF_END) {
            int row = (current_vr - VR_K_TRC_FF_START) / NUM_FEEDFORWARD_INPUTS;
            int col = (current_vr - VR_K_TRC_FF_START) % NUM_FEEDFORWARD_INPUTS;
            comp->K_trc_ff_param[row][col] = value[i];
        } else if (current_vr >= VR_K_YRC_STATE_START && current_vr <= VR_K_YRC_STATE_END) {
            int row = (current_vr - VR_K_YRC_STATE_START) / NUM_STATES;
            int col = (current_vr - VR_K_YRC_STATE_START) % NUM_STATES;
            comp->K_yrc_x_param[row][col] = value[i];
        } else if (current_vr >= VR_K_YRC_PSI_START && current_vr <= VR_K_YRC_PSI_END) {
            int row = (current_vr - VR_K_YRC_PSI_START) / NUM_INPUTS;
            int col = (current_vr - VR_K_YRC_PSI_START) % NUM_INPUTS;
            comp->K_yrc_psi_param[row][col] = value[i];
        }
        else if (current_vr >= VR_NOISE_MEAN_START && current_vr <= VR_NOISE_MEAN_END) {
            comp->s_phys.mean_noise[current_vr - VR_NOISE_MEAN_START] = value[i];
        }
        else if (current_vr >= VR_NOISE_STD_DEV_START && current_vr <= VR_NOISE_STD_DEV_END) {
            comp->s_phys.st_dev_noise[current_vr - VR_NOISE_STD_DEV_START] = value[i];
        }
        // Set ff_ref inputs
        else if (current_vr >= VR_FF_REF_START && current_vr <= VR_FF_REF_END) {
            comp->ff_ref_current_input[current_vr - VR_FF_REF_START] = value[i];
        }
        // Set initial state
        else if (current_vr >= VR_X0_START && current_vr <= VR_X0_END) {
            comp->s_phys.x0_param[current_vr - VR_X0_START] = value[i];
        }
        // Set initial velocity
        else if (current_vr == VR_INIT_VELOCITY) {
            comp->s_phys.v0_param = value[i];
            //update_parametric_matrices(comp, true);
        }
        // Set Velocity
        else if (current_vr == VR_VELOCITY) {
            if (fabs(comp->s_phys.velocity_param - value[i]) > TOL) { // Only update if value changed significantly
                comp->s_phys.velocity_param = value[i];
                // Recalculate matrices when velocity changes
                //update_parametric_matrices(comp, false);
            }
        }
        else {
            logMessage(c, comp->instanceName, fmi2Error, "error", "fmi2SetReal: Unknown value reference.");
            return fmi2Error;
        }
    }
    return fmi2OK;
}




fmi2Status LateralMotionControl_fmi2GetInteger(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, fmi2Integer value[]) {
    ModelInstance* comp = getModelInstance(c);
    if (!comp) return fmi2Error;

    for (size_t i = 0; i < nvr; ++i) {
        fmi2ValueReference current_vr = vr[i];
        if (current_vr == VR_CURRENT_STEP_OUT) {
            value[i] = comp->s_phys.step;
        }
        else if (current_vr == VR_IN_LOCAL_PLATFORM_THRESHOLD_ERROR_CNTR) {
            value[i] = comp->perf_metrics_internal.num_violations;
        }
        else if (current_vr == VR_IN_REMOTE_PLATFORM_THRESHOLD_ERROR_CNTR) {
            value[i] = comp->perf_metrics_remote_platform.num_violations;
        }
        else if (current_vr == VR_REAL_THRESHOLD_ERROR_CNTR) {
            value[i] = comp->perf_metrics_real.num_violations;
        }
        else {
            logMessage(c, comp->instanceName, fmi2Error, "error", "fmi2GetInteger: Unknown value reference.");
            return fmi2Error;
        }
    }
    return fmi2OK;
}

fmi2Status LateralMotionControl_fmi2SetInteger(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, const fmi2Integer value[]) {
    ModelInstance* comp = getModelInstance(c);
    if (!comp) return fmi2Error;

    for (size_t i = 0; i < nvr; ++i) {
        fmi2ValueReference current_vr = vr[i];
        // No integer parameters to set currently
        logMessage(c, comp->instanceName, fmi2Error, "error", "fmi2SetInteger: Unknown or unsupported value reference.");
        return fmi2Error;
    }
    return fmi2OK;
}


fmi2Status LateralMotionControl_fmi2GetBoolean(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, fmi2Boolean value[]) {
    ModelInstance* comp = getModelInstance(c);
    if (!comp) return fmi2Error;

    for (size_t i = 0; i < nvr; ++i) {
        fmi2ValueReference current_vr = vr[i];
        if (current_vr == VR_SENSOR_TRIGGER_ACTIVATED_INPUT) {
            value[i] = comp->s_triggers.sensor_trigger_activated;
        } else if (current_vr == VR_SENSOR_TRIGGER_FINISHED_INPUT) {
            value[i] = comp->s_triggers.sensor_trigger_finished;
        } else if (current_vr == VR_NETWORK_SC_TRIGGER_SENT_INPUT) {
            value[i] = comp->s_triggers.network_sc_trigger_sent;
        } else if (current_vr == VR_NETWORK_SC_TRIGGER_RECEIVED_INPUT) {
            value[i] = comp->s_triggers.network_sc_trigger_rec;
        } else if (current_vr == VR_ESTIMATOR_TRIGGER_ACTIVATED_INPUT) {
            value[i] = comp->s_triggers.estimator_trigger_activated;
        } else if (current_vr == VR_ESTIMATOR_TRIGGER_FINISHED_INPUT) {
            value[i] = comp->s_triggers.estimator_trigger_finished;
        } else if (current_vr == VR_CONTROLLER_TRIGGER_ACTIVATED_INPUT) {
            value[i] = comp->s_triggers.controller_trigger_activated;
        } else if (current_vr == VR_CONTROLLER_TRIGGER_FINISHED_INPUT) {
            value[i] = comp->s_triggers.controller_trigger_finished;
        } else if (current_vr == VR_FEEDFORWARD_TRIGGER_ACTIVATED_INPUT) {
            value[i] = comp->s_triggers.feedforward_trigger_activated;
        } else if (current_vr == VR_FEEDFORWARD_TRIGGER_FINISHED_INPUT) {
            value[i] = comp->s_triggers.feedforward_trigger_finished;
        } else if (current_vr == VR_MERGER_TRIGGER_ACTIVATED_INPUT) {
            value[i] = comp->s_triggers.merger_trigger_activated;
        } else if (current_vr == VR_MERGER_TRIGGER_FINISHED_INPUT) {
            value[i] = comp->s_triggers.merger_trigger_finished;
        } else if (current_vr == VR_NETWORK_CA_TRIGGER_SENT_INPUT) {
            value[i] = comp->s_triggers.network_ca_trigger_sent;
        } else if (current_vr == VR_NETWORK_CA_TRIGGER_RECEIVED_INPUT) {
            value[i] = comp->s_triggers.network_ca_trigger_rec;
        } else if (current_vr == VR_ACTUATOR_TRIGGER_ACTIVATED_INPUT) {
            value[i] = comp->s_triggers.actuator_trigger_activated;
        } else if (current_vr == VR_ACTUATOR_TRIGGER_FINISHED_INPUT) {
            value[i] = comp->s_triggers.actuator_trigger_finished;
        } else if (current_vr == VR_IN_LOCAL_PLATFORM_CRITICAL_SECTION) {
            value[i] = comp->perf_metrics_internal.critical_section;
        } else if (current_vr == VR_IN_REMOTE_PLATFORM_CRITICAL_SECTION) {
            value[i] = comp->perf_metrics_remote_platform.critical_section;
        } else if (current_vr == VR_REAL_CRITICAL_SECTION) {
            value[i] = comp->perf_metrics_real.critical_section;
        } else if (current_vr == VR_IN_LOCAL_PLATFORM_VIOLATED_CONSTRAINT) {
            value[i] = comp->perf_metrics_internal.violated_threshold;
        } else if (current_vr == VR_IN_REMOTE_PLATFORM_VIOLATED_CONSTRAINT) {
            value[i] = comp->perf_metrics_remote_platform.violated_threshold;
        } else if (current_vr == VR_REAL_VIOLATED_CONSTRAINT) {
            value[i] = comp->perf_metrics_real.violated_threshold;
        } else {
            logMessage(c, comp->instanceName, fmi2Error, "error", "fmi2GetBoolean: Unknown value reference.");
            return fmi2Error;
        }
    }
    return fmi2OK;
}

fmi2Status LateralMotionControl_fmi2SetBoolean(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, const fmi2Boolean value[]) {
    ModelInstance* comp = getModelInstance(c);
    if (!comp) return fmi2Error;

    for (size_t i = 0; i < nvr; ++i) {
        fmi2ValueReference current_vr = vr[i];
        if (current_vr == VR_SENSOR_TRIGGER_ACTIVATED_INPUT) {
            comp->s_triggers.sensor_trigger_activated = value[i];
        } else if (current_vr == VR_SENSOR_TRIGGER_FINISHED_INPUT) {
            comp->s_triggers.sensor_trigger_finished = value[i];
        } else if (current_vr == VR_NETWORK_SC_TRIGGER_SENT_INPUT) {
            comp->s_triggers.network_sc_trigger_sent = value[i];
        } else if (current_vr == VR_NETWORK_SC_TRIGGER_RECEIVED_INPUT) {
            comp->s_triggers.network_sc_trigger_rec = value[i];
        } else if (current_vr == VR_ESTIMATOR_TRIGGER_ACTIVATED_INPUT) {
            comp->s_triggers.estimator_trigger_activated = value[i];
        } else if (current_vr == VR_ESTIMATOR_TRIGGER_FINISHED_INPUT) {
            comp->s_triggers.estimator_trigger_finished = value[i];
        } else if (current_vr == VR_CONTROLLER_TRIGGER_ACTIVATED_INPUT) {
            comp->s_triggers.controller_trigger_activated = value[i];
        } else if (current_vr == VR_CONTROLLER_TRIGGER_FINISHED_INPUT) {
            comp->s_triggers.controller_trigger_finished = value[i];
        } else if (current_vr == VR_FEEDFORWARD_TRIGGER_ACTIVATED_INPUT) {
            comp->s_triggers.feedforward_trigger_activated = value[i];
        } else if (current_vr == VR_FEEDFORWARD_TRIGGER_FINISHED_INPUT) {
            comp->s_triggers.feedforward_trigger_finished = value[i];
        } else if (current_vr == VR_MERGER_TRIGGER_ACTIVATED_INPUT) {
            comp->s_triggers.merger_trigger_activated = value[i];
        } else if (current_vr == VR_MERGER_TRIGGER_FINISHED_INPUT) {
            comp->s_triggers.merger_trigger_finished = value[i];
        } else if (current_vr == VR_NETWORK_CA_TRIGGER_SENT_INPUT) {
            comp->s_triggers.network_ca_trigger_sent = value[i];
        } else if (current_vr == VR_NETWORK_CA_TRIGGER_RECEIVED_INPUT) {
            comp->s_triggers.network_ca_trigger_rec = value[i];
        } else if (current_vr == VR_ACTUATOR_TRIGGER_ACTIVATED_INPUT) {
            comp->s_triggers.actuator_trigger_activated = value[i];
        } else if (current_vr == VR_ACTUATOR_TRIGGER_FINISHED_INPUT) {
            comp->s_triggers.actuator_trigger_finished = value[i];
        } else {
            logMessage(c, comp->instanceName, fmi2Error, "error", "fmi2SetBoolean: Unknown value reference.");
            return fmi2Error;
        }
    }
    return fmi2OK;
}

fmi2Status LateralMotionControl_fmi2Terminate(fmi2Component c) {
    ModelInstance* comp = getModelInstance(c);
    if (!comp) return fmi2Error;
    // Nothing special to do here for this model
    logMessage(c, comp->instanceName, fmi2OK, "log", "fmi2Terminate successful.");
    return fmi2OK;
}

fmi2Status LateralMotionControl_fmi2Reset(fmi2Component c) {
    ModelInstance* comp = getModelInstance(c);
    if (!comp) return fmi2Error;

    // Reset time and step counters
    comp->currentTime = comp->startTime;
    comp->s_phys.current_time = comp->startTime;
    comp->s_phys.step = 0;

    // Re-initialize physical state to the configured initial state
    vec_copy(comp->s_phys.x0_param, comp->s_phys.current_phys_state, NUM_STATES);

    // Re-initialize all task chain variables to a clean state
    resetTaskChainsVariables(&comp->s_taskchains, &comp->s_phys.v0_param);
    
    // Reset trigger inputs
    initializeSimTriggerVariables(&comp->s_triggers);
    // Reset network buffers
    initializeNetworkBuffer(&comp->network_sc_buffer, NETWORK_BUFFER_SIZE, NUM_SENSED_STATES, comp->callbacks->allocateMemory);
    initializeNetworkBuffer(&comp->network_ca_buffer, NETWORK_BUFFER_SIZE, NETWORK_CA_PACKET_SIZE, comp->callbacks->allocateMemory);

    // Reset current ff_ref input to zero
    memset(comp->ff_ref_current_input, 0, NUM_FEEDFORWARD_INPUTS * sizeof(double));

    // Reset performance metrics
    initializePerformanceMetrics(&comp->perf_metrics_internal);
    initializePerformanceMetrics(&comp->perf_metrics_remote_platform);
    initializePerformanceMetrics(&comp->perf_metrics_real);

    logMessage(c, comp->instanceName, fmi2OK, "log", "fmi2Reset successful.");
    return fmi2OK;
}

fmi2Status LateralMotionControl_fmi2SetDebugLogging(fmi2Component c, fmi2Boolean loggingOn, size_t nCategories, const fmi2String categories[]) { 
    ModelInstance* comp = getModelInstance(c);
    if (!comp) return fmi2Error;
    comp->loggingOn = loggingOn;
    return fmi2OK;
}

const char* LateralMotionControl_fmi2GetTypesPlatform() { return fmi2TypesPlatform; }
const char* LateralMotionControl_fmi2GetVersion() { return fmi2Version; }

// =====================================================================================
// FMI 2.0 UNUSED/UNSUPPORTED FUNCTIONS
// These functions are not implemented in this FMU, but must exist in the DLL.
// They return fmi2Error or fmi2Discard to indicate they are not supported.
// =====================================================================================
fmi2Status LateralMotionControl_fmi2GetString(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, fmi2String value[]) { return fmi2Error; }
fmi2Status LateralMotionControl_fmi2SetString(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, const fmi2String value[]) { return fmi2Error; }
fmi2Status LateralMotionControl_fmi2CancelStep(fmi2Component c) { return fmi2Error; }
fmi2Status LateralMotionControl_fmi2GetStatus(fmi2Component c, const fmi2StatusKind s, fmi2Status* value) { return fmi2Error; }
fmi2Status LateralMotionControl_fmi2GetRealStatus(fmi2Component c, const fmi2StatusKind s, fmi2Real* value) { return fmi2Error; }
fmi2Status LateralMotionControl_fmi2GetIntegerStatus(fmi2Component c, const fmi2StatusKind s, fmi2Integer* value) { return fmi2Error; }
fmi2Status LateralMotionControl_fmi2GetBooleanStatus(fmi2Component c, const fmi2StatusKind s, fmi2Boolean* value) { return fmi2Error; }
fmi2Status LateralMotionControl_fmi2GetStringStatus(fmi2Component c, const fmi2StatusKind s, fmi2String* value) { return fmi2Error; }
fmi2Status LateralMotionControl_fmi2GetFMUstate(fmi2Component c, fmi2FMUstate* FMUstate) { return fmi2Error; }
fmi2Status LateralMotionControl_fmi2SetFMUstate(fmi2Component c, fmi2FMUstate FMUstate) { return fmi2Error; }
fmi2Status LateralMotionControl_fmi2FreeFMUstate(fmi2Component c, fmi2FMUstate* FMUstate) { return fmi2Error; }
fmi2Status LateralMotionControl_fmi2SerializedFMUstateSize(fmi2Component c, fmi2FMUstate FMUstate, size_t* size) { return fmi2Error; }
fmi2Status LateralMotionControl_fmi2SerializeFMUstate(fmi2Component c, fmi2FMUstate FMUstate, fmi2Byte serializedState[], size_t size) { return fmi2Error; }
fmi2Status LateralMotionControl_fmi2DeSerializeFMUstate(fmi2Component c, const fmi2Byte serializedState[], size_t size, fmi2FMUstate* FMUstate) { return fmi2Error; }
fmi2Status LateralMotionControl_fmi2GetDirectionalDerivative(fmi2Component c, const fmi2ValueReference vUnknown_ref[], size_t nUnknown, const fmi2ValueReference vKnown_ref[], size_t nKnown, const fmi2Real dvKnown[], fmi2Real dvUnknown[]) { return fmi2Error; }
fmi2Status LateralMotionControl_fmi2SetRealInputDerivatives(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, const fmi2Integer order[], const fmi2Real value[]) { return fmi2Error; }
fmi2Status LateralMotionControl_fmi2GetRealOutputDerivatives(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, const fmi2Integer order[], fmi2Real value[]) { return fmi2Error; }
