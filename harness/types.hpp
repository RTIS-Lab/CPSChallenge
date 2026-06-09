#ifndef HARNESS_TYPES_HPP
#define HARNESS_TYPES_HPP

#include <string>
#include <map>
#include <vector>

namespace BoschChallenge {

enum class TaskType {
    Estimator,
    Controller,
    Feedforward,
    Merger,
    // Note: Sensor and Actuator are "Edge" tasks handled automatically by the runner
};

struct TaskInstance {
    int vehicleId;
    TaskType type;

    bool operator<(const TaskInstance& other) const {
        if (vehicleId != other.vehicleId) return vehicleId < other.vehicleId;
        return type < other.type;
    }

    bool operator==(const TaskInstance& other) const {
        return vehicleId == other.vehicleId && type == other.type;
    }
};

enum class TaskState {
    Idle,      // Not yet released this period
    Released,  // Period hit, waiting for scheduler
    Running,   // Occupying a core
    Preempted, // Timer paused, waiting for core to free
    Completed  // Execution finished, output latched, waiting for next period
};

/**
 * Performance metrics as seen by the scheduler (Remote Platform Estimates).
 * These are calculated by the Estimator task inside the FMU.
 */
struct VehicleState {
    int vehicleId;
    double estimatedLateralError;
    double estimatedRollingPerformance;
    int estimatedThresholdErrors;
    bool inCriticalSection;
    bool ffRefActive; // Trigger for Feedforward/Merger immediate activation
};

/**
 * Ground truth metrics for scoring (not seen by the scheduler in realistic mode).
 */
struct GroundTruthState {
    double realLateralError;
    double realRollingPerformance;
    double realAveragePerformance;
    int realThresholdErrors;
    bool violated;
};

} // namespace BoschChallenge

#endif // HARNESS_TYPES_HPP
