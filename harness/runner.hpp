#ifndef HARNESS_RUNNER_HPP
#define HARNESS_RUNNER_HPP

#include "types.hpp"
#include "vr.hpp"
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace BoschChallenge {

enum class ExecTimeModel { WCET, BCET, RandomUniform, PERT };

struct TaskTimingConfig {
    double period; // seconds
    double bcet;   // seconds
    double avet;   // seconds
    double wcet;   // seconds
};

struct NetworkTimingConfig {
    double bcDelay;  // seconds
    double avgDelay; // seconds
    double wcDelay;  // seconds
};

struct RunnerConfig {
    std::string fmuSoPath;
    std::string exampleDir;
    double simDuration = 240.0;
    double commStepSize = 0.0001;
    int numVehicles = 1;
    int numCloudCores = 3;
    ExecTimeModel timingModel = ExecTimeModel::WCET;

    // Task and Network configs (defaults from parameters.md)
    std::map<TaskType, TaskTimingConfig> taskConfigs;
    NetworkTimingConfig netSCConfig; // Sensor -> Cloud
    NetworkTimingConfig netCAConfig; // Cloud -> Actuator

    double sensorPeriod = 0.005;
    double actuatorPeriod = 0.030;
};

class SimulationRunner {
public:
    explicit SimulationRunner(const RunnerConfig& config);
    ~SimulationRunner();

    /**
     * Advances simulation time by commStepSize.
     * 1. Checks execution timers of running tasks.
     * 2. Checks network delay timers.
     * 3. Triggers Sensor/Actuator edge tasks.
     * 4. Releases periodic cloud tasks.
     * @return false if simDuration is reached.
     */
    bool beginStep();

    /**
     * Activates a task on a core with given priority.
     * Preempts lowest-priority task if all cores busy.
     */
    void activateTask(const TaskInstance& task, double priority);

    /**
     * Sets real-time inputs for a vehicle (ff_ref and velocity).
     */
    void setVehicleInputs(int vehicleId, double ff_ref_0, double ff_ref_1, double velocity);

    /**
     * Finalizes the step by applying all triggers to FMUs and calling fmi2DoStep.
     */
    void endStep();

    // Accessors for the scheduler
    double currentTime() const { return m_currentTime; }
    int availableCores() const { return m_availableCores; }
    const std::vector<TaskInstance>& newlyReleased() const { return m_newlyReleased; }
    const std::vector<TaskInstance>& newlyCompleted() const { return m_newlyCompleted; }
    const std::map<TaskInstance, TaskState>& taskStates() const { return m_taskStates; }
    const std::map<int, VehicleState>& vehicleStates() const { return m_vehicleStates; }
    
    // Detailed data for logging
    const std::map<int, GroundTruthState>& groundTruthStates() const { return m_groundTruthStates; }
    void getVehiclePhysState(int vehicleId, double state[6]);
    void getVehicleTriggers(int vehicleId, std::map<VR_Triggers, bool>& triggers);
    void getVehicleLastTriggers(int vehicleId, std::map<VR_Triggers, bool>& triggers);
    double getVehicleActOut(int vehicleId);

private:
    class Impl;
    std::unique_ptr<Impl> m_impl;

    RunnerConfig m_config;
    double m_currentTime = 0.0;
    int m_availableCores;

    // Internal state management
    std::map<TaskInstance, TaskState> m_taskStates;
    std::map<int, VehicleState> m_vehicleStates;
    std::map<int, GroundTruthState> m_groundTruthStates;
    
    std::vector<TaskInstance> m_newlyReleased;
    std::vector<TaskInstance> m_newlyCompleted;

    // TODO: Add FMU instance wrappers and internal timers
};

} // namespace BoschChallenge

#endif // HARNESS_RUNNER_HPP
