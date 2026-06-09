#include "runner.hpp"
#include "fmu_wrapper.hpp"
#include <cmath>
#include <algorithm>
#include <iostream>
#include <fstream>

namespace BoschChallenge {

struct InternalTaskData {
    TaskInstance instance;
    TaskState state = TaskState::Idle;
    double remainingExecTime = 0.0;
    double priority = 1000.0;
    double periodTimer = 0.0;
};

struct InternalNetworkData {
    int vehicleId;
    bool active = false;
    double remainingDelay = 0.0;
    bool pulse = false; // Latch for the 1-step RECEIVED pulse
};

class SimulationRunner::Impl {
public:
    RunnerConfig config;
    double currentTime = 0.0;
    
    std::vector<std::unique_ptr<FmuWrapper>> fmus;
    std::map<TaskInstance, InternalTaskData> taskData;
    
    std::map<int, InternalNetworkData> netSC; 
    std::map<int, InternalNetworkData> netCA;
    std::map<int, std::map<VR_Triggers, bool>> lastTriggers;

    Impl(const RunnerConfig& c) : config(c) {
        for (int i = 0; i < config.numVehicles; ++i) {
            auto fmu = std::make_unique<FmuWrapper>();
            if (!fmu->load(config.fmuSoPath)) throw std::runtime_error("Failed to load FMU SO");
            if (!fmu->instantiate("vehicle_" + std::to_string(i))) throw std::runtime_error("Failed to instantiate FMU");
            if (!fmu->setupExperiment(0.0, config.simDuration)) throw std::runtime_error("Failed to setup experiment");
            fmus.push_back(std::move(fmu));

            for (auto type : {TaskType::Estimator, TaskType::Controller, TaskType::Feedforward, TaskType::Merger}) {
                TaskInstance inst{i, type};
                double initialTimer = 0.0;
                if (config.taskConfigs.count(type)) {
                    initialTimer = config.taskConfigs.at(type).period;
                }
                taskData[inst] = {inst, TaskState::Idle, 0.0, 1000.0, initialTimer};
            }
        }
    }

    double getExecTime(TaskType type) {
        auto const& cfg = config.taskConfigs.at(type);
        switch (config.timingModel) {
            case ExecTimeModel::WCET: return cfg.wcet;
            case ExecTimeModel::BCET: return cfg.bcet;
            default: return cfg.wcet;
        }
    }

    double getNetDelay(const NetworkTimingConfig& cfg) {
        switch (config.timingModel) {
            case ExecTimeModel::WCET: return cfg.wcDelay;
            case ExecTimeModel::BCET: return cfg.bcDelay;
            default: return cfg.wcDelay;
        }
    }
};

SimulationRunner::SimulationRunner(const RunnerConfig& config) 
    : m_config(config), m_availableCores(config.numCloudCores) {
    m_impl = std::make_unique<Impl>(config);
}

SimulationRunner::~SimulationRunner() = default;

bool SimulationRunner::beginStep() {
    if (m_impl->currentTime >= m_config.simDuration - 1e-12) return false;

    m_newlyReleased.clear();
    m_newlyCompleted.clear();

    double dt = m_config.commStepSize;

    // Reset network pulses
    for (int i = 0; i < m_config.numVehicles; ++i) {
        m_impl->netSC[i].pulse = false;
        m_impl->netCA[i].pulse = false;
    }

    // 1. Advance Running tasks
    for (auto& [inst, data] : m_impl->taskData) {
        if (data.state == TaskState::Running) {
            data.remainingExecTime -= dt;
            if (data.remainingExecTime <= 1e-12) {
                data.state = TaskState::Completed;
                data.remainingExecTime = 0;
                m_newlyCompleted.push_back(inst);
                m_availableCores++;
                if (inst.type == TaskType::Merger) {
                    auto& net = m_impl->netCA[inst.vehicleId];
                    if (!net.active) {
                        net.active = true;
                        net.remainingDelay = m_impl->getNetDelay(m_config.netCAConfig);
                    }
                }
            }
        }
    }

    // 2. Advance Network Delays
    for (int i = 0; i < m_config.numVehicles; ++i) {
        auto& sc = m_impl->netSC[i];
        if (sc.active) {
            sc.remainingDelay -= dt;
            if (sc.remainingDelay <= 1e-12) {
                sc.remainingDelay = 0;
                sc.active = false;
                sc.pulse = true; // Trigger fires in endStep
            }
        }
        auto& ca = m_impl->netCA[i];
        if (ca.active) {
            ca.remainingDelay -= dt;
            if (ca.remainingDelay <= 1e-12) {
                ca.remainingDelay = 0;
                ca.active = false;
                ca.pulse = true; // Trigger fires in endStep
            }
        }
    }

    // 3. Periodic Releases
    for (auto& [inst, data] : m_impl->taskData) {
        data.periodTimer += dt;
        double period = m_config.taskConfigs.count(inst.type) ? m_config.taskConfigs.at(inst.type).period : 0.01;
        if (period > 0 && data.periodTimer >= period - 1e-12) {
            data.periodTimer = 0;
            if (data.state == TaskState::Idle) {
                data.state = TaskState::Released;
                m_newlyReleased.push_back(inst);
            }
        }
    }

    // 4. ff_ref triggers
    for (int i = 0; i < m_config.numVehicles; ++i) {
        if (m_vehicleStates[i].ffRefActive) {
            auto& ff = m_impl->taskData[{i, TaskType::Feedforward}];
            if (ff.state == TaskState::Idle) {
                ff.state = TaskState::Released;
                ff.periodTimer = 0;
                m_newlyReleased.push_back({i, TaskType::Feedforward});
            }
            auto& merg = m_impl->taskData[{i, TaskType::Merger}];
            if (merg.state == TaskState::Idle) {
                merg.state = TaskState::Released;
                merg.periodTimer = 0;
                m_newlyReleased.push_back({i, TaskType::Merger});
            }
        }
    }

    m_currentTime = m_impl->currentTime;
    m_taskStates.clear();
    for (auto const& [inst, data] : m_impl->taskData) m_taskStates[inst] = data.state;
    for (int i = 0; i < m_config.numVehicles; ++i) {
        m_impl->fmus[i]->getEstimatedState(m_vehicleStates[i]);
        m_impl->fmus[i]->getGroundTruthState(m_groundTruthStates[i]);
    }
    return true;
}

void SimulationRunner::getVehiclePhysState(int vehicleId, double state[6]) { m_impl->fmus[vehicleId]->getPhysState(state); }
void SimulationRunner::getVehicleTriggers(int vehicleId, std::map<VR_Triggers, bool>& triggers) { m_impl->fmus[vehicleId]->getTriggers(triggers); }
void SimulationRunner::getVehicleLastTriggers(int vehicleId, std::map<VR_Triggers, bool>& triggers) { triggers = m_impl->lastTriggers[vehicleId]; }
double SimulationRunner::getVehicleActOut(int vehicleId) { return m_impl->fmus[vehicleId]->getActOut(); }
void SimulationRunner::setVehicleInputs(int vehicleId, double ff_ref_0, double ff_ref_1, double velocity) { m_impl->fmus[vehicleId]->setRealInputs(ff_ref_0, ff_ref_1, velocity); }

void SimulationRunner::activateTask(const TaskInstance& task, double priority) {
    auto& data = m_impl->taskData[task];
    if (data.state != TaskState::Released && data.state != TaskState::Preempted) return;
    if (m_availableCores > 0) {
        data.state = TaskState::Running;
        data.priority = priority;
        if (data.remainingExecTime <= 0) data.remainingExecTime = m_impl->getExecTime(task.type);
        m_availableCores--;
    } else {
        InternalTaskData* lowest = nullptr;
        for (auto& [inst, d] : m_impl->taskData) {
            if (d.state == TaskState::Running) { if (!lowest || d.priority > lowest->priority) lowest = &d; }
        }
        if (lowest && priority < lowest->priority) {
            lowest->state = TaskState::Preempted;
            data.state = TaskState::Running;
            data.priority = priority;
            if (data.remainingExecTime <= 0) data.remainingExecTime = m_impl->getExecTime(task.type);
        }
    }
}

void SimulationRunner::endStep() {
    double dt = m_config.commStepSize;
    double time = m_impl->currentTime;
    long long step = (long long)(time / dt + 0.5);

    for (int i = 0; i < m_config.numVehicles; ++i) {
        std::map<VR_Triggers, bool> trigs;
        for (int v = 100; v <= 115; ++v) trigs[static_cast<VR_Triggers>(v)] = false;

        long long sSteps = (long long)(m_config.sensorPeriod / dt + 0.5);
        bool sFire = (step % sSteps == 0);
        trigs[VR_SENSOR_TRIGGER_ACTIVATED_INPUT] = sFire;
        trigs[VR_SENSOR_TRIGGER_FINISHED_INPUT] = sFire;
        
        // Network SC SENT fires when sensor fires
        trigs[VR_NETWORK_SC_TRIGGER_SENT_INPUT] = sFire;
        if (sFire) {
            auto& sc = m_impl->netSC[i];
            if (!sc.active) {
                sc.active = true;
                sc.remainingDelay = m_impl->getNetDelay(m_config.netSCConfig);
            }
        }

        auto& sc = m_impl->netSC[i];
        trigs[VR_NETWORK_SC_TRIGGER_RECEIVED_INPUT] = sc.pulse;

        auto check = [&](TaskType type, bool activated) {
            auto const& d = m_impl->taskData[{i, type}];
            if (activated) {
                bool justStarted = (d.state == TaskState::Running && std::abs(d.remainingExecTime - m_impl->getExecTime(type)) < 1e-12);
                bool zeroDelayFinished = (d.state == TaskState::Completed && m_impl->getExecTime(type) < 1e-12);
                return justStarted || zeroDelayFinished;
            } else {
                return d.state == TaskState::Completed;
            }
        };

        trigs[VR_ESTIMATOR_TRIGGER_ACTIVATED_INPUT] = check(TaskType::Estimator, true);
        trigs[VR_ESTIMATOR_TRIGGER_FINISHED_INPUT] = check(TaskType::Estimator, false);
        trigs[VR_CONTROLLER_TRIGGER_ACTIVATED_INPUT] = check(TaskType::Controller, true);
        trigs[VR_CONTROLLER_TRIGGER_FINISHED_INPUT] = check(TaskType::Controller, false);
        trigs[VR_FEEDFORWARD_TRIGGER_ACTIVATED_INPUT] = check(TaskType::Feedforward, true);
        trigs[VR_FEEDFORWARD_TRIGGER_FINISHED_INPUT] = check(TaskType::Feedforward, false);
        trigs[VR_MERGER_TRIGGER_ACTIVATED_INPUT] = check(TaskType::Merger, true);
        trigs[VR_MERGER_TRIGGER_FINISHED_INPUT] = check(TaskType::Merger, false);

        // Network CA SENT fires when merger finishes
        trigs[VR_NETWORK_CA_TRIGGER_SENT_INPUT] = check(TaskType::Merger, false);

        auto& ca = m_impl->netCA[i];
        trigs[VR_NETWORK_CA_TRIGGER_RECEIVED_INPUT] = ca.pulse;
        
        long long aSteps = (long long)(m_config.actuatorPeriod / dt + 0.5);
        bool aFire = (step % aSteps == 0);
        trigs[VR_ACTUATOR_TRIGGER_ACTIVATED_INPUT] = aFire;
        trigs[VR_ACTUATOR_TRIGGER_FINISHED_INPUT] = aFire;

        m_impl->lastTriggers[i] = trigs;
        m_impl->fmus[i]->setTriggers(trigs);
        m_impl->fmus[i]->doStep(time, dt);
    }

    for (auto& [inst, data] : m_impl->taskData) {
        if (data.state == TaskState::Completed) data.state = TaskState::Idle;
    }

    m_impl->currentTime += dt;
}

} // namespace BoschChallenge
