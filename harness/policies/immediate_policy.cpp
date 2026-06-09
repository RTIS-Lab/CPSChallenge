#include "immediate_policy.hpp"
#include <algorithm>
#include <map>

namespace BoschChallenge {

std::vector<std::pair<TaskInstance, double>> ImmediateActivationPolicy::schedule(
    double currentTime,
    const std::vector<TaskInstance>& newlyReleased,
    const std::vector<TaskInstance>& newlyCompleted,
    const std::map<int, VehicleState>& vehicleStates,
    const std::map<TaskInstance, TaskState>& taskStates,
    int availableCores
) {
    std::vector<std::pair<TaskInstance, double>> decisions;

    // Use a very simple priority: Sensor > Estimator > Controller > Merger > Feedforward
    auto getPriority = [](TaskType type) {
        if (type == TaskType::Estimator) return 2.0;
        if (type == TaskType::Controller) return 1.0;
        if (type == TaskType::Merger) return 3.0;
        if (type == TaskType::Feedforward) return 4.0;
        return 10.0;
    };

    for (auto const& [inst, state] : taskStates) {
        if (state == TaskState::Released || state == TaskState::Preempted) {
            decisions.push_back({inst, getPriority(inst.type)});
        }
    }

    return decisions;
}

} // namespace BoschChallenge
