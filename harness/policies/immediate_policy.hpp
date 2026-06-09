#ifndef HARNESS_IMMEDIATE_POLICY_HPP
#define HARNESS_IMMEDIATE_POLICY_HPP

#include "../scheduler.hpp"

namespace BoschChallenge {

/**
 * Baseline policy that activates every task as soon as it is released.
 * Assigns equal priority (0.0) to all tasks.
 */
class ImmediateActivationPolicy : public SchedulePolicy {
public:
    std::vector<std::pair<TaskInstance, double>> schedule(
        double time,
        const std::vector<TaskInstance>& newlyReleased,
        const std::vector<TaskInstance>& newlyCompleted,
        const std::map<int, VehicleState>& vehicleStates,
        const std::map<TaskInstance, TaskState>& taskStates,
        int availableCores) override;

    std::string name() const override { return "ImmediateActivation"; }
};

} // namespace BoschChallenge

#endif // HARNESS_IMMEDIATE_POLICY_HPP
