#ifndef HARNESS_SCHEDULER_HPP
#define HARNESS_SCHEDULER_HPP

#include "types.hpp"
#include <vector>
#include <utility>
#include <map>

namespace BoschChallenge {

class SchedulePolicy {
public:
    virtual ~SchedulePolicy() = default;

    /**
     * Called at each communication step (e.g., every 1ms).
     *
     * @param time The current simulation time in seconds.
     * @param newlyReleased Tasks that hit their period boundary or ff_ref trigger this step.
     * @param newlyCompleted Tasks that finished their execution timer this step.
     * @param vehicleStates The latest ESTIMATED performance metrics for all vehicles.
     * @param taskStates The current execution state of all cloud tasks.
     * @param availableCores Number of cloud cores currently free.
     *
     * @return A list of (TaskInstance, priority) pairs to activate.
     *         Lower priority value = higher priority.
     */
    virtual std::vector<std::pair<TaskInstance, double>> schedule(
        double time,
        const std::vector<TaskInstance>& newlyReleased,
        const std::vector<TaskInstance>& newlyCompleted,
        const std::map<int, VehicleState>& vehicleStates,
        const std::map<TaskInstance, TaskState>& taskStates,
        int availableCores) = 0;

    virtual std::string name() const = 0;
};

} // namespace BoschChallenge

#endif // HARNESS_SCHEDULER_HPP
