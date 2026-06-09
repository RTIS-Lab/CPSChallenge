# Scheduler API Design Plan

## Goal

Extract the hardcoded trigger logic from `harness.cpp` into an extensible scheduling API so different scheduling policies can be plugged in — starting with an ideal baseline, but designed so realistic policies can model BCET/WCET execution times, network delays, core allocation, and preemption.

## Architecture

```
harness/
  runner.hpp + runner.cpp   — SimulationRunner class + config structs
  scheduler.hpp              — SchedulePolicy abstract base (header-only)
  experiment.hpp + exp.cpp  — ExperimentResult + runExperiment()
  run_experiment.cpp         — CLI entry point
  harness.cpp                — Left as-is for regression reference
```

## Layer 1: SimulationRunner (`runner.hpp`)

Owns the FMU and enforces timing. Does NOT make scheduling decisions.

### Config structs

```cpp
enum class TaskId { Estimator, Controller, Feedforward, Merger };

enum class ExecTimeModel { WCET, BCET, RandomUniform, PERT };

struct TaskTimingConfig {
    double period;   // seconds
    double bcet;      // seconds
    double avet;      // seconds
    double wcet;      // seconds
};

enum class NetworkId { NetSC, NetCA };

struct NetworkTimingConfig {
    double bcDelay;    // seconds
    double avgDelay;   // seconds
    double wcDelay;    // seconds
};

struct RunnerConfig {
    std::string fmuSoPath;
    std::string exampleDir;
    double simDuration = 5.0;
    double commStepSize = 0.001;
    int numCloudCores = 3;
    ExecTimeModel execTimeModel = ExecTimeModel::WCET;

    std::map<TaskId, TaskTimingConfig> tasks = {
        {TaskId::Estimator,    {.period=0.010, .bcet=0.0007, .avet=0.0009, .wcet=0.0011}},
        {TaskId::Controller,   {.period=0.020, .bcet=0.0002, .avet=0.0003, .wcet=0.0005}},
        {TaskId::Feedforward, {.period=0.020, .bcet=0.0002, .avet=0.0012, .wcet=0.0025}},
        {TaskId::Merger,      {.period=0.020, .bcet=0.0002, .avet=0.0003, .wcet=0.0005}},
    };

    std::map<NetworkId, NetworkTimingConfig> networks = {
        {NetworkId::NetSC, {.bcDelay=0.001, .avgDelay=0.008, .wcDelay=0.016}},
        {NetworkId::NetCA, {.bcDelay=0.001, .avgDelay=0.008, .wcDelay=0.016}},
    };

    double sensorPeriod   = 0.005;  // 5 ms
    double actuatorPeriod = 0.030;  // 30 ms
};
```

Default values come from `parameters.md`.

### Task state machine

```
Idle → Released → Running → Completed
                  ↑    ↓       ↑
                  └─ Preempted ─┘
```

- **Idle**: not yet released this period
- **Released**: period boundary hit, waiting for scheduler to activate
- **Running**: `activated` trigger set, execution timer counting down
- **Preempted**: lower-priority task kicked off core, timer paused, waiting to resume
- **Completed**: execution time elapsed, `finished` trigger set, output available

### Edge tasks (Sensor, Actuator)

- Run on dedicated vehicle hardware, not on shared cloud cores
- Auto-activated at their period boundaries
- `activated` + `finished` set simultaneously (instant execution, no scheduling decision)
- **Hidden from the scheduler** — the scheduler only sees the 4 cloud tasks

### Network handling

- **Automatic**: `sent` trigger fires when source task transitions to Completed
- **Automatic**: `received` trigger fires after network delay elapses
- Delay is determined by `ExecTimeModel` applied to network config (WC delay, BC delay, etc.)
- The scheduler does not control network timing
- Can be extended later if needed (e.g., holding results before sending)

### Preemption mechanics

- When `activateTask(task, priority)` is called and all cores are busy, the runner preempts the **lowest-priority** running task
- Preempted tasks keep their remaining execution time (timer paused)
- When a core frees up (a task completes), the runner **automatically resumes** the highest-priority preempted task — no scheduler intervention needed
- The FMU `activated` trigger is set once when the task first starts; preemption/resumption is invisible to the FMU

### Core constraint

- At most `numCloudCores` (default 3) cloud tasks running simultaneously
- `activateTask()` returns `false` if no core is available and no lower-priority task can be preempted (all running tasks have equal or higher priority)

### Computation model: compute-on-start

- `activated` trigger fires when the scheduler starts a task → FMU immediately computes `_comp` from current inputs
- `finished` trigger fires after execution time elapses → FMU latches `_comp → _out`
- This is the realistic model (reads inputs at start, produces output after delay)
- Preemption just delays the `finished` trigger; no re-computation needed

### Two-phase step

The simulation step is split into two phases so the scheduler can make decisions between them:

```cpp
bool beginStep();  // returns false if simDuration reached
  // 1. Advance time by commStepSize
  // 2. Check running tasks — if execution time elapsed, transition to Completed,
  //    queue network sends (set finished trigger)
  // 3. Check pending network transmissions — if delay elapsed, set received trigger
  // 4. Check Sensor/Actuator period boundaries — auto-activate
  // 5. Check cloud task period boundaries — mark as Released
  // 6. Collect newlyReleased, newlyCompleted, newlySent, newlyReceived

void activateTask(TaskId task, double priority);
  // Assign to core. Preempt lowest-priority if all cores busy.
  // Set activated trigger on FMU.

void endStep();
  // Set all collected triggers on FMU
  // Call fmi2DoStep()
  // Read outputs
```

## Layer 2: SchedulePolicy (`scheduler.hpp`)

Abstract interface. Students/researchers implement this to define scheduling algorithms.

```cpp
class SchedulePolicy {
public:
    virtual ~SchedulePolicy() = default;

    // Called each timestep after beginStep().
    // Returns (task, priority) pairs for tasks to activate.
    // Lower priority number = higher priority.
    virtual std::vector<std::pair<TaskId, double>> schedule(
        double time,
        const std::vector<TaskId>& newlyReleased,
        const std::vector<TaskId>& newlyCompleted,
        const std::map<TaskId, TaskState>& taskStates,
        int availableCores) = 0;

    virtual std::string name() const = 0;
};
```

The scheduler only sees the 4 cloud tasks. Edge tasks and network timing are handled automatically by the runner.

### IdealSchedulePolicy (baseline)

```cpp
class IdealSchedulePolicy : public SchedulePolicy {
public:
    std::vector<std::pair<TaskId, double>> schedule(
        double time,
        const std::vector<TaskId>& newlyReleased,
        const std::vector<TaskId>& newlyCompleted,
        const std::map<TaskId, TaskState>& taskStates,
        int availableCores) override;

    std::string name() const override { return "IdealSchedule"; }
};
```

Activates every released task immediately with equal priority. Uses WCET for execution times and WC delay for networks. This provides a safe deterministic baseline — not identical to the current harness (which has zero execution time) but represents the "best case under realistic timing."

### Example: EDF Scheduler (future)

```cpp
class EDFPolicy : public SchedulePolicy {
    // Activates released tasks ordered by deadline (earliest period boundary first)
    // Preempts running tasks if a higher-priority (earlier deadline) task is released
};
```

### Example: Rate-Monotonic Scheduler (future)

```cpp
class RMPolicy : public SchedulePolicy {
    // Assigns priority inversely proportional to period
    // Shorter period = higher priority
};
```

## Layer 3: Experiment Host (`experiment.hpp`)

```cpp
struct ExperimentResult {
    double duration;
    double avgPerformance;
    double rollingPerformance;
    int thresholdErrors;
    bool violated;
    std::string policyName;
};

ExperimentResult runExperiment(const RunnerConfig& config, SchedulePolicy& policy);
```

The main loop:

```cpp
ExperimentResult runExperiment(const RunnerConfig& config, SchedulePolicy& policy) {
    SimulationRunner runner(config);
    ExperimentResult result;

    while (runner.beginStep()) {
        auto decisions = policy.schedule(
            runner.currentTime(),
            runner.newlyReleased(),
            runner.newlyCompleted(),
            runner.taskStates(),
            runner.availableCores());

        for (auto& [task, priority] : decisions) {
            runner.activateTask(task, priority);
        }

        runner.endStep();
    }

    // Collect final results from runner
    return result;
}
```

## FMU Trigger Processing Order (critical)

Inside `fmi2DoStep`, the FMU processes triggers in this order:
1. **All `finished` triggers first** — latches `_comp → _out`
2. **All `activated` triggers second** — computes new `_comp`

This means:
- If `activated` and `finished` fire in the same step, `finished` latches the *previous* computation, and `activated` starts a new computation that will be latched by a *future* `finished`.
- The runner must set `finished` triggers for completed tasks and `activated` triggers for newly-started tasks in the same `doStep` call. The FMU ordering ensures correctness.

## Execution Time Model

The `ExecTimeModel` determines how the runner picks actual execution time:

| Model | Behavior |
|-------|----------|
| `WCET` | Always use `wcet` from config (deterministic, pessimistic) |
| `BCET` | Always use `bcet` from config (deterministic, optimistic) |
| `RandomUniform` | Uniform random between `bcet` and `wcet` each invocation |
| `PERT` | PERT distribution using `bcet`, `avet`, `wcet` (beta distribution stub) |

Similarly for network delays, the model determines which delay value to use.

## Special behavior from current harness

### Feedforward/Merger activation on ff_ref

The current harness activates Feedforward and Merger not just at their period boundaries, but also whenever `ff_ref` is meaningfully non-zero (`|ff_ref_0| > 1e-6 || |ff_ref_1| > 1e-6`). This is a heuristic to ensure immediate response.

**Open question**: Should this behavior be:
1. Built into the runner (always release FF/Merger on non-zero ff_ref)?
2. Exposed to the scheduler (runner signals ff_ref activity, scheduler decides)?
3. Left to the scheduler implementation (IdealSchedulePolicy replicates it, others may not)?

Recommendation: Option 2 — the runner provides `bool ffRefActive()` as a query, and `IdealSchedulePolicy` uses it. Other policies can ignore it.

## Task Chain DAG

```
Sensor ──→ Net_SC ──→ Estimator ──→ Controller ──┐
 (5ms)    (network)     (10ms)         (20ms)      │
                                                     ▼
                          Feedforward ──→ Merger ──→ Net_CA ──→ Actuator
                               (20ms)      (20ms)   (network)    (30ms)
```

**Data dependencies:**
- Estimator depends on Sensor (via Net_SC)
- Controller depends on Estimator
- Merger depends on Controller + Feedforward
- Actuator depends on Merger (via Net_CA)

The scheduler does NOT need to enforce these dependencies — the FMU handles data flow internally. If the scheduler activates Controller before Estimator finishes, the FMU will use stale Estimator output. This is correct behavior (it's what happens in a real system with timing variations).

## Verification Strategy

1. `IdealSchedulePolicy` with `ExecTimeModel::WCET` should produce a valid control simulation with realistic timing delays
2. The current `harness.cpp` serves as a regression reference for the "zero-delay" case
3. A `ZeroDelayPolicy` (activated=finished in same step, sent=received in same step) should produce bit-identical output to the current harness — this is a verification checkpoint

## Open Items

- [ ] Should `activateTask()` be idempotent for already-running tasks? (recommend: yes, no-op)
- [ ] Should the scheduler be able to explicitly defer a released task? (recommend: yes — just don't call activateTask for it; it stays Released until cores free up)
- [ ] How to handle the case where a task's period boundary hits but the previous instance is still running? (recommend: skip the release, log a warning — this is a deadline miss)
- [ ] CSV output format: extend current format, or add new columns for scheduler state?
- [ ] Multi-vehicle support: current design is single-vehicle. The challenge mentions N vehicles sharing cores. Future extension needed.