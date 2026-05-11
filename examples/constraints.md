# Performance Metrics Constraints

The performance metrics are updated every 10ms in-car, and every activation of the estimator task in the remote platform.
The following performance constraints must be checked for each vehicle in the test.

1.  **Constraint Violation of lateral Error**:
   * Hard constraint (safety): The lateral error must always be within a threshold of 0.8 meters.
   * Soft constraint (comfort): The lateral error should be preferably within the predefined `ERROR_THRESHOLD` of 0.2 meters. This threshold should not be violated for more than 5% of the total simulation time (chance constraint).
2.  **Rolling Performance**: The rolling performance models the impact of lateral error and lateral error rate in a rolling window of 1 second. Its maximum value should be minimized.
3.  **Average Performance**: The average cumulative performance models the impact of lateral error and lateral error rate across the entire simulation and should be minimized, with secondary importance w.r.t. the rolling performance.

Different solutions of the Challenge can be ranked by comparing the values obtained in the metrics above.

**Notes**
* One set of performance monitors is defined for each vehicle.
* The combined (or a subset of) measurements obtained from the performance monitors of all vehicles should drive the decisions of the scheduler at runtime.
* The performance monitors are evaluated with a granularity of 10ms (in vehicle and in the remote platform). Thus, the Threshold Error Counter monitor counts how many iterations of 10ms the vehicle error is above the `ERROR_THRESHOLD` of 0.2 meters. As an example, for a 150 seconds simulation and a chance constraint of 5%, it means that the counter value should not exceed 750.
* The constraints above should be evaluated on the "real" performance monitors. Nonetheless, the scheduler in the remote platform has access only to an estimate of the performance based on the transmitted data and estimated states from the Estimator task. Due to delays and/or skipped executions, there might be a mismatch between the real and estimated performance metrics.
* Additional performance constraints can be added by leveraging the FMU outputs.
