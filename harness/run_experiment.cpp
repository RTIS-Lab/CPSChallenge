#include <iomanip>
#include <cmath>
#include <iostream>
#include <map>

#include "runner.hpp"
#include "policies/immediate_policy.hpp"
#include "csv_loader.hpp"

using namespace BoschChallenge;

static double computePathHeading(const std::vector<double>& xTrack, const std::vector<double>& yTrack, size_t idx) {
    if (xTrack.size() < 2) return 0.0;
    size_t idx_plus = (idx + 100 < xTrack.size()) ? idx + 100 : idx;
    size_t idx_minus = (idx > 100) ? idx - 100 : idx;
    double dx = xTrack[idx_plus] - xTrack[idx_minus];
    double dy = yTrack[idx_plus] - yTrack[idx_minus];
    return std::atan2(dy, dx);
}

int main(int argc, char* argv[]) {
    std::cerr << "=== Bosch CPS Challenge: Modular Scheduler Harness ===" << std::endl;

    RunnerConfig config;
    config.fmuSoPath = FMU_SO_PATH;
    config.exampleDir = EXAMPLE_DIR;
    config.numVehicles = 1;
    config.numCloudCores = 3; 
    config.commStepSize = 0.001;
    bool baselineMode = false;
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--baseline") baselineMode = true;
    }

    if (argc >= 2 && std::string(argv[1]) != "--baseline") config.numVehicles = std::stoi(argv[1]);
    if (argc >= 3 && std::string(argv[2]) != "--baseline") config.simDuration = std::stod(argv[2]);

    if (baselineMode) {
        std::cerr << "Running in BASELINE mode (Zero delays, original periods)" << std::endl;
        config.timingModel = ExecTimeModel::WCET; // Doesn't matter, all are 0
        config.taskConfigs[TaskType::Estimator]   = {0.010, 0.0, 0.0, 0.0};
        config.taskConfigs[TaskType::Controller]  = {0.020, 0.0, 0.0, 0.0};
        config.taskConfigs[TaskType::Feedforward] = {0.020, 0.0, 0.0, 0.0};
        config.taskConfigs[TaskType::Merger]      = {0.020, 0.0, 0.0, 0.0};
        config.sensorPeriod = 0.005;
        config.actuatorPeriod = 0.030;
        config.netSCConfig = {0.0, 0.0, 0.0};
        config.netCAConfig = {0.0, 0.0, 0.0};
    } else {
        std::cerr << "Running in CHALLENGE mode (Realistic delays)" << std::endl;
        config.timingModel = ExecTimeModel::WCET;
        config.taskConfigs[TaskType::Estimator]   = {0.010, 0.0007, 0.0009, 0.0011};
        config.taskConfigs[TaskType::Controller]  = {0.020, 0.0002, 0.0003, 0.0005};
        config.taskConfigs[TaskType::Feedforward] = {0.020, 0.0002, 0.0012, 0.0025};
        config.taskConfigs[TaskType::Merger]      = {0.020, 0.0002, 0.0003, 0.0005};
        config.sensorPeriod = 0.005;
        config.actuatorPeriod = 0.030;
        config.netSCConfig = {0.001, 0.008, 0.016};
        config.netCAConfig = {0.001, 0.008, 0.016};
    }

    auto track_x = loadCSVColumn(config.exampleDir + "/x_position_track.csv");
    auto track_y = loadCSVColumn(config.exampleDir + "/y_position_track.csv");
    auto velocity_data = loadCSVColumn(config.exampleDir + "/velocity.csv");
    auto ff_ref_0 = loadCSVColumn(config.exampleDir + "/feedforward_sequence_0.csv");
    auto ff_ref_1 = loadCSVColumn(config.exampleDir + "/feedforward_sequence_1.csv");

    SimulationRunner runner(config);
    ImmediateActivationPolicy policy;

    std::cout << "time,vehicle_id,phi_dot,beta,delta,delta_dot,e_y,e_y_dot,act_out,velocity,x_track,y_track,xveh,yveh,"
              << "real_rolling_perf,real_avg_perf,real_thresh_errors,real_critical,real_violated,"
              << "ff_ref_0,ff_ref_1,"
              << "trig_sens,trig_sc_sent,trig_sc_recv,trig_est_act,trig_est_fin,trig_ctrl_act,trig_ctrl_fin,trig_ff_act,trig_ff_fin,trig_merg_act,trig_merg_fin,trig_ca_sent,trig_ca_recv,trig_act_act"
              << std::endl;

    std::map<int, std::map<VR_Triggers, bool>> latchedTriggers;
    long long lastLogStep = -1;

    while (runner.beginStep()) {
        auto decisions = policy.schedule(
            runner.currentTime(), runner.newlyReleased(), runner.newlyCompleted(),
            runner.vehicleStates(), runner.taskStates(), runner.availableCores()
        );
        for (auto const& [task, priority] : decisions) runner.activateTask(task, priority);

        size_t idx = (size_t)(runner.currentTime() / 0.0001 + 0.5);
        for (int i = 0; i < config.numVehicles; ++i) {
            double ff0 = (idx < ff_ref_0.size()) ? ff_ref_0[idx] : 0.0;
            double ff1 = (idx < ff_ref_1.size()) ? ff_ref_1[idx] : 0.0;
            double vel = (idx < velocity_data.size()) ? velocity_data[idx] : 10.0;
            runner.setVehicleInputs(i, ff0, ff1, vel);
        }

        runner.endStep();

        for (int i = 0; i < config.numVehicles; ++i) {
            std::map<VR_Triggers, bool> current;
            runner.getVehicleLastTriggers(i, current);
            for (auto const& [vr, val] : current) if (val) latchedTriggers[i][vr] = true;
        }

        double time = runner.currentTime();
        long long currentLogStep = (long long)(time / 0.001 + 0.5);
        if (currentLogStep > lastLogStep) {
            lastLogStep = currentLogStep;
            for (int i = 0; i < config.numVehicles; ++i) {
                double phys[6];
                runner.getVehiclePhysState(i, phys);
                double act_out = runner.getVehicleActOut(i);
                const auto& gt = runner.groundTruthStates().at(i);
                auto& latched = latchedTriggers[i];

                size_t logIdx = (size_t)(time / 0.0001 + 0.5);
                if (logIdx >= track_x.size()) logIdx = track_x.size() - 1;
                double xt = track_x[logIdx], yt = track_y[logIdx];
                double vel = (logIdx < velocity_data.size()) ? velocity_data[logIdx] : 10.0;
                double psi_path = computePathHeading(track_x, track_y, logIdx);
                double e_y = phys[4];
                
                double xv = xt + e_y * std::sin(psi_path);
                double yv = yt - e_y * std::cos(psi_path);

                std::cout << std::fixed << std::setprecision(6) << time << "," << i << ","
                          << phys[0] << "," << phys[1] << "," << phys[2] << "," << phys[3] << "," << phys[4] << "," << phys[5] << ","
                          << act_out << "," << vel << "," << xt << "," << yt << "," << xv << "," << yv << ","
                          << gt.realRollingPerformance << "," << gt.realAveragePerformance << "," << gt.realThresholdErrors << ","
                          << (gt.realRollingPerformance > 0.1 ? 1 : 0) << "," << (gt.violated ? 1 : 0) << ","
                          << ff_ref_0[logIdx] << "," << ff_ref_1[logIdx] << ","
                          << latched[VR_SENSOR_TRIGGER_ACTIVATED_INPUT] << ","
                          << latched[VR_NETWORK_SC_TRIGGER_SENT_INPUT] << ","
                          << latched[VR_NETWORK_SC_TRIGGER_RECEIVED_INPUT] << ","
                          << latched[VR_ESTIMATOR_TRIGGER_ACTIVATED_INPUT] << ","
                          << latched[VR_ESTIMATOR_TRIGGER_FINISHED_INPUT] << ","
                          << latched[VR_CONTROLLER_TRIGGER_ACTIVATED_INPUT] << ","
                          << latched[VR_CONTROLLER_TRIGGER_FINISHED_INPUT] << ","
                          << latched[VR_FEEDFORWARD_TRIGGER_ACTIVATED_INPUT] << ","
                          << latched[VR_FEEDFORWARD_TRIGGER_FINISHED_INPUT] << ","
                          << latched[VR_MERGER_TRIGGER_ACTIVATED_INPUT] << ","
                          << latched[VR_MERGER_TRIGGER_FINISHED_INPUT] << ","
                          << latched[VR_NETWORK_CA_TRIGGER_SENT_INPUT] << ","
                          << latched[VR_NETWORK_CA_TRIGGER_RECEIVED_INPUT] << ","
                          << latched[VR_ACTUATOR_TRIGGER_ACTIVATED_INPUT] << std::endl;
                
                latched.clear();
            }
        }

        if (std::fmod(time, 1.0) < 0.00005) {
            std::cerr << "Progress: " << std::fixed << std::setprecision(1) << time << "s / " << config.simDuration << "s\r" << std::flush;
        }
    }
    std::cerr << std::endl << "Experiment Finished." << std::endl;
    return 0;
}
