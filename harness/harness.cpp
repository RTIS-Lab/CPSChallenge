/*
 * FMU Harness for LateralMotionControl
 * 
 * Dynamically loads LateralMotionControl.so via dlopen and runs a simulation
 * using the FMI 2.0 Co-Simulation interface.
 *
 * Build: mkdir build && cd build && cmake ../harness && make
 * Run:   ./fmu_harness [path_to_so] [example_dir] [duration] [step_size]
 */

#include <cstddef>
#include <dlfcn.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <cassert>
#include <iomanip>

// ─── FMI 2.0 Type Definitions ────────────────────────────────────────

typedef void*           fmi2Component;
typedef void*           fmi2ComponentEnvironment;
typedef void*           fmi2FMUstate;
typedef unsigned int    fmi2ValueReference;
typedef double          fmi2Real;
typedef int             fmi2Integer;
typedef int             fmi2Boolean;
typedef const char*     fmi2String;
typedef char            fmi2Byte;

#define fmi2True  1
#define fmi2False 0

typedef enum {
    fmi2OK,
    fmi2Warning,
    fmi2Discard,
    fmi2Error,
    fmi2Fatal,
    fmi2Pending
} fmi2Status;

typedef enum {
    fmi2ModelExchange,
    fmi2CoSimulation
} fmi2Type;

// Callback function types
typedef void      (*fmi2CallbackLogger)(fmi2ComponentEnvironment, fmi2String, fmi2Status, fmi2String, fmi2String, ...);
typedef void*     (*fmi2CallbackAllocateMemory)(size_t, size_t);
typedef void      (*fmi2CallbackFreeMemory)(void*);
typedef void      (*fmi2StepFinished)(fmi2ComponentEnvironment, fmi2Status);

typedef struct {
    fmi2CallbackLogger         logger;
    fmi2CallbackAllocateMemory allocateMemory;
    fmi2CallbackFreeMemory     freeMemory;
    fmi2StepFinished           stepFinished;
    fmi2ComponentEnvironment   componentEnvironment;
} fmi2CallbackFunctions;

// FMI 2.0 function pointer types
typedef const char*             (*fmi2GetTypesPlatformTYPE)(void);
typedef const char*             (*fmi2GetVersionTYPE)(void);
typedef fmi2Status              (*fmi2SetDebugLoggingTYPE)(fmi2Component, fmi2Boolean, size_t, const fmi2String[]);
typedef fmi2Component           (*fmi2InstantiateTYPE)(fmi2String, fmi2Type, fmi2String, fmi2String, const fmi2CallbackFunctions*, fmi2Boolean, fmi2Boolean);
typedef void                    (*fmi2FreeInstanceTYPE)(fmi2Component);
typedef fmi2Status              (*fmi2SetupExperimentTYPE)(fmi2Component, fmi2Boolean, fmi2Real, fmi2Real, fmi2Boolean, fmi2Real);
typedef fmi2Status              (*fmi2EnterInitializationModeTYPE)(fmi2Component);
typedef fmi2Status              (*fmi2ExitInitializationModeTYPE)(fmi2Component);
typedef fmi2Status              (*fmi2TerminateTYPE)(fmi2Component);
typedef fmi2Status              (*fmi2ResetTYPE)(fmi2Component);
typedef fmi2Status              (*fmi2GetRealTYPE)(fmi2Component, const fmi2ValueReference[], size_t, fmi2Real[]);
typedef fmi2Status              (*fmi2GetIntegerTYPE)(fmi2Component, const fmi2ValueReference[], size_t, fmi2Integer[]);
typedef fmi2Status              (*fmi2GetBooleanTYPE)(fmi2Component, const fmi2ValueReference[], size_t, fmi2Boolean[]);
typedef fmi2Status              (*fmi2SetRealTYPE)(fmi2Component, const fmi2ValueReference[], size_t, const fmi2Real[]);
typedef fmi2Status              (*fmi2SetIntegerTYPE)(fmi2Component, const fmi2ValueReference[], size_t, const fmi2Integer[]);
typedef fmi2Status              (*fmi2SetBooleanTYPE)(fmi2Component, const fmi2ValueReference[], size_t, const fmi2Boolean[]);
typedef fmi2Status              (*fmi2DoStepTYPE)(fmi2Component, fmi2Real, fmi2Real, fmi2Boolean);

// ─── Value References ───────────────────────────────────────────────

// Trigger inputs (Boolean)
enum VR_Triggers {
    VR_SENSOR_TRIGGER_ACTIVATED_INPUT      = 100,
    VR_SENSOR_TRIGGER_FINISHED_INPUT       = 101,
    VR_NETWORK_SC_TRIGGER_SENT_INPUT       = 102,
    VR_NETWORK_SC_TRIGGER_RECEIVED_INPUT   = 103,
    VR_ESTIMATOR_TRIGGER_ACTIVATED_INPUT   = 104,
    VR_ESTIMATOR_TRIGGER_FINISHED_INPUT    = 105,
    VR_CONTROLLER_TRIGGER_ACTIVATED_INPUT  = 106,
    VR_CONTROLLER_TRIGGER_FINISHED_INPUT   = 107,
    VR_FEEDFORWARD_TRIGGER_ACTIVATED_INPUT = 108,
    VR_FEEDFORWARD_TRIGGER_FINISHED_INPUT  = 109,
    VR_MERGER_TRIGGER_ACTIVATED_INPUT     = 110,
    VR_MERGER_TRIGGER_FINISHED_INPUT      = 111,
    VR_NETWORK_CA_TRIGGER_SENT_INPUT      = 112,
    VR_NETWORK_CA_TRIGGER_RECEIVED_INPUT  = 113,
    VR_ACTUATOR_TRIGGER_ACTIVATED_INPUT   = 114,
    VR_ACTUATOR_TRIGGER_FINISHED_INPUT    = 115,
};

// Real inputs
enum VR_RealInputs {
    VR_FF_REF_0 = 116,
    VR_FF_REF_1 = 117,
    VR_VELOCITY = 118,
};

// Real outputs
enum VR_RealOutputs {
    VR_CURRENT_PHYS_STATE_START = 1000,
    VR_CURRENT_PHYS_STATE_END   = 1005,
    VR_SENS_OUT_START           = 1006,
    VR_SENS_OUT_END             = 1010,
    VR_EST_STATES_OUT_START     = 1011,
    VR_EST_STATES_OUT_END       = 1016,
    VR_CNTRL_OUT_START          = 1017,
    VR_CNTRL_OUT_END            = 1017,
    VR_FFOUT_START              = 1018,
    VR_FFOUT_END                = 1019,
    VR_FFOUT_PSI_DOT_START     = 1020,
    VR_FFOUT_PSI_DOT_END       = 1020,
    VR_AGG_OUT_START           = 1021,
    VR_AGG_OUT_END             = 1021,
    VR_ACT_OUT_START           = 1022,
    VR_ACT_OUT_END             = 1022,
    VR_CURRENT_TIME_OUT        = 1023,
    VR_CURRENT_STEP_OUT        = 1024,  // Integer
};

// Performance outputs
enum VR_Performance {
    VR_IN_LOCAL_PLATFORM_CRITICAL_SECTION     = 1025,
    VR_IN_REMOTE_PLATFORM_CRITICAL_SECTION    = 1026,
    VR_REAL_CRITICAL_SECTION                  = 1027,
    VR_IN_LOCAL_PLATFORM_ROLLING_PERFORMANCE  = 1028,
    VR_IN_REMOTE_PLATFORM_ROLLING_PERFORMANCE = 1029,
    VR_REAL_ROLLING_PERFORMANCE               = 1030,
    VR_IN_LOCAL_PLATFORM_AVERAGE_PERFORMANCE  = 1031,
    VR_IN_REMOTE_PLATFORM_AVERAGE_PERFORMANCE = 1032,
    VR_REAL_AVERAGE_PERFORMANCE               = 1033,
    VR_IN_LOCAL_PLATFORM_THRESHOLD_ERROR_CNTR = 1034,  // Integer
    VR_IN_REMOTE_PLATFORM_THRESHOLD_ERROR_CNTR= 1035,  // Integer
    VR_REAL_THRESHOLD_ERROR_CNTR              = 1036,  // Integer
    VR_IN_LOCAL_PLATFORM_VIOLATED_CONSTRAINT  = 1037,
    VR_IN_REMOTE_PLATFORM_VIOLATED_CONSTRAINT = 1038,
    VR_REAL_VIOLATED_CONSTRAINT              = 1039,
};

// ─── Task Periods (seconds) ──────────────────────────────────────────
// From parameters.md
static constexpr double PERIOD_SENSOR     = 0.005;   // 5 ms
static constexpr double PERIOD_ESTIMATOR   = 0.010;   // 10 ms
static constexpr double PERIOD_CONTROLLER  = 0.020;   // 20 ms
static constexpr double PERIOD_FEEDFORWARD = 0.020;   // 20 ms
static constexpr double PERIOD_MERGER      = 0.020;   // 20 ms
static constexpr double PERIOD_ACTUATOR    = 0.030;   // 30 ms

// Base simulation step
static constexpr double SIM_STEP = 0.0001;  // 0.1 ms

// GUID from modelDescription.xml
static const char* FMU_GUID = "{ec101913-52ec-40d8-afe6-5fbb52430f74}";

// ─── FMU Wrapper Class ──────────────────────────────────────────────

class LateralMotionControlFMU {
public:
    LateralMotionControlFMU() : m_handle(nullptr), m_comp(nullptr) {
        memset(&m_callbacks, 0, sizeof(m_callbacks));
    }

    ~LateralMotionControlFMU() {
        if (m_comp && m_fmi2Terminate) {
            m_fmi2Terminate(m_comp);
        }
        if (m_comp && m_fmi2FreeInstance) {
            m_fmi2FreeInstance(m_comp);
            m_comp = nullptr;
        }
        if (m_handle) {
            dlclose(m_handle);
            m_handle = nullptr;
        }
    }

    // Non-copyable
    LateralMotionControlFMU(const LateralMotionControlFMU&) = delete;
    LateralMotionControlFMU& operator=(const LateralMotionControlFMU&) = delete;

    bool load(const std::string& soPath) {
        m_handle = dlopen(soPath.c_str(), RTLD_NOW);
        if (!m_handle) {
            std::cerr << "ERROR: Failed to load '" << soPath << "': " << dlerror() << std::endl;
            return false;
        }

        // Resolve all FMI function pointers
        m_fmi2GetTypesPlatform     = (fmi2GetTypesPlatformTYPE)dlsym(m_handle, "LateralMotionControl_fmi2GetTypesPlatform");
        m_fmi2GetVersion           = (fmi2GetVersionTYPE)dlsym(m_handle, "LateralMotionControl_fmi2GetVersion");
        m_fmi2SetDebugLogging      = (fmi2SetDebugLoggingTYPE)dlsym(m_handle, "LateralMotionControl_fmi2SetDebugLogging");
        m_fmi2Instantiate          = (fmi2InstantiateTYPE)dlsym(m_handle, "LateralMotionControl_fmi2Instantiate");
        m_fmi2FreeInstance         = (fmi2FreeInstanceTYPE)dlsym(m_handle, "LateralMotionControl_fmi2FreeInstance");
        m_fmi2SetupExperiment      = (fmi2SetupExperimentTYPE)dlsym(m_handle, "LateralMotionControl_fmi2SetupExperiment");
        m_fmi2EnterInitializationMode = (fmi2EnterInitializationModeTYPE)dlsym(m_handle, "LateralMotionControl_fmi2EnterInitializationMode");
        m_fmi2ExitInitializationMode  = (fmi2ExitInitializationModeTYPE)dlsym(m_handle, "LateralMotionControl_fmi2ExitInitializationMode");
        m_fmi2Terminate            = (fmi2TerminateTYPE)dlsym(m_handle, "LateralMotionControl_fmi2Terminate");
        m_fmi2Reset                = (fmi2ResetTYPE)dlsym(m_handle, "LateralMotionControl_fmi2Reset");
        m_fmi2GetReal              = (fmi2GetRealTYPE)dlsym(m_handle, "LateralMotionControl_fmi2GetReal");
        m_fmi2GetInteger           = (fmi2GetIntegerTYPE)dlsym(m_handle, "LateralMotionControl_fmi2GetInteger");
        m_fmi2GetBoolean           = (fmi2GetBooleanTYPE)dlsym(m_handle, "LateralMotionControl_fmi2GetBoolean");
        m_fmi2SetReal              = (fmi2SetRealTYPE)dlsym(m_handle, "LateralMotionControl_fmi2SetReal");
        m_fmi2SetInteger           = (fmi2SetIntegerTYPE)dlsym(m_handle, "LateralMotionControl_fmi2SetInteger");
        m_fmi2SetBoolean           = (fmi2SetBooleanTYPE)dlsym(m_handle, "LateralMotionControl_fmi2SetBoolean");
        m_fmi2DoStep               = (fmi2DoStepTYPE)dlsym(m_handle, "LateralMotionControl_fmi2DoStep");

        // Check all required functions resolved
        if (!m_fmi2Instantiate || !m_fmi2FreeInstance || !m_fmi2SetupExperiment ||
            !m_fmi2EnterInitializationMode || !m_fmi2ExitInitializationMode ||
            !m_fmi2Terminate || !m_fmi2DoStep ||
            !m_fmi2GetReal || !m_fmi2SetReal || !m_fmi2GetInteger ||
            !m_fmi2GetBoolean || !m_fmi2SetBoolean)
        {
            std::cerr << "ERROR: Failed to resolve one or more FMI functions" << std::endl;
            dlclose(m_handle);
            m_handle = nullptr;
            return false;
        }

        std::cout << "Successfully loaded FMU from: " << soPath << std::endl;
        if (m_fmi2GetVersion) {
            std::cout << "  FMI Version: " << m_fmi2GetVersion() << std::endl;
        }
        if (m_fmi2GetTypesPlatform) {
            std::cout << "  Types Platform: " << m_fmi2GetTypesPlatform() << std::endl;
        }
        return true;
    }

    bool instantiate(const std::string& instanceName) {
        m_callbacks.logger = &fmiLogger;
        m_callbacks.allocateMemory = &fmiAllocateMemory;
        m_callbacks.freeMemory = &fmiFreeMemory;
        m_callbacks.stepFinished = nullptr;
        m_callbacks.componentEnvironment = nullptr;

        m_comp = m_fmi2Instantiate(
            instanceName.c_str(),
            fmi2CoSimulation,
            FMU_GUID,
            "",  // resourceLocation (not used)
            &m_callbacks,
            fmi2False,  // visible
            fmi2True     // loggingOn
        );

        if (!m_comp) {
            std::cerr << "ERROR: fmi2Instantiate failed" << std::endl;
            return false;
        }

        std::cout << "FMU instance '" << instanceName << "' created successfully." << std::endl;
        return true;
    }

    bool setupExperiment(double startTime, double stopTime) {
        fmi2Status status;

        status = m_fmi2SetupExperiment(m_comp, fmi2False, 0.0, startTime, fmi2True, stopTime);
        if (status != fmi2OK) {
            std::cerr << "ERROR: fmi2SetupExperiment returned " << status << std::endl;
            return false;
        }

        status = m_fmi2EnterInitializationMode(m_comp);
        if (status != fmi2OK) {
            std::cerr << "ERROR: fmi2EnterInitializationMode returned " << status << std::endl;
            return false;
        }

        // Set initial parameters here if needed (before exiting init mode)
        // For example, set init_velocity:
        //   fmi2ValueReference vrVel = 44;  // VR_INIT_VELOCITY
        //   fmi2Real velValue = 10.0;
        //   m_fmi2SetReal(m_comp, &vrVel, 1, &velValue);

        status = m_fmi2ExitInitializationMode(m_comp);
        if (status != fmi2OK) {
            std::cerr << "ERROR: fmi2ExitInitializationMode returned " << status << std::endl;
            return false;
        }

        std::cout << "Experiment setup: t=[" << startTime << ", " << stopTime << "]" << std::endl;
        return true;
    }

    void setTriggers(bool sensor_act, bool sensor_fin,
                      bool net_sc_sent, bool net_sc_recv,
                      bool est_act, bool est_fin,
                      bool ctrl_act, bool ctrl_fin,
                      bool ff_act, bool ff_fin,
                      bool merger_act, bool merger_fin,
                      bool net_ca_sent, bool net_ca_recv,
                      bool act_act, bool act_fin)
    {
        fmi2ValueReference vrs[16] = {
            VR_SENSOR_TRIGGER_ACTIVATED_INPUT,
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
        };
        fmi2Boolean values[16] = {
            sensor_act ? fmi2True : fmi2False,
            sensor_fin ? fmi2True : fmi2False,
            net_sc_sent ? fmi2True : fmi2False,
            net_sc_recv ? fmi2True : fmi2False,
            est_act ? fmi2True : fmi2False,
            est_fin ? fmi2True : fmi2False,
            ctrl_act ? fmi2True : fmi2False,
            ctrl_fin ? fmi2True : fmi2False,
            ff_act ? fmi2True : fmi2False,
            ff_fin ? fmi2True : fmi2False,
            merger_act ? fmi2True : fmi2False,
            merger_fin ? fmi2True : fmi2False,
            net_ca_sent ? fmi2True : fmi2False,
            net_ca_recv ? fmi2True : fmi2False,
            act_act ? fmi2True : fmi2False,
            act_fin ? fmi2True : fmi2False,
        };
        m_fmi2SetBoolean(m_comp, vrs, 16, values);
    }

    void setRealInputs(double ff_ref_0, double ff_ref_1, double velocity) {
        fmi2ValueReference vrs[3] = {VR_FF_REF_0, VR_FF_REF_1, VR_VELOCITY};
        fmi2Real values[3] = {ff_ref_0, ff_ref_1, velocity};
        m_fmi2SetReal(m_comp, vrs, 3, values);
    }

    void setVelocity(double velocity) {
        fmi2ValueReference vr = VR_VELOCITY;
        m_fmi2SetReal(m_comp, &vr, 1, &velocity);
    }

    fmi2Status doStep(double currentTime, double stepSize) {
        return m_fmi2DoStep(m_comp, currentTime, stepSize, fmi2True);
    }

    void getPhysState(double state[6]) {
        fmi2ValueReference vrs[6] = {
            (fmi2ValueReference)(VR_CURRENT_PHYS_STATE_START + 0),
            (fmi2ValueReference)(VR_CURRENT_PHYS_STATE_START + 1),
            (fmi2ValueReference)(VR_CURRENT_PHYS_STATE_START + 2),
            (fmi2ValueReference)(VR_CURRENT_PHYS_STATE_START + 3),
            (fmi2ValueReference)(VR_CURRENT_PHYS_STATE_START + 4),
            (fmi2ValueReference)(VR_CURRENT_PHYS_STATE_START + 5),
        };
        m_fmi2GetReal(m_comp, vrs, 6, state);
    }

    void getActOut(double& act_out) {
        fmi2ValueReference vr = VR_ACT_OUT_START;
        m_fmi2GetReal(m_comp, &vr, 1, &act_out);
    }

    void getPerformance(double& real_rolling, double& real_average,
                        fmi2Integer& real_threshold_errors,
                        fmi2Boolean& real_critical_section,
                        fmi2Boolean& real_violated_constraint)
    {
        fmi2ValueReference realVRs[2] = {
            VR_REAL_ROLLING_PERFORMANCE,
            VR_REAL_AVERAGE_PERFORMANCE
        };
        fmi2Real realVals[2];
        m_fmi2GetReal(m_comp, realVRs, 2, realVals);
        real_rolling = realVals[0];
        real_average = realVals[1];

        fmi2ValueReference vrThreshold = (fmi2ValueReference)VR_REAL_THRESHOLD_ERROR_CNTR;
        m_fmi2GetInteger(m_comp, &vrThreshold, 1, &real_threshold_errors);

        fmi2ValueReference boolVRs[2] = {
            VR_REAL_CRITICAL_SECTION,
            VR_REAL_VIOLATED_CONSTRAINT
        };
        fmi2Boolean boolVals[2];
        m_fmi2GetBoolean(m_comp, boolVRs, 2, boolVals);
        real_critical_section = boolVals[0];
        real_violated_constraint = boolVals[1];
    }

    void getCurrentTimeAndStep(double& time, fmi2Integer& step) {
        fmi2ValueReference vrTime = VR_CURRENT_TIME_OUT;
        m_fmi2GetReal(m_comp, &vrTime, 1, &time);
        fmi2ValueReference vrStep = (fmi2ValueReference)VR_CURRENT_STEP_OUT;
        m_fmi2GetInteger(m_comp, &vrStep, 1, &step);
    }

private:
    // Logging callback
    static void fmiLogger(fmi2ComponentEnvironment env, fmi2String instanceName,
                           fmi2Status status, fmi2String category, fmi2String message, ...) {
        (void)env;
        const char* statusStr[] = {"OK", "Warning", "Discard", "Error", "Fatal", "Pending"};
        std::cerr << "[FMU " << (instanceName ? instanceName : "?") << " / "
                  << (category ? category : "?") << "] "
                  << (status >= 0 && status <= 5 ? statusStr[status] : "Unknown") << ": "
                  << (message ? message : "") << std::endl;
    }

    static void* fmiAllocateMemory(size_t n, size_t size) {
        return calloc(n, size);
    }

    static void fmiFreeMemory(void* ptr) {
        free(ptr);
    }

    void* m_handle;
    fmi2Component m_comp;
    fmi2CallbackFunctions m_callbacks;

    // Resolved FMI function pointers (TYPE aliases already include the pointer)
    fmi2GetTypesPlatformTYPE      m_fmi2GetTypesPlatform = nullptr;
    fmi2GetVersionTYPE            m_fmi2GetVersion = nullptr;
    fmi2SetDebugLoggingTYPE       m_fmi2SetDebugLogging = nullptr;
    fmi2InstantiateTYPE           m_fmi2Instantiate = nullptr;
    fmi2FreeInstanceTYPE          m_fmi2FreeInstance = nullptr;
    fmi2SetupExperimentTYPE       m_fmi2SetupExperiment = nullptr;
    fmi2EnterInitializationModeTYPE m_fmi2EnterInitializationMode = nullptr;
    fmi2ExitInitializationModeTYPE  m_fmi2ExitInitializationMode = nullptr;
    fmi2TerminateTYPE             m_fmi2Terminate = nullptr;
    fmi2ResetTYPE                 m_fmi2Reset = nullptr;
    fmi2GetRealTYPE               m_fmi2GetReal = nullptr;
    fmi2GetIntegerTYPE            m_fmi2GetInteger = nullptr;
    fmi2GetBooleanTYPE            m_fmi2GetBoolean = nullptr;
    fmi2SetRealTYPE               m_fmi2SetReal = nullptr;
    fmi2SetIntegerTYPE            m_fmi2SetInteger = nullptr;
    fmi2SetBooleanTYPE            m_fmi2SetBoolean = nullptr;
    fmi2DoStepTYPE                m_fmi2DoStep = nullptr;
};

// ─── Helper: Load CSV column (single column, no header) ─────────────

static std::vector<double> loadCSVColumn(const std::string& path) {
    std::vector<double> data;
    std::ifstream file(path);
    if (!file.is_open()) {
        std::cerr << "WARNING: Cannot open '" << path << "'" << std::endl;
        return data;
    }
    double val;
    while (file >> val) {
        data.push_back(val);
    }
    return data;
}

// ─── Helper: Get value from a time-indexed vector ────────────────────
// Uses nearest-sample lookup into dataVec. If timeVec is provided,
// binary-searches for the nearest time index; otherwise assumes
// data is sampled at SIM_STEP (0.1ms) granularity.

static double lookupAtTime(const std::vector<double>& timeVec,
                            const std::vector<double>& dataVec,
                            double t)
{
    if (dataVec.empty()) return 0.0;
    if (timeVec.empty()) {
        // Fallback: assume data is sampled at SIM_STEP granularity
        size_t idx = (size_t)(t / SIM_STEP + 0.5);
        if (idx >= dataVec.size()) idx = dataVec.size() - 1;
        return dataVec[idx];
    }
    // Binary search for the closest time index
    size_t lo = 0, hi = timeVec.size() - 1;
    while (lo < hi) {
        size_t mid = lo + (hi - lo) / 2;
        if (timeVec[mid] < t) {
            lo = mid + 1;
        } else {
            hi = mid;
        }
    }
    // lo is the first index where timeVec[lo] >= t
    // Pick the closest of lo and lo-1
    if (lo > 0) {
        double diff_lo = std::abs(timeVec[lo] - t);
        double diff_prev = std::abs(timeVec[lo - 1] - t);
        if (diff_prev < diff_lo) lo = lo - 1;
    }
    if (lo >= dataVec.size()) lo = dataVec.size() - 1;
    return dataVec[lo];
}

// ─── Main ───────────────────────────────────────────────────────────

int main(int argc, char* argv[]) {
    std::cout << "=== LateralMotionControl FMU Harness ===" << std::endl;

    // ── Configuration ────────────────────────────────────────────────
    std::string soPath       = FMU_SO_PATH;
    std::string exampleDir   = EXAMPLE_DIR;
    double simDuration       = 5.0;      // seconds (short demo)
    double commStepSize      = 0.001;    // 1 ms communication step size

    // Override from command line
    if (argc >= 2) soPath = argv[1];
    if (argc >= 3) exampleDir = argv[2];
    if (argc >= 4) simDuration = std::stod(argv[3]);
    if (argc >= 5) commStepSize = std::stod(argv[4]);

    // ── Load the FMU ────────────────────────────────────────────────
    LateralMotionControlFMU fmu;
    if (!fmu.load(soPath)) {
        return 1;
    }

    if (!fmu.instantiate("lateral_sim")) {
        return 1;
    }

    if (!fmu.setupExperiment(0.0, simDuration)) {
        return 1;
    }

    // ── Load example data ───────────────────────────────────────────
    std::vector<double> timeVec       = loadCSVColumn(exampleDir + "/time_vector.csv");
    std::vector<double> ffRef0Data   = loadCSVColumn(exampleDir + "/feedforward_sequence_0.csv");
    std::vector<double> ffRef1Data   = loadCSVColumn(exampleDir + "/feedforward_sequence_1.csv");
    std::vector<double> velocityData = loadCSVColumn(exampleDir + "/velocity.csv");
    std::vector<double> xTrackData   = loadCSVColumn(exampleDir + "/x_position_track.csv");
    std::vector<double> yTrackData   = loadCSVColumn(exampleDir + "/y_position_track.csv");

    if (ffRef0Data.empty() || velocityData.empty()) {
        std::cerr << "WARNING: Could not load example data from '" << exampleDir
                  << "'. Using constant defaults (ff=0, v=10)." << std::endl;
    }

    // ── Simulation Loop ──────────────────────────────────────────────
    double time = 0.0;

    std::cout << std::endl;
    std::cout << "Starting simulation: duration=" << simDuration
              << "s, commStepSize=" << commStepSize * 1000 << "ms" << std::endl;
    std::cout << std::endl;

    // CSV header — full state + track position approximation
    // State: phi_dot, beta, delta, delta_dot, e_y, e_y_dot
    // Position: x_track(t) and y_track(t)+e_y approximate the vehicle position
    std::cout << "time,phi_dot,beta,delta,delta_dot,e_y,e_y_dot,"
              << "act_out,velocity,x_track,y_track,xveh,yveh,"
              << "real_rolling_perf,real_avg_perf,"
              << "real_thresh_errors,real_critical,real_violated"
              << std::endl;

    double lastPrintedTime = -1.0;  // ensures first step always prints

    while (time < simDuration - 1e-12) {
        // ── Set inputs based on example data ─────────────────────────
        double ff_ref_0 = lookupAtTime(timeVec, ffRef0Data, time);
        double ff_ref_1 = lookupAtTime(timeVec, ffRef1Data, time);
        double velocity  = lookupAtTime(timeVec, velocityData, time);

        fmu.setRealInputs(ff_ref_0, ff_ref_1, velocity);

        // ── Determine which triggers fire at this timestep ────────────
        // Simplified schedule: tasks fire at their periods, with both
        // activated + finished pulsed together in the same step.
        //
        // In a real co-simulation, a scheduling simulator would
        // determine when tasks start and finish. Here we use a naive
        // periodic schedule where each task's triggers are pulsed
        // together at the task's period boundary.

        double eps = 1e-12;
        bool sensor_fire     = (fmod(time + eps, PERIOD_SENSOR) < commStepSize + eps);
        bool net_sc_fire     = sensor_fire;  // network SC sends right after sensor
        bool estimator_fire  = (fmod(time + eps, PERIOD_ESTIMATOR) < commStepSize + eps);
        bool controller_fire = (fmod(time + eps, PERIOD_CONTROLLER) < commStepSize + eps);
        bool ff_fire         = (fmod(time + eps, PERIOD_FEEDFORWARD) < commStepSize + eps);
        bool merger_fire     = (fmod(time + eps, PERIOD_MERGER) < commStepSize + eps);
        bool net_ca_fire     = merger_fire;   // network CA sends right after merger
        bool actuator_fire   = (fmod(time + eps, PERIOD_ACTUATOR) < commStepSize + eps);

        // Pulse both activated and finished together on the same step.
        // A more realistic simulation would separate activation (start)
        // and finished (completion) by the task's execution time.
        fmu.setTriggers(
            sensor_fire, sensor_fire,        // sensor: activated + finished
            net_sc_fire, net_sc_fire,         // network SC: sent + received
            estimator_fire, estimator_fire,   // estimator: activated + finished
            controller_fire, controller_fire, // controller: activated + finished
            ff_fire, ff_fire,                 // feedforward: activated + finished
            merger_fire, merger_fire,         // merger: activated + finished
            net_ca_fire, net_ca_fire,         // network CA: sent + received
            actuator_fire, actuator_fire      // actuator: activated + finished
        );

        // ── Step the FMU ─────────────────────────────────────────────
        fmi2Status status = fmu.doStep(time, commStepSize);
        if (status != fmi2OK) {
            std::cerr << "ERROR: fmi2DoStep failed at t=" << time
                      << " with status " << status << std::endl;
            break;
        }

        // ── Read outputs periodically ────────────────────────────────
        // Print every ~100ms. Use lastPrintedTime to avoid double-prints
        // that the modulo approach produces at boundary steps.
        double printInterval = 0.1;
        bool shouldPrint = ((time - lastPrintedTime) >= printInterval - eps);

        if (shouldPrint) {
            double physState[6] = {0};
            fmu.getPhysState(physState);

            double act_out = 0.0;
            fmu.getActOut(act_out);

            double real_rolling = 0.0, real_average = 0.0;
            fmi2Integer real_thresh_errors = 0;
            fmi2Boolean real_critical = fmi2False, real_violated = fmi2False;
            fmu.getPerformance(real_rolling, real_average, real_thresh_errors,
                               real_critical, real_violated);

            // Approximate vehicle position from track reference + lateral error
            double x_track = lookupAtTime(timeVec, xTrackData, time);
            double y_track = lookupAtTime(timeVec, yTrackData, time);
            double x_veh = x_track;
            double y_veh = y_track + physState[4]; // y_track + e_y

            std::cout << std::fixed << std::setprecision(4)
                      << time << ","
                      << physState[0] << ","  // phi_dot (yaw rate)
                      << physState[1] << ","  // beta (slip angle)
                      << physState[2] << ","  // delta (steering angle)
                      << physState[3] << ","  // delta_dot (steering rate)
                      << physState[4] << ","  // e_y (lateral error)
                      << physState[5] << ","  // e_y_dot (lateral error rate)
                      << act_out << ","
                      << velocity << ","
                      << x_track << ","
                      << y_track << ","
                      << x_veh << ","
                      << y_veh << ","
                      << real_rolling << ","
                      << real_average << ","
                      << real_thresh_errors << ","
                      << real_critical << ","
                      << real_violated
                      << std::endl;
            lastPrintedTime = time;
        }

        // ── Clear all triggers for next step ──────────────────────────
        // After processing, set all triggers to false for the next cycle.
        // Triggers should only be true for the step where they fire.
        fmu.setTriggers(
            false, false,  // sensor
            false, false,  // network SC
            false, false,  // estimator
            false, false,  // controller
            false, false,  // feedforward
            false, false,  // merger
            false, false,  // network CA
            false, false   // actuator
        );

        time += commStepSize;
    }

    std::cerr << std::endl;
    std::cerr << "Simulation completed. Final time: " << time << "s" << std::endl;

    // Final performance summary
    {
        double real_rolling = 0.0, real_average = 0.0;
        fmi2Integer real_thresh_errors = 0;
        fmi2Boolean real_critical = fmi2False, real_violated = fmi2False;
        fmu.getPerformance(real_rolling, real_average, real_thresh_errors,
                           real_critical, real_violated);

        double physState[6] = {0};
        fmu.getPhysState(physState);
        double act_out = 0.0;
        fmu.getActOut(act_out);

        std::cerr << std::endl;
        std::cerr << "=== Final State ===" << std::endl;
        std::cerr << "  Physical state: [";
        for (int i = 0; i < 6; i++) {
            std::cerr << physState[i];
            if (i < 5) std::cerr << ", ";
        }
        std::cerr << "]" << std::endl;
        double x_track_final = lookupAtTime(timeVec, xTrackData, time);
        double y_track_final = lookupAtTime(timeVec, yTrackData, time);

        std::cerr << "  Steering output (act_out): " << act_out << std::endl;
        std::cerr << "  Lateral error (e_y):       " << physState[4] << std::endl;
        std::cerr << "  Track position:             (" << x_track_final << ", " << y_track_final << ")" << std::endl;
        std::cerr << "  Vehicle position (approx):  (" << x_track_final << ", " << y_track_final + physState[4] << ")" << std::endl;
        std::cerr << "  Real avg performance:      " << real_average << std::endl;
        std::cerr << "  Real rolling performance:  " << real_rolling << std::endl;
        std::cerr << "  Threshold error counter:   " << real_thresh_errors << std::endl;
        std::cerr << "  In critical section:       " << (real_critical ? "YES" : "NO") << std::endl;
        std::cerr << "  Constraint violated:       " << (real_violated ? "YES" : "NO") << std::endl;
    }

    // ── Cleanup ──────────────────────────────────────────────────────
    // Destructor handles terminate + freeInstance + dlclose

    return 0;
}