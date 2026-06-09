#include "fmu_wrapper.hpp"
#include <dlfcn.h>
#include <iostream>
#include <cstring>
#include <cstdarg>
#include <vector>

namespace BoschChallenge {

FmuWrapper::FmuWrapper() {
    memset(&m_callbacks, 0, sizeof(m_callbacks));
}

FmuWrapper::~FmuWrapper() {
    if (m_comp && m_fmi2Terminate) (*m_fmi2Terminate)(m_comp);
    if (m_comp && m_fmi2FreeInstance) (*m_fmi2FreeInstance)(m_comp);
    if (m_handle) dlclose(m_handle);
}

bool FmuWrapper::load(const std::string& soPath) {
    m_handle = dlopen(soPath.c_str(), RTLD_NOW);
    if (!m_handle) {
        std::cerr << "ERROR: Failed to load '" << soPath << "': " << dlerror() << std::endl;
        return false;
    }

    auto resolve = [&](const char* name) -> void* {
        std::string fullName = "LateralMotionControl_" + std::string(name);
        void* ptr = dlsym(m_handle, fullName.c_str());
        if (!ptr) ptr = dlsym(m_handle, name);
        return ptr;
    };

    m_fmi2GetTypesPlatform = (fmi2GetTypesPlatformTYPE*)resolve("fmi2GetTypesPlatform");
    m_fmi2GetVersion = (fmi2GetVersionTYPE*)resolve("fmi2GetVersion");
    m_fmi2Instantiate = (fmi2InstantiateTYPE*)resolve("fmi2Instantiate");
    m_fmi2FreeInstance = (fmi2FreeInstanceTYPE*)resolve("fmi2FreeInstance");
    m_fmi2SetupExperiment = (fmi2SetupExperimentTYPE*)resolve("fmi2SetupExperiment");
    m_fmi2EnterInitializationMode = (fmi2EnterInitializationModeTYPE*)resolve("fmi2EnterInitializationMode");
    m_fmi2ExitInitializationMode = (fmi2ExitInitializationModeTYPE*)resolve("fmi2ExitInitializationMode");
    m_fmi2Terminate = (fmi2TerminateTYPE*)resolve("fmi2Terminate");
    m_fmi2GetReal = (fmi2GetRealTYPE*)resolve("fmi2GetReal");
    m_fmi2GetInteger = (fmi2GetIntegerTYPE*)resolve("fmi2GetInteger");
    m_fmi2GetBoolean = (fmi2GetBooleanTYPE*)resolve("fmi2GetBoolean");
    m_fmi2SetReal = (fmi2SetRealTYPE*)resolve("fmi2SetReal");
    m_fmi2SetBoolean = (fmi2SetBooleanTYPE*)resolve("fmi2SetBoolean");
    m_fmi2DoStep = (fmi2DoStepTYPE*)resolve("fmi2DoStep");

    return (m_fmi2Instantiate != nullptr && m_fmi2DoStep != nullptr && 
            m_fmi2GetReal != nullptr && m_fmi2SetReal != nullptr);
}

bool FmuWrapper::instantiate(const std::string& instanceName) {
    m_instanceName = instanceName;
    m_callbacks.logger = &fmiLogger;
    m_callbacks.allocateMemory = &fmiAllocateMemory;
    m_callbacks.freeMemory = &fmiFreeMemory;

    m_comp = (*m_fmi2Instantiate)(m_instanceName.c_str(), fmi2CoSimulation, 
                               "{ec101913-52ec-40d8-afe6-5fbb52430f74}", // GUID
                               "", &m_callbacks, fmi2False, fmi2True);
    return m_comp != nullptr;
}

void FmuWrapper::setGains() {
    // K_trc_fb Matrix (feedback gain)
    fmi2ValueReference vr_fb[] = {1, 2, 3, 4, 5, 6};
    fmi2Real val_fb[] = {-0.987, 0.0, 0.0, 0.0, 8.0, 1.75};
    (*m_fmi2SetReal)(m_comp, vr_fb, 6, val_fb);

    // K_trc_ff Matrix (feedforward gain)
    fmi2ValueReference vr_ff[] = {7, 8};
    fmi2Real val_ff[] = {1.987, 0.1974};
    (*m_fmi2SetReal)(m_comp, vr_ff, 2, val_ff);

    // K_yrc_x Matrix (yaw rate controller state gain)
    fmi2ValueReference vr_yrc_x[] = {9, 10, 11, 12, 13, 14};
    fmi2Real val_yrc_x[] = {-0.2876, 0.0, 1.0, 0.0, 0.0, 0.0};
    (*m_fmi2SetReal)(m_comp, vr_yrc_x, 6, val_yrc_x);

    // K_yrc_psi Matrix (yaw rate controller psi_dot gain)
    fmi2ValueReference vr_yrc_psi = 15;
    fmi2Real val_yrc_psi = 0.2876;
    (*m_fmi2SetReal)(m_comp, &vr_yrc_psi, 1, &val_yrc_psi);

    // Initial State x0
    fmi2ValueReference vr_x0[] = {16, 17, 18, 19, 20, 21};
    fmi2Real val_x0[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    (*m_fmi2SetReal)(m_comp, vr_x0, 6, val_x0);

    // Initial Velocity
    fmi2ValueReference vr_v0 = 34;
    fmi2Real val_v0 = 10.0;
    (*m_fmi2SetReal)(m_comp, &vr_v0, 1, &val_v0);
}

bool FmuWrapper::setupExperiment(double startTime, double stopTime) {
    if ((*m_fmi2SetupExperiment)(m_comp, fmi2False, 0.0, startTime, fmi2False, 0.0) != fmi2OK) return false;
    if ((*m_fmi2EnterInitializationMode)(m_comp) != fmi2OK) return false;
    setGains();
    if ((*m_fmi2ExitInitializationMode)(m_comp) != fmi2OK) return false;
    return true;
}

void FmuWrapper::setTriggers(const std::map<VR_Triggers, bool>& triggers) {
    std::vector<fmi2ValueReference> vrs;
    std::vector<fmi2Boolean> vals;
    for (auto const& [vr, val] : triggers) {
        vrs.push_back(vr);
        vals.push_back(val ? fmi2True : fmi2False);
    }
    (*m_fmi2SetBoolean)(m_comp, vrs.data(), vrs.size(), vals.data());
}

void FmuWrapper::setRealInputs(double ff_ref_0, double ff_ref_1, double velocity) {
    fmi2ValueReference vrs[] = {VR_FF_REF_0_INPUT, VR_FF_REF_1_INPUT, VR_VELOCITY_INPUT, VR_INIT_VELOCITY};
    fmi2Real vals[] = {ff_ref_0, ff_ref_1, velocity, velocity};
    (*m_fmi2SetReal)(m_comp, vrs, 4, vals);
}

fmi2Status FmuWrapper::doStep(double currentTime, double stepSize) {
    return (*m_fmi2DoStep)(m_comp, currentTime, stepSize, fmi2True);
}

void FmuWrapper::getPhysState(double state[6]) {
    fmi2ValueReference vrs[] = {1000, 1001, 1002, 1003, 1004, 1005};
    (*m_fmi2GetReal)(m_comp, vrs, 6, state);
}

void FmuWrapper::getEstimatedState(VehicleState& state) {
    state.estimatedLateralError = getReal(VR_EST_STATE_OUT_START + 4);
    state.estimatedRollingPerformance = getReal(VR_REMOTE_PERF);
    state.estimatedThresholdErrors = getInteger(VR_REMOTE_ERR_CNTR);
    state.inCriticalSection = getBoolean(VR_REMOTE_CRITICAL);
    
    double ff0 = getReal(VR_FF_REF_0_INPUT);
    double ff1 = getReal(VR_FF_REF_1_INPUT);
    state.ffRefActive = (std::abs(ff0) > 1e-6 || std::abs(ff1) > 1e-6);
}

void FmuWrapper::getGroundTruthState(GroundTruthState& state) {
    state.realLateralError = getReal(VR_PHYS_STATE_START + 4);
    state.realRollingPerformance = getReal(VR_REAL_PERF);
    state.realAveragePerformance = getReal(VR_REAL_AVG_PERF);
    state.realThresholdErrors = getInteger(VR_REAL_ERR_CNTR);
    state.violated = getBoolean(VR_REAL_VIOLATED);
}

void FmuWrapper::getTriggers(std::map<VR_Triggers, bool>& triggers) {
    for (int i = 100; i <= 115; ++i) {
        VR_Triggers vr = static_cast<VR_Triggers>(i);
        triggers[vr] = getBoolean(vr);
    }
}

double FmuWrapper::getActOut() {
    return getReal(VR_ACT_OUT);
}

double FmuWrapper::getReal(fmi2ValueReference vr) {
    fmi2Real val = 0;
    (*m_fmi2GetReal)(m_comp, &vr, 1, &val);
    return val;
}

int FmuWrapper::getInteger(fmi2ValueReference vr) {
    fmi2Integer val = 0;
    (*m_fmi2GetInteger)(m_comp, &vr, 1, &val);
    return (int)val;
}

bool FmuWrapper::getBoolean(fmi2ValueReference vr) {
    fmi2Boolean val = fmi2False;
    (*m_fmi2GetBoolean)(m_comp, &vr, 1, &val);
    return val == fmi2True;
}

void FmuWrapper::fmiLogger(fmi2ComponentEnvironment env, fmi2String instanceName,
                          fmi2Status status, fmi2String category,
                          fmi2String message, ...) {
}

void* FmuWrapper::fmiAllocateMemory(size_t n, size_t size) { return calloc(n, size); }
void FmuWrapper::fmiFreeMemory(void* ptr) { free(ptr); }

} // namespace BoschChallenge
