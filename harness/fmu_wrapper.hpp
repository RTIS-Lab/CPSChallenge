#ifndef HARNESS_FMU_WRAPPER_HPP
#define HARNESS_FMU_WRAPPER_HPP

#include "vr.hpp"
#include "types.hpp"
#include "LateralMotionControl/sources/fmi2Functions.h"
#include <string>
#include <vector>

namespace BoschChallenge {

class FmuWrapper {
public:
    FmuWrapper();
    ~FmuWrapper();

    // Non-copyable
    FmuWrapper(const FmuWrapper&) = delete;
    FmuWrapper& operator=(const FmuWrapper&) = delete;

    bool load(const std::string& soPath);
    bool instantiate(const std::string& instanceName);
    bool setupExperiment(double startTime, double stopTime);

    // Inputs
    void setTriggers(const std::map<VR_Triggers, bool>& triggers);
    void setRealInputs(double ff_ref_0, double ff_ref_1, double velocity);
    
    // Step
    fmi2Status doStep(double currentTime, double stepSize);

    // Outputs
    void getPhysState(double state[6]);
    void getEstimatedState(VehicleState& state);
    void getGroundTruthState(GroundTruthState& state);
    void getTriggers(std::map<VR_Triggers, bool>& triggers);
    double getActOut();
    double getReal(fmi2ValueReference vr);
    int getInteger(fmi2ValueReference vr);
    bool getBoolean(fmi2ValueReference vr);

private:
    void setGains();
    static void fmiLogger(fmi2ComponentEnvironment env, fmi2String instanceName,
                         fmi2Status status, fmi2String category,
                         fmi2String message, ...);
    static void* fmiAllocateMemory(size_t n, size_t size);
    static void fmiFreeMemory(void* ptr);

    void* m_handle = nullptr;
    fmi2Component m_comp = nullptr;
    fmi2CallbackFunctions m_callbacks;
    std::string m_instanceName;

    // FMI Function Pointers
    fmi2GetTypesPlatformTYPE* m_fmi2GetTypesPlatform = nullptr;
    fmi2GetVersionTYPE* m_fmi2GetVersion = nullptr;
    fmi2SetDebugLoggingTYPE* m_fmi2SetDebugLogging = nullptr;
    fmi2InstantiateTYPE* m_fmi2Instantiate = nullptr;
    fmi2FreeInstanceTYPE* m_fmi2FreeInstance = nullptr;
    fmi2SetupExperimentTYPE* m_fmi2SetupExperiment = nullptr;
    fmi2EnterInitializationModeTYPE* m_fmi2EnterInitializationMode = nullptr;
    fmi2ExitInitializationModeTYPE* m_fmi2ExitInitializationMode = nullptr;
    fmi2TerminateTYPE* m_fmi2Terminate = nullptr;
    fmi2ResetTYPE* m_fmi2Reset = nullptr;
    fmi2GetRealTYPE* m_fmi2GetReal = nullptr;
    fmi2GetIntegerTYPE* m_fmi2GetInteger = nullptr;
    fmi2GetBooleanTYPE* m_fmi2GetBoolean = nullptr;
    fmi2SetRealTYPE* m_fmi2SetReal = nullptr;
    fmi2SetIntegerTYPE* m_fmi2SetInteger = nullptr;
    fmi2SetBooleanTYPE* m_fmi2SetBoolean = nullptr;
    fmi2DoStepTYPE* m_fmi2DoStep = nullptr;
};

} // namespace BoschChallenge

#endif // HARNESS_FMU_WRAPPER_HPP
