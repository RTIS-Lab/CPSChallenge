/*
  Bosch RTAS Challenge
  Copyright (c) 2026 Robert Bosch GmbH
  SPDX-License-Identifier: AGPL-3.0
*/

#ifndef LATERAL_MOTION_CONTROL_H
#define LATERAL_MOTION_CONTROL_H

// Include the standard FMI types needed for the function declarations
#include "fmi2FunctionTypes.h"
#include "fmi2TypesPlatform.h"

/*
This header file contains the function declarations for the
FMI functions exported by LateralMotionControl.c. Including this
in the test harness ensures that the compiler knows the correct
signatures for these functions.
*/

// --- Function Declarations for the FMU ---

// Use the standard FMI 2.0 function macros to declare the functions,
// but with the specific prefix for this FMU.
#define FMI2_FUNCTION_PREFIX LateralMotionControl_
#include "fmi2Functions.h"


#endif // LATERAL_MOTION_CONTROL_H
