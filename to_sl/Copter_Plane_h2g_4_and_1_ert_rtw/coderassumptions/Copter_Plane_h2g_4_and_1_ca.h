/*
 * File: Copter_Plane_h2g_4_and_1_ca.h
 *
 * Abstract: Tests assumptions in the generated code.
 */

#ifndef COPTER_PLANE_H2G_4_AND_1_CA_H
#define COPTER_PLANE_H2G_4_AND_1_CA_H

/* preprocessor validation checks */
#include "Copter_Plane_h2g_4_and_1_ca_preproc.h"
#include "coder_assumptions_hwimpl.h"

/* variables holding test results */
extern CA_HWImpl_TestResults CA_Copter_Plane_h2g_4_and_1_HWRes;
extern CA_PWS_TestResults CA_Copter_Plane_h2g_4_and_1_PWSRes;

/* variables holding "expected" and "actual" hardware implementation */
extern const CA_HWImpl CA_Copter_Plane_h2g_4_and_1_ExpHW;
extern CA_HWImpl CA_Copter_Plane_h2g_4_and_1_ActHW;

/* entry point function to run tests */
void Copter_Plane_h2g_4_and_1_caRunTests(void);

#endif                                 /* COPTER_PLANE_H2G_4_AND_1_CA_H */
