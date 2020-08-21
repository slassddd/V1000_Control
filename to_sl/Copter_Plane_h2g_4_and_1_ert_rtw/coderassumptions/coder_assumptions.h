/*
 * File: coder_assumptions.h
 *
 * Abstract: Coder assumptions header file
 */

#ifndef CODER_ASSUMPTIONS_H
#define CODER_ASSUMPTIONS_H

/* include model specific checks */
#include "Copter_Plane_h2g_4_and_1_ca.h"

/* global results variable mapping for static code */
#define CA_Expected_HWImpl             CA_Copter_Plane_h2g_4_and_1_ExpHW
#define CA_Actual_HWImpl               CA_Copter_Plane_h2g_4_and_1_ActHW
#define CA_HWImpl_Results              CA_Copter_Plane_h2g_4_and_1_HWRes
#define CA_PortableWordSizes_Results   CA_Copter_Plane_h2g_4_and_1_PWSRes

/* entry point function mapping for static code */
#define CA_Run_Tests                   Copter_Plane_h2g_4_and_1_caRunTests
#endif                                 /* CODER_ASSUMPTIONS_H */
