/*
 * File: Copter_Plane_h2g_4_and_1_ca.c
 *
 * Abstract: Tests assumptions in the generated code.
 */

#include "Copter_Plane_h2g_4_and_1_ca.h"

CA_HWImpl_TestResults CA_Copter_Plane_h2g_4_and_1_HWRes;
CA_PWS_TestResults CA_Copter_Plane_h2g_4_and_1_PWSRes;
const CA_HWImpl CA_Copter_Plane_h2g_4_and_1_ExpHW = {
  8,                                   /* BitPerChar */
  16,                                  /* BitPerShort */
  32,                                  /* BitPerInt */
  32,                                  /* BitPerLong */
  64,                                  /* BitPerLongLong */
  32,                                  /* BitPerFloat */
  64,                                  /* BitPerDouble */
  32,                                  /* BitPerPointer */
  32,                                  /* BitPerSizeT */
  32,                                  /* BitPerPtrDiffT */
  CA_LITTLE_ENDIAN,                    /* Endianess */
  CA_ZERO,                             /* IntDivRoundTo */
  1,                                   /* ShiftRightIntArith */
  1,                                   /* LongLongMode */
  0,                                   /* PortableWordSizes */
  "Custom Processor->Custom Processor",/* HWDeviceType */
  0,                                   /* MemoryAtStartup */
  0,                                   /* DenormalFlushToZero */
  0                                    /* DenormalAsZero */
};

CA_HWImpl CA_Copter_Plane_h2g_4_and_1_ActHW;
void Copter_Plane_h2g_4_and_1_caRunTests(void)
{
  /* verify hardware implementation */
  caVerifyPortableWordSizes(&CA_Copter_Plane_h2g_4_and_1_ActHW,
    &CA_Copter_Plane_h2g_4_and_1_ExpHW, &CA_Copter_Plane_h2g_4_and_1_PWSRes);
  caVerifyHWImpl(&CA_Copter_Plane_h2g_4_and_1_ActHW,
                 &CA_Copter_Plane_h2g_4_and_1_ExpHW,
                 &CA_Copter_Plane_h2g_4_and_1_HWRes);
}
