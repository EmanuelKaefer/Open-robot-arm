/*
 * Primary and Secondary School License - for use in teaching and meeting
 * course requirements at primary and secondary schools only.  Not for
 * government, commercial, university, or other organizational use.
 *
 * File: raspitest.c
 *
 * Code generated for Simulink model 'raspitest'.
 *
 * Model version                  : 1.5
 * Simulink Coder version         : 25.2 (R2025b) 28-Jul-2025
 * C/C++ source code generated on : Thu Apr  9 15:55:19 2026
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex-A (64-bit)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "raspitest.h"
#include "raspitest_private.h"
#include "rtwtypes.h"
#include <math.h>

/* Block states (default storage) */
DW_raspitest_T raspitest_DW;

/* Real-time model */
static RT_MODEL_raspitest_T raspitest_M_;
RT_MODEL_raspitest_T *const raspitest_M = &raspitest_M_;
real_T rt_roundd_snf(real_T u)
{
  real_T y;
  if (fabs(u) < 4.503599627370496E+15) {
    if (u >= 0.5) {
      y = floor(u + 0.5);
    } else if (u > -0.5) {
      y = u * 0.0;
    } else {
      y = ceil(u - 0.5);
    }
  } else {
    y = u;
  }

  return y;
}

/* Model step function */
void raspitest_step(void)
{
  real_T y;
  uint8_T tmp;

  /* MATLABSystem: '<Root>/GPIO Write1' incorporates:
   *  Constant: '<Root>/Constant'
   */
  y = rt_roundd_snf(raspitest_P.Constant_Value);
  if (y < 256.0) {
    if (y >= 0.0) {
      tmp = (uint8_T)y;
    } else {
      tmp = 0U;
    }
  } else {
    tmp = MAX_uint8_T;
  }

  EXT_GPIO_write(20U, tmp);

  /* End of MATLABSystem: '<Root>/GPIO Write1' */

  /* Start for MATLABSystem: '<Root>/PWM' incorporates:
   *  Constant: '<Root>/Constant1'
   */
  if (raspitest_P.Constant1_Value <= 1.0) {
    y = raspitest_P.Constant1_Value;
  } else {
    y = 1.0;
  }

  if (!(y >= 0.0)) {
    y = 0.0;
  }

  /* MATLABSystem: '<Root>/PWM' */
  EXT_PWMBlock_setDutyCycle(raspitest_DW.obj.PinNumber, y);

  /* Update absolute time for base rate */
  /* The "clockTick0" counts the number of times the code of this task has
   * been executed. The absolute time is the multiplication of "clockTick0"
   * and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
   * overflow during the application lifespan selected.
   */
  raspitest_M->Timing.taskTime0 =
    ((time_T)(++raspitest_M->Timing.clockTick0)) * raspitest_M->Timing.stepSize0;
}

/* Model initialize function */
void raspitest_initialize(void)
{
  /* Registration code */
  rtmSetTFinal(raspitest_M, 20.0);
  raspitest_M->Timing.stepSize0 = 0.4;

  /* External mode info */
  raspitest_M->Sizes.checksums[0] = (2644618522U);
  raspitest_M->Sizes.checksums[1] = (1065808538U);
  raspitest_M->Sizes.checksums[2] = (2211953803U);
  raspitest_M->Sizes.checksums[3] = (765059149U);

  {
    static const sysRanDType rtAlwaysEnabled = SUBSYS_RAN_BC_ENABLE;
    static RTWExtModeInfo rt_ExtModeInfo;
    static const sysRanDType *systemRan[3];
    raspitest_M->extModeInfo = (&rt_ExtModeInfo);
    rteiSetSubSystemActiveVectorAddresses(&rt_ExtModeInfo, systemRan);
    systemRan[0] = &rtAlwaysEnabled;
    systemRan[1] = &rtAlwaysEnabled;
    systemRan[2] = &rtAlwaysEnabled;
    rteiSetModelMappingInfoPtr(raspitest_M->extModeInfo,
      &raspitest_M->SpecialInfo.mappingInfo);
    rteiSetChecksumsPtr(raspitest_M->extModeInfo, raspitest_M->Sizes.checksums);
    rteiSetTPtr(raspitest_M->extModeInfo, rtmGetTPtr(raspitest_M));
  }

  {
    int32_T ret;
    char_T b[45];
    static const char_T b_0[45] =
      "Unable to configure pin %u for PWM output.\\n";

    /* Start for MATLABSystem: '<Root>/GPIO Write1' */
    raspitest_DW.obj_l.matlabCodegenIsDeleted = false;
    raspitest_DW.obj_l.isInitialized = 1;
    EXT_GPIO_init(20U, true);
    raspitest_DW.obj_l.isSetupComplete = true;

    /* Start for MATLABSystem: '<Root>/PWM' */
    raspitest_DW.obj.matlabCodegenIsDeleted = false;
    raspitest_DW.obj.isInitialized = 1;
    raspitest_DW.obj.PinNumber = 21U;
    ret = EXT_PWMBlock_init(21U, 800U, 0.0);
    if (ret != 0) {
      for (ret = 0; ret < 45; ret++) {
        b[ret] = b_0[ret];
      }

      printf(&b[0], 21U);
    }

    raspitest_DW.obj.isSetupComplete = true;

    /* End of Start for MATLABSystem: '<Root>/PWM' */
  }
}

/* Model terminate function */
void raspitest_terminate(void)
{
  /* Terminate for MATLABSystem: '<Root>/GPIO Write1' */
  if (!raspitest_DW.obj_l.matlabCodegenIsDeleted) {
    raspitest_DW.obj_l.matlabCodegenIsDeleted = true;
    if ((raspitest_DW.obj_l.isInitialized == 1) &&
        raspitest_DW.obj_l.isSetupComplete) {
      EXT_GPIO_terminate(20U);
    }
  }

  /* End of Terminate for MATLABSystem: '<Root>/GPIO Write1' */

  /* Terminate for MATLABSystem: '<Root>/PWM' */
  if (!raspitest_DW.obj.matlabCodegenIsDeleted) {
    raspitest_DW.obj.matlabCodegenIsDeleted = true;
    if ((raspitest_DW.obj.isInitialized == 1) &&
        raspitest_DW.obj.isSetupComplete) {
      EXT_PWMBlock_terminate(raspitest_DW.obj.PinNumber);
    }
  }

  /* End of Terminate for MATLABSystem: '<Root>/PWM' */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
