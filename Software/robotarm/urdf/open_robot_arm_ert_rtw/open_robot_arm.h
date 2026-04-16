/*
 * Primary and Secondary School License - for use in teaching and meeting
 * course requirements at primary and secondary schools only.  Not for
 * government, commercial, university, or other organizational use.
 *
 * File: open_robot_arm.h
 *
 * Code generated for Simulink model 'open_robot_arm'.
 *
 * Model version                  : 2.4
 * Simulink Coder version         : 25.2 (R2025b) 28-Jul-2025
 * C/C++ source code generated on : Sat Apr 11 14:57:06 2026
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex-A (64-bit)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef open_robot_arm_h_
#define open_robot_arm_h_
#ifndef open_robot_arm_COMMON_INCLUDES_
#define open_robot_arm_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "rtw_extmode.h"
#include "sysran_types.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "ext_mode.h"
#include "MW_gpio.h"
#include "MW_pigs.h"
#endif                                 /* open_robot_arm_COMMON_INCLUDES_ */

#include "open_robot_arm_types.h"
#include "rt_nonfinite.h"
#include "rtGetInf.h"
#include "rtGetNaN.h"
#include <stddef.h>

/* Macros for accessing real-time model data structure */
#ifndef rtmGetFinalTime
#define rtmGetFinalTime(rtm)           ((rtm)->Timing.tFinal)
#endif

#ifndef rtmGetRTWExtModeInfo
#define rtmGetRTWExtModeInfo(rtm)      ((rtm)->extModeInfo)
#endif

#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

#ifndef rtmGetStopRequested
#define rtmGetStopRequested(rtm)       ((rtm)->Timing.stopRequestedFlag)
#endif

#ifndef rtmSetStopRequested
#define rtmSetStopRequested(rtm, val)  ((rtm)->Timing.stopRequestedFlag = (val))
#endif

#ifndef rtmGetStopRequestedPtr
#define rtmGetStopRequestedPtr(rtm)    (&((rtm)->Timing.stopRequestedFlag))
#endif

#ifndef rtmGetT
#define rtmGetT(rtm)                   (rtmGetTPtr((rtm))[0])
#endif

#ifndef rtmGetTFinal
#define rtmGetTFinal(rtm)              ((rtm)->Timing.tFinal)
#endif

#ifndef rtmGetTPtr
#define rtmGetTPtr(rtm)                ((rtm)->Timing.t)
#endif

/* Block signals (default storage) */
typedef struct {
  s_2pO8TjqrE1iQ6274Qd0KoF_open_T trajPP_data[6];
  g_cell_wrap_open_robot_arm_T coeffsCell[6];
  real_T Round;                        /* '<S1>/Round' */
  real_T Abs;                          /* '<S1>/Abs' */
  real_T TrapezoidalVelocityProfileTraje[6];
                          /* '<Root>/Trapezoidal Velocity Profile Trajectory' */
  real_T TrapezoidalVelocityProfileTra_d[6];
                          /* '<Root>/Trapezoidal Velocity Profile Trajectory' */
  real_T TrapezoidalVelocityProfileTra_j[6];
                          /* '<Root>/Trapezoidal Velocity Profile Trajectory' */
} B_open_robot_arm_T;

/* Block states (default storage) for system '<Root>' */
typedef struct {
  robotics_slcore_internal_bloc_T obj;
                          /* '<Root>/Trapezoidal Velocity Profile Trajectory' */
  raspi_internal_PWMBlock_open__T obj_l;/* '<Root>/PWM' */
  codertarget_raspi_internal_Ra_T obj_m;/* '<Root>/GPIO Write' */
  real_T UnitDelay_DSTATE;             /* '<S1>/Unit Delay' */
} DW_open_robot_arm_T;

/* Parameters (default storage) */
struct P_open_robot_arm_T_ {
  real_T CompareToConstant_const;     /* Mask Parameter: CompareToConstant_const
                                       * Referenced by: '<S2>/Constant'
                                       */
  real_T TrapezoidalVelocityProfileTraje[42];
  /* Expression: [[0.0171; -0.3470; 0.3363; -1.0126; -0.0202; 2.1291], [-0.4433; -1.0328; 0.2438; 0.5897; -0.8809; 3.5442], [-0.4433; -1.2029; 0.3003; 0.5440; -0.9768; 3.4680], [0.0277; 0.2324; -0.7566; -0.0552; -0.5249; 3.0938], [0.6507; -1.4503; 0.9933; -1.0454; -0.7756; 2.2527], [0.6507; -1.5708; 0.9936; -0.9490; -0.8409; 2.3923], [0.0171; -0.3470; 0.3363; -1.0126; -0.0202; 2.1291]]
   * Referenced by: '<Root>/Trapezoidal Velocity Profile Trajectory'
   */
  real_T TrapezoidalVelocityProfileTra_p[6];/* Expression: [ 3;3;3;3;3;3]
                                             * Referenced by: '<Root>/Trapezoidal Velocity Profile Trajectory'
                                             */
  real_T Gain_Gain;                    /* Expression: 10186
                                        * Referenced by: '<S1>/Gain'
                                        */
  real_T UnitDelay_InitialCondition;   /* Expression: 0
                                        * Referenced by: '<S1>/Unit Delay'
                                        */
};

/* Real-time Model Data Structure */
struct tag_RTM_open_robot_arm_T {
  const char_T *errorStatus;
  RTWExtModeInfo *extModeInfo;
  RTWSolverInfo solverInfo;

  /*
   * Sizes:
   * The following substructure contains sizes information
   * for many of the model attributes such as inputs, outputs,
   * dwork, sample times, etc.
   */
  struct {
    uint32_T checksums[4];
  } Sizes;

  /*
   * SpecialInfo:
   * The following substructure contains special information
   * related to other components that are dependent on RTW.
   */
  struct {
    const void *mappingInfo;
  } SpecialInfo;

  /*
   * Timing:
   * The following substructure contains information regarding
   * the timing information for the model.
   */
  struct {
    uint32_T clockTick0;
    time_T stepSize0;
    uint32_T clockTick1;
    time_T tFinal;
    SimTimeStep simTimeStep;
    boolean_T stopRequestedFlag;
    time_T *t;
    time_T tArray[2];
  } Timing;
};

/* Block parameters (default storage) */
extern P_open_robot_arm_T open_robot_arm_P;

/* Block signals (default storage) */
extern B_open_robot_arm_T open_robot_arm_B;

/* Block states (default storage) */
extern DW_open_robot_arm_T open_robot_arm_DW;

/* Model entry point functions */
extern void open_robot_arm_initialize(void);
extern void open_robot_arm_step(void);
extern void open_robot_arm_terminate(void);

/* Real-time Model object */
extern RT_MODEL_open_robot_arm_T *const open_robot_arm_M;
extern volatile boolean_T stopRequested;
extern volatile boolean_T runModel;

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'open_robot_arm'
 * '<S1>'   : 'open_robot_arm/q1'
 * '<S2>'   : 'open_robot_arm/q1/Compare To Constant'
 */
#endif                                 /* open_robot_arm_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
