/*
 * Primary and Secondary School License - for use in teaching and meeting
 * course requirements at primary and secondary schools only.  Not for
 * government, commercial, university, or other organizational use.
 *
 * File: open_robot_arm_data.c
 *
 * Code generated for Simulink model 'open_robot_arm'.
 *
 * Model version                  : 2.3
 * Simulink Coder version         : 25.2 (R2025b) 28-Jul-2025
 * C/C++ source code generated on : Fri Apr 10 21:45:59 2026
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex-A (64-bit)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "open_robot_arm.h"

/* Block parameters (default storage) */
P_open_robot_arm_T open_robot_arm_P = {
  /* Mask Parameter: CompareToConstant_const
   * Referenced by: '<S2>/Constant'
   */
  0.0,

  /* Expression: [[0.0171; -0.3470; 0.3363; -1.0126; -0.0202; 2.1291], [-0.4433; -1.0328; 0.2438; 0.5897; -0.8809; 3.5442], [-0.4433; -1.2029; 0.3003; 0.5440; -0.9768; 3.4680], [0.0277; 0.2324; -0.7566; -0.0552; -0.5249; 3.0938], [0.6507; -1.4503; 0.9933; -1.0454; -0.7756; 2.2527], [0.6507; -1.5708; 0.9936; -0.9490; -0.8409; 2.3923], [0.0171; -0.3470; 0.3363; -1.0126; -0.0202; 2.1291]]
   * Referenced by: '<Root>/Trapezoidal Velocity Profile Trajectory'
   */
  { 0.0171, -0.347, 0.3363, -1.0126, -0.0202, 2.1291, -0.4433, -1.0328, 0.2438,
    0.5897, -0.8809, 3.5442, -0.4433, -1.2029, 0.3003, 0.544, -0.9768, 3.468,
    0.0277, 0.2324, -0.7566, -0.0552, -0.5249, 3.0938, 0.6507, -1.4503, 0.9933,
    -1.0454, -0.7756, 2.2527, 0.6507, -1.5708, 0.9936, -0.949, -0.8409, 2.3923,
    0.0171, -0.347, 0.3363, -1.0126, -0.0202, 2.1291 },

  /* Expression: [ 3;3;3;3;3;3]
   * Referenced by: '<Root>/Trapezoidal Velocity Profile Trajectory'
   */
  { 3.0, 3.0, 3.0, 3.0, 3.0, 3.0 },

  /* Expression: 10186
   * Referenced by: '<S1>/Gain'
   */
  10186.0,

  /* Expression: 0
   * Referenced by: '<S1>/Unit Delay'
   */
  0.0
};

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
