/*
 * Primary and Secondary School License - for use in teaching and meeting
 * course requirements at primary and secondary schools only.  Not for
 * government, commercial, university, or other organizational use.
 *
 * File: open_robot_arm.c
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
#include "open_robot_arm_types.h"
#include "rtwtypes.h"
#include <string.h>
#include "open_robot_arm_private.h"
#include "rt_nonfinite.h"
#include <math.h>

/* Block signals (default storage) */
B_open_robot_arm_T open_robot_arm_B;

/* Block states (default storage) */
DW_open_robot_arm_T open_robot_arm_DW;

/* Real-time model */
static RT_MODEL_open_robot_arm_T open_robot_arm_M_;
RT_MODEL_open_robot_arm_T *const open_robot_arm_M = &open_robot_arm_M_;

/* Forward declaration for local functions */
static void open_robot_computeProfileParams(real_T i, real_T j, const real_T
  wayPoints[42], const real_T TFi_data[], real_T *vParam, real_T *aParam, real_T
  *tAParam, real_T *tFParam);
static boolean_T open_checkPolyForMultipleBreaks(const real_T breakMat[114]);
static void open_r_processPolynomialResults(const real_T breakMat[114], const
  real_T coeffMat[324], boolean_T hasMultipleBreaks,
  f_cell_wrap_open_robot_arm_T breaksCell[6], g_cell_wrap_open_robot_arm_T
  coeffCell[6]);
static real_T open_robot_arm_maximum(const real_T x[6]);
static void open_robot_arm_ppval(const real_T pp_breaks[21], const real_T
  pp_coefs_data[], const int32_T pp_coefs_size[3], const real_T x[2], real_T
  v_data[], int32_T v_size[2]);
static void open_robot_arm_trapveltraj(const real_T wayPoints[42], const real_T
  varargin_2[6], real_T q[12], real_T qd[12], real_T qdd[12], real_T t[2],
  s_2pO8TjqrE1iQ6274Qd0KoF_open_T ppCell_data[], int32_T *ppCell_size);
static void TrapVelTrajSys_generate1DPVAP_g(const real_T breaks[21], const
  real_T coefs_data[], const int32_T coefs_size[2], real_T pp_breaks[21], real_T
  pp_coefs[60], real_T ppd_breaks[21], real_T ppd_coefs[60], real_T ppdd_coefs
  [60]);
static void TrapVelTrajSys_regeneratePPForm(robotics_slcore_internal_bloc_T *obj,
  const real_T wPts[42]);
static real_T open_robot_arm_ppval_g(const real_T pp_breaks[21], const real_T
  pp_coefs[60], real_T x);
static void open_robot_computeProfileParams(real_T i, real_T j, const real_T
  wayPoints[42], const real_T TFi_data[], real_T *vParam, real_T *aParam, real_T
  *tAParam, real_T *tFParam)
{
  real_T s0;
  real_T s0_tmp;
  real_T sF;
  int32_T deltaSign;
  int32_T s0_tmp_tmp;
  s0_tmp_tmp = ((int32_T)j - 1) * 6;

  /* Start for MATLABSystem: '<Root>/Trapezoidal Velocity Profile Trajectory' */
  s0_tmp = wayPoints[(s0_tmp_tmp + (int32_T)i) - 1];
  s0 = s0_tmp;
  sF = wayPoints[(((int32_T)(j + 1.0) - 1) * 6 + (int32_T)i) - 1];
  deltaSign = 1;

  /* Start for MATLABSystem: '<Root>/Trapezoidal Velocity Profile Trajectory' */
  if (sF < s0_tmp) {
    s0 = sF;
    sF = s0_tmp;
    deltaSign = -1;
  }

  *tFParam = TFi_data[((int32_T)i + s0_tmp_tmp) - 1];

  /* Start for MATLABSystem: '<Root>/Trapezoidal Velocity Profile Trajectory' */
  *vParam = (sF - s0) * 1.5 / *tFParam;
  *tAParam = (*tFParam * *vParam + (s0 - sF)) / *vParam;
  *aParam = *vParam / *tAParam;
  if (s0 == sF) {
    *aParam = 0.0;
    *vParam = 0.0;

    /* Start for MATLABSystem: '<Root>/Trapezoidal Velocity Profile Trajectory' */
    if (rtIsNaN(*tFParam) || (*tFParam == 0.0)) {
      *tFParam = 1.0;
    }

    *tAParam = *tFParam / 3.0;
  }

  *vParam *= (real_T)deltaSign;
  *aParam *= (real_T)deltaSign;
}

static boolean_T open_checkPolyForMultipleBreaks(const real_T breakMat[114])
{
  real_T y[19];
  int32_T b_i;
  int32_T b_k;
  int32_T y_tmp;
  boolean_T exitg1;
  boolean_T hasMultipleBreaks;
  boolean_T y_0;
  hasMultipleBreaks = false;
  for (b_i = 0; b_i < 5; b_i++) {
    for (b_k = 0; b_k < 19; b_k++) {
      /* Start for MATLABSystem: '<Root>/Trapezoidal Velocity Profile Trajectory' */
      y_tmp = 6 * b_k + b_i;
      y[b_k] = fabs(breakMat[y_tmp] - breakMat[y_tmp + 1]);
    }

    y_0 = false;
    b_k = 0;
    exitg1 = false;
    while ((!exitg1) && (b_k < 19)) {
      if (y[b_k] > 2.2204460492503131E-16) {
        y_0 = true;
        exitg1 = true;
      } else {
        b_k++;
      }
    }

    /* Start for MATLABSystem: '<Root>/Trapezoidal Velocity Profile Trajectory' */
    hasMultipleBreaks = (y_0 || hasMultipleBreaks);
  }

  return hasMultipleBreaks;
}

static void open_r_processPolynomialResults(const real_T breakMat[114], const
  real_T coeffMat[324], boolean_T hasMultipleBreaks,
  f_cell_wrap_open_robot_arm_T breaksCell[6], g_cell_wrap_open_robot_arm_T
  coeffCell[6])
{
  int32_T i;
  int32_T i_0;
  if (hasMultipleBreaks) {
    coeffCell[0].f1.size[0] = 18;
    coeffCell[0].f1.size[1] = 3;
    for (i_0 = 0; i_0 < 3; i_0++) {
      for (i = 0; i < 18; i++) {
        /* Start for MATLABSystem: '<Root>/Trapezoidal Velocity Profile Trajectory' */
        coeffCell[0].f1.data[i + coeffCell[0].f1.size[0] * i_0] = coeffMat[6 * i
          + 108 * i_0];
      }
    }

    for (i_0 = 0; i_0 < 19; i_0++) {
      breaksCell[0].f1[i_0] = breakMat[6 * i_0];
    }

    coeffCell[1].f1.size[0] = 18;
    coeffCell[1].f1.size[1] = 3;
    for (i_0 = 0; i_0 < 3; i_0++) {
      for (i = 0; i < 18; i++) {
        /* Start for MATLABSystem: '<Root>/Trapezoidal Velocity Profile Trajectory' */
        coeffCell[1].f1.data[i + coeffCell[1].f1.size[0] * i_0] = coeffMat[(6 *
          i + 108 * i_0) + 1];
      }
    }

    for (i_0 = 0; i_0 < 19; i_0++) {
      breaksCell[1].f1[i_0] = breakMat[6 * i_0 + 1];
    }

    coeffCell[2].f1.size[0] = 18;
    coeffCell[2].f1.size[1] = 3;
    for (i_0 = 0; i_0 < 3; i_0++) {
      for (i = 0; i < 18; i++) {
        /* Start for MATLABSystem: '<Root>/Trapezoidal Velocity Profile Trajectory' */
        coeffCell[2].f1.data[i + coeffCell[2].f1.size[0] * i_0] = coeffMat[(6 *
          i + 108 * i_0) + 2];
      }
    }

    for (i_0 = 0; i_0 < 19; i_0++) {
      breaksCell[2].f1[i_0] = breakMat[6 * i_0 + 2];
    }

    coeffCell[3].f1.size[0] = 18;
    coeffCell[3].f1.size[1] = 3;
    for (i_0 = 0; i_0 < 3; i_0++) {
      for (i = 0; i < 18; i++) {
        /* Start for MATLABSystem: '<Root>/Trapezoidal Velocity Profile Trajectory' */
        coeffCell[3].f1.data[i + coeffCell[3].f1.size[0] * i_0] = coeffMat[(6 *
          i + 108 * i_0) + 3];
      }
    }

    for (i_0 = 0; i_0 < 19; i_0++) {
      breaksCell[3].f1[i_0] = breakMat[6 * i_0 + 3];
    }

    coeffCell[4].f1.size[0] = 18;
    coeffCell[4].f1.size[1] = 3;
    for (i_0 = 0; i_0 < 3; i_0++) {
      for (i = 0; i < 18; i++) {
        /* Start for MATLABSystem: '<Root>/Trapezoidal Velocity Profile Trajectory' */
        coeffCell[4].f1.data[i + coeffCell[4].f1.size[0] * i_0] = coeffMat[(6 *
          i + 108 * i_0) + 4];
      }
    }

    for (i_0 = 0; i_0 < 19; i_0++) {
      breaksCell[4].f1[i_0] = breakMat[6 * i_0 + 4];
    }

    coeffCell[5].f1.size[0] = 18;
    coeffCell[5].f1.size[1] = 3;
    for (i_0 = 0; i_0 < 3; i_0++) {
      for (i = 0; i < 18; i++) {
        /* Start for MATLABSystem: '<Root>/Trapezoidal Velocity Profile Trajectory' */
        coeffCell[5].f1.data[i + coeffCell[5].f1.size[0] * i_0] = coeffMat[(6 *
          i + 108 * i_0) + 5];
      }
    }

    for (i_0 = 0; i_0 < 19; i_0++) {
      breaksCell[5].f1[i_0] = breakMat[6 * i_0 + 5];
    }
  } else {
    coeffCell[0].f1.size[0] = 108;
    coeffCell[0].f1.size[1] = 3;
    memcpy(&coeffCell[0].f1.data[0], &coeffMat[0], 324U * sizeof(real_T));
    for (i_0 = 0; i_0 < 19; i_0++) {
      breaksCell[0].f1[i_0] = breakMat[6 * i_0];
    }

    coeffCell[1].f1.size[0] = 108;
    coeffCell[1].f1.size[1] = 3;
    memcpy(&coeffCell[1].f1.data[0], &coeffMat[0], 324U * sizeof(real_T));
    for (i_0 = 0; i_0 < 19; i_0++) {
      breaksCell[1].f1[i_0] = breakMat[6 * i_0];
    }

    coeffCell[2].f1.size[0] = 108;
    coeffCell[2].f1.size[1] = 3;
    memcpy(&coeffCell[2].f1.data[0], &coeffMat[0], 324U * sizeof(real_T));
    for (i_0 = 0; i_0 < 19; i_0++) {
      breaksCell[2].f1[i_0] = breakMat[6 * i_0];
    }

    coeffCell[3].f1.size[0] = 108;
    coeffCell[3].f1.size[1] = 3;
    memcpy(&coeffCell[3].f1.data[0], &coeffMat[0], 324U * sizeof(real_T));
    for (i_0 = 0; i_0 < 19; i_0++) {
      breaksCell[3].f1[i_0] = breakMat[6 * i_0];
    }

    coeffCell[4].f1.size[0] = 108;
    coeffCell[4].f1.size[1] = 3;
    memcpy(&coeffCell[4].f1.data[0], &coeffMat[0], 324U * sizeof(real_T));
    for (i_0 = 0; i_0 < 19; i_0++) {
      breaksCell[4].f1[i_0] = breakMat[6 * i_0];
    }

    coeffCell[5].f1.size[0] = 108;
    coeffCell[5].f1.size[1] = 3;
    memcpy(&coeffCell[5].f1.data[0], &coeffMat[0], 324U * sizeof(real_T));
    for (i_0 = 0; i_0 < 19; i_0++) {
      breaksCell[5].f1[i_0] = breakMat[6 * i_0];
    }
  }
}

static real_T open_robot_arm_maximum(const real_T x[6])
{
  real_T ex;
  real_T x_0;
  int32_T b_idx;
  int32_T k;
  boolean_T exitg1;

  /* Start for MATLABSystem: '<Root>/Trapezoidal Velocity Profile Trajectory' */
  if (!rtIsNaN(x[0])) {
    b_idx = 1;
  } else {
    b_idx = 0;
    k = 2;
    exitg1 = false;
    while ((!exitg1) && (k < 7)) {
      if (!rtIsNaN(x[k - 1])) {
        b_idx = k;
        exitg1 = true;
      } else {
        k++;
      }
    }
  }

  if (b_idx == 0) {
    /* Start for MATLABSystem: '<Root>/Trapezoidal Velocity Profile Trajectory' */
    ex = x[0];
  } else {
    /* Start for MATLABSystem: '<Root>/Trapezoidal Velocity Profile Trajectory' */
    ex = x[b_idx - 1];
    for (k = b_idx + 1; k < 7; k++) {
      x_0 = x[k - 1];
      if (ex < x_0) {
        ex = x_0;
      }
    }
  }

  return ex;
}

real_T rt_powd_snf(real_T u0, real_T u1)
{
  real_T tmp;
  real_T tmp_0;
  real_T y;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = (rtNaN);
  } else {
    tmp = fabs(u0);
    tmp_0 = fabs(u1);
    if (rtIsInf(u1)) {
      if (tmp == 1.0) {
        y = 1.0;
      } else if (tmp > 1.0) {
        if (u1 > 0.0) {
          y = (rtInf);
        } else {
          y = 0.0;
        }
      } else if (u1 > 0.0) {
        y = 0.0;
      } else {
        y = (rtInf);
      }
    } else if (tmp_0 == 0.0) {
      y = 1.0;
    } else if (tmp_0 == 1.0) {
      if (u1 > 0.0) {
        y = u0;
      } else {
        y = 1.0 / u0;
      }
    } else if (u1 == 2.0) {
      y = u0 * u0;
    } else if ((u1 == 0.5) && (u0 >= 0.0)) {
      y = sqrt(u0);
    } else if ((u0 < 0.0) && (u1 > floor(u1))) {
      y = (rtNaN);
    } else {
      y = pow(u0, u1);
    }
  }

  return y;
}

static void open_robot_arm_ppval(const real_T pp_breaks[21], const real_T
  pp_coefs_data[], const int32_T pp_coefs_size[3], const real_T x[2], real_T
  v_data[], int32_T v_size[2])
{
  real_T xloc;
  int32_T c_ix;
  int32_T coefStride;
  int32_T elementsPerPage;
  int32_T high_i;
  int32_T iv0;
  int32_T low_i;
  int32_T low_ip1;
  int32_T mid_i;
  int32_T tmp;
  elementsPerPage = pp_coefs_size[0];

  /* Start for MATLABSystem: '<Root>/Trapezoidal Velocity Profile Trajectory' */
  coefStride = pp_coefs_size[0] * 20;
  v_size[0] = pp_coefs_size[0];
  v_size[1] = 2;

  /* Start for MATLABSystem: '<Root>/Trapezoidal Velocity Profile Trajectory' */
  if (pp_coefs_size[0] == 1) {
    if (rtIsNaN(x[0])) {
      v_data[0] = (rtNaN);
    } else {
      low_i = 0;
      low_ip1 = 1;
      high_i = 21;
      while (high_i > low_ip1 + 1) {
        mid_i = ((low_i + high_i) + 1) >> 1;
        if (x[0] >= pp_breaks[mid_i - 1]) {
          low_i = mid_i - 1;
          low_ip1 = mid_i;
        } else {
          high_i = mid_i;
        }
      }

      xloc = x[0] - pp_breaks[low_i];
      v_data[0] = (pp_coefs_data[low_i] * xloc + pp_coefs_data[low_i +
                   coefStride]) * xloc + pp_coefs_data[(coefStride << 1) + low_i];
    }

    if (rtIsNaN(x[1])) {
      v_data[1] = (rtNaN);
    } else {
      low_i = 0;
      low_ip1 = 1;
      high_i = 21;
      while (high_i > low_ip1 + 1) {
        mid_i = ((low_i + high_i) + 1) >> 1;
        if (x[1] >= pp_breaks[mid_i - 1]) {
          low_i = mid_i - 1;
          low_ip1 = mid_i;
        } else {
          high_i = mid_i;
        }
      }

      xloc = x[1] - pp_breaks[low_i];
      v_data[1] = (pp_coefs_data[low_i] * xloc + pp_coefs_data[low_i +
                   coefStride]) * xloc + pp_coefs_data[(coefStride << 1) + low_i];
    }
  } else {
    for (c_ix = 0; c_ix < 2; c_ix++) {
      iv0 = c_ix * elementsPerPage - 1;
      if (rtIsNaN(x[c_ix])) {
        for (low_ip1 = 0; low_ip1 < elementsPerPage; low_ip1++) {
          v_data[(iv0 + low_ip1) + 1] = x[c_ix];
        }
      } else {
        low_i = 0;
        low_ip1 = 1;
        high_i = 21;
        while (high_i > low_ip1 + 1) {
          mid_i = ((low_i + high_i) + 1) >> 1;
          if (x[c_ix] >= pp_breaks[mid_i - 1]) {
            low_i = mid_i - 1;
            low_ip1 = mid_i;
          } else {
            high_i = mid_i;
          }
        }

        low_ip1 = low_i * elementsPerPage;
        xloc = x[c_ix] - pp_breaks[low_i];
        for (low_i = 0; low_i < elementsPerPage; low_i++) {
          v_data[(iv0 + low_i) + 1] = pp_coefs_data[low_ip1 + low_i];
        }

        for (low_i = 0; low_i < 2; low_i++) {
          high_i = ((low_i + 1) * coefStride + low_ip1) - 1;
          for (mid_i = 0; mid_i < elementsPerPage; mid_i++) {
            tmp = (mid_i + iv0) + 1;
            v_data[tmp] = pp_coefs_data[(mid_i + high_i) + 1] + v_data[tmp] *
              xloc;
          }
        }
      }
    }
  }
}

static void open_robot_arm_trapveltraj(const real_T wayPoints[42], const real_T
  varargin_2[6], real_T q[12], real_T qd[12], real_T qdd[12], real_T t[2],
  s_2pO8TjqrE1iQ6274Qd0KoF_open_T ppCell_data[], int32_T *ppCell_size)
{
  f_cell_wrap_open_robot_arm_T breaksCell[6];
  real_T b_newCoefs_data[360];
  real_T dCoeffs_data[360];
  real_T ddCoeffs_data[360];
  real_T coefsWithFlatStart_data[342];
  real_T coeffMat[324];
  real_T parameterMat[216];
  real_T breakMat[114];
  real_T b[36];
  real_T b_newBreaks[21];
  real_T breaksWithFlatStart[20];
  real_T newSegmentCoeffs_data[18];
  real_T tmp_data_0[12];
  real_T tmp_data_1[12];
  real_T tmp_data_2[12];
  real_T coefs[9];
  real_T varargin_1[6];
  real_T evalPointVector[3];
  real_T coefs_tmp;
  real_T holdPoint;
  real_T parameterMat_tmp;
  real_T parameterMat_tmp_0;
  real_T segAcc;
  real_T segFTime;
  real_T segVel;
  int32_T evalPointVector_0[3];
  int32_T evalPointVector_1[3];
  int32_T evalPointVector_2[3];
  int32_T b_i;
  int32_T b_jj;
  int32_T cellSelection;
  int32_T g_idx_0;
  int32_T i;
  int32_T ibtile;
  int32_T indivPolyDim;
  int32_T loop_ub;
  int32_T loop_ub_0;
  int32_T loop_ub_1;
  int32_T numComputedPolynomials;
  int8_T tmp_data[108];
  int8_T l_data[6];
  int8_T rowSelection_data[6];
  int8_T tmp[2];
  uint8_T f_data[19];
  uint8_T lspbSegIndices_data[19];
  boolean_T coefIndex[108];
  boolean_T hasMultipleBreaks;
  static const int8_T tmp_0[3] = { 0, 0, 1 };

  int32_T tmp_size[2];
  int32_T tmp_size_0[2];
  int32_T tmp_size_1[2];
  int32_T lspbSegIndices_size_idx_1;
  int32_T rowSelection_size_idx_1;
  for (i = 0; i < 6; i++) {
    /* Start for MATLABSystem: '<Root>/Trapezoidal Velocity Profile Trajectory' */
    ibtile = i * 6 - 1;
    for (numComputedPolynomials = 0; numComputedPolynomials < 6;
         numComputedPolynomials++) {
      /* Start for MATLABSystem: '<Root>/Trapezoidal Velocity Profile Trajectory' */
      b[(ibtile + numComputedPolynomials) + 1] =
        varargin_2[numComputedPolynomials];
    }
  }

  /* Start for MATLABSystem: '<Root>/Trapezoidal Velocity Profile Trajectory' */
  memset(&q[0], 0, 12U * sizeof(real_T));
  memset(&qd[0], 0, 12U * sizeof(real_T));
  memset(&qdd[0], 0, 12U * sizeof(real_T));
  memset(&coeffMat[0], 0, 324U * sizeof(real_T));
  memset(&breakMat[0], 0, 114U * sizeof(real_T));
  for (b_i = 0; b_i < 6; b_i++) {
    for (numComputedPolynomials = 0; numComputedPolynomials < 6;
         numComputedPolynomials++) {
      /* Start for MATLABSystem: '<Root>/Trapezoidal Velocity Profile Trajectory' */
      open_robot_computeProfileParams((real_T)b_i + 1.0, (real_T)
        numComputedPolynomials + 1.0, wayPoints, b, &segVel, &segAcc, &holdPoint,
        &segFTime);
      i = 6 * numComputedPolynomials + b_i;
      parameterMat_tmp = wayPoints[i];
      parameterMat[i] = parameterMat_tmp;

      /* Start for MATLABSystem: '<Root>/Trapezoidal Velocity Profile Trajectory' */
      parameterMat_tmp_0 = wayPoints[(numComputedPolynomials + 1) * 6 + b_i];
      parameterMat[i + 36] = parameterMat_tmp_0;
      parameterMat[i + 72] = segVel;
      parameterMat[i + 108] = segAcc;
      parameterMat[i + 144] = holdPoint;
      parameterMat[i + 180] = segFTime;
      memset(&coefs[0], 0, 9U * sizeof(real_T));

      /* Start for MATLABSystem: '<Root>/Trapezoidal Velocity Profile Trajectory' */
      if (segVel == 0.0) {
        coefs[6] = parameterMat_tmp;
        coefs[7] = parameterMat_tmp;
        coefs[8] = parameterMat_tmp;
      } else {
        coefs_tmp = segAcc / 2.0;
        coefs[0] = coefs_tmp;
        coefs[3] = 0.0;
        coefs[6] = parameterMat_tmp;
        coefs[1] = 0.0;
        coefs[4] = segVel;
        coefs_tmp *= holdPoint * holdPoint;
        coefs[7] = coefs_tmp + parameterMat_tmp;
        coefs[2] = -segAcc / 2.0;
        coefs[5] = segVel;
        coefs[8] = (parameterMat_tmp_0 + coefs_tmp) - segVel * holdPoint;
      }

      memset(&coefIndex[0], 0, 108U * sizeof(boolean_T));

      /* Start for MATLABSystem: '<Root>/Trapezoidal Velocity Profile Trajectory' */
      indivPolyDim = 3 * numComputedPolynomials * 6 + b_i;
      if (indivPolyDim + 13 < indivPolyDim + 1) {
        lspbSegIndices_size_idx_1 = 0;
      } else {
        lspbSegIndices_size_idx_1 = 3;
        for (ibtile = 0; ibtile < 3; ibtile++) {
          lspbSegIndices_data[ibtile] = (uint8_T)((6 * ibtile + indivPolyDim) +
            1);
        }
      }

      for (ibtile = 0; ibtile < lspbSegIndices_size_idx_1; ibtile++) {
        f_data[ibtile] = lspbSegIndices_data[ibtile];
        coefIndex[f_data[ibtile] - 1] = true;
      }

      ibtile = 0;
      for (i = 0; i < 108; i++) {
        if (coefIndex[i]) {
          ibtile++;
        }
      }

      lspbSegIndices_size_idx_1 = ibtile;
      ibtile = 0;
      for (i = 0; i < 108; i++) {
        if (coefIndex[i]) {
          tmp_data[ibtile] = (int8_T)i;
          ibtile++;
        }
      }

      tmp[0] = (int8_T)lspbSegIndices_size_idx_1;
      i = (int8_T)lspbSegIndices_size_idx_1;
      for (ibtile = 0; ibtile < 3; ibtile++) {
        for (lspbSegIndices_size_idx_1 = 0; lspbSegIndices_size_idx_1 < i;
             lspbSegIndices_size_idx_1++) {
          coeffMat[tmp_data[lspbSegIndices_size_idx_1] + 108 * ibtile] =
            coefs[tmp[0] * ibtile + lspbSegIndices_size_idx_1];
        }
      }

      /* Start for MATLABSystem: '<Root>/Trapezoidal Velocity Profile Trajectory' */
      i = (numComputedPolynomials + 1) * 3;
      segVel = breakMat[(i - 3) * 6 + b_i];
      breakMat[b_i + 6 * (i - 2)] = holdPoint + segVel;
      breakMat[b_i + 6 * (i - 1)] = (segFTime - holdPoint) + segVel;
      breakMat[b_i + 6 * i] = segFTime + segVel;
    }
  }

  hasMultipleBreaks = open_checkPolyForMultipleBreaks(breakMat);
  open_r_processPolynomialResults(breakMat, coeffMat, hasMultipleBreaks,
    breaksCell, open_robot_arm_B.coeffsCell);
  for (i = 0; i < 6; i++) {
    /* Start for MATLABSystem: '<Root>/Trapezoidal Velocity Profile Trajectory' */
    varargin_1[i] = parameterMat[i + 180];
  }

  for (ibtile = 0; ibtile < 5; ibtile++) {
    /* Start for MATLABSystem: '<Root>/Trapezoidal Velocity Profile Trajectory' */
    i = (ibtile + 1) * 6 + 179;
    for (numComputedPolynomials = 0; numComputedPolynomials < 6;
         numComputedPolynomials++) {
      /* Start for MATLABSystem: '<Root>/Trapezoidal Velocity Profile Trajectory' */
      varargin_1[numComputedPolynomials] += parameterMat[(i +
        numComputedPolynomials) + 1];
    }
  }

  /* Start for MATLABSystem: '<Root>/Trapezoidal Velocity Profile Trajectory' */
  t[1] = open_robot_arm_maximum(varargin_1);
  t[0] = 0.0;
  if (hasMultipleBreaks) {
    numComputedPolynomials = 6;
    indivPolyDim = 1;
  } else {
    numComputedPolynomials = 1;
    indivPolyDim = 6;
  }

  /* Start for MATLABSystem: '<Root>/Trapezoidal Velocity Profile Trajectory' */
  *ppCell_size = numComputedPolynomials;
  i = indivPolyDim * 3;
  loop_ub_0 = indivPolyDim * 3;
  loop_ub_1 = indivPolyDim * 20 * 3;
  evalPointVector_0[0] = indivPolyDim;
  evalPointVector_0[1] = 20;
  evalPointVector_0[2] = 3;
  evalPointVector_1[0] = indivPolyDim;
  evalPointVector_1[1] = 20;
  evalPointVector_1[2] = 3;
  evalPointVector_2[0] = indivPolyDim;
  evalPointVector_2[1] = 20;
  evalPointVector_2[2] = 3;
  for (b_jj = 0; b_jj < numComputedPolynomials; b_jj++) {
    if (hasMultipleBreaks) {
      rowSelection_size_idx_1 = 1;

      /* Start for MATLABSystem: '<Root>/Trapezoidal Velocity Profile Trajectory' */
      rowSelection_data[0] = (int8_T)(b_jj + 1);
      cellSelection = b_jj + 1;
    } else {
      rowSelection_size_idx_1 = 6;
      for (ibtile = 0; ibtile < 6; ibtile++) {
        rowSelection_data[ibtile] = (int8_T)(ibtile + 1);
      }

      cellSelection = 1;
    }

    memset(&newSegmentCoeffs_data[0], 0, (uint32_T)i * sizeof(real_T));

    /* Start for MATLABSystem: '<Root>/Trapezoidal Velocity Profile Trajectory' */
    memset(&varargin_1[0], 0, (uint32_T)indivPolyDim * sizeof(real_T));
    for (ibtile = 0; ibtile < 3; ibtile++) {
      lspbSegIndices_size_idx_1 = tmp_0[ibtile];
      for (loop_ub = 0; loop_ub < indivPolyDim; loop_ub++) {
        varargin_1[loop_ub] += open_robot_arm_B.coeffsCell[cellSelection - 1].
          f1.data[open_robot_arm_B.coeffsCell[cellSelection - 1].f1.size[0] *
          ibtile + loop_ub] * (real_T)lspbSegIndices_size_idx_1;
      }
    }

    for (ibtile = 0; ibtile < indivPolyDim; ibtile++) {
      newSegmentCoeffs_data[ibtile + (indivPolyDim << 1)] = varargin_1[ibtile];
    }

    g_idx_0 = open_robot_arm_B.coeffsCell[cellSelection - 1].f1.size[0];
    b_i = g_idx_0 + indivPolyDim;
    memset(&coefsWithFlatStart_data[0], 0, (uint32_T)(b_i * 3) * sizeof(real_T));

    /* Start for MATLABSystem: '<Root>/Trapezoidal Velocity Profile Trajectory' */
    ibtile = b_i - indivPolyDim;
    for (lspbSegIndices_size_idx_1 = 0; lspbSegIndices_size_idx_1 < 3;
         lspbSegIndices_size_idx_1++) {
      /* Start for MATLABSystem: '<Root>/Trapezoidal Velocity Profile Trajectory' */
      /* Start for MATLABSystem: '<Root>/Trapezoidal Velocity Profile Trajectory' */
      for (loop_ub = 0; loop_ub < indivPolyDim; loop_ub++) {
        coefsWithFlatStart_data[loop_ub + b_i * lspbSegIndices_size_idx_1] =
          newSegmentCoeffs_data[indivPolyDim * lspbSegIndices_size_idx_1 +
          loop_ub];
      }

      for (loop_ub = 0; loop_ub < g_idx_0; loop_ub++) {
        /* Start for MATLABSystem: '<Root>/Trapezoidal Velocity Profile Trajectory' */
        coefsWithFlatStart_data[(indivPolyDim + loop_ub) + b_i *
          lspbSegIndices_size_idx_1] = open_robot_arm_B.coeffsCell[cellSelection
          - 1].f1.data[open_robot_arm_B.coeffsCell[cellSelection - 1].f1.size[0]
          * lspbSegIndices_size_idx_1 + loop_ub];
      }
    }

    /* Start for MATLABSystem: '<Root>/Trapezoidal Velocity Profile Trajectory' */
    breaksWithFlatStart[0] = breaksCell[cellSelection - 1].f1[0] - 1.0;
    memcpy(&breaksWithFlatStart[1], &breaksCell[cellSelection - 1].f1[0], 19U *
           sizeof(real_T));
    holdPoint = breaksWithFlatStart[19] - breaksWithFlatStart[18];
    evalPointVector[0] = rt_powd_snf(holdPoint, 2.0);
    evalPointVector[1] = rt_powd_snf(holdPoint, 1.0);
    evalPointVector[2] = rt_powd_snf(holdPoint, 0.0);
    if (ibtile + 1 > b_i) {
      cellSelection = 0;
      lspbSegIndices_size_idx_1 = 0;
    } else {
      cellSelection = ibtile;
      lspbSegIndices_size_idx_1 = b_i;
    }

    memset(&newSegmentCoeffs_data[0], 0, (uint32_T)loop_ub_0 * sizeof(real_T));

    /* Start for MATLABSystem: '<Root>/Trapezoidal Velocity Profile Trajectory' */
    ibtile = lspbSegIndices_size_idx_1 - cellSelection;
    if (ibtile - 1 >= 0) {
      memset(&breakMat[0], 0, (uint32_T)ibtile * sizeof(real_T));
    }

    for (lspbSegIndices_size_idx_1 = 0; lspbSegIndices_size_idx_1 < 3;
         lspbSegIndices_size_idx_1++) {
      holdPoint = evalPointVector[lspbSegIndices_size_idx_1];
      for (loop_ub = 0; loop_ub < ibtile; loop_ub++) {
        breakMat[loop_ub] += coefsWithFlatStart_data[(cellSelection + loop_ub) +
          b_i * lspbSegIndices_size_idx_1] * holdPoint;
      }
    }

    for (ibtile = 0; ibtile < indivPolyDim; ibtile++) {
      newSegmentCoeffs_data[ibtile + (indivPolyDim << 1)] = breakMat[ibtile];
    }

    cellSelection = b_i + indivPolyDim;
    loop_ub = cellSelection * 3;
    memset(&b_newCoefs_data[0], 0, (uint32_T)loop_ub * sizeof(real_T));
    for (ibtile = 0; ibtile < 3; ibtile++) {
      for (lspbSegIndices_size_idx_1 = 0; lspbSegIndices_size_idx_1 < b_i;
           lspbSegIndices_size_idx_1++) {
        /* Start for MATLABSystem: '<Root>/Trapezoidal Velocity Profile Trajectory' */
        b_newCoefs_data[lspbSegIndices_size_idx_1 + cellSelection * ibtile] =
          coefsWithFlatStart_data[b_i * ibtile + lspbSegIndices_size_idx_1];
      }
    }

    /* Start for MATLABSystem: '<Root>/Trapezoidal Velocity Profile Trajectory' */
    if (b_i + 1 > cellSelection) {
      b_i = 0;
      ibtile = 0;
    } else {
      ibtile = cellSelection;
    }

    g_idx_0 = ibtile - b_i;
    for (ibtile = 0; ibtile < 3; ibtile++) {
      for (lspbSegIndices_size_idx_1 = 0; lspbSegIndices_size_idx_1 < g_idx_0;
           lspbSegIndices_size_idx_1++) {
        /* Start for MATLABSystem: '<Root>/Trapezoidal Velocity Profile Trajectory' */
        b_newCoefs_data[(b_i + lspbSegIndices_size_idx_1) + cellSelection *
          ibtile] = newSegmentCoeffs_data[indivPolyDim * ibtile +
          lspbSegIndices_size_idx_1];
      }
    }

    /* Start for MATLABSystem: '<Root>/Trapezoidal Velocity Profile Trajectory' */
    memcpy(&b_newBreaks[0], &breaksWithFlatStart[0], 20U * sizeof(real_T));
    b_newBreaks[20] = breaksWithFlatStart[19] + 1.0;
    memset(&dCoeffs_data[0], 0, (uint32_T)loop_ub * sizeof(real_T));
    for (b_i = 0; b_i < 2; b_i++) {
      for (ibtile = 0; ibtile < cellSelection; ibtile++) {
        /* Start for MATLABSystem: '<Root>/Trapezoidal Velocity Profile Trajectory' */
        dCoeffs_data[ibtile + cellSelection * (b_i + 1)] =
          b_newCoefs_data[cellSelection * b_i + ibtile] * (2.0 - (real_T)b_i);
      }
    }

    memset(&ddCoeffs_data[0], 0, (uint32_T)loop_ub * sizeof(real_T));
    for (b_i = 0; b_i < 2; b_i++) {
      for (ibtile = 0; ibtile < cellSelection; ibtile++) {
        /* Start for MATLABSystem: '<Root>/Trapezoidal Velocity Profile Trajectory' */
        ddCoeffs_data[ibtile + cellSelection * (b_i + 1)] =
          dCoeffs_data[cellSelection * b_i + ibtile] * (2.0 - (real_T)b_i);
      }
    }

    /* Start for MATLABSystem: '<Root>/Trapezoidal Velocity Profile Trajectory' */
    memcpy(&ppCell_data[b_jj].breaks[0], &b_newBreaks[0], 21U * sizeof(real_T));
    ppCell_data[b_jj].coefs.size[0] = indivPolyDim;
    ppCell_data[b_jj].coefs.size[1] = 20;
    ppCell_data[b_jj].coefs.size[2] = 3;
    for (ibtile = 0; ibtile < loop_ub_1; ibtile++) {
      /* Start for MATLABSystem: '<Root>/Trapezoidal Velocity Profile Trajectory' */
      ppCell_data[b_jj].coefs.data[ibtile] = b_newCoefs_data[ibtile];
    }

    for (ibtile = 0; ibtile < rowSelection_size_idx_1; ibtile++) {
      l_data[ibtile] = (int8_T)(rowSelection_data[ibtile] - 1);
    }

    /* Start for MATLABSystem: '<Root>/Trapezoidal Velocity Profile Trajectory' */
    open_robot_arm_ppval(b_newBreaks, b_newCoefs_data, evalPointVector_0, t,
                         tmp_data_0, tmp_size);
    open_robot_arm_ppval(b_newBreaks, dCoeffs_data, evalPointVector_1, t,
                         tmp_data_1, tmp_size_0);
    open_robot_arm_ppval(b_newBreaks, ddCoeffs_data, evalPointVector_2, t,
                         tmp_data_2, tmp_size_1);
    for (ibtile = 0; ibtile < 2; ibtile++) {
      for (lspbSegIndices_size_idx_1 = 0; lspbSegIndices_size_idx_1 <
           rowSelection_size_idx_1; lspbSegIndices_size_idx_1++) {
        b_i = 6 * ibtile + l_data[lspbSegIndices_size_idx_1];
        q[b_i] = tmp_data_0[tmp_size[0] * ibtile + lspbSegIndices_size_idx_1];
        qd[b_i] = tmp_data_1[tmp_size_0[0] * ibtile + lspbSegIndices_size_idx_1];
        qdd[b_i] = tmp_data_2[tmp_size_1[0] * ibtile + lspbSegIndices_size_idx_1];
      }
    }
  }
}

static void TrapVelTrajSys_generate1DPVAP_g(const real_T breaks[21], const
  real_T coefs_data[], const int32_T coefs_size[2], real_T pp_breaks[21], real_T
  pp_coefs[60], real_T ppd_breaks[21], real_T ppd_coefs[60], real_T ppdd_coefs
  [60])
{
  real_T dCoefs_data[360];
  real_T ddCoefs_data[360];
  int32_T b_i;
  int32_T b_idx_0;
  int32_T dCoefs_size_idx_0;
  int32_T ddCoefs_size_idx_0;
  int32_T i;
  int32_T loop_ub;
  b_idx_0 = coefs_size[0];
  dCoefs_size_idx_0 = coefs_size[0];
  loop_ub = coefs_size[0] * 3;
  if (loop_ub - 1 >= 0) {
    memset(&dCoefs_data[0], 0, (uint32_T)loop_ub * sizeof(real_T));
  }

  for (b_i = 0; b_i < 2; b_i++) {
    /* Start for MATLABSystem: '<Root>/Trapezoidal Velocity Profile Trajectory' */
    for (i = 0; i < b_idx_0; i++) {
      dCoefs_data[i + dCoefs_size_idx_0 * (b_i + 1)] = coefs_data[coefs_size[0] *
        b_i + i] * (2.0 - (real_T)b_i);
    }
  }

  b_idx_0 = coefs_size[0];
  ddCoefs_size_idx_0 = coefs_size[0];
  if (loop_ub - 1 >= 0) {
    memset(&ddCoefs_data[0], 0, (uint32_T)loop_ub * sizeof(real_T));
  }

  for (b_i = 0; b_i < 2; b_i++) {
    /* Start for MATLABSystem: '<Root>/Trapezoidal Velocity Profile Trajectory' */
    for (i = 0; i < b_idx_0; i++) {
      ddCoefs_data[i + ddCoefs_size_idx_0 * (b_i + 1)] =
        dCoefs_data[dCoefs_size_idx_0 * b_i + i] * (2.0 - (real_T)b_i);
    }
  }

  /* Start for MATLABSystem: '<Root>/Trapezoidal Velocity Profile Trajectory' */
  memcpy(&pp_breaks[0], &breaks[0], 21U * sizeof(real_T));
  memcpy(&pp_coefs[0], &coefs_data[0], 60U * sizeof(real_T));
  memcpy(&ppd_breaks[0], &breaks[0], 21U * sizeof(real_T));
  memcpy(&ppd_coefs[0], &dCoefs_data[0], 60U * sizeof(real_T));
  memcpy(&ppdd_coefs[0], &ddCoefs_data[0], 60U * sizeof(real_T));
}

static void TrapVelTrajSys_regeneratePPForm(robotics_slcore_internal_bloc_T *obj,
  const real_T wPts[42])
{
  real_T evalCoeffs_data[360];
  real_T oneDimCoeffs_data[360];
  real_T a__10[12];
  real_T a__11[12];
  real_T a__9[12];
  real_T a__12[2];
  real_T *breaks;
  int32_T evalCoeffs_size[2];
  int32_T evalCoeffs_data_tmp;
  int32_T i;
  int32_T loop_ub;
  int32_T oneDimCoeffs;
  int32_T trajPP_size;
  int8_T trajPP[2];

  /* Start for MATLABSystem: '<Root>/Trapezoidal Velocity Profile Trajectory' */
  open_robot_arm_trapveltraj(wPts, obj->EndTime, a__9, a__10, a__11, a__12,
    open_robot_arm_B.trajPP_data, &trajPP_size);
  if (trajPP_size > 1) {
    /* Start for MATLABSystem: '<Root>/Trapezoidal Velocity Profile Trajectory' */
    breaks = &open_robot_arm_B.trajPP_data[0].breaks[0];
    oneDimCoeffs = (int32_T)((uint32_T)(open_robot_arm_B.trajPP_data[0].
      coefs.size[0] * 20 * 3) / 3U);
    loop_ub = oneDimCoeffs * 3;
    for (i = 0; i < loop_ub; i++) {
      /* Start for MATLABSystem: '<Root>/Trapezoidal Velocity Profile Trajectory' */
      oneDimCoeffs_data[i] = open_robot_arm_B.trajPP_data[0].coefs.data[i];
    }
  } else {
    /* Start for MATLABSystem: '<Root>/Trapezoidal Velocity Profile Trajectory' */
    breaks = &open_robot_arm_B.trajPP_data[0].breaks[0];
    trajPP[0] = (int8_T)((uint32_T)(open_robot_arm_B.trajPP_data[0].coefs.size[0]
      * 20 * 3) / 3U);
    oneDimCoeffs = 20;
    for (i = 0; i < 3; i++) {
      for (loop_ub = 0; loop_ub < 20; loop_ub++) {
        /* Start for MATLABSystem: '<Root>/Trapezoidal Velocity Profile Trajectory' */
        oneDimCoeffs_data[loop_ub + 20 * i] = open_robot_arm_B.trajPP_data[0].
          coefs.data[6 * loop_ub + trajPP[0] * i];
      }
    }
  }

  evalCoeffs_size[0] = oneDimCoeffs;
  evalCoeffs_size[1] = 3;
  loop_ub = oneDimCoeffs * 3;
  if (loop_ub - 1 >= 0) {
    memset(&evalCoeffs_data[0], 0, (uint32_T)loop_ub * sizeof(real_T));
  }

  for (i = 0; i < 3; i++) {
    for (loop_ub = 0; loop_ub < oneDimCoeffs; loop_ub++) {
      evalCoeffs_data_tmp = oneDimCoeffs * i;

      /* Start for MATLABSystem: '<Root>/Trapezoidal Velocity Profile Trajectory' */
      evalCoeffs_data[loop_ub + evalCoeffs_data_tmp] = oneDimCoeffs_data[loop_ub
        + evalCoeffs_data_tmp];
    }
  }

  /* Start for MATLABSystem: '<Root>/Trapezoidal Velocity Profile Trajectory' */
  memcpy(&obj->PPDDCell[0].breaks[0], &breaks[0], 21U * sizeof(real_T));
  TrapVelTrajSys_generate1DPVAP_g(obj->PPDDCell[0].breaks, evalCoeffs_data,
    evalCoeffs_size, obj->PPCell[0].breaks, obj->PPCell[0].coefs, obj->PPDCell[0]
    .breaks, obj->PPDCell[0].coefs, obj->PPDDCell[0].coefs);
  if (trajPP_size > 1) {
    /* Start for MATLABSystem: '<Root>/Trapezoidal Velocity Profile Trajectory' */
    breaks = &open_robot_arm_B.trajPP_data[1].breaks[0];
    oneDimCoeffs = (int32_T)((uint32_T)(open_robot_arm_B.trajPP_data[1].
      coefs.size[0] * 20 * 3) / 3U);
    loop_ub = oneDimCoeffs * 3;
    for (i = 0; i < loop_ub; i++) {
      /* Start for MATLABSystem: '<Root>/Trapezoidal Velocity Profile Trajectory' */
      oneDimCoeffs_data[i] = open_robot_arm_B.trajPP_data[1].coefs.data[i];
    }
  } else {
    /* Start for MATLABSystem: '<Root>/Trapezoidal Velocity Profile Trajectory' */
    trajPP[0] = (int8_T)((uint32_T)(open_robot_arm_B.trajPP_data[0].coefs.size[0]
      * 20 * 3) / 3U);
    oneDimCoeffs = 20;
    for (i = 0; i < 3; i++) {
      for (loop_ub = 0; loop_ub < 20; loop_ub++) {
        /* Start for MATLABSystem: '<Root>/Trapezoidal Velocity Profile Trajectory' */
        oneDimCoeffs_data[loop_ub + 20 * i] = open_robot_arm_B.trajPP_data[0].
          coefs.data[(6 * loop_ub + trajPP[0] * i) + 1];
      }
    }
  }

  evalCoeffs_size[0] = oneDimCoeffs;
  evalCoeffs_size[1] = 3;
  loop_ub = oneDimCoeffs * 3;
  if (loop_ub - 1 >= 0) {
    memset(&evalCoeffs_data[0], 0, (uint32_T)loop_ub * sizeof(real_T));
  }

  for (i = 0; i < 3; i++) {
    for (loop_ub = 0; loop_ub < oneDimCoeffs; loop_ub++) {
      evalCoeffs_data_tmp = oneDimCoeffs * i;

      /* Start for MATLABSystem: '<Root>/Trapezoidal Velocity Profile Trajectory' */
      evalCoeffs_data[loop_ub + evalCoeffs_data_tmp] = oneDimCoeffs_data[loop_ub
        + evalCoeffs_data_tmp];
    }
  }

  /* Start for MATLABSystem: '<Root>/Trapezoidal Velocity Profile Trajectory' */
  memcpy(&obj->PPDDCell[1].breaks[0], &breaks[0], 21U * sizeof(real_T));
  TrapVelTrajSys_generate1DPVAP_g(obj->PPDDCell[1].breaks, evalCoeffs_data,
    evalCoeffs_size, obj->PPCell[1].breaks, obj->PPCell[1].coefs, obj->PPDCell[1]
    .breaks, obj->PPDCell[1].coefs, obj->PPDDCell[1].coefs);
  if (trajPP_size > 1) {
    /* Start for MATLABSystem: '<Root>/Trapezoidal Velocity Profile Trajectory' */
    breaks = &open_robot_arm_B.trajPP_data[2].breaks[0];
    oneDimCoeffs = (int32_T)((uint32_T)(open_robot_arm_B.trajPP_data[2].
      coefs.size[0] * 20 * 3) / 3U);
    loop_ub = oneDimCoeffs * 3;
    for (i = 0; i < loop_ub; i++) {
      /* Start for MATLABSystem: '<Root>/Trapezoidal Velocity Profile Trajectory' */
      oneDimCoeffs_data[i] = open_robot_arm_B.trajPP_data[2].coefs.data[i];
    }
  } else {
    /* Start for MATLABSystem: '<Root>/Trapezoidal Velocity Profile Trajectory' */
    breaks = &open_robot_arm_B.trajPP_data[0].breaks[0];
    trajPP[0] = (int8_T)((uint32_T)(open_robot_arm_B.trajPP_data[0].coefs.size[0]
      * 20 * 3) / 3U);
    oneDimCoeffs = 20;
    for (i = 0; i < 3; i++) {
      for (loop_ub = 0; loop_ub < 20; loop_ub++) {
        /* Start for MATLABSystem: '<Root>/Trapezoidal Velocity Profile Trajectory' */
        oneDimCoeffs_data[loop_ub + 20 * i] = open_robot_arm_B.trajPP_data[0].
          coefs.data[(6 * loop_ub + trajPP[0] * i) + 2];
      }
    }
  }

  evalCoeffs_size[0] = oneDimCoeffs;
  evalCoeffs_size[1] = 3;
  loop_ub = oneDimCoeffs * 3;
  if (loop_ub - 1 >= 0) {
    memset(&evalCoeffs_data[0], 0, (uint32_T)loop_ub * sizeof(real_T));
  }

  for (i = 0; i < 3; i++) {
    for (loop_ub = 0; loop_ub < oneDimCoeffs; loop_ub++) {
      evalCoeffs_data_tmp = oneDimCoeffs * i;

      /* Start for MATLABSystem: '<Root>/Trapezoidal Velocity Profile Trajectory' */
      evalCoeffs_data[loop_ub + evalCoeffs_data_tmp] = oneDimCoeffs_data[loop_ub
        + evalCoeffs_data_tmp];
    }
  }

  /* Start for MATLABSystem: '<Root>/Trapezoidal Velocity Profile Trajectory' */
  memcpy(&obj->PPDDCell[2].breaks[0], &breaks[0], 21U * sizeof(real_T));
  TrapVelTrajSys_generate1DPVAP_g(obj->PPDDCell[2].breaks, evalCoeffs_data,
    evalCoeffs_size, obj->PPCell[2].breaks, obj->PPCell[2].coefs, obj->PPDCell[2]
    .breaks, obj->PPDCell[2].coefs, obj->PPDDCell[2].coefs);
  if (trajPP_size > 1) {
    /* Start for MATLABSystem: '<Root>/Trapezoidal Velocity Profile Trajectory' */
    breaks = &open_robot_arm_B.trajPP_data[3].breaks[0];
    oneDimCoeffs = (int32_T)((uint32_T)(open_robot_arm_B.trajPP_data[3].
      coefs.size[0] * 20 * 3) / 3U);
    loop_ub = oneDimCoeffs * 3;
    for (i = 0; i < loop_ub; i++) {
      /* Start for MATLABSystem: '<Root>/Trapezoidal Velocity Profile Trajectory' */
      oneDimCoeffs_data[i] = open_robot_arm_B.trajPP_data[3].coefs.data[i];
    }
  } else {
    /* Start for MATLABSystem: '<Root>/Trapezoidal Velocity Profile Trajectory' */
    breaks = &open_robot_arm_B.trajPP_data[0].breaks[0];
    trajPP[0] = (int8_T)((uint32_T)(open_robot_arm_B.trajPP_data[0].coefs.size[0]
      * 20 * 3) / 3U);
    oneDimCoeffs = 20;
    for (i = 0; i < 3; i++) {
      for (loop_ub = 0; loop_ub < 20; loop_ub++) {
        /* Start for MATLABSystem: '<Root>/Trapezoidal Velocity Profile Trajectory' */
        oneDimCoeffs_data[loop_ub + 20 * i] = open_robot_arm_B.trajPP_data[0].
          coefs.data[(6 * loop_ub + trajPP[0] * i) + 3];
      }
    }
  }

  evalCoeffs_size[0] = oneDimCoeffs;
  evalCoeffs_size[1] = 3;
  loop_ub = oneDimCoeffs * 3;
  if (loop_ub - 1 >= 0) {
    memset(&evalCoeffs_data[0], 0, (uint32_T)loop_ub * sizeof(real_T));
  }

  for (i = 0; i < 3; i++) {
    for (loop_ub = 0; loop_ub < oneDimCoeffs; loop_ub++) {
      evalCoeffs_data_tmp = oneDimCoeffs * i;

      /* Start for MATLABSystem: '<Root>/Trapezoidal Velocity Profile Trajectory' */
      evalCoeffs_data[loop_ub + evalCoeffs_data_tmp] = oneDimCoeffs_data[loop_ub
        + evalCoeffs_data_tmp];
    }
  }

  /* Start for MATLABSystem: '<Root>/Trapezoidal Velocity Profile Trajectory' */
  memcpy(&obj->PPDDCell[3].breaks[0], &breaks[0], 21U * sizeof(real_T));
  TrapVelTrajSys_generate1DPVAP_g(obj->PPDDCell[3].breaks, evalCoeffs_data,
    evalCoeffs_size, obj->PPCell[3].breaks, obj->PPCell[3].coefs, obj->PPDCell[3]
    .breaks, obj->PPDCell[3].coefs, obj->PPDDCell[3].coefs);
  if (trajPP_size > 1) {
    /* Start for MATLABSystem: '<Root>/Trapezoidal Velocity Profile Trajectory' */
    breaks = &open_robot_arm_B.trajPP_data[4].breaks[0];
    oneDimCoeffs = (int32_T)((uint32_T)(open_robot_arm_B.trajPP_data[4].
      coefs.size[0] * 20 * 3) / 3U);
    loop_ub = oneDimCoeffs * 3;
    for (i = 0; i < loop_ub; i++) {
      /* Start for MATLABSystem: '<Root>/Trapezoidal Velocity Profile Trajectory' */
      oneDimCoeffs_data[i] = open_robot_arm_B.trajPP_data[4].coefs.data[i];
    }
  } else {
    /* Start for MATLABSystem: '<Root>/Trapezoidal Velocity Profile Trajectory' */
    breaks = &open_robot_arm_B.trajPP_data[0].breaks[0];
    trajPP[0] = (int8_T)((uint32_T)(open_robot_arm_B.trajPP_data[0].coefs.size[0]
      * 20 * 3) / 3U);
    oneDimCoeffs = 20;
    for (i = 0; i < 3; i++) {
      for (loop_ub = 0; loop_ub < 20; loop_ub++) {
        /* Start for MATLABSystem: '<Root>/Trapezoidal Velocity Profile Trajectory' */
        oneDimCoeffs_data[loop_ub + 20 * i] = open_robot_arm_B.trajPP_data[0].
          coefs.data[(6 * loop_ub + trajPP[0] * i) + 4];
      }
    }
  }

  evalCoeffs_size[0] = oneDimCoeffs;
  evalCoeffs_size[1] = 3;
  loop_ub = oneDimCoeffs * 3;
  if (loop_ub - 1 >= 0) {
    memset(&evalCoeffs_data[0], 0, (uint32_T)loop_ub * sizeof(real_T));
  }

  for (i = 0; i < 3; i++) {
    for (loop_ub = 0; loop_ub < oneDimCoeffs; loop_ub++) {
      evalCoeffs_data_tmp = oneDimCoeffs * i;

      /* Start for MATLABSystem: '<Root>/Trapezoidal Velocity Profile Trajectory' */
      evalCoeffs_data[loop_ub + evalCoeffs_data_tmp] = oneDimCoeffs_data[loop_ub
        + evalCoeffs_data_tmp];
    }
  }

  /* Start for MATLABSystem: '<Root>/Trapezoidal Velocity Profile Trajectory' */
  memcpy(&obj->PPDDCell[4].breaks[0], &breaks[0], 21U * sizeof(real_T));
  TrapVelTrajSys_generate1DPVAP_g(obj->PPDDCell[4].breaks, evalCoeffs_data,
    evalCoeffs_size, obj->PPCell[4].breaks, obj->PPCell[4].coefs, obj->PPDCell[4]
    .breaks, obj->PPDCell[4].coefs, obj->PPDDCell[4].coefs);
  if (trajPP_size > 1) {
    /* Start for MATLABSystem: '<Root>/Trapezoidal Velocity Profile Trajectory' */
    breaks = &open_robot_arm_B.trajPP_data[5].breaks[0];
    oneDimCoeffs = (int32_T)((uint32_T)(open_robot_arm_B.trajPP_data[5].
      coefs.size[0] * 20 * 3) / 3U);
    loop_ub = oneDimCoeffs * 3;
    for (i = 0; i < loop_ub; i++) {
      /* Start for MATLABSystem: '<Root>/Trapezoidal Velocity Profile Trajectory' */
      oneDimCoeffs_data[i] = open_robot_arm_B.trajPP_data[5].coefs.data[i];
    }
  } else {
    /* Start for MATLABSystem: '<Root>/Trapezoidal Velocity Profile Trajectory' */
    breaks = &open_robot_arm_B.trajPP_data[0].breaks[0];
    trajPP[0] = (int8_T)((uint32_T)(open_robot_arm_B.trajPP_data[0].coefs.size[0]
      * 20 * 3) / 3U);
    oneDimCoeffs = 20;
    for (i = 0; i < 3; i++) {
      for (loop_ub = 0; loop_ub < 20; loop_ub++) {
        /* Start for MATLABSystem: '<Root>/Trapezoidal Velocity Profile Trajectory' */
        oneDimCoeffs_data[loop_ub + 20 * i] = open_robot_arm_B.trajPP_data[0].
          coefs.data[(6 * loop_ub + trajPP[0] * i) + 5];
      }
    }
  }

  evalCoeffs_size[0] = oneDimCoeffs;
  evalCoeffs_size[1] = 3;
  loop_ub = oneDimCoeffs * 3;
  if (loop_ub - 1 >= 0) {
    memset(&evalCoeffs_data[0], 0, (uint32_T)loop_ub * sizeof(real_T));
  }

  for (i = 0; i < 3; i++) {
    for (loop_ub = 0; loop_ub < oneDimCoeffs; loop_ub++) {
      evalCoeffs_data_tmp = oneDimCoeffs * i;

      /* Start for MATLABSystem: '<Root>/Trapezoidal Velocity Profile Trajectory' */
      evalCoeffs_data[loop_ub + evalCoeffs_data_tmp] = oneDimCoeffs_data[loop_ub
        + evalCoeffs_data_tmp];
    }
  }

  /* Start for MATLABSystem: '<Root>/Trapezoidal Velocity Profile Trajectory' */
  memcpy(&obj->PPDDCell[5].breaks[0], &breaks[0], 21U * sizeof(real_T));
  TrapVelTrajSys_generate1DPVAP_g(obj->PPDDCell[5].breaks, evalCoeffs_data,
    evalCoeffs_size, obj->PPCell[5].breaks, obj->PPCell[5].coefs, obj->PPDCell[5]
    .breaks, obj->PPDCell[5].coefs, obj->PPDDCell[5].coefs);
  obj->PPFormUpdatedNeeded = false;
}

static real_T open_robot_arm_ppval_g(const real_T pp_breaks[21], const real_T
  pp_coefs[60], real_T x)
{
  real_T v;
  real_T xloc;
  int32_T high_i;
  int32_T low_i;
  int32_T low_ip1;
  int32_T mid_i;

  /* Start for MATLABSystem: '<Root>/Trapezoidal Velocity Profile Trajectory' */
  if (rtIsNaN(x)) {
    v = (rtNaN);
  } else {
    low_i = 0;
    low_ip1 = 1;
    high_i = 21;
    while (high_i > low_ip1 + 1) {
      mid_i = ((low_i + high_i) + 1) >> 1;
      if (x >= pp_breaks[mid_i - 1]) {
        low_i = mid_i - 1;
        low_ip1 = mid_i;
      } else {
        high_i = mid_i;
      }
    }

    xloc = x - pp_breaks[low_i];
    v = (xloc * pp_coefs[low_i] + pp_coefs[low_i + 20]) * xloc + pp_coefs[low_i
      + 40];
  }

  /* End of Start for MATLABSystem: '<Root>/Trapezoidal Velocity Profile Trajectory' */
  return v;
}

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
void open_robot_arm_step(void)
{
  real_T rtb_Clock;
  real_T tmp;
  int32_T i;
  boolean_T exitg1;
  boolean_T p;
  boolean_T propChange;

  /* Clock: '<Root>/Clock' */
  rtb_Clock = open_robot_arm_M->Timing.t[0];

  /* MATLABSystem: '<Root>/Trapezoidal Velocity Profile Trajectory' */
  propChange = false;
  p = true;
  i = 0;
  exitg1 = false;
  while ((!exitg1) && (i < 42)) {
    if (!(open_robot_arm_DW.obj.Waypoints[i] ==
          open_robot_arm_P.TrapezoidalVelocityProfileTraje[i])) {
      p = false;
      exitg1 = true;
    } else {
      i++;
    }
  }

  if (p) {
    propChange = true;
  }

  if (!propChange) {
    if (open_robot_arm_DW.obj.isInitialized == 1) {
      open_robot_arm_DW.obj.TunablePropsChanged = true;
      open_robot_arm_DW.obj.tunablePropertyChanged[0] = true;
    }

    memcpy(&open_robot_arm_DW.obj.Waypoints[0],
           &open_robot_arm_P.TrapezoidalVelocityProfileTraje[0], 42U * sizeof
           (real_T));
  }

  propChange = false;
  p = true;
  i = 0;
  exitg1 = false;
  while ((!exitg1) && (i < 6)) {
    if (!(open_robot_arm_DW.obj.EndTime[i] ==
          open_robot_arm_P.TrapezoidalVelocityProfileTra_p[i])) {
      p = false;
      exitg1 = true;
    } else {
      i++;
    }
  }

  if (p) {
    propChange = true;
  }

  if (!propChange) {
    if (open_robot_arm_DW.obj.isInitialized == 1) {
      open_robot_arm_DW.obj.TunablePropsChanged = true;
      open_robot_arm_DW.obj.tunablePropertyChanged[3] = true;
    }

    for (i = 0; i < 6; i++) {
      open_robot_arm_DW.obj.EndTime[i] =
        open_robot_arm_P.TrapezoidalVelocityProfileTra_p[i];
    }
  }

  if (open_robot_arm_DW.obj.TunablePropsChanged) {
    open_robot_arm_DW.obj.TunablePropsChanged = false;
    propChange = open_robot_arm_DW.obj.tunablePropertyChanged[0];
    if (!open_robot_arm_DW.obj.tunablePropertyChanged[0]) {
      if (open_robot_arm_DW.obj.tunablePropertyChanged[1] ||
          open_robot_arm_DW.obj.tunablePropertyChanged[2] ||
          open_robot_arm_DW.obj.tunablePropertyChanged[3] ||
          open_robot_arm_DW.obj.tunablePropertyChanged[4]) {
        propChange = true;
      } else {
        propChange = false;
      }
    }

    open_robot_arm_DW.obj.PPFormUpdatedNeeded = propChange;
    if (open_robot_arm_DW.obj.PPFormUpdatedNeeded) {
      TrapVelTrajSys_regeneratePPForm(&open_robot_arm_DW.obj,
        open_robot_arm_DW.obj.Waypoints);
    }

    for (i = 0; i < 5; i++) {
      open_robot_arm_DW.obj.tunablePropertyChanged[i] = false;
    }
  }

  /* Start for MATLABSystem: '<Root>/Trapezoidal Velocity Profile Trajectory' */
  open_robot_arm_B.TrapezoidalVelocityProfileTraje[0] = open_robot_arm_ppval_g
    (open_robot_arm_DW.obj.PPCell[0].breaks, open_robot_arm_DW.obj.PPCell[0].
     coefs, rtb_Clock);
  open_robot_arm_B.TrapezoidalVelocityProfileTraje[1] = open_robot_arm_ppval_g
    (open_robot_arm_DW.obj.PPCell[1].breaks, open_robot_arm_DW.obj.PPCell[1].
     coefs, rtb_Clock);
  open_robot_arm_B.TrapezoidalVelocityProfileTraje[2] = open_robot_arm_ppval_g
    (open_robot_arm_DW.obj.PPCell[2].breaks, open_robot_arm_DW.obj.PPCell[2].
     coefs, rtb_Clock);
  open_robot_arm_B.TrapezoidalVelocityProfileTraje[3] = open_robot_arm_ppval_g
    (open_robot_arm_DW.obj.PPCell[3].breaks, open_robot_arm_DW.obj.PPCell[3].
     coefs, rtb_Clock);
  open_robot_arm_B.TrapezoidalVelocityProfileTraje[4] = open_robot_arm_ppval_g
    (open_robot_arm_DW.obj.PPCell[4].breaks, open_robot_arm_DW.obj.PPCell[4].
     coefs, rtb_Clock);
  open_robot_arm_B.TrapezoidalVelocityProfileTraje[5] = open_robot_arm_ppval_g
    (open_robot_arm_DW.obj.PPCell[5].breaks, open_robot_arm_DW.obj.PPCell[5].
     coefs, rtb_Clock);
  open_robot_arm_B.TrapezoidalVelocityProfileTra_d[0] = open_robot_arm_ppval_g
    (open_robot_arm_DW.obj.PPDCell[0].breaks, open_robot_arm_DW.obj.PPDCell[0].
     coefs, rtb_Clock);
  open_robot_arm_B.TrapezoidalVelocityProfileTra_d[1] = open_robot_arm_ppval_g
    (open_robot_arm_DW.obj.PPDCell[1].breaks, open_robot_arm_DW.obj.PPDCell[1].
     coefs, rtb_Clock);
  open_robot_arm_B.TrapezoidalVelocityProfileTra_d[2] = open_robot_arm_ppval_g
    (open_robot_arm_DW.obj.PPDCell[2].breaks, open_robot_arm_DW.obj.PPDCell[2].
     coefs, rtb_Clock);
  open_robot_arm_B.TrapezoidalVelocityProfileTra_d[3] = open_robot_arm_ppval_g
    (open_robot_arm_DW.obj.PPDCell[3].breaks, open_robot_arm_DW.obj.PPDCell[3].
     coefs, rtb_Clock);
  open_robot_arm_B.TrapezoidalVelocityProfileTra_d[4] = open_robot_arm_ppval_g
    (open_robot_arm_DW.obj.PPDCell[4].breaks, open_robot_arm_DW.obj.PPDCell[4].
     coefs, rtb_Clock);
  open_robot_arm_B.TrapezoidalVelocityProfileTra_d[5] = open_robot_arm_ppval_g
    (open_robot_arm_DW.obj.PPDCell[5].breaks, open_robot_arm_DW.obj.PPDCell[5].
     coefs, rtb_Clock);
  open_robot_arm_B.TrapezoidalVelocityProfileTra_j[0] = open_robot_arm_ppval_g
    (open_robot_arm_DW.obj.PPDDCell[0].breaks, open_robot_arm_DW.obj.PPDDCell[0]
     .coefs, rtb_Clock);
  open_robot_arm_B.TrapezoidalVelocityProfileTra_j[1] = open_robot_arm_ppval_g
    (open_robot_arm_DW.obj.PPDDCell[1].breaks, open_robot_arm_DW.obj.PPDDCell[1]
     .coefs, rtb_Clock);
  open_robot_arm_B.TrapezoidalVelocityProfileTra_j[2] = open_robot_arm_ppval_g
    (open_robot_arm_DW.obj.PPDDCell[2].breaks, open_robot_arm_DW.obj.PPDDCell[2]
     .coefs, rtb_Clock);
  open_robot_arm_B.TrapezoidalVelocityProfileTra_j[3] = open_robot_arm_ppval_g
    (open_robot_arm_DW.obj.PPDDCell[3].breaks, open_robot_arm_DW.obj.PPDDCell[3]
     .coefs, rtb_Clock);
  open_robot_arm_B.TrapezoidalVelocityProfileTra_j[4] = open_robot_arm_ppval_g
    (open_robot_arm_DW.obj.PPDDCell[4].breaks, open_robot_arm_DW.obj.PPDDCell[4]
     .coefs, rtb_Clock);
  open_robot_arm_B.TrapezoidalVelocityProfileTra_j[5] = open_robot_arm_ppval_g
    (open_robot_arm_DW.obj.PPDDCell[5].breaks, open_robot_arm_DW.obj.PPDDCell[5]
     .coefs, rtb_Clock);

  /* Rounding: '<S1>/Round' incorporates:
   *  Gain: '<S1>/Gain'
   */
  open_robot_arm_B.Round = rt_roundd_snf(open_robot_arm_P.Gain_Gain *
    open_robot_arm_B.TrapezoidalVelocityProfileTraje[0]);

  /* Sum: '<S1>/Add' incorporates:
   *  UnitDelay: '<S1>/Unit Delay'
   */
  rtb_Clock = open_robot_arm_B.Round - open_robot_arm_DW.UnitDelay_DSTATE;

  /* Signum: '<S1>/Sign' */
  if (rtIsNaN(rtb_Clock)) {
    tmp = (rtNaN);
  } else if (rtb_Clock < 0.0) {
    tmp = -1.0;
  } else {
    tmp = (rtb_Clock > 0.0);
  }

  /* RelationalOperator: '<S2>/Compare' incorporates:
   *  Constant: '<S2>/Constant'
   *  Signum: '<S1>/Sign'
   */
  open_robot_arm_B.Compare = (tmp >= open_robot_arm_P.CompareToConstant_const);

  /* MATLABSystem: '<Root>/GPIO Write' */
  EXT_GPIO_write(20U, (uint8_T)open_robot_arm_B.Compare);

  /* Abs: '<S1>/Abs' */
  open_robot_arm_B.Abs = fabs(rtb_Clock);

  /* Start for MATLABSystem: '<Root>/PWM' */
  if (open_robot_arm_B.Abs <= 1.0) {
    rtb_Clock = open_robot_arm_B.Abs;
  } else {
    rtb_Clock = 1.0;
  }

  if (!(rtb_Clock >= 0.0)) {
    rtb_Clock = 0.0;
  }

  /* MATLABSystem: '<Root>/PWM' */
  EXT_PWMBlock_setDutyCycle(open_robot_arm_DW.obj_l.PinNumber, rtb_Clock);

  /* Update for UnitDelay: '<S1>/Unit Delay' */
  open_robot_arm_DW.UnitDelay_DSTATE = open_robot_arm_B.Round;

  {                                    /* Sample time: [0.001s, 0.0s] */
    extmodeErrorCode_T errorCode = EXTMODE_SUCCESS;
    extmodeSimulationTime_T extmodeTime = (extmodeSimulationTime_T)
      ((open_robot_arm_M->Timing.clockTick1) * 0.001);

    /* Trigger External Mode event */
    errorCode = extmodeEvent(1, extmodeTime);
    if (errorCode != EXTMODE_SUCCESS) {
      /* Code to handle External Mode event errors
         may be added here */
    }
  }

  /* Update absolute time for base rate */
  /* The "clockTick0" counts the number of times the code of this task has
   * been executed. The absolute time is the multiplication of "clockTick0"
   * and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
   * overflow during the application lifespan selected.
   */
  open_robot_arm_M->Timing.t[0] =
    ((time_T)(++open_robot_arm_M->Timing.clockTick0)) *
    open_robot_arm_M->Timing.stepSize0;

  {
    /* Update absolute timer for sample time: [0.001s, 0.0s] */
    /* The "clockTick1" counts the number of times the code of this task has
     * been executed. The resolution of this integer timer is 0.001, which is the step size
     * of the task. Size of "clockTick1" ensures timer will not overflow during the
     * application lifespan selected.
     */
    open_robot_arm_M->Timing.clockTick1++;
  }
}

/* Model initialize function */
void open_robot_arm_initialize(void)
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  {
    /* Setup solver object */
    rtsiSetSimTimeStepPtr(&open_robot_arm_M->solverInfo,
                          &open_robot_arm_M->Timing.simTimeStep);
    rtsiSetTPtr(&open_robot_arm_M->solverInfo, &rtmGetTPtr(open_robot_arm_M));
    rtsiSetStepSizePtr(&open_robot_arm_M->solverInfo,
                       &open_robot_arm_M->Timing.stepSize0);
    rtsiSetErrorStatusPtr(&open_robot_arm_M->solverInfo, (&rtmGetErrorStatus
      (open_robot_arm_M)));
    rtsiSetRTModelPtr(&open_robot_arm_M->solverInfo, open_robot_arm_M);
  }

  rtsiSetSimTimeStep(&open_robot_arm_M->solverInfo, MAJOR_TIME_STEP);
  rtsiSetIsMinorTimeStepWithModeChange(&open_robot_arm_M->solverInfo, false);
  rtsiSetIsContModeFrozen(&open_robot_arm_M->solverInfo, false);
  rtsiSetSolverName(&open_robot_arm_M->solverInfo,"FixedStepDiscrete");
  rtmSetTPtr(open_robot_arm_M, &open_robot_arm_M->Timing.tArray[0]);
  rtmSetTFinal(open_robot_arm_M, 22.0);
  open_robot_arm_M->Timing.stepSize0 = 0.001;

  /* External mode info */
  open_robot_arm_M->Sizes.checksums[0] = (3046052321U);
  open_robot_arm_M->Sizes.checksums[1] = (1171764642U);
  open_robot_arm_M->Sizes.checksums[2] = (1073275869U);
  open_robot_arm_M->Sizes.checksums[3] = (4053188882U);

  {
    static const sysRanDType rtAlwaysEnabled = SUBSYS_RAN_BC_ENABLE;
    static RTWExtModeInfo rt_ExtModeInfo;
    static const sysRanDType *systemRan[4];
    open_robot_arm_M->extModeInfo = (&rt_ExtModeInfo);
    rteiSetSubSystemActiveVectorAddresses(&rt_ExtModeInfo, systemRan);
    systemRan[0] = &rtAlwaysEnabled;
    systemRan[1] = &rtAlwaysEnabled;
    systemRan[2] = &rtAlwaysEnabled;
    systemRan[3] = &rtAlwaysEnabled;
    rteiSetModelMappingInfoPtr(open_robot_arm_M->extModeInfo,
      &open_robot_arm_M->SpecialInfo.mappingInfo);
    rteiSetChecksumsPtr(open_robot_arm_M->extModeInfo,
                        open_robot_arm_M->Sizes.checksums);
    rteiSetTPtr(open_robot_arm_M->extModeInfo, rtmGetTPtr(open_robot_arm_M));
  }

  {
    int32_T i;
    char_T b[45];
    static const char_T b_0[45] =
      "Unable to configure pin %u for PWM output.\\n";

    /* InitializeConditions for UnitDelay: '<S1>/Unit Delay' */
    open_robot_arm_DW.UnitDelay_DSTATE =
      open_robot_arm_P.UnitDelay_InitialCondition;

    /* Start for MATLABSystem: '<Root>/Trapezoidal Velocity Profile Trajectory' */
    for (i = 0; i < 5; i++) {
      open_robot_arm_DW.obj.tunablePropertyChanged[i] = false;
    }

    memcpy(&open_robot_arm_DW.obj.Waypoints[0],
           &open_robot_arm_P.TrapezoidalVelocityProfileTraje[0], 42U * sizeof
           (real_T));
    for (i = 0; i < 6; i++) {
      open_robot_arm_DW.obj.EndTime[i] =
        open_robot_arm_P.TrapezoidalVelocityProfileTra_p[i];
    }

    open_robot_arm_DW.obj.isInitialized = 1;
    TrapVelTrajSys_regeneratePPForm(&open_robot_arm_DW.obj,
      open_robot_arm_DW.obj.Waypoints);
    open_robot_arm_DW.obj.TunablePropsChanged = false;

    /* End of Start for MATLABSystem: '<Root>/Trapezoidal Velocity Profile Trajectory' */

    /* Start for MATLABSystem: '<Root>/GPIO Write' */
    open_robot_arm_DW.obj_m.matlabCodegenIsDeleted = false;
    open_robot_arm_DW.obj_m.isInitialized = 1;
    EXT_GPIO_init(20U, true);
    open_robot_arm_DW.obj_m.isSetupComplete = true;

    /* Start for MATLABSystem: '<Root>/PWM' */
    open_robot_arm_DW.obj_l.matlabCodegenIsDeleted = false;
    open_robot_arm_DW.obj_l.isInitialized = 1;
    open_robot_arm_DW.obj_l.PinNumber = 21U;
    i = EXT_PWMBlock_init(21U, 800U, 0.0);
    if (i != 0) {
      for (i = 0; i < 45; i++) {
        b[i] = b_0[i];
      }

      printf(&b[0], 21U);
    }

    open_robot_arm_DW.obj_l.isSetupComplete = true;

    /* End of Start for MATLABSystem: '<Root>/PWM' */
  }
}

/* Model terminate function */
void open_robot_arm_terminate(void)
{
  /* Terminate for MATLABSystem: '<Root>/GPIO Write' */
  if (!open_robot_arm_DW.obj_m.matlabCodegenIsDeleted) {
    open_robot_arm_DW.obj_m.matlabCodegenIsDeleted = true;
    if ((open_robot_arm_DW.obj_m.isInitialized == 1) &&
        open_robot_arm_DW.obj_m.isSetupComplete) {
      EXT_GPIO_terminate(20U);
    }
  }

  /* End of Terminate for MATLABSystem: '<Root>/GPIO Write' */
  /* Terminate for MATLABSystem: '<Root>/PWM' */
  if (!open_robot_arm_DW.obj_l.matlabCodegenIsDeleted) {
    open_robot_arm_DW.obj_l.matlabCodegenIsDeleted = true;
    if ((open_robot_arm_DW.obj_l.isInitialized == 1) &&
        open_robot_arm_DW.obj_l.isSetupComplete) {
      EXT_PWMBlock_terminate(open_robot_arm_DW.obj_l.PinNumber);
    }
  }

  /* End of Terminate for MATLABSystem: '<Root>/PWM' */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
