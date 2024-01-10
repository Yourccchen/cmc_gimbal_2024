/*
 * File: YawSpeedPID.h
 *
 * Code generated for Simulink model :YawSpeedPID.
 *
 * Model version      : 1.2
 * Simulink Coder version    : 23.2 (R2023b) 01-Aug-2023
 * TLC version       : 23.2 (Jan 09 2024)
 * C/C++ source code generated on  : Tue Jan  9 22:31:22 2024
 *
 * Target selection: stm32.tlc
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 *
 *
 *
 * ******************************************************************************
 * * attention
 * *
 * * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
 * * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
 * * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
 * * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
 * * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
 * * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 * *
 * ******************************************************************************
 */

#ifndef RTW_HEADER_YawSpeedPID_h_
#define RTW_HEADER_YawSpeedPID_h_
#ifndef YawSpeedPID_COMMON_INCLUDES_
#define YawSpeedPID_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 /* YawSpeedPID_COMMON_INCLUDES_ */

#include "STM32_Config.h"
#include "YawSpeedPID_External_Functions.h"
#include "YawSpeedPID_types.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

/* Block states (default storage) for system '<Root>' */
typedef struct {
  real_T Integrator_DSTATE;            /* '<S36>/Integrator' */
  real_T Filter_DSTATE;                /* '<S31>/Filter' */
} DW_YawSpeedPID;

/* External inputs (root inport signals with default storage) */
typedef struct {
  real_T YawSpeed_set;                 /* '<Root>/YawSpeed_set' */
  real_T YawSpeed_Now;                 /* '<Root>/YawSpeed_Now' */
  real_T YawS_P;                       /* '<Root>/YawS_P' */
  real_T YawS_I;                       /* '<Root>/YawS_I' */
  real_T YawS_D;                       /* '<Root>/YawS_D' */
  real_T YawS_N;                       /* '<Root>/YawS_N' */
  real_T YawS_MO;                      /* '<Root>/YawS_MO' */
  real_T YawS_LO;                      /* '<Root>/YawS_LO' */
} ExtU_YawSpeedPID;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real_T YawCurrent;                   /* '<Root>/YawCurrent' */
} ExtY_YawSpeedPID;

/* Real-time Model Data Structure */
struct tag_RTM_YawSpeedPID {
  const char_T * volatile errorStatus;

  /*
   * Timing:
   * The following substructure contains information regarding
   * the timing information for the model.
   */
  struct {
    struct {
      uint8_T TID[2];
    } TaskCounters;
  } Timing;
};

/* Block states (default storage) */
extern DW_YawSpeedPID YawSpeedPID_DW;

/* External inputs (root inport signals with default storage) */
extern ExtU_YawSpeedPID YawSpeedPID_U;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY_YawSpeedPID YawSpeedPID_Y;

/* Model entry point functions */
extern void YawSpeedPID_initialize(void);
extern void YawSpeedPID_step(void);

/* Real-time Model object */
extern RT_MODEL_YawSpeedPID *const YawSpeedPID_M;

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<S44>/Data Type Duplicate' : Unused code path elimination
 * Block '<S44>/Data Type Propagation' : Unused code path elimination
 */

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
 * '<Root>' : 'YawSpeedPID'
 * '<S1>'   : 'YawSpeedPID/PID Controller'
 * '<S2>'   : 'YawSpeedPID/PID Controller/Anti-windup'
 * '<S3>'   : 'YawSpeedPID/PID Controller/D Gain'
 * '<S4>'   : 'YawSpeedPID/PID Controller/Filter'
 * '<S5>'   : 'YawSpeedPID/PID Controller/Filter ICs'
 * '<S6>'   : 'YawSpeedPID/PID Controller/I Gain'
 * '<S7>'   : 'YawSpeedPID/PID Controller/Ideal P Gain'
 * '<S8>'   : 'YawSpeedPID/PID Controller/Ideal P Gain Fdbk'
 * '<S9>'   : 'YawSpeedPID/PID Controller/Integrator'
 * '<S10>'  : 'YawSpeedPID/PID Controller/Integrator ICs'
 * '<S11>'  : 'YawSpeedPID/PID Controller/N Copy'
 * '<S12>'  : 'YawSpeedPID/PID Controller/N Gain'
 * '<S13>'  : 'YawSpeedPID/PID Controller/P Copy'
 * '<S14>'  : 'YawSpeedPID/PID Controller/Parallel P Gain'
 * '<S15>'  : 'YawSpeedPID/PID Controller/Reset Signal'
 * '<S16>'  : 'YawSpeedPID/PID Controller/Saturation'
 * '<S17>'  : 'YawSpeedPID/PID Controller/Saturation Fdbk'
 * '<S18>'  : 'YawSpeedPID/PID Controller/Sum'
 * '<S19>'  : 'YawSpeedPID/PID Controller/Sum Fdbk'
 * '<S20>'  : 'YawSpeedPID/PID Controller/Tracking Mode'
 * '<S21>'  : 'YawSpeedPID/PID Controller/Tracking Mode Sum'
 * '<S22>'  : 'YawSpeedPID/PID Controller/Tsamp - Integral'
 * '<S23>'  : 'YawSpeedPID/PID Controller/Tsamp - Ngain'
 * '<S24>'  : 'YawSpeedPID/PID Controller/postSat Signal'
 * '<S25>'  : 'YawSpeedPID/PID Controller/preSat Signal'
 * '<S26>'  : 'YawSpeedPID/PID Controller/Anti-windup/Disc. Clamping Ideal'
 * '<S27>'  : 'YawSpeedPID/PID Controller/Anti-windup/Disc. Clamping Ideal/Dead Zone'
 * '<S28>'  : 'YawSpeedPID/PID Controller/Anti-windup/Disc. Clamping Ideal/Dead Zone/External'
 * '<S29>'  : 'YawSpeedPID/PID Controller/Anti-windup/Disc. Clamping Ideal/Dead Zone/External/Dead Zone Dynamic'
 * '<S30>'  : 'YawSpeedPID/PID Controller/D Gain/External Parameters'
 * '<S31>'  : 'YawSpeedPID/PID Controller/Filter/Disc. Forward Euler Filter'
 * '<S32>'  : 'YawSpeedPID/PID Controller/Filter ICs/Internal IC - Filter'
 * '<S33>'  : 'YawSpeedPID/PID Controller/I Gain/External Parameters'
 * '<S34>'  : 'YawSpeedPID/PID Controller/Ideal P Gain/External Parameters'
 * '<S35>'  : 'YawSpeedPID/PID Controller/Ideal P Gain Fdbk/Disabled'
 * '<S36>'  : 'YawSpeedPID/PID Controller/Integrator/Discrete'
 * '<S37>'  : 'YawSpeedPID/PID Controller/Integrator ICs/Internal IC'
 * '<S38>'  : 'YawSpeedPID/PID Controller/N Copy/Disabled'
 * '<S39>'  : 'YawSpeedPID/PID Controller/N Gain/External Parameters'
 * '<S40>'  : 'YawSpeedPID/PID Controller/P Copy/External Parameters Ideal'
 * '<S41>'  : 'YawSpeedPID/PID Controller/Parallel P Gain/Passthrough'
 * '<S42>'  : 'YawSpeedPID/PID Controller/Reset Signal/Disabled'
 * '<S43>'  : 'YawSpeedPID/PID Controller/Saturation/External'
 * '<S44>'  : 'YawSpeedPID/PID Controller/Saturation/External/Saturation Dynamic'
 * '<S45>'  : 'YawSpeedPID/PID Controller/Saturation Fdbk/Disabled'
 * '<S46>'  : 'YawSpeedPID/PID Controller/Sum/Sum_PID'
 * '<S47>'  : 'YawSpeedPID/PID Controller/Sum Fdbk/Disabled'
 * '<S48>'  : 'YawSpeedPID/PID Controller/Tracking Mode/Disabled'
 * '<S49>'  : 'YawSpeedPID/PID Controller/Tracking Mode Sum/Passthrough'
 * '<S50>'  : 'YawSpeedPID/PID Controller/Tsamp - Integral/TsSignalSpecification'
 * '<S51>'  : 'YawSpeedPID/PID Controller/Tsamp - Ngain/Passthrough'
 * '<S52>'  : 'YawSpeedPID/PID Controller/postSat Signal/Forward_Path'
 * '<S53>'  : 'YawSpeedPID/PID Controller/preSat Signal/Forward_Path'
 */
#endif                                 /* RTW_HEADER_YawSpeedPID_h_ */

/* File trailer for Real-Time Workshop generated code.
 *
 * [EOF] YawSpeedPID.h
 */
