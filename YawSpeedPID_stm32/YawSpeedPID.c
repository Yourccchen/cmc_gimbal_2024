/*
 * File: YawSpeedPID.c
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

#include "YawSpeedPID.h"
#include "rtwtypes.h"

/* Block states (default storage) */
DW_YawSpeedPID YawSpeedPID_DW;

/* External inputs (root inport signals with default storage) */
ExtU_YawSpeedPID YawSpeedPID_U;

/* External outputs (root outports fed by signals with default storage) */
ExtY_YawSpeedPID YawSpeedPID_Y;

/* Real-time model */
static RT_MODEL_YawSpeedPID YawSpeedPID_M_;
RT_MODEL_YawSpeedPID *const YawSpeedPID_M = &YawSpeedPID_M_;
static void rate_scheduler(void);

/*
 *         This function updates active task flag for each subrate.
 *         The function is called at model base rate, hence the
 *         generated code self-manages all its subrates.
 */
static void rate_scheduler(void)
{
  /* Compute which subrates run during the next base time step.  Subrates
   * are an integer multiple of the base rate counter.  Therefore, the subtask
   * counter is reset when it reaches its limit (zero means run).
   */
  (YawSpeedPID_M->Timing.TaskCounters.TID[1])++;
  if ((YawSpeedPID_M->Timing.TaskCounters.TID[1]) > 49) {/* Sample time: [0.005s, 0.0s] */
    YawSpeedPID_M->Timing.TaskCounters.TID[1] = 0;
  }
}

/* Model step function */
void YawSpeedPID_step(void)
{
  real_T rtb_IProdOut;
  real_T rtb_NProdOut;
  real_T rtb_Switch2;
  int8_T tmp;
  int8_T tmp_0;
  boolean_T rtb_Equal1;
  boolean_T rtb_RelationalOperator1;
  if (YawSpeedPID_M->Timing.TaskCounters.TID[1] == 0) {
    rtb_RelationalOperator1 = (YawSpeedPID_U.YawS_P > 0.0);
    rtb_IProdOut = YawSpeedPID_U.YawSpeed_set - YawSpeedPID_U.YawSpeed_Now;
    rtb_NProdOut = (rtb_IProdOut * YawSpeedPID_U.YawS_D -
                    YawSpeedPID_DW.Filter_DSTATE) * YawSpeedPID_U.YawS_N;
    YawSpeedPID_Y.YawCurrent = ((rtb_IProdOut + YawSpeedPID_DW.Integrator_DSTATE)
      + rtb_NProdOut) * YawSpeedPID_U.YawS_P;
    if (YawSpeedPID_Y.YawCurrent >= YawSpeedPID_U.YawS_MO) {
      rtb_Switch2 = YawSpeedPID_U.YawS_MO;
    } else if (YawSpeedPID_Y.YawCurrent > YawSpeedPID_U.YawS_LO) {
      rtb_Switch2 = YawSpeedPID_Y.YawCurrent;
    } else {
      rtb_Switch2 = YawSpeedPID_U.YawS_LO;
    }

    rtb_Switch2 = YawSpeedPID_Y.YawCurrent - rtb_Switch2;
    rtb_IProdOut *= YawSpeedPID_U.YawS_I;
    if (rtb_Switch2 > 0.0) {
      tmp = 1;
    } else {
      tmp = -1;
    }

    if (rtb_IProdOut > 0.0) {
      tmp_0 = 1;
    } else {
      tmp_0 = -1;
    }

    rtb_Equal1 = (tmp == tmp_0);
    if (YawSpeedPID_Y.YawCurrent > YawSpeedPID_U.YawS_MO) {
      YawSpeedPID_Y.YawCurrent = YawSpeedPID_U.YawS_MO;
    } else if (YawSpeedPID_Y.YawCurrent < YawSpeedPID_U.YawS_LO) {
      YawSpeedPID_Y.YawCurrent = YawSpeedPID_U.YawS_LO;
    }

    if ((rtb_Switch2 != 0.0) && ((rtb_Equal1 && rtb_RelationalOperator1) ||
         ((!rtb_Equal1) && (!rtb_RelationalOperator1)))) {
      rtb_IProdOut = 0.0;
    }

    YawSpeedPID_DW.Integrator_DSTATE += 0.005 * rtb_IProdOut;
    YawSpeedPID_DW.Filter_DSTATE += 0.005 * rtb_NProdOut;
  }

  rate_scheduler();
}

/* Model initialize function */
void YawSpeedPID_initialize(void)
{
  /* (no initialization code required) */
}

/* File trailer for Real-Time Workshop generated code.
 *
 * [EOF] YawSpeedPID.c
 */
