<<<<<<< HEAD
/* ----------------------------------------------------------------------
 * Project:      CMSIS DSP Library
 * Title:        arm_biquad_cascade_df1_init_f32.c
 * Description:  Floating-point Biquad cascade DirectFormI(DF1) filter initialization function
 *
 * $Date:        27. January 2017
 * $Revision:    V.1.5.1
 *
 * Target Processor: Cortex-M cores
 * -------------------------------------------------------------------- */
/*
 * Copyright (C) 2010-2017 ARM Limited or its affiliates. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "arm_math.h"

/**
 * @ingroup groupFilters
 */

/**
 * @addtogroup BiquadCascadeDF1
 * @{
 */

/**
 * @details
 * @brief  Initialization function for the floating-point Biquad cascade filter.
 * @param[in,out] *S           points to an instance of the floating-point Biquad cascade structure.
 * @param[in]     numStages    number of 2nd order stages in the filter.
 * @param[in]     *pCoeffs     points to the filter coefficients array.
 * @param[in]     *pState      points to the state array.
 * @return        none
 *
 *
 * <b>Coefficient and State Ordering:</b>
 *
 * \par
 * The coefficients are stored in the array <code>pCoeffs</code> in the following order:
 * <pre>
 *     {b10, b11, b12, a11, a12, b20, b21, b22, a21, a22, ...}
 * </pre>
 *
 * \par
 * where <code>b1x</code> and <code>a1x</code> are the coefficients for the first stage,
 * <code>b2x</code> and <code>a2x</code> are the coefficients for the second stage,
 * and so on.  The <code>pCoeffs</code> array contains a total of <code>5*numStages</code> values.
 *
 * \par
 * The <code>pState</code> is a pointer to state array.
 * Each Biquad stage has 4 state variables <code>x[n-1], x[n-2], y[n-1],</code> and <code>y[n-2]</code>.
 * The state variables are arranged in the <code>pState</code> array as:
 * <pre>
 *     {x[n-1], x[n-2], y[n-1], y[n-2]}
 * </pre>
 * The 4 state variables for stage 1 are first, then the 4 state variables for stage 2, and so on.
 * The state array has a total length of <code>4*numStages</code> values.
 * The state variables are updated after each block of data is processed; the coefficients are untouched.
 *
 */

void arm_biquad_cascade_df1_init_f32(
  arm_biquad_casd_df1_inst_f32 * S,
  uint8_t numStages,
  float32_t * pCoeffs,
  float32_t * pState)
{
  /* Assign filter stages */
  S->numStages = numStages;

  /* Assign coefficient pointer */
  S->pCoeffs = pCoeffs;

  /* Clear state buffer and size is always 4 * numStages */
  memset(pState, 0, (4U * (uint32_t) numStages) * sizeof(float32_t));

  /* Assign state pointer */
  S->pState = pState;
}

/**
 * @} end of BiquadCascadeDF1 group
 */
=======
/* ----------------------------------------------------------------------
 * Project:      CMSIS DSP Library
 * Title:        arm_biquad_cascade_df1_init_f32.c
 * Description:  Floating-point Biquad cascade DirectFormI(DF1) filter initialization function
 *
 * $Date:        27. January 2017
 * $Revision:    V.1.5.1
 *
 * Target Processor: Cortex-M cores
 * -------------------------------------------------------------------- */
/*
 * Copyright (C) 2010-2017 ARM Limited or its affiliates. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "arm_math.h"

/**
 * @ingroup groupFilters
 */

/**
 * @addtogroup BiquadCascadeDF1
 * @{
 */

/**
 * @details
 * @brief  Initialization function for the floating-point Biquad cascade filter.
 * @param[in,out] *S           points to an instance of the floating-point Biquad cascade structure.
 * @param[in]     numStages    number of 2nd order stages in the filter.
 * @param[in]     *pCoeffs     points to the filter coefficients array.
 * @param[in]     *pState      points to the state array.
 * @return        none
 *
 *
 * <b>Coefficient and State Ordering:</b>
 *
 * \par
 * The coefficients are stored in the array <code>pCoeffs</code> in the following order:
 * <pre>
 *     {b10, b11, b12, a11, a12, b20, b21, b22, a21, a22, ...}
 * </pre>
 *
 * \par
 * where <code>b1x</code> and <code>a1x</code> are the coefficients for the first stage,
 * <code>b2x</code> and <code>a2x</code> are the coefficients for the second stage,
 * and so on.  The <code>pCoeffs</code> array contains a total of <code>5*numStages</code> values.
 *
 * \par
 * The <code>pState</code> is a pointer to state array.
 * Each Biquad stage has 4 state variables <code>x[n-1], x[n-2], y[n-1],</code> and <code>y[n-2]</code>.
 * The state variables are arranged in the <code>pState</code> array as:
 * <pre>
 *     {x[n-1], x[n-2], y[n-1], y[n-2]}
 * </pre>
 * The 4 state variables for stage 1 are first, then the 4 state variables for stage 2, and so on.
 * The state array has a total length of <code>4*numStages</code> values.
 * The state variables are updated after each block of data is processed; the coefficients are untouched.
 *
 */

void arm_biquad_cascade_df1_init_f32(
  arm_biquad_casd_df1_inst_f32 * S,
  uint8_t numStages,
  float32_t * pCoeffs,
  float32_t * pState)
{
  /* Assign filter stages */
  S->numStages = numStages;

  /* Assign coefficient pointer */
  S->pCoeffs = pCoeffs;

  /* Clear state buffer and size is always 4 * numStages */
  memset(pState, 0, (4U * (uint32_t) numStages) * sizeof(float32_t));

  /* Assign state pointer */
  S->pState = pState;
}

/**
 * @} end of BiquadCascadeDF1 group
 */
>>>>>>> b334a05 (Week 3 Tasks)
