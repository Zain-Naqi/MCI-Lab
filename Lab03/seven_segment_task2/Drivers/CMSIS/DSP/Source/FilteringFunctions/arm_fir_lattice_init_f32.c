<<<<<<< HEAD
/* ----------------------------------------------------------------------
 * Project:      CMSIS DSP Library
 * Title:        arm_fir_lattice_init_f32.c
 * Description:  Floating-point FIR Lattice filter initialization function
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
 * @addtogroup FIR_Lattice
 * @{
 */

/**
 * @brief Initialization function for the floating-point FIR lattice filter.
 * @param[in] *S points to an instance of the floating-point FIR lattice structure.
 * @param[in] numStages  number of filter stages.
 * @param[in] *pCoeffs points to the coefficient buffer.  The array is of length numStages.
 * @param[in] *pState points to the state buffer.  The array is of length numStages.
 * @return none.
 */

void arm_fir_lattice_init_f32(
  arm_fir_lattice_instance_f32 * S,
  uint16_t numStages,
  float32_t * pCoeffs,
  float32_t * pState)
{
  /* Assign filter taps */
  S->numStages = numStages;

  /* Assign coefficient pointer */
  S->pCoeffs = pCoeffs;

  /* Clear state buffer and size is always numStages */
  memset(pState, 0, (numStages) * sizeof(float32_t));

  /* Assign state pointer */
  S->pState = pState;

}

/**
 * @} end of FIR_Lattice group
 */
=======
/* ----------------------------------------------------------------------
 * Project:      CMSIS DSP Library
 * Title:        arm_fir_lattice_init_f32.c
 * Description:  Floating-point FIR Lattice filter initialization function
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
 * @addtogroup FIR_Lattice
 * @{
 */

/**
 * @brief Initialization function for the floating-point FIR lattice filter.
 * @param[in] *S points to an instance of the floating-point FIR lattice structure.
 * @param[in] numStages  number of filter stages.
 * @param[in] *pCoeffs points to the coefficient buffer.  The array is of length numStages.
 * @param[in] *pState points to the state buffer.  The array is of length numStages.
 * @return none.
 */

void arm_fir_lattice_init_f32(
  arm_fir_lattice_instance_f32 * S,
  uint16_t numStages,
  float32_t * pCoeffs,
  float32_t * pState)
{
  /* Assign filter taps */
  S->numStages = numStages;

  /* Assign coefficient pointer */
  S->pCoeffs = pCoeffs;

  /* Clear state buffer and size is always numStages */
  memset(pState, 0, (numStages) * sizeof(float32_t));

  /* Assign state pointer */
  S->pState = pState;

}

/**
 * @} end of FIR_Lattice group
 */
>>>>>>> b334a05 (Week 3 Tasks)
