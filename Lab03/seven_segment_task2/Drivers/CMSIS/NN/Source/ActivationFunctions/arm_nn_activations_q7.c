<<<<<<< HEAD
/*
 * Copyright (C) 2010-2018 Arm Limited or its affiliates. All rights reserved.
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

/* ----------------------------------------------------------------------
 * Project:      CMSIS NN Library
 * Title:        arm_nn_activations_q7.c
 * Description:  Q7 neural network activation function using direct table look-up
 *
 * $Date:        17. January 2018
 * $Revision:    V.1.0.0
 *
 * Target Processor:  Cortex-M cores
 *
 * -------------------------------------------------------------------- */

#include "arm_math.h"
#include "arm_common_tables.h"
#include "arm_nnfunctions.h"

/**
 *  @ingroup groupNN
 */

/**
 * @addtogroup Acti
 * @{
 */

  /**
   * @brief Q7 neural network activation function using direct table look-up
   * @param[in,out]   data        pointer to input
   * @param[in]       size        number of elements
   * @param[in]       int_width   bit-width of the integer part, assume to be smaller than 3
   * @param[in]       type        type of activation functions
   * @return none.
   *
   * @details
   * 
   * This is the direct table look-up approach.
   *
   * Assume here the integer part of the fixed-point is <= 3.
   * More than 3 just not making much sense, makes no difference with
   * saturation followed by any of these activation functions. 
   */

void arm_nn_activations_direct_q7(q7_t * data, uint16_t size, uint16_t int_width, arm_nn_activation_type type)
{
    uint16_t  i = size;
    q7_t     *pIn = data;
    q7_t     *pOut = data;
    q7_t      in;
    q7_t      out;
    uint16_t  shift_size = 3 - int_width;
    const q7_t *lookup_table;
    switch (type)
    {
    case ARM_SIGMOID:
        lookup_table = sigmoidTable_q7;
        break;
    case ARM_TANH:
    default:
        lookup_table = tanhTable_q7;
        break;
    }
    while (i)
    {
        in = *pIn++;
        out = lookup_table[(uint8_t) (in >> shift_size)];
        *pOut++ = out;
        i--;
    }
}

/**
 * @} end of Acti group
 */
=======
/*
 * Copyright (C) 2010-2018 Arm Limited or its affiliates. All rights reserved.
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

/* ----------------------------------------------------------------------
 * Project:      CMSIS NN Library
 * Title:        arm_nn_activations_q7.c
 * Description:  Q7 neural network activation function using direct table look-up
 *
 * $Date:        17. January 2018
 * $Revision:    V.1.0.0
 *
 * Target Processor:  Cortex-M cores
 *
 * -------------------------------------------------------------------- */

#include "arm_math.h"
#include "arm_common_tables.h"
#include "arm_nnfunctions.h"

/**
 *  @ingroup groupNN
 */

/**
 * @addtogroup Acti
 * @{
 */

  /**
   * @brief Q7 neural network activation function using direct table look-up
   * @param[in,out]   data        pointer to input
   * @param[in]       size        number of elements
   * @param[in]       int_width   bit-width of the integer part, assume to be smaller than 3
   * @param[in]       type        type of activation functions
   * @return none.
   *
   * @details
   * 
   * This is the direct table look-up approach.
   *
   * Assume here the integer part of the fixed-point is <= 3.
   * More than 3 just not making much sense, makes no difference with
   * saturation followed by any of these activation functions. 
   */

void arm_nn_activations_direct_q7(q7_t * data, uint16_t size, uint16_t int_width, arm_nn_activation_type type)
{
    uint16_t  i = size;
    q7_t     *pIn = data;
    q7_t     *pOut = data;
    q7_t      in;
    q7_t      out;
    uint16_t  shift_size = 3 - int_width;
    const q7_t *lookup_table;
    switch (type)
    {
    case ARM_SIGMOID:
        lookup_table = sigmoidTable_q7;
        break;
    case ARM_TANH:
    default:
        lookup_table = tanhTable_q7;
        break;
    }
    while (i)
    {
        in = *pIn++;
        out = lookup_table[(uint8_t) (in >> shift_size)];
        *pOut++ = out;
        i--;
    }
}

/**
 * @} end of Acti group
 */
>>>>>>> b334a05 (Week 3 Tasks)
