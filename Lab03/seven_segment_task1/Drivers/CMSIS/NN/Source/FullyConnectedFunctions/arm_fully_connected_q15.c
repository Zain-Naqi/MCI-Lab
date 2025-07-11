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
 * Title:        arm_fully_connected_q15.c
 * Description:  Q15 basic fully-connected layer function
 *
 * $Date:        17. January 2018
 * $Revision:    V.1.0.0
 *
 * Target Processor:  Cortex-M cores
 *
 * -------------------------------------------------------------------- */

#include "arm_math.h"
#include "arm_nnfunctions.h"

/**
 *  @ingroup groupNN
 */

/**
 * @addtogroup FC
 * @{
 */

  /**
   * @brief Q15 opt fully-connected layer function
   * @param[in]       pV          pointer to input vector
   * @param[in]       pM          pointer to matrix weights
   * @param[in]       dim_vec     length of the vector
   * @param[in]       num_of_rows number of rows in weight matrix
   * @param[in]       bias_shift  amount of left-shift for bias
   * @param[in]       out_shift   amount of right-shift for output
   * @param[in]       bias        pointer to bias
   * @param[in,out]   pOut        pointer to output vector
   * @param[in,out]   vec_buffer  pointer to buffer space for input
   * @return     The function returns <code>ARM_MATH_SUCCESS</code>
   *
   *
   * @details
   *
   * <b>Buffer size:</b>
   *
   * vec_buffer size: 0
   *
   */

arm_status
arm_fully_connected_q15(const q15_t * pV,
                        const q15_t * pM,
                        const uint16_t dim_vec,
                        const uint16_t num_of_rows,
                        const uint16_t bias_shift,
                        const uint16_t out_shift, 
                        const q15_t * bias, 
                        q15_t * pOut,
                        q15_t * vec_buffer)
{

#if defined (ARM_MATH_DSP)
    /* Run the following code for Cortex-M4 and Cortex-M7 */

    const q15_t *pB = pM;
    const q15_t *pB2 = pB + dim_vec;
    q15_t    *pO = pOut;
    const q15_t    *pA;
    const q15_t    *pBias = bias;
    uint16_t rowCnt = num_of_rows >> 1;

    /* this loop loops over different output */
    while (rowCnt) {
        q31_t     sum =  ((q31_t)(*pBias++) << bias_shift) + NN_ROUND(out_shift);
        q31_t     sum2 = ((q31_t)(*pBias++) << bias_shift) + NN_ROUND(out_shift);

        uint16_t  colCnt = dim_vec >> 2;

        pA = pV;
        pB2 = pB + dim_vec;

        while (colCnt)
        {
            q31_t     inV1, inM1, inM2;
            inV1 = *__SIMD32(pA)++;
            inM1 = *__SIMD32(pB)++;
            sum = __SMLAD(inV1, inM1, sum);
            inM2 = *__SIMD32(pB2)++;
            sum2 = __SMLAD(inV1, inM2, sum2);

            inV1 = *__SIMD32(pA)++;
            inM1 = *__SIMD32(pB)++;
            sum = __SMLAD(inV1, inM1, sum);
            inM2 = *__SIMD32(pB2)++;
            sum2 = __SMLAD(inV1, inM2, sum2);

            colCnt--;
        }
        colCnt = dim_vec & 0x3;
        while (colCnt)
        {
            q15_t     inV = *pA++;
            q15_t     inM = *pB++;
            q15_t     inM2 = *pB2++;

            sum += inV * inM;
            sum2 += inV * inM2;
            colCnt--;
        }                       /* while over colCnt */
        *pO++ =  (q15_t) (__SSAT((sum >> out_shift), 16));
        *pO++ = (q15_t) (__SSAT((sum2>> out_shift), 16));
		
        /* adjust the pointers and counters */
        pB = pB + dim_vec;
        rowCnt --;
    }

    rowCnt = num_of_rows & 0x1;

    while (rowCnt) {
        q31_t     sum = ((q31_t)(*pBias++) << bias_shift) + NN_ROUND(out_shift);

        uint16_t  colCnt = dim_vec >> 2;

        pA = pV;
      
        while (colCnt) {
            q31_t     inV1, inM1;
            inV1 = *__SIMD32(pA)++;
            inM1 = *__SIMD32(pB)++;
            sum = __SMLAD(inV1, inM1, sum);
            
            inV1 = *__SIMD32(pA)++;
            inM1 = *__SIMD32(pB)++;
            sum = __SMLAD(inV1, inM1, sum);
				
            colCnt--;
	}
			
	/* left-over of the vector */
	colCnt = dim_vec & 0x3;
	while(colCnt) {
            q15_t     inV = *pA++;
            q15_t     inM = *pB++;

            sum += inV * inM;

            colCnt--;
	}

        *pO++ =  (q15_t) (__SSAT((sum >> out_shift), 16));
			
        rowCnt --;
    }

#else
    int       i, j;
    /* Run the following code as reference implementation for Cortex-M0 and Cortex-M3 */
    for (i = 0; i < num_of_rows; i++)
    {
        int       ip_out = ((q31_t)(bias[i]) << bias_shift) + NN_ROUND(out_shift);
        for (j = 0; j < dim_vec; j++)
        {
            ip_out += pV[j] * pM[i * dim_vec + j];
        }
        pOut[i] = (q15_t) __SSAT((ip_out >> out_shift), 16);
    }

#endif                          /* ARM_MATH_DSP */

    /* Return to application */
    return (ARM_MATH_SUCCESS);

}

/**
 * @} end of FC group
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
 * Title:        arm_fully_connected_q15.c
 * Description:  Q15 basic fully-connected layer function
 *
 * $Date:        17. January 2018
 * $Revision:    V.1.0.0
 *
 * Target Processor:  Cortex-M cores
 *
 * -------------------------------------------------------------------- */

#include "arm_math.h"
#include "arm_nnfunctions.h"

/**
 *  @ingroup groupNN
 */

/**
 * @addtogroup FC
 * @{
 */

  /**
   * @brief Q15 opt fully-connected layer function
   * @param[in]       pV          pointer to input vector
   * @param[in]       pM          pointer to matrix weights
   * @param[in]       dim_vec     length of the vector
   * @param[in]       num_of_rows number of rows in weight matrix
   * @param[in]       bias_shift  amount of left-shift for bias
   * @param[in]       out_shift   amount of right-shift for output
   * @param[in]       bias        pointer to bias
   * @param[in,out]   pOut        pointer to output vector
   * @param[in,out]   vec_buffer  pointer to buffer space for input
   * @return     The function returns <code>ARM_MATH_SUCCESS</code>
   *
   *
   * @details
   *
   * <b>Buffer size:</b>
   *
   * vec_buffer size: 0
   *
   */

arm_status
arm_fully_connected_q15(const q15_t * pV,
                        const q15_t * pM,
                        const uint16_t dim_vec,
                        const uint16_t num_of_rows,
                        const uint16_t bias_shift,
                        const uint16_t out_shift, 
                        const q15_t * bias, 
                        q15_t * pOut,
                        q15_t * vec_buffer)
{

#if defined (ARM_MATH_DSP)
    /* Run the following code for Cortex-M4 and Cortex-M7 */

    const q15_t *pB = pM;
    const q15_t *pB2 = pB + dim_vec;
    q15_t    *pO = pOut;
    const q15_t    *pA;
    const q15_t    *pBias = bias;
    uint16_t rowCnt = num_of_rows >> 1;

    /* this loop loops over different output */
    while (rowCnt) {
        q31_t     sum =  ((q31_t)(*pBias++) << bias_shift) + NN_ROUND(out_shift);
        q31_t     sum2 = ((q31_t)(*pBias++) << bias_shift) + NN_ROUND(out_shift);

        uint16_t  colCnt = dim_vec >> 2;

        pA = pV;
        pB2 = pB + dim_vec;

        while (colCnt)
        {
            q31_t     inV1, inM1, inM2;
            inV1 = *__SIMD32(pA)++;
            inM1 = *__SIMD32(pB)++;
            sum = __SMLAD(inV1, inM1, sum);
            inM2 = *__SIMD32(pB2)++;
            sum2 = __SMLAD(inV1, inM2, sum2);

            inV1 = *__SIMD32(pA)++;
            inM1 = *__SIMD32(pB)++;
            sum = __SMLAD(inV1, inM1, sum);
            inM2 = *__SIMD32(pB2)++;
            sum2 = __SMLAD(inV1, inM2, sum2);

            colCnt--;
        }
        colCnt = dim_vec & 0x3;
        while (colCnt)
        {
            q15_t     inV = *pA++;
            q15_t     inM = *pB++;
            q15_t     inM2 = *pB2++;

            sum += inV * inM;
            sum2 += inV * inM2;
            colCnt--;
        }                       /* while over colCnt */
        *pO++ =  (q15_t) (__SSAT((sum >> out_shift), 16));
        *pO++ = (q15_t) (__SSAT((sum2>> out_shift), 16));
		
        /* adjust the pointers and counters */
        pB = pB + dim_vec;
        rowCnt --;
    }

    rowCnt = num_of_rows & 0x1;

    while (rowCnt) {
        q31_t     sum = ((q31_t)(*pBias++) << bias_shift) + NN_ROUND(out_shift);

        uint16_t  colCnt = dim_vec >> 2;

        pA = pV;
      
        while (colCnt) {
            q31_t     inV1, inM1;
            inV1 = *__SIMD32(pA)++;
            inM1 = *__SIMD32(pB)++;
            sum = __SMLAD(inV1, inM1, sum);
            
            inV1 = *__SIMD32(pA)++;
            inM1 = *__SIMD32(pB)++;
            sum = __SMLAD(inV1, inM1, sum);
				
            colCnt--;
	}
			
	/* left-over of the vector */
	colCnt = dim_vec & 0x3;
	while(colCnt) {
            q15_t     inV = *pA++;
            q15_t     inM = *pB++;

            sum += inV * inM;

            colCnt--;
	}

        *pO++ =  (q15_t) (__SSAT((sum >> out_shift), 16));
			
        rowCnt --;
    }

#else
    int       i, j;
    /* Run the following code as reference implementation for Cortex-M0 and Cortex-M3 */
    for (i = 0; i < num_of_rows; i++)
    {
        int       ip_out = ((q31_t)(bias[i]) << bias_shift) + NN_ROUND(out_shift);
        for (j = 0; j < dim_vec; j++)
        {
            ip_out += pV[j] * pM[i * dim_vec + j];
        }
        pOut[i] = (q15_t) __SSAT((ip_out >> out_shift), 16);
    }

#endif                          /* ARM_MATH_DSP */

    /* Return to application */
    return (ARM_MATH_SUCCESS);

}

/**
 * @} end of FC group
 */
>>>>>>> b334a05 (Week 3 Tasks)
