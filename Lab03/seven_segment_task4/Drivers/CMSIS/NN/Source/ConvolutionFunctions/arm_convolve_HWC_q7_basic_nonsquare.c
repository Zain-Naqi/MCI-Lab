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
 * Title:        arm_convolve_HWC_q7_basic.c
 * Description:	 Q7 version of convolution
 *
 * $Date:        13. July 2018
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
 * @addtogroup NNConv
 * @{
 */

  /**
   * @brief Basic Q7 convolution function (non-sqaure shape)
   * @param[in]       Im_in        pointer to input tensor
   * @param[in]       dim_im_in_x  input tensor dimention x
   * @param[in]       dim_im_in_y  input tensor dimention y
   * @param[in]       ch_im_in     number of input tensor channels
   * @param[in]       wt           pointer to kernel weights
   * @param[in]       ch_im_out    number of filters, i.e., output tensor channels
   * @param[in]       dim_kernel_x filter kernel size x
   * @param[in]       dim_kernel_y filter kernel size y
   * @param[in]       padding_x    padding size x
   * @param[in]       padding_y    padding size y
   * @param[in]       stride_x     convolution stride x
   * @param[in]       stride_y     convolution stride y
   * @param[in]       bias         pointer to bias
   * @param[in]       bias_shift   amount of left-shift for bias
   * @param[in]       out_shift    amount of right-shift for output
   * @param[in,out]   Im_out       pointer to output tensor
   * @param[in]       dim_im_out_x output tensor dimension x
   * @param[in]       dim_im_out_y output tensor dimension y
   * @param[in,out]   bufferA      pointer to buffer space for input
   * @param[in,out]   bufferB      pointer to buffer space for output
   * @return     The function returns <code>ARM_MATH_SUCCESS</code>
   */

arm_status arm_convolve_HWC_q7_basic_nonsquare(const q7_t * Im_in,
                                               const uint16_t dim_im_in_x,
                                               const uint16_t dim_im_in_y,
                                               const uint16_t ch_im_in,
                                               const q7_t * wt,
                                               const uint16_t ch_im_out,
                                               const uint16_t dim_kernel_x,
                                               const uint16_t dim_kernel_y,
                                               const uint16_t padding_x,
                                               const uint16_t padding_y,
                                               const uint16_t stride_x,
                                               const uint16_t stride_y,
                                               const q7_t * bias,
                                               const uint16_t bias_shift,
                                               const uint16_t out_shift,
                                               q7_t * Im_out,
                                               const uint16_t dim_im_out_x,
                                               const uint16_t dim_im_out_y,
                                               q15_t * bufferA,
                                               q7_t * bufferB)
{

#if defined (ARM_MATH_DSP)
    /* Run the following code for Cortex-M4 and Cortex-M7 */

    int16_t   i_out_y, i_out_x, i_ker_y, i_ker_x;

    /* 
     *  Here we use bufferA as q15_t internally as computation are done with q15_t level
     *  im2col are done to output in q15_t format from q7_t input
     */
    q15_t    *pBuffer = bufferA;
    q7_t     *pOut = Im_out;

    /* This part implements the im2col function */
    for (i_out_y = 0; i_out_y < dim_im_out_y; i_out_y++)
    {
        for (i_out_x = 0; i_out_x < dim_im_out_x; i_out_x++)
        {
            for (i_ker_y = i_out_y * stride_y - padding_y; i_ker_y < i_out_y * stride_y - padding_y + dim_kernel_y; i_ker_y++)
            {
                for (i_ker_x = i_out_x * stride_x - padding_x; i_ker_x < i_out_x * stride_x - padding_x + dim_kernel_x; i_ker_x++)
                {
                    if (i_ker_y < 0 || i_ker_y >= dim_im_in_y || i_ker_x < 0 || i_ker_x >= dim_im_in_x)
                    {
                        /* Filling 0 for out-of-bound paddings */
                        /* arm_fill_q15(0, pBuffer, ch_im_in); */
                        memset(pBuffer, 0, sizeof(q15_t)*ch_im_in);
                    } else
                    {
                        /* Copying the pixel data to column */
                        arm_q7_to_q15_no_shift((q7_t *)
                                               Im_in + (i_ker_y * dim_im_in_x + i_ker_x) * ch_im_in, pBuffer, ch_im_in);
                    }
                    pBuffer += ch_im_in;
                }
            }

            /* Computation is filed for every 2 columns */
            if (pBuffer == bufferA + 2 * ch_im_in * dim_kernel_y * dim_kernel_x)
            {
                pOut =
                    arm_nn_mat_mult_kernel_q7_q15(wt, bufferA,
                                                  ch_im_out,
                                                  ch_im_in *
                                                  dim_kernel_y * dim_kernel_x, bias_shift, out_shift, bias, pOut);

                /* counter reset */
                pBuffer = bufferA;
            }
        }
    }

    /* left-over because odd number of output pixels */
    if (pBuffer != bufferA)
    {
        const q7_t *pA = wt;
        int       i;

        for (i = 0; i < ch_im_out; i++)
        {
            /* Load the accumulator with bias first */
            q31_t     sum = ((q31_t)bias[i] << bias_shift) + NN_ROUND(out_shift);

            /* Point to the beging of the im2col buffer */
            q15_t    *pB = bufferA;

            /* Each time it process 4 entries */
            uint16_t  colCnt = ch_im_in * dim_kernel_y * dim_kernel_x >> 2;

            while (colCnt)
            {
                q31_t     inA1, inA2;
                q31_t     inB1, inB2;

                pA = (q7_t *) read_and_pad((void *)pA, &inA1, &inA2);

                inB1 = *__SIMD32(pB)++;
                sum = __SMLAD(inA1, inB1, sum);
                inB2 = *__SIMD32(pB)++;
                sum = __SMLAD(inA2, inB2, sum);

                colCnt--;
            }
            colCnt = ch_im_in * dim_kernel_y * dim_kernel_x & 0x3;
            while (colCnt)
            {
                q7_t      inA1 = *pA++;
                q15_t     inB1 = *pB++;
                sum += inA1 * inB1;
                colCnt--;
            }
            *pOut++ = (q7_t) __SSAT((sum >> out_shift), 8);
        }
    }
#else
    /* Run the following code as reference implementation for Cortex-M0 and Cortex-M3 */

    uint16_t  i, j, k, l, m, n;
    int       conv_out;
    signed char in_row, in_col;

    for (i = 0; i < ch_im_out; i++)
    {
        for (j = 0; j < dim_im_out_y; j++)
        {
            for (k = 0; k < dim_im_out_x; k++)
            {
                conv_out = ((q31_t)bias[i] << bias_shift) + NN_ROUND(out_shift);
                for (m = 0; m < dim_kernel_y; m++)
                {
                    for (n = 0; n < dim_kernel_x; n++)
                    {
                        // if-for implementation
                        in_row = stride_y * j + m - padding_y;
                        in_col = stride_x * k + n - padding_x;
                        if (in_row >= 0 && in_col >= 0 && in_row < dim_im_in_y && in_col < dim_im_in_x)
                        {
                            for (l = 0; l < ch_im_in; l++)
                            {
                                conv_out +=
                                    Im_in[(in_row * dim_im_in_x + in_col) * ch_im_in + l] * 
                                         wt[i * ch_im_in * dim_kernel_y * dim_kernel_x + 
                                         (m * dim_kernel_x + n) * ch_im_in + l];
                            }
                        }
                    }
                }
                Im_out[i + (j * dim_im_out_x + k) * ch_im_out] = (q7_t) __SSAT((conv_out >> out_shift), 8);
            }
        }
    }

#endif                          /* ARM_MATH_DSP */

    /* Return to application */
    return ARM_MATH_SUCCESS;
}

/**
 * @} end of NNConv group
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
 * Title:        arm_convolve_HWC_q7_basic.c
 * Description:	 Q7 version of convolution
 *
 * $Date:        13. July 2018
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
 * @addtogroup NNConv
 * @{
 */

  /**
   * @brief Basic Q7 convolution function (non-sqaure shape)
   * @param[in]       Im_in        pointer to input tensor
   * @param[in]       dim_im_in_x  input tensor dimention x
   * @param[in]       dim_im_in_y  input tensor dimention y
   * @param[in]       ch_im_in     number of input tensor channels
   * @param[in]       wt           pointer to kernel weights
   * @param[in]       ch_im_out    number of filters, i.e., output tensor channels
   * @param[in]       dim_kernel_x filter kernel size x
   * @param[in]       dim_kernel_y filter kernel size y
   * @param[in]       padding_x    padding size x
   * @param[in]       padding_y    padding size y
   * @param[in]       stride_x     convolution stride x
   * @param[in]       stride_y     convolution stride y
   * @param[in]       bias         pointer to bias
   * @param[in]       bias_shift   amount of left-shift for bias
   * @param[in]       out_shift    amount of right-shift for output
   * @param[in,out]   Im_out       pointer to output tensor
   * @param[in]       dim_im_out_x output tensor dimension x
   * @param[in]       dim_im_out_y output tensor dimension y
   * @param[in,out]   bufferA      pointer to buffer space for input
   * @param[in,out]   bufferB      pointer to buffer space for output
   * @return     The function returns <code>ARM_MATH_SUCCESS</code>
   */

arm_status arm_convolve_HWC_q7_basic_nonsquare(const q7_t * Im_in,
                                               const uint16_t dim_im_in_x,
                                               const uint16_t dim_im_in_y,
                                               const uint16_t ch_im_in,
                                               const q7_t * wt,
                                               const uint16_t ch_im_out,
                                               const uint16_t dim_kernel_x,
                                               const uint16_t dim_kernel_y,
                                               const uint16_t padding_x,
                                               const uint16_t padding_y,
                                               const uint16_t stride_x,
                                               const uint16_t stride_y,
                                               const q7_t * bias,
                                               const uint16_t bias_shift,
                                               const uint16_t out_shift,
                                               q7_t * Im_out,
                                               const uint16_t dim_im_out_x,
                                               const uint16_t dim_im_out_y,
                                               q15_t * bufferA,
                                               q7_t * bufferB)
{

#if defined (ARM_MATH_DSP)
    /* Run the following code for Cortex-M4 and Cortex-M7 */

    int16_t   i_out_y, i_out_x, i_ker_y, i_ker_x;

    /* 
     *  Here we use bufferA as q15_t internally as computation are done with q15_t level
     *  im2col are done to output in q15_t format from q7_t input
     */
    q15_t    *pBuffer = bufferA;
    q7_t     *pOut = Im_out;

    /* This part implements the im2col function */
    for (i_out_y = 0; i_out_y < dim_im_out_y; i_out_y++)
    {
        for (i_out_x = 0; i_out_x < dim_im_out_x; i_out_x++)
        {
            for (i_ker_y = i_out_y * stride_y - padding_y; i_ker_y < i_out_y * stride_y - padding_y + dim_kernel_y; i_ker_y++)
            {
                for (i_ker_x = i_out_x * stride_x - padding_x; i_ker_x < i_out_x * stride_x - padding_x + dim_kernel_x; i_ker_x++)
                {
                    if (i_ker_y < 0 || i_ker_y >= dim_im_in_y || i_ker_x < 0 || i_ker_x >= dim_im_in_x)
                    {
                        /* Filling 0 for out-of-bound paddings */
                        /* arm_fill_q15(0, pBuffer, ch_im_in); */
                        memset(pBuffer, 0, sizeof(q15_t)*ch_im_in);
                    } else
                    {
                        /* Copying the pixel data to column */
                        arm_q7_to_q15_no_shift((q7_t *)
                                               Im_in + (i_ker_y * dim_im_in_x + i_ker_x) * ch_im_in, pBuffer, ch_im_in);
                    }
                    pBuffer += ch_im_in;
                }
            }

            /* Computation is filed for every 2 columns */
            if (pBuffer == bufferA + 2 * ch_im_in * dim_kernel_y * dim_kernel_x)
            {
                pOut =
                    arm_nn_mat_mult_kernel_q7_q15(wt, bufferA,
                                                  ch_im_out,
                                                  ch_im_in *
                                                  dim_kernel_y * dim_kernel_x, bias_shift, out_shift, bias, pOut);

                /* counter reset */
                pBuffer = bufferA;
            }
        }
    }

    /* left-over because odd number of output pixels */
    if (pBuffer != bufferA)
    {
        const q7_t *pA = wt;
        int       i;

        for (i = 0; i < ch_im_out; i++)
        {
            /* Load the accumulator with bias first */
            q31_t     sum = ((q31_t)bias[i] << bias_shift) + NN_ROUND(out_shift);

            /* Point to the beging of the im2col buffer */
            q15_t    *pB = bufferA;

            /* Each time it process 4 entries */
            uint16_t  colCnt = ch_im_in * dim_kernel_y * dim_kernel_x >> 2;

            while (colCnt)
            {
                q31_t     inA1, inA2;
                q31_t     inB1, inB2;

                pA = (q7_t *) read_and_pad((void *)pA, &inA1, &inA2);

                inB1 = *__SIMD32(pB)++;
                sum = __SMLAD(inA1, inB1, sum);
                inB2 = *__SIMD32(pB)++;
                sum = __SMLAD(inA2, inB2, sum);

                colCnt--;
            }
            colCnt = ch_im_in * dim_kernel_y * dim_kernel_x & 0x3;
            while (colCnt)
            {
                q7_t      inA1 = *pA++;
                q15_t     inB1 = *pB++;
                sum += inA1 * inB1;
                colCnt--;
            }
            *pOut++ = (q7_t) __SSAT((sum >> out_shift), 8);
        }
    }
#else
    /* Run the following code as reference implementation for Cortex-M0 and Cortex-M3 */

    uint16_t  i, j, k, l, m, n;
    int       conv_out;
    signed char in_row, in_col;

    for (i = 0; i < ch_im_out; i++)
    {
        for (j = 0; j < dim_im_out_y; j++)
        {
            for (k = 0; k < dim_im_out_x; k++)
            {
                conv_out = ((q31_t)bias[i] << bias_shift) + NN_ROUND(out_shift);
                for (m = 0; m < dim_kernel_y; m++)
                {
                    for (n = 0; n < dim_kernel_x; n++)
                    {
                        // if-for implementation
                        in_row = stride_y * j + m - padding_y;
                        in_col = stride_x * k + n - padding_x;
                        if (in_row >= 0 && in_col >= 0 && in_row < dim_im_in_y && in_col < dim_im_in_x)
                        {
                            for (l = 0; l < ch_im_in; l++)
                            {
                                conv_out +=
                                    Im_in[(in_row * dim_im_in_x + in_col) * ch_im_in + l] * 
                                         wt[i * ch_im_in * dim_kernel_y * dim_kernel_x + 
                                         (m * dim_kernel_x + n) * ch_im_in + l];
                            }
                        }
                    }
                }
                Im_out[i + (j * dim_im_out_x + k) * ch_im_out] = (q7_t) __SSAT((conv_out >> out_shift), 8);
            }
        }
    }

#endif                          /* ARM_MATH_DSP */

    /* Return to application */
    return ARM_MATH_SUCCESS;
}

/**
 * @} end of NNConv group
 */
>>>>>>> b334a05 (Week 3 Tasks)
