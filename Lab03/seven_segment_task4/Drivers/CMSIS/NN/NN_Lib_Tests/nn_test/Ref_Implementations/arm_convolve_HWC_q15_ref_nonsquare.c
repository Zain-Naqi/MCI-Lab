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

#include "ref_functions.h"

void
arm_convolve_HWC_q15_nonsquare_ref(const q15_t * Im_in,
                          const uint16_t dim_im_in_x,
                          const uint16_t dim_im_in_y,
                          const uint16_t ch_im_in,
                          const q15_t * wt,
                          const uint16_t ch_im_out,
                          const uint16_t dim_kernel_x,
                          const uint16_t dim_kernel_y,
                          const uint16_t padding_x,
                          const uint16_t padding_y,
                          const uint16_t stride_x,
                          const uint16_t stride_y,
                          const q15_t * bias,
                          const uint16_t bias_shift,
                          const uint16_t out_shift,
                          q15_t * Im_out,
                          const uint16_t dim_im_out_x,
                          const uint16_t dim_im_out_y, 
                          q15_t * bufferA, 
                          q7_t * bufferB)

{	
    uint16_t  i, j, k, l, m, n;
    int       conv_out;
    signed char in_row, in_col;

    for (i = 0; i < ch_im_out; i++)
    {
        for (j = 0; j < dim_im_out_y; j++)
        {
            for (k = 0; k < dim_im_out_x; k++)
            {
#ifndef ARM_NN_TRUNCATE
                conv_out = (bias[i] << bias_shift) + (0x1 << (out_shift - 1));
#else
                conv_out = bias[i] << bias_shift;
#endif
                for (m = 0; m < dim_kernel_y; m++)
                {
                    for (n = 0; n < dim_kernel_x; n++)
                    {
                        in_row = stride_y * j + m - padding_y;
                        in_col = stride_x * k + n - padding_x;
                        if (in_row >= 0 && in_col >= 0 && in_row < dim_im_in_y && in_col < dim_im_in_x)
                        {
                            for (l = 0; l < ch_im_in; l++)
                            {
                                conv_out +=
                                    Im_in[(in_row * dim_im_in_x + in_col) * ch_im_in +
                                          l] * wt[i * ch_im_in * dim_kernel_x * dim_kernel_y + (m * dim_kernel_x +
                                                                                            n) * ch_im_in + l];
                            }
                        }
                    }
                }
                Im_out[i + (j * dim_im_out_x + k) * ch_im_out] = (q15_t) __SSAT((conv_out >> out_shift), 16);
            }
        }
    }
}	

	
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

#include "ref_functions.h"

void
arm_convolve_HWC_q15_nonsquare_ref(const q15_t * Im_in,
                          const uint16_t dim_im_in_x,
                          const uint16_t dim_im_in_y,
                          const uint16_t ch_im_in,
                          const q15_t * wt,
                          const uint16_t ch_im_out,
                          const uint16_t dim_kernel_x,
                          const uint16_t dim_kernel_y,
                          const uint16_t padding_x,
                          const uint16_t padding_y,
                          const uint16_t stride_x,
                          const uint16_t stride_y,
                          const q15_t * bias,
                          const uint16_t bias_shift,
                          const uint16_t out_shift,
                          q15_t * Im_out,
                          const uint16_t dim_im_out_x,
                          const uint16_t dim_im_out_y, 
                          q15_t * bufferA, 
                          q7_t * bufferB)

{	
    uint16_t  i, j, k, l, m, n;
    int       conv_out;
    signed char in_row, in_col;

    for (i = 0; i < ch_im_out; i++)
    {
        for (j = 0; j < dim_im_out_y; j++)
        {
            for (k = 0; k < dim_im_out_x; k++)
            {
#ifndef ARM_NN_TRUNCATE
                conv_out = (bias[i] << bias_shift) + (0x1 << (out_shift - 1));
#else
                conv_out = bias[i] << bias_shift;
#endif
                for (m = 0; m < dim_kernel_y; m++)
                {
                    for (n = 0; n < dim_kernel_x; n++)
                    {
                        in_row = stride_y * j + m - padding_y;
                        in_col = stride_x * k + n - padding_x;
                        if (in_row >= 0 && in_col >= 0 && in_row < dim_im_in_y && in_col < dim_im_in_x)
                        {
                            for (l = 0; l < ch_im_in; l++)
                            {
                                conv_out +=
                                    Im_in[(in_row * dim_im_in_x + in_col) * ch_im_in +
                                          l] * wt[i * ch_im_in * dim_kernel_x * dim_kernel_y + (m * dim_kernel_x +
                                                                                            n) * ch_im_in + l];
                            }
                        }
                    }
                }
                Im_out[i + (j * dim_im_out_x + k) * ch_im_out] = (q15_t) __SSAT((conv_out >> out_shift), 16);
            }
        }
    }
}	

	
>>>>>>> b334a05 (Week 3 Tasks)
