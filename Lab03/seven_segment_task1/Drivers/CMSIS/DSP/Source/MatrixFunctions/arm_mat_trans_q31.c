<<<<<<< HEAD
/* ----------------------------------------------------------------------
 * Project:      CMSIS DSP Library
 * Title:        arm_mat_trans_q31.c
 * Description:  Q31 matrix transpose
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
 * @ingroup groupMatrix
 */

/**
 * @addtogroup MatrixTrans
 * @{
 */

/*
  * @brief Q31 matrix transpose.
  * @param[in]  *pSrc points to the input matrix
  * @param[out] *pDst points to the output matrix
  * @return 	The function returns either  <code>ARM_MATH_SIZE_MISMATCH</code>
  * or <code>ARM_MATH_SUCCESS</code> based on the outcome of size checking.
 */

arm_status arm_mat_trans_q31(
  const arm_matrix_instance_q31 * pSrc,
  arm_matrix_instance_q31 * pDst)
{
  q31_t *pIn = pSrc->pData;                      /* input data matrix pointer  */
  q31_t *pOut = pDst->pData;                     /* output data matrix pointer  */
  q31_t *px;                                     /* Temporary output data matrix pointer */
  uint16_t nRows = pSrc->numRows;                /* number of nRows */
  uint16_t nColumns = pSrc->numCols;             /* number of nColumns  */

#if defined (ARM_MATH_DSP)

  /* Run the below code for Cortex-M4 and Cortex-M3 */

  uint16_t blkCnt, i = 0U, row = nRows;          /* loop counters */
  arm_status status;                             /* status of matrix transpose */


#ifdef ARM_MATH_MATRIX_CHECK


  /* Check for matrix mismatch condition */
  if ((pSrc->numRows != pDst->numCols) || (pSrc->numCols != pDst->numRows))
  {
    /* Set status as ARM_MATH_SIZE_MISMATCH */
    status = ARM_MATH_SIZE_MISMATCH;
  }
  else
#endif /*    #ifdef ARM_MATH_MATRIX_CHECK    */

  {
    /* Matrix transpose by exchanging the rows with columns */
    /* row loop     */
    do
    {
      /* Apply loop unrolling and exchange the columns with row elements */
      blkCnt = nColumns >> 2U;

      /* The pointer px is set to starting address of the column being processed */
      px = pOut + i;

      /* First part of the processing with loop unrolling.  Compute 4 outputs at a time.
       ** a second loop below computes the remaining 1 to 3 samples. */
      while (blkCnt > 0U)
      {
        /* Read and store the input element in the destination */
        *px = *pIn++;

        /* Update the pointer px to point to the next row of the transposed matrix */
        px += nRows;

        /* Read and store the input element in the destination */
        *px = *pIn++;

        /* Update the pointer px to point to the next row of the transposed matrix */
        px += nRows;

        /* Read and store the input element in the destination */
        *px = *pIn++;

        /* Update the pointer px to point to the next row of the transposed matrix */
        px += nRows;

        /* Read and store the input element in the destination */
        *px = *pIn++;

        /* Update the pointer px to point to the next row of the transposed matrix */
        px += nRows;

        /* Decrement the column loop counter */
        blkCnt--;
      }

      /* Perform matrix transpose for last 3 samples here. */
      blkCnt = nColumns % 0x4U;

      while (blkCnt > 0U)
      {
        /* Read and store the input element in the destination */
        *px = *pIn++;

        /* Update the pointer px to point to the next row of the transposed matrix */
        px += nRows;

        /* Decrement the column loop counter */
        blkCnt--;
      }

#else

  /* Run the below code for Cortex-M0 */

  uint16_t col, i = 0U, row = nRows;             /* loop counters */
  arm_status status;                             /* status of matrix transpose */


#ifdef ARM_MATH_MATRIX_CHECK

  /* Check for matrix mismatch condition */
  if ((pSrc->numRows != pDst->numCols) || (pSrc->numCols != pDst->numRows))
  {
    /* Set status as ARM_MATH_SIZE_MISMATCH */
    status = ARM_MATH_SIZE_MISMATCH;
  }
  else
#endif /*    #ifdef ARM_MATH_MATRIX_CHECK    */

  {
    /* Matrix transpose by exchanging the rows with columns */
    /* row loop     */
    do
    {
      /* The pointer px is set to starting address of the column being processed */
      px = pOut + i;

      /* Initialize column loop counter */
      col = nColumns;

      while (col > 0U)
      {
        /* Read and store the input element in the destination */
        *px = *pIn++;

        /* Update the pointer px to point to the next row of the transposed matrix */
        px += nRows;

        /* Decrement the column loop counter */
        col--;
      }

#endif /* #if defined (ARM_MATH_DSP) */

      i++;

      /* Decrement the row loop counter */
      row--;

    }
    while (row > 0U);            /* row loop end */

    /* set status as ARM_MATH_SUCCESS */
    status = ARM_MATH_SUCCESS;
  }

  /* Return to application */
  return (status);
}

/**
 * @} end of MatrixTrans group
 */
=======
/* ----------------------------------------------------------------------
 * Project:      CMSIS DSP Library
 * Title:        arm_mat_trans_q31.c
 * Description:  Q31 matrix transpose
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
 * @ingroup groupMatrix
 */

/**
 * @addtogroup MatrixTrans
 * @{
 */

/*
  * @brief Q31 matrix transpose.
  * @param[in]  *pSrc points to the input matrix
  * @param[out] *pDst points to the output matrix
  * @return 	The function returns either  <code>ARM_MATH_SIZE_MISMATCH</code>
  * or <code>ARM_MATH_SUCCESS</code> based on the outcome of size checking.
 */

arm_status arm_mat_trans_q31(
  const arm_matrix_instance_q31 * pSrc,
  arm_matrix_instance_q31 * pDst)
{
  q31_t *pIn = pSrc->pData;                      /* input data matrix pointer  */
  q31_t *pOut = pDst->pData;                     /* output data matrix pointer  */
  q31_t *px;                                     /* Temporary output data matrix pointer */
  uint16_t nRows = pSrc->numRows;                /* number of nRows */
  uint16_t nColumns = pSrc->numCols;             /* number of nColumns  */

#if defined (ARM_MATH_DSP)

  /* Run the below code for Cortex-M4 and Cortex-M3 */

  uint16_t blkCnt, i = 0U, row = nRows;          /* loop counters */
  arm_status status;                             /* status of matrix transpose */


#ifdef ARM_MATH_MATRIX_CHECK


  /* Check for matrix mismatch condition */
  if ((pSrc->numRows != pDst->numCols) || (pSrc->numCols != pDst->numRows))
  {
    /* Set status as ARM_MATH_SIZE_MISMATCH */
    status = ARM_MATH_SIZE_MISMATCH;
  }
  else
#endif /*    #ifdef ARM_MATH_MATRIX_CHECK    */

  {
    /* Matrix transpose by exchanging the rows with columns */
    /* row loop     */
    do
    {
      /* Apply loop unrolling and exchange the columns with row elements */
      blkCnt = nColumns >> 2U;

      /* The pointer px is set to starting address of the column being processed */
      px = pOut + i;

      /* First part of the processing with loop unrolling.  Compute 4 outputs at a time.
       ** a second loop below computes the remaining 1 to 3 samples. */
      while (blkCnt > 0U)
      {
        /* Read and store the input element in the destination */
        *px = *pIn++;

        /* Update the pointer px to point to the next row of the transposed matrix */
        px += nRows;

        /* Read and store the input element in the destination */
        *px = *pIn++;

        /* Update the pointer px to point to the next row of the transposed matrix */
        px += nRows;

        /* Read and store the input element in the destination */
        *px = *pIn++;

        /* Update the pointer px to point to the next row of the transposed matrix */
        px += nRows;

        /* Read and store the input element in the destination */
        *px = *pIn++;

        /* Update the pointer px to point to the next row of the transposed matrix */
        px += nRows;

        /* Decrement the column loop counter */
        blkCnt--;
      }

      /* Perform matrix transpose for last 3 samples here. */
      blkCnt = nColumns % 0x4U;

      while (blkCnt > 0U)
      {
        /* Read and store the input element in the destination */
        *px = *pIn++;

        /* Update the pointer px to point to the next row of the transposed matrix */
        px += nRows;

        /* Decrement the column loop counter */
        blkCnt--;
      }

#else

  /* Run the below code for Cortex-M0 */

  uint16_t col, i = 0U, row = nRows;             /* loop counters */
  arm_status status;                             /* status of matrix transpose */


#ifdef ARM_MATH_MATRIX_CHECK

  /* Check for matrix mismatch condition */
  if ((pSrc->numRows != pDst->numCols) || (pSrc->numCols != pDst->numRows))
  {
    /* Set status as ARM_MATH_SIZE_MISMATCH */
    status = ARM_MATH_SIZE_MISMATCH;
  }
  else
#endif /*    #ifdef ARM_MATH_MATRIX_CHECK    */

  {
    /* Matrix transpose by exchanging the rows with columns */
    /* row loop     */
    do
    {
      /* The pointer px is set to starting address of the column being processed */
      px = pOut + i;

      /* Initialize column loop counter */
      col = nColumns;

      while (col > 0U)
      {
        /* Read and store the input element in the destination */
        *px = *pIn++;

        /* Update the pointer px to point to the next row of the transposed matrix */
        px += nRows;

        /* Decrement the column loop counter */
        col--;
      }

#endif /* #if defined (ARM_MATH_DSP) */

      i++;

      /* Decrement the row loop counter */
      row--;

    }
    while (row > 0U);            /* row loop end */

    /* set status as ARM_MATH_SUCCESS */
    status = ARM_MATH_SUCCESS;
  }

  /* Return to application */
  return (status);
}

/**
 * @} end of MatrixTrans group
 */
>>>>>>> b334a05 (Week 3 Tasks)
