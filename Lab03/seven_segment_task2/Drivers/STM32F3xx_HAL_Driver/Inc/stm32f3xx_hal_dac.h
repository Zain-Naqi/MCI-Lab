<<<<<<< HEAD
/**
  ******************************************************************************
  * @file    stm32f3xx_hal_dac.h
  * @author  MCD Application Team
  * @brief   Header file of DAC HAL module.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2016 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef STM32F3xx_HAL_DAC_H
#define STM32F3xx_HAL_DAC_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal_def.h"
   
/** @addtogroup STM32F3xx_HAL_Driver
  * @{
  */

/** @addtogroup DAC
  * @{
  */

/* Exported types ------------------------------------------------------------*/

/** @defgroup DAC_Exported_Types DAC Exported Types
  * @{
  */

/** 
  * @brief  HAL State structures definition  
  */ 
typedef enum
{
  HAL_DAC_STATE_RESET             = 0x00U,  /*!< DAC not yet initialized or disabled  */
  HAL_DAC_STATE_READY             = 0x01U,  /*!< DAC initialized and ready for use    */
  HAL_DAC_STATE_BUSY              = 0x02U,  /*!< DAC internal processing is ongoing   */
  HAL_DAC_STATE_TIMEOUT           = 0x03U,  /*!< DAC timeout state                    */
  HAL_DAC_STATE_ERROR             = 0x04U   /*!< DAC error state                      */
 
}HAL_DAC_StateTypeDef;
   
/** 
  * @brief   DAC Configuration regular Channel structure definition  
  */ 
typedef struct
{
  uint32_t DAC_Trigger;                 /*!< Specifies the external trigger for the selected DAC channel.
                                        This parameter can be a value of @ref DACEx_trigger_selection */
  
  uint32_t DAC_OutputBuffer;            /*!< Specifies whether the DAC channel output buffer is enabled or disabled.
                                        This parameter can be a value of @ref DAC_output_buffer 
                                        For a given DAC channel, is this parameter applies then DAC_OutputSwitch 
                                        does not apply */
  
  uint32_t DAC_OutputSwitch;            /*!< Specifies whether the DAC channel output switch is enabled or disabled.
                                        This parameter can be a value of @ref DAC_OutputSwitch 
                                        For a given DAC channel, is this parameter applies then DAC_OutputBuffer 
                                        does not apply */
  
}DAC_ChannelConfTypeDef;

/** 
  * @brief  DAC handle Structure definition  
  */ 
typedef struct __DAC_HandleTypeDef
{
  DAC_TypeDef                 *Instance;      /*!< Register base address             */
  
  __IO HAL_DAC_StateTypeDef   State;          /*!< DAC communication state           */

  HAL_LockTypeDef             Lock;          /*!< DAC locking object                */
  
  DMA_HandleTypeDef           *DMA_Handle1;  /*!< Pointer DMA handler for channel 1U */
  
  DMA_HandleTypeDef           *DMA_Handle2;  /*!< Pointer DMA handler for channel 2U */ 
  
  __IO uint32_t               ErrorCode;     /*!< DAC Error code                    */

#if (USE_HAL_DAC_REGISTER_CALLBACKS == 1)
  void (* ConvCpltCallbackCh1)            (struct __DAC_HandleTypeDef *hdac);
  void (* ConvHalfCpltCallbackCh1)        (struct __DAC_HandleTypeDef *hdac);
  void (* ErrorCallbackCh1)               (struct __DAC_HandleTypeDef *hdac);
  void (* DMAUnderrunCallbackCh1)         (struct __DAC_HandleTypeDef *hdac);
  void (* ConvCpltCallbackCh2)            (struct __DAC_HandleTypeDef* hdac);
  void (* ConvHalfCpltCallbackCh2)        (struct __DAC_HandleTypeDef* hdac);
  void (* ErrorCallbackCh2)               (struct __DAC_HandleTypeDef* hdac);
  void (* DMAUnderrunCallbackCh2)         (struct __DAC_HandleTypeDef* hdac); 

  void (* MspInitCallback)                (struct __DAC_HandleTypeDef *hdac);
  void (* MspDeInitCallback )             (struct __DAC_HandleTypeDef *hdac); 
#endif /* USE_HAL_DAC_REGISTER_CALLBACKS */

}DAC_HandleTypeDef;

#if (USE_HAL_DAC_REGISTER_CALLBACKS == 1)
/**
  * @brief  HAL DAC Callback ID enumeration definition
  */
typedef enum
{
  HAL_DAC_CH1_COMPLETE_CB_ID                 = 0x00U,  /*!< DAC CH1 Complete Callback ID      */
  HAL_DAC_CH1_HALF_COMPLETE_CB_ID            = 0x01U,  /*!< DAC CH1 half Complete Callback ID */
  HAL_DAC_CH1_ERROR_ID                       = 0x02U,  /*!< DAC CH1 error Callback ID         */
  HAL_DAC_CH1_UNDERRUN_CB_ID                 = 0x03U,  /*!< DAC CH1 underrun Callback ID      */
  HAL_DAC_CH2_COMPLETE_CB_ID                 = 0x04U,  /*!< DAC CH2 Complete Callback ID      */
  HAL_DAC_CH2_HALF_COMPLETE_CB_ID            = 0x05U,  /*!< DAC CH2 half Complete Callback ID */
  HAL_DAC_CH2_ERROR_ID                       = 0x06U,  /*!< DAC CH2 error Callback ID         */
  HAL_DAC_CH2_UNDERRUN_CB_ID                 = 0x07U,  /*!< DAC CH2 underrun Callback ID      */
  HAL_DAC_MSP_INIT_CB_ID                     = 0x08U,  /*!< DAC MspInit Callback ID           */
  HAL_DAC_MSP_DEINIT_CB_ID                   = 0x09U,  /*!< DAC MspDeInit Callback ID         */
  HAL_DAC_ALL_CB_ID                          = 0x0AU   /*!< DAC All ID                        */
}HAL_DAC_CallbackIDTypeDef;                            
                                                       
/**
  * @brief  HAL DAC Callback pointer definition
  */
typedef void (*pDAC_CallbackTypeDef)(DAC_HandleTypeDef *hdac);
#endif /* USE_HAL_DAC_REGISTER_CALLBACKS */

/**
  * @}
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup DAC_Exported_Constants DAC Exported Constants
  * @{
  */

/** @defgroup DAC_Error_Code DAC Error Code
  * @{
  */
#define  HAL_DAC_ERROR_NONE              0x00U    /*!< No error                          */
#define  HAL_DAC_ERROR_DMAUNDERRUNCH1    0x01U    /*!< DAC channel1 DMA underrun error   */
#define  HAL_DAC_ERROR_DMAUNDERRUNCH2    0x02U    /*!< DAC channel2 DMA underrun error   */
#define  HAL_DAC_ERROR_DMA               0x04U    /*!< DMA error                         */
#if (USE_HAL_DAC_REGISTER_CALLBACKS == 1)
#define HAL_DAC_ERROR_INVALID_CALLBACK   0x10U    /*!< Invalid callback error            */
#endif /* USE_HAL_DAC_REGISTER_CALLBACKS */  
/**
  * @}
  */

/** @defgroup DAC_lfsrunmask_triangleamplitude DAC lfsrunmask triangleamplitude
  * @{
  */
#define DAC_LFSRUNMASK_BIT0                (0x00000000U) /*!< Unmask DAC channel LFSR bit0 for noise wave generation */
#define DAC_LFSRUNMASK_BITS1_0             ((uint32_t)DAC_CR_MAMP1_0) /*!< Unmask DAC channel LFSR bit[1:0] for noise wave generation */
#define DAC_LFSRUNMASK_BITS2_0             ((uint32_t)DAC_CR_MAMP1_1) /*!< Unmask DAC channel LFSR bit[2:0] for noise wave generation */
#define DAC_LFSRUNMASK_BITS3_0             ((uint32_t)DAC_CR_MAMP1_1 | DAC_CR_MAMP1_0)/*!< Unmask DAC channel LFSR bit[3:0] for noise wave generation */
#define DAC_LFSRUNMASK_BITS4_0             ((uint32_t)DAC_CR_MAMP1_2) /*!< Unmask DAC channel LFSR bit[4:0] for noise wave generation */
#define DAC_LFSRUNMASK_BITS5_0             ((uint32_t)DAC_CR_MAMP1_2 | DAC_CR_MAMP1_0) /*!< Unmask DAC channel LFSR bit[5:0] for noise wave generation */
#define DAC_LFSRUNMASK_BITS6_0             ((uint32_t)DAC_CR_MAMP1_2 | DAC_CR_MAMP1_1) /*!< Unmask DAC channel LFSR bit[6:0] for noise wave generation */
#define DAC_LFSRUNMASK_BITS7_0             ((uint32_t)DAC_CR_MAMP1_2 | DAC_CR_MAMP1_1 | DAC_CR_MAMP1_0) /*!< Unmask DAC channel LFSR bit[7:0] for noise wave generation */
#define DAC_LFSRUNMASK_BITS8_0             ((uint32_t)DAC_CR_MAMP1_3) /*!< Unmask DAC channel LFSR bit[8:0] for noise wave generation */
#define DAC_LFSRUNMASK_BITS9_0             ((uint32_t)DAC_CR_MAMP1_3 | DAC_CR_MAMP1_0) /*!< Unmask DAC channel LFSR bit[9:0] for noise wave generation */
#define DAC_LFSRUNMASK_BITS10_0            ((uint32_t)DAC_CR_MAMP1_3 | DAC_CR_MAMP1_1) /*!< Unmask DAC channel LFSR bit[10:0] for noise wave generation */
#define DAC_LFSRUNMASK_BITS11_0            ((uint32_t)DAC_CR_MAMP1_3 | DAC_CR_MAMP1_1 | DAC_CR_MAMP1_0) /*!< Unmask DAC channel LFSR bit[11:0] for noise wave generation */
#define DAC_TRIANGLEAMPLITUDE_1            (0x00000000U) /*!< Select max triangle amplitude of 1U */
#define DAC_TRIANGLEAMPLITUDE_3            ((uint32_t)DAC_CR_MAMP1_0) /*!< Select max triangle amplitude of 3U */
#define DAC_TRIANGLEAMPLITUDE_7            ((uint32_t)DAC_CR_MAMP1_1) /*!< Select max triangle amplitude of 7U */
#define DAC_TRIANGLEAMPLITUDE_15           ((uint32_t)DAC_CR_MAMP1_1 | DAC_CR_MAMP1_0) /*!< Select max triangle amplitude of 15U */
#define DAC_TRIANGLEAMPLITUDE_31           ((uint32_t)DAC_CR_MAMP1_2) /*!< Select max triangle amplitude of 31U */
#define DAC_TRIANGLEAMPLITUDE_63           ((uint32_t)DAC_CR_MAMP1_2 | DAC_CR_MAMP1_0) /*!< Select max triangle amplitude of 63U */
#define DAC_TRIANGLEAMPLITUDE_127          ((uint32_t)DAC_CR_MAMP1_2 | DAC_CR_MAMP1_1) /*!< Select max triangle amplitude of 127U */
#define DAC_TRIANGLEAMPLITUDE_255          ((uint32_t)DAC_CR_MAMP1_2 | DAC_CR_MAMP1_1 | DAC_CR_MAMP1_0) /*!< Select max triangle amplitude of 255U */
#define DAC_TRIANGLEAMPLITUDE_511          ((uint32_t)DAC_CR_MAMP1_3) /*!< Select max triangle amplitude of 511U */
#define DAC_TRIANGLEAMPLITUDE_1023         ((uint32_t)DAC_CR_MAMP1_3 | DAC_CR_MAMP1_0) /*!< Select max triangle amplitude of 1023U */
#define DAC_TRIANGLEAMPLITUDE_2047         ((uint32_t)DAC_CR_MAMP1_3 | DAC_CR_MAMP1_1) /*!< Select max triangle amplitude of 2047U */
#define DAC_TRIANGLEAMPLITUDE_4095         ((uint32_t)DAC_CR_MAMP1_3 | DAC_CR_MAMP1_1 | DAC_CR_MAMP1_0) /*!< Select max triangle amplitude of 4095U */

/**
  * @}
  */

/** @defgroup DAC_output_buffer DAC output buffer
  * @{
  */
#define DAC_OUTPUTBUFFER_ENABLE            (0x00000000U)
#define DAC_OUTPUTBUFFER_DISABLE           ((uint32_t)DAC_CR_BOFF1)

/**
  * @}
  */

#if defined(DAC_CR_OUTEN1)
/** @defgroup DAC_output_switch DAC output switch
  * @{
  */
#define DAC_OUTPUTSWITCH_DISABLE           (0x00000000U)
#define DAC_OUTPUTSWITCH_ENABLE            ((uint32_t)DAC_CR_OUTEN1)

/**
  * @}
  */

#endif /* DAC_CR_OUTEN1 */
/** @defgroup DAC_data_alignment DAC data alignment
  * @{
  */
#define DAC_ALIGN_12B_R                    (0x00000000U)
#define DAC_ALIGN_12B_L                    (0x00000004U)
#define DAC_ALIGN_8B_R                     (0x00000008U)

/**
  * @}
  */

/** @defgroup DAC_flags_definition DAC flags definition
  * @{
  */ 
#define DAC_FLAG_DMAUDR1                   ((uint32_t)DAC_SR_DMAUDR1)
#define DAC_FLAG_DMAUDR2                   ((uint32_t)DAC_SR_DMAUDR2)   
/**
  * @}
  */

/** @defgroup DAC_interrupts_definition DAC interrupts definition
  * @{
  */ 
#define DAC_IT_DMAUDR1                   ((uint32_t)DAC_CR_DMAUDRIE1)
#define DAC_IT_DMAUDR2                   ((uint32_t)DAC_CR_DMAUDRIE2) 

/**
  * @}
  */

/**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/

/** @defgroup DAC_Exported_Macros DAC Exported Macros
  * @{
  */

/** @brief Reset DAC handle state
  * @param  __HANDLE__ specifies the DAC handle.
  * @retval None
  */
#if (USE_HAL_DAC_REGISTER_CALLBACKS == 1)
#define __HAL_DAC_RESET_HANDLE_STATE(__HANDLE__)           do {                                              \
                                                                 (__HANDLE__)->State = HAL_DAC_STATE_RESET;  \
                                                                 (__HANDLE__)->MspInitCallback = NULL;       \
                                                                 (__HANDLE__)->MspDeInitCallback = NULL;     \
                                                               } while(0)
#else
#define __HAL_DAC_RESET_HANDLE_STATE(__HANDLE__) ((__HANDLE__)->State = HAL_DAC_STATE_RESET)
#endif /* USE_HAL_DAC_REGISTER_CALLBACKS */

/** @brief Enable the DAC channel
  * @param  __HANDLE__ specifies the DAC handle.
  * @param  __DAC_Channel__ specifies the DAC channel
  * @retval None
  */
#define __HAL_DAC_ENABLE(__HANDLE__, __DAC_Channel__) \
((__HANDLE__)->Instance->CR |=  (DAC_CR_EN1 << (__DAC_Channel__)))

/** @brief Disable the DAC channel
  * @param  __HANDLE__ specifies the DAC handle
  * @param  __DAC_Channel__ specifies the DAC channel.
  * @retval None
  */
#define __HAL_DAC_DISABLE(__HANDLE__, __DAC_Channel__) \
((__HANDLE__)->Instance->CR &=  ~(DAC_CR_EN1 << (__DAC_Channel__)))
 
/** @brief Set DHR12R1 alignment
  * @param  __ALIGNMENT__ specifies the DAC alignment
  * @retval None
  */
#define DAC_DHR12R1_ALIGNMENT(__ALIGNMENT__) ((0x00000008U) + (__ALIGNMENT__))

/** @brief  Set DHR12R2 alignment
  * @param  __ALIGNMENT__ specifies the DAC alignment
  * @retval None
  */
#define DAC_DHR12R2_ALIGNMENT(__ALIGNMENT__) ((0x00000014U) + (__ALIGNMENT__))

/** @brief  Set DHR12RD alignment
  * @param  __ALIGNMENT__ specifies the DAC alignment
  * @retval None
  */
#define DAC_DHR12RD_ALIGNMENT(__ALIGNMENT__) ((0x00000020U) + (__ALIGNMENT__))

/** @brief Enable the DAC interrupt
  * @param  __HANDLE__ specifies the DAC handle
  * @param  __INTERRUPT__ specifies the DAC interrupt.
  *          This parameter can be any combination of the following values:
  *            @arg DAC_IT_DMAUDR1: DAC channel 1 DMA underrun interrupt
  *            @arg DAC_IT_DMAUDR2: DAC channel 2 DMA underrun interrupt
  * @retval None
  */
#define __HAL_DAC_ENABLE_IT(__HANDLE__, __INTERRUPT__) (((__HANDLE__)->Instance->CR) |= (__INTERRUPT__))

/** @brief Disable the DAC interrupt
  * @param  __HANDLE__ specifies the DAC handle
  * @param  __INTERRUPT__ specifies the DAC interrupt.
  *          This parameter can be any combination of the following values:
  *            @arg DAC_IT_DMAUDR1: DAC channel 1 DMA underrun interrupt
  *            @arg DAC_IT_DMAUDR2: DAC channel 2 DMA underrun interrupt
  * @retval None
  */
#define __HAL_DAC_DISABLE_IT(__HANDLE__, __INTERRUPT__) (((__HANDLE__)->Instance->CR) &= ~(__INTERRUPT__))

/** @brief  Check whether the specified DAC interrupt source is enabled or not
  * @param __HANDLE__ DAC handle
  * @param __INTERRUPT__ DAC interrupt source to check
  *          This parameter can be any combination of the following values:
  *            @arg DAC_IT_DMAUDR1: DAC channel 1 DMA underrun interrupt
  *            @arg DAC_IT_DMAUDR2: DAC channel 2 DMA underrun interrupt
  * @retval State of interruption (SET or RESET)
  */
#define __HAL_DAC_GET_IT_SOURCE(__HANDLE__, __INTERRUPT__) (((__HANDLE__)->Instance->CR & (__INTERRUPT__)) == (__INTERRUPT__))

/** @brief  Get the selected DAC's flag status
  * @param  __HANDLE__ specifies the DAC handle.
  * @param  __FLAG__ specifies the DAC flag to get.
  *          This parameter can be any combination of the following values:
  *            @arg DAC_FLAG_DMAUDR1: DAC channel 1 DMA underrun flag
  *            @arg DAC_FLAG_DMAUDR2: DAC channel 2 DMA underrun flag
  * @retval None
  */
#define __HAL_DAC_GET_FLAG(__HANDLE__, __FLAG__) ((((__HANDLE__)->Instance->SR) & (__FLAG__)) == (__FLAG__))

/** @brief  Clear the DAC's flag
  * @param  __HANDLE__ specifies the DAC handle.
  * @param  __FLAG__ specifies the DAC flag to clear.
  *          This parameter can be any combination of the following values:
  *            @arg DAC_FLAG_DMAUDR1: DAC channel 1 DMA underrun flag
  *            @arg DAC_FLAG_DMAUDR2: DAC channel 2 DMA underrun flag
  * @retval None
  */
#define __HAL_DAC_CLEAR_FLAG(__HANDLE__, __FLAG__) (((__HANDLE__)->Instance->SR) = (__FLAG__))

/**
  * @}
  */

/* Private macro -------------------------------------------------------------*/

/** @addtogroup DAC_Private_Macros
  * @{
  */

#define IS_DAC_LFSR_UNMASK_TRIANGLE_AMPLITUDE(VALUE) (((VALUE) == DAC_LFSRUNMASK_BIT0) || \
                                                      ((VALUE) == DAC_LFSRUNMASK_BITS1_0) || \
                                                      ((VALUE) == DAC_LFSRUNMASK_BITS2_0) || \
                                                      ((VALUE) == DAC_LFSRUNMASK_BITS3_0) || \
                                                      ((VALUE) == DAC_LFSRUNMASK_BITS4_0) || \
                                                      ((VALUE) == DAC_LFSRUNMASK_BITS5_0) || \
                                                      ((VALUE) == DAC_LFSRUNMASK_BITS6_0) || \
                                                      ((VALUE) == DAC_LFSRUNMASK_BITS7_0) || \
                                                      ((VALUE) == DAC_LFSRUNMASK_BITS8_0) || \
                                                      ((VALUE) == DAC_LFSRUNMASK_BITS9_0) || \
                                                      ((VALUE) == DAC_LFSRUNMASK_BITS10_0) || \
                                                      ((VALUE) == DAC_LFSRUNMASK_BITS11_0) || \
                                                      ((VALUE) == DAC_TRIANGLEAMPLITUDE_1) || \
                                                      ((VALUE) == DAC_TRIANGLEAMPLITUDE_3) || \
                                                      ((VALUE) == DAC_TRIANGLEAMPLITUDE_7) || \
                                                      ((VALUE) == DAC_TRIANGLEAMPLITUDE_15) || \
                                                      ((VALUE) == DAC_TRIANGLEAMPLITUDE_31) || \
                                                      ((VALUE) == DAC_TRIANGLEAMPLITUDE_63) || \
                                                      ((VALUE) == DAC_TRIANGLEAMPLITUDE_127) || \
                                                      ((VALUE) == DAC_TRIANGLEAMPLITUDE_255) || \
                                                      ((VALUE) == DAC_TRIANGLEAMPLITUDE_511) || \
                                                      ((VALUE) == DAC_TRIANGLEAMPLITUDE_1023) || \
                                                      ((VALUE) == DAC_TRIANGLEAMPLITUDE_2047) || \
                                                      ((VALUE) == DAC_TRIANGLEAMPLITUDE_4095))

#define IS_DAC_OUTPUT_BUFFER_STATE(STATE) (((STATE) == DAC_OUTPUTBUFFER_ENABLE) || \
                                           ((STATE) == DAC_OUTPUTBUFFER_DISABLE))

#if defined(DAC_CR_OUTEN1)
#define IS_DAC_OUTPUT_SWITCH_STATE(STATE) (((STATE) == DAC_OUTPUTSWITCH_DISABLE) || \
                                           ((STATE) == DAC_OUTPUTSWITCH_ENABLE))

#endif /* DAC_CR_OUTEN1 */
#define IS_DAC_ALIGN(ALIGN) (((ALIGN) == DAC_ALIGN_12B_R) || \
                             ((ALIGN) == DAC_ALIGN_12B_L) || \
                             ((ALIGN) == DAC_ALIGN_8B_R))

#define IS_DAC_DATA(DATA) ((DATA) <= 0xFFF0U) 



/**
  * @}
  */


/* Include DAC HAL Extended module */
#include "stm32f3xx_hal_dac_ex.h" 

/* Exported functions --------------------------------------------------------*/  

/** @addtogroup DAC_Exported_Functions
  * @{
  */

/** @addtogroup DAC_Exported_Functions_Group1
  * @{
  */
/* Initialization and de-initialization functions *****************************/ 
HAL_StatusTypeDef HAL_DAC_Init(DAC_HandleTypeDef* hdac);
HAL_StatusTypeDef HAL_DAC_DeInit(DAC_HandleTypeDef* hdac);
void HAL_DAC_MspInit(DAC_HandleTypeDef* hdac);
void HAL_DAC_MspDeInit(DAC_HandleTypeDef* hdac);

/**
  * @}
  */

/** @addtogroup DAC_Exported_Functions_Group2
  * @{
  */
/* IO operation functions *****************************************************/
HAL_StatusTypeDef HAL_DAC_Start(DAC_HandleTypeDef* hdac, uint32_t Channel);
HAL_StatusTypeDef HAL_DAC_Stop(DAC_HandleTypeDef* hdac, uint32_t Channel);
HAL_StatusTypeDef HAL_DAC_Start_DMA(DAC_HandleTypeDef* hdac, uint32_t Channel, uint32_t* pData, uint32_t Length, uint32_t Alignment);
HAL_StatusTypeDef HAL_DAC_Stop_DMA(DAC_HandleTypeDef* hdac, uint32_t Channel);
uint32_t HAL_DAC_GetValue(DAC_HandleTypeDef* hdac, uint32_t Channel);
HAL_StatusTypeDef HAL_DAC_ConfigChannel(DAC_HandleTypeDef* hdac, DAC_ChannelConfTypeDef* sConfig, uint32_t Channel);

void HAL_DAC_IRQHandler(DAC_HandleTypeDef* hdac);
void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef* hdac);
void HAL_DAC_ConvHalfCpltCallbackCh1(DAC_HandleTypeDef* hdac);
void HAL_DAC_ErrorCallbackCh1(DAC_HandleTypeDef *hdac);
void HAL_DAC_DMAUnderrunCallbackCh1(DAC_HandleTypeDef *hdac);

#if (USE_HAL_DAC_REGISTER_CALLBACKS == 1)
/* DAC callback registering/unregistering */
HAL_StatusTypeDef     HAL_DAC_RegisterCallback (DAC_HandleTypeDef *hdac, HAL_DAC_CallbackIDTypeDef CallbackId, pDAC_CallbackTypeDef pCallback);
HAL_StatusTypeDef     HAL_DAC_UnRegisterCallback (DAC_HandleTypeDef *hdac, HAL_DAC_CallbackIDTypeDef CallbackId);
#endif /* USE_HAL_DAC_REGISTER_CALLBACKS */

/**
  * @}
  */
  
/** @addtogroup DAC_Exported_Functions_Group3
  * @{
  */
/* Peripheral Control functions ***********************************************/
HAL_StatusTypeDef HAL_DAC_SetValue(DAC_HandleTypeDef* hdac, uint32_t Channel, uint32_t Alignment, uint32_t Data);

/**
  * @}
  */

/** @addtogroup DAC_Exported_Functions_Group4
  * @{
  */
/* Peripheral State and Error functions ***************************************/
HAL_DAC_StateTypeDef HAL_DAC_GetState(DAC_HandleTypeDef* hdac);
uint32_t HAL_DAC_GetError(DAC_HandleTypeDef *hdac);

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */
  
#ifdef __cplusplus
}
#endif

#endif /* STM32F3xx_HAL_DAC_H */

=======
/**
  ******************************************************************************
  * @file    stm32f3xx_hal_dac.h
  * @author  MCD Application Team
  * @brief   Header file of DAC HAL module.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2016 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef STM32F3xx_HAL_DAC_H
#define STM32F3xx_HAL_DAC_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal_def.h"
   
/** @addtogroup STM32F3xx_HAL_Driver
  * @{
  */

/** @addtogroup DAC
  * @{
  */

/* Exported types ------------------------------------------------------------*/

/** @defgroup DAC_Exported_Types DAC Exported Types
  * @{
  */

/** 
  * @brief  HAL State structures definition  
  */ 
typedef enum
{
  HAL_DAC_STATE_RESET             = 0x00U,  /*!< DAC not yet initialized or disabled  */
  HAL_DAC_STATE_READY             = 0x01U,  /*!< DAC initialized and ready for use    */
  HAL_DAC_STATE_BUSY              = 0x02U,  /*!< DAC internal processing is ongoing   */
  HAL_DAC_STATE_TIMEOUT           = 0x03U,  /*!< DAC timeout state                    */
  HAL_DAC_STATE_ERROR             = 0x04U   /*!< DAC error state                      */
 
}HAL_DAC_StateTypeDef;
   
/** 
  * @brief   DAC Configuration regular Channel structure definition  
  */ 
typedef struct
{
  uint32_t DAC_Trigger;                 /*!< Specifies the external trigger for the selected DAC channel.
                                        This parameter can be a value of @ref DACEx_trigger_selection */
  
  uint32_t DAC_OutputBuffer;            /*!< Specifies whether the DAC channel output buffer is enabled or disabled.
                                        This parameter can be a value of @ref DAC_output_buffer 
                                        For a given DAC channel, is this parameter applies then DAC_OutputSwitch 
                                        does not apply */
  
  uint32_t DAC_OutputSwitch;            /*!< Specifies whether the DAC channel output switch is enabled or disabled.
                                        This parameter can be a value of @ref DAC_OutputSwitch 
                                        For a given DAC channel, is this parameter applies then DAC_OutputBuffer 
                                        does not apply */
  
}DAC_ChannelConfTypeDef;

/** 
  * @brief  DAC handle Structure definition  
  */ 
typedef struct __DAC_HandleTypeDef
{
  DAC_TypeDef                 *Instance;      /*!< Register base address             */
  
  __IO HAL_DAC_StateTypeDef   State;          /*!< DAC communication state           */

  HAL_LockTypeDef             Lock;          /*!< DAC locking object                */
  
  DMA_HandleTypeDef           *DMA_Handle1;  /*!< Pointer DMA handler for channel 1U */
  
  DMA_HandleTypeDef           *DMA_Handle2;  /*!< Pointer DMA handler for channel 2U */ 
  
  __IO uint32_t               ErrorCode;     /*!< DAC Error code                    */

#if (USE_HAL_DAC_REGISTER_CALLBACKS == 1)
  void (* ConvCpltCallbackCh1)            (struct __DAC_HandleTypeDef *hdac);
  void (* ConvHalfCpltCallbackCh1)        (struct __DAC_HandleTypeDef *hdac);
  void (* ErrorCallbackCh1)               (struct __DAC_HandleTypeDef *hdac);
  void (* DMAUnderrunCallbackCh1)         (struct __DAC_HandleTypeDef *hdac);
  void (* ConvCpltCallbackCh2)            (struct __DAC_HandleTypeDef* hdac);
  void (* ConvHalfCpltCallbackCh2)        (struct __DAC_HandleTypeDef* hdac);
  void (* ErrorCallbackCh2)               (struct __DAC_HandleTypeDef* hdac);
  void (* DMAUnderrunCallbackCh2)         (struct __DAC_HandleTypeDef* hdac); 

  void (* MspInitCallback)                (struct __DAC_HandleTypeDef *hdac);
  void (* MspDeInitCallback )             (struct __DAC_HandleTypeDef *hdac); 
#endif /* USE_HAL_DAC_REGISTER_CALLBACKS */

}DAC_HandleTypeDef;

#if (USE_HAL_DAC_REGISTER_CALLBACKS == 1)
/**
  * @brief  HAL DAC Callback ID enumeration definition
  */
typedef enum
{
  HAL_DAC_CH1_COMPLETE_CB_ID                 = 0x00U,  /*!< DAC CH1 Complete Callback ID      */
  HAL_DAC_CH1_HALF_COMPLETE_CB_ID            = 0x01U,  /*!< DAC CH1 half Complete Callback ID */
  HAL_DAC_CH1_ERROR_ID                       = 0x02U,  /*!< DAC CH1 error Callback ID         */
  HAL_DAC_CH1_UNDERRUN_CB_ID                 = 0x03U,  /*!< DAC CH1 underrun Callback ID      */
  HAL_DAC_CH2_COMPLETE_CB_ID                 = 0x04U,  /*!< DAC CH2 Complete Callback ID      */
  HAL_DAC_CH2_HALF_COMPLETE_CB_ID            = 0x05U,  /*!< DAC CH2 half Complete Callback ID */
  HAL_DAC_CH2_ERROR_ID                       = 0x06U,  /*!< DAC CH2 error Callback ID         */
  HAL_DAC_CH2_UNDERRUN_CB_ID                 = 0x07U,  /*!< DAC CH2 underrun Callback ID      */
  HAL_DAC_MSP_INIT_CB_ID                     = 0x08U,  /*!< DAC MspInit Callback ID           */
  HAL_DAC_MSP_DEINIT_CB_ID                   = 0x09U,  /*!< DAC MspDeInit Callback ID         */
  HAL_DAC_ALL_CB_ID                          = 0x0AU   /*!< DAC All ID                        */
}HAL_DAC_CallbackIDTypeDef;                            
                                                       
/**
  * @brief  HAL DAC Callback pointer definition
  */
typedef void (*pDAC_CallbackTypeDef)(DAC_HandleTypeDef *hdac);
#endif /* USE_HAL_DAC_REGISTER_CALLBACKS */

/**
  * @}
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup DAC_Exported_Constants DAC Exported Constants
  * @{
  */

/** @defgroup DAC_Error_Code DAC Error Code
  * @{
  */
#define  HAL_DAC_ERROR_NONE              0x00U    /*!< No error                          */
#define  HAL_DAC_ERROR_DMAUNDERRUNCH1    0x01U    /*!< DAC channel1 DMA underrun error   */
#define  HAL_DAC_ERROR_DMAUNDERRUNCH2    0x02U    /*!< DAC channel2 DMA underrun error   */
#define  HAL_DAC_ERROR_DMA               0x04U    /*!< DMA error                         */
#if (USE_HAL_DAC_REGISTER_CALLBACKS == 1)
#define HAL_DAC_ERROR_INVALID_CALLBACK   0x10U    /*!< Invalid callback error            */
#endif /* USE_HAL_DAC_REGISTER_CALLBACKS */  
/**
  * @}
  */

/** @defgroup DAC_lfsrunmask_triangleamplitude DAC lfsrunmask triangleamplitude
  * @{
  */
#define DAC_LFSRUNMASK_BIT0                (0x00000000U) /*!< Unmask DAC channel LFSR bit0 for noise wave generation */
#define DAC_LFSRUNMASK_BITS1_0             ((uint32_t)DAC_CR_MAMP1_0) /*!< Unmask DAC channel LFSR bit[1:0] for noise wave generation */
#define DAC_LFSRUNMASK_BITS2_0             ((uint32_t)DAC_CR_MAMP1_1) /*!< Unmask DAC channel LFSR bit[2:0] for noise wave generation */
#define DAC_LFSRUNMASK_BITS3_0             ((uint32_t)DAC_CR_MAMP1_1 | DAC_CR_MAMP1_0)/*!< Unmask DAC channel LFSR bit[3:0] for noise wave generation */
#define DAC_LFSRUNMASK_BITS4_0             ((uint32_t)DAC_CR_MAMP1_2) /*!< Unmask DAC channel LFSR bit[4:0] for noise wave generation */
#define DAC_LFSRUNMASK_BITS5_0             ((uint32_t)DAC_CR_MAMP1_2 | DAC_CR_MAMP1_0) /*!< Unmask DAC channel LFSR bit[5:0] for noise wave generation */
#define DAC_LFSRUNMASK_BITS6_0             ((uint32_t)DAC_CR_MAMP1_2 | DAC_CR_MAMP1_1) /*!< Unmask DAC channel LFSR bit[6:0] for noise wave generation */
#define DAC_LFSRUNMASK_BITS7_0             ((uint32_t)DAC_CR_MAMP1_2 | DAC_CR_MAMP1_1 | DAC_CR_MAMP1_0) /*!< Unmask DAC channel LFSR bit[7:0] for noise wave generation */
#define DAC_LFSRUNMASK_BITS8_0             ((uint32_t)DAC_CR_MAMP1_3) /*!< Unmask DAC channel LFSR bit[8:0] for noise wave generation */
#define DAC_LFSRUNMASK_BITS9_0             ((uint32_t)DAC_CR_MAMP1_3 | DAC_CR_MAMP1_0) /*!< Unmask DAC channel LFSR bit[9:0] for noise wave generation */
#define DAC_LFSRUNMASK_BITS10_0            ((uint32_t)DAC_CR_MAMP1_3 | DAC_CR_MAMP1_1) /*!< Unmask DAC channel LFSR bit[10:0] for noise wave generation */
#define DAC_LFSRUNMASK_BITS11_0            ((uint32_t)DAC_CR_MAMP1_3 | DAC_CR_MAMP1_1 | DAC_CR_MAMP1_0) /*!< Unmask DAC channel LFSR bit[11:0] for noise wave generation */
#define DAC_TRIANGLEAMPLITUDE_1            (0x00000000U) /*!< Select max triangle amplitude of 1U */
#define DAC_TRIANGLEAMPLITUDE_3            ((uint32_t)DAC_CR_MAMP1_0) /*!< Select max triangle amplitude of 3U */
#define DAC_TRIANGLEAMPLITUDE_7            ((uint32_t)DAC_CR_MAMP1_1) /*!< Select max triangle amplitude of 7U */
#define DAC_TRIANGLEAMPLITUDE_15           ((uint32_t)DAC_CR_MAMP1_1 | DAC_CR_MAMP1_0) /*!< Select max triangle amplitude of 15U */
#define DAC_TRIANGLEAMPLITUDE_31           ((uint32_t)DAC_CR_MAMP1_2) /*!< Select max triangle amplitude of 31U */
#define DAC_TRIANGLEAMPLITUDE_63           ((uint32_t)DAC_CR_MAMP1_2 | DAC_CR_MAMP1_0) /*!< Select max triangle amplitude of 63U */
#define DAC_TRIANGLEAMPLITUDE_127          ((uint32_t)DAC_CR_MAMP1_2 | DAC_CR_MAMP1_1) /*!< Select max triangle amplitude of 127U */
#define DAC_TRIANGLEAMPLITUDE_255          ((uint32_t)DAC_CR_MAMP1_2 | DAC_CR_MAMP1_1 | DAC_CR_MAMP1_0) /*!< Select max triangle amplitude of 255U */
#define DAC_TRIANGLEAMPLITUDE_511          ((uint32_t)DAC_CR_MAMP1_3) /*!< Select max triangle amplitude of 511U */
#define DAC_TRIANGLEAMPLITUDE_1023         ((uint32_t)DAC_CR_MAMP1_3 | DAC_CR_MAMP1_0) /*!< Select max triangle amplitude of 1023U */
#define DAC_TRIANGLEAMPLITUDE_2047         ((uint32_t)DAC_CR_MAMP1_3 | DAC_CR_MAMP1_1) /*!< Select max triangle amplitude of 2047U */
#define DAC_TRIANGLEAMPLITUDE_4095         ((uint32_t)DAC_CR_MAMP1_3 | DAC_CR_MAMP1_1 | DAC_CR_MAMP1_0) /*!< Select max triangle amplitude of 4095U */

/**
  * @}
  */

/** @defgroup DAC_output_buffer DAC output buffer
  * @{
  */
#define DAC_OUTPUTBUFFER_ENABLE            (0x00000000U)
#define DAC_OUTPUTBUFFER_DISABLE           ((uint32_t)DAC_CR_BOFF1)

/**
  * @}
  */

#if defined(DAC_CR_OUTEN1)
/** @defgroup DAC_output_switch DAC output switch
  * @{
  */
#define DAC_OUTPUTSWITCH_DISABLE           (0x00000000U)
#define DAC_OUTPUTSWITCH_ENABLE            ((uint32_t)DAC_CR_OUTEN1)

/**
  * @}
  */

#endif /* DAC_CR_OUTEN1 */
/** @defgroup DAC_data_alignment DAC data alignment
  * @{
  */
#define DAC_ALIGN_12B_R                    (0x00000000U)
#define DAC_ALIGN_12B_L                    (0x00000004U)
#define DAC_ALIGN_8B_R                     (0x00000008U)

/**
  * @}
  */

/** @defgroup DAC_flags_definition DAC flags definition
  * @{
  */ 
#define DAC_FLAG_DMAUDR1                   ((uint32_t)DAC_SR_DMAUDR1)
#define DAC_FLAG_DMAUDR2                   ((uint32_t)DAC_SR_DMAUDR2)   
/**
  * @}
  */

/** @defgroup DAC_interrupts_definition DAC interrupts definition
  * @{
  */ 
#define DAC_IT_DMAUDR1                   ((uint32_t)DAC_CR_DMAUDRIE1)
#define DAC_IT_DMAUDR2                   ((uint32_t)DAC_CR_DMAUDRIE2) 

/**
  * @}
  */

/**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/

/** @defgroup DAC_Exported_Macros DAC Exported Macros
  * @{
  */

/** @brief Reset DAC handle state
  * @param  __HANDLE__ specifies the DAC handle.
  * @retval None
  */
#if (USE_HAL_DAC_REGISTER_CALLBACKS == 1)
#define __HAL_DAC_RESET_HANDLE_STATE(__HANDLE__)           do {                                              \
                                                                 (__HANDLE__)->State = HAL_DAC_STATE_RESET;  \
                                                                 (__HANDLE__)->MspInitCallback = NULL;       \
                                                                 (__HANDLE__)->MspDeInitCallback = NULL;     \
                                                               } while(0)
#else
#define __HAL_DAC_RESET_HANDLE_STATE(__HANDLE__) ((__HANDLE__)->State = HAL_DAC_STATE_RESET)
#endif /* USE_HAL_DAC_REGISTER_CALLBACKS */

/** @brief Enable the DAC channel
  * @param  __HANDLE__ specifies the DAC handle.
  * @param  __DAC_Channel__ specifies the DAC channel
  * @retval None
  */
#define __HAL_DAC_ENABLE(__HANDLE__, __DAC_Channel__) \
((__HANDLE__)->Instance->CR |=  (DAC_CR_EN1 << (__DAC_Channel__)))

/** @brief Disable the DAC channel
  * @param  __HANDLE__ specifies the DAC handle
  * @param  __DAC_Channel__ specifies the DAC channel.
  * @retval None
  */
#define __HAL_DAC_DISABLE(__HANDLE__, __DAC_Channel__) \
((__HANDLE__)->Instance->CR &=  ~(DAC_CR_EN1 << (__DAC_Channel__)))
 
/** @brief Set DHR12R1 alignment
  * @param  __ALIGNMENT__ specifies the DAC alignment
  * @retval None
  */
#define DAC_DHR12R1_ALIGNMENT(__ALIGNMENT__) ((0x00000008U) + (__ALIGNMENT__))

/** @brief  Set DHR12R2 alignment
  * @param  __ALIGNMENT__ specifies the DAC alignment
  * @retval None
  */
#define DAC_DHR12R2_ALIGNMENT(__ALIGNMENT__) ((0x00000014U) + (__ALIGNMENT__))

/** @brief  Set DHR12RD alignment
  * @param  __ALIGNMENT__ specifies the DAC alignment
  * @retval None
  */
#define DAC_DHR12RD_ALIGNMENT(__ALIGNMENT__) ((0x00000020U) + (__ALIGNMENT__))

/** @brief Enable the DAC interrupt
  * @param  __HANDLE__ specifies the DAC handle
  * @param  __INTERRUPT__ specifies the DAC interrupt.
  *          This parameter can be any combination of the following values:
  *            @arg DAC_IT_DMAUDR1: DAC channel 1 DMA underrun interrupt
  *            @arg DAC_IT_DMAUDR2: DAC channel 2 DMA underrun interrupt
  * @retval None
  */
#define __HAL_DAC_ENABLE_IT(__HANDLE__, __INTERRUPT__) (((__HANDLE__)->Instance->CR) |= (__INTERRUPT__))

/** @brief Disable the DAC interrupt
  * @param  __HANDLE__ specifies the DAC handle
  * @param  __INTERRUPT__ specifies the DAC interrupt.
  *          This parameter can be any combination of the following values:
  *            @arg DAC_IT_DMAUDR1: DAC channel 1 DMA underrun interrupt
  *            @arg DAC_IT_DMAUDR2: DAC channel 2 DMA underrun interrupt
  * @retval None
  */
#define __HAL_DAC_DISABLE_IT(__HANDLE__, __INTERRUPT__) (((__HANDLE__)->Instance->CR) &= ~(__INTERRUPT__))

/** @brief  Check whether the specified DAC interrupt source is enabled or not
  * @param __HANDLE__ DAC handle
  * @param __INTERRUPT__ DAC interrupt source to check
  *          This parameter can be any combination of the following values:
  *            @arg DAC_IT_DMAUDR1: DAC channel 1 DMA underrun interrupt
  *            @arg DAC_IT_DMAUDR2: DAC channel 2 DMA underrun interrupt
  * @retval State of interruption (SET or RESET)
  */
#define __HAL_DAC_GET_IT_SOURCE(__HANDLE__, __INTERRUPT__) (((__HANDLE__)->Instance->CR & (__INTERRUPT__)) == (__INTERRUPT__))

/** @brief  Get the selected DAC's flag status
  * @param  __HANDLE__ specifies the DAC handle.
  * @param  __FLAG__ specifies the DAC flag to get.
  *          This parameter can be any combination of the following values:
  *            @arg DAC_FLAG_DMAUDR1: DAC channel 1 DMA underrun flag
  *            @arg DAC_FLAG_DMAUDR2: DAC channel 2 DMA underrun flag
  * @retval None
  */
#define __HAL_DAC_GET_FLAG(__HANDLE__, __FLAG__) ((((__HANDLE__)->Instance->SR) & (__FLAG__)) == (__FLAG__))

/** @brief  Clear the DAC's flag
  * @param  __HANDLE__ specifies the DAC handle.
  * @param  __FLAG__ specifies the DAC flag to clear.
  *          This parameter can be any combination of the following values:
  *            @arg DAC_FLAG_DMAUDR1: DAC channel 1 DMA underrun flag
  *            @arg DAC_FLAG_DMAUDR2: DAC channel 2 DMA underrun flag
  * @retval None
  */
#define __HAL_DAC_CLEAR_FLAG(__HANDLE__, __FLAG__) (((__HANDLE__)->Instance->SR) = (__FLAG__))

/**
  * @}
  */

/* Private macro -------------------------------------------------------------*/

/** @addtogroup DAC_Private_Macros
  * @{
  */

#define IS_DAC_LFSR_UNMASK_TRIANGLE_AMPLITUDE(VALUE) (((VALUE) == DAC_LFSRUNMASK_BIT0) || \
                                                      ((VALUE) == DAC_LFSRUNMASK_BITS1_0) || \
                                                      ((VALUE) == DAC_LFSRUNMASK_BITS2_0) || \
                                                      ((VALUE) == DAC_LFSRUNMASK_BITS3_0) || \
                                                      ((VALUE) == DAC_LFSRUNMASK_BITS4_0) || \
                                                      ((VALUE) == DAC_LFSRUNMASK_BITS5_0) || \
                                                      ((VALUE) == DAC_LFSRUNMASK_BITS6_0) || \
                                                      ((VALUE) == DAC_LFSRUNMASK_BITS7_0) || \
                                                      ((VALUE) == DAC_LFSRUNMASK_BITS8_0) || \
                                                      ((VALUE) == DAC_LFSRUNMASK_BITS9_0) || \
                                                      ((VALUE) == DAC_LFSRUNMASK_BITS10_0) || \
                                                      ((VALUE) == DAC_LFSRUNMASK_BITS11_0) || \
                                                      ((VALUE) == DAC_TRIANGLEAMPLITUDE_1) || \
                                                      ((VALUE) == DAC_TRIANGLEAMPLITUDE_3) || \
                                                      ((VALUE) == DAC_TRIANGLEAMPLITUDE_7) || \
                                                      ((VALUE) == DAC_TRIANGLEAMPLITUDE_15) || \
                                                      ((VALUE) == DAC_TRIANGLEAMPLITUDE_31) || \
                                                      ((VALUE) == DAC_TRIANGLEAMPLITUDE_63) || \
                                                      ((VALUE) == DAC_TRIANGLEAMPLITUDE_127) || \
                                                      ((VALUE) == DAC_TRIANGLEAMPLITUDE_255) || \
                                                      ((VALUE) == DAC_TRIANGLEAMPLITUDE_511) || \
                                                      ((VALUE) == DAC_TRIANGLEAMPLITUDE_1023) || \
                                                      ((VALUE) == DAC_TRIANGLEAMPLITUDE_2047) || \
                                                      ((VALUE) == DAC_TRIANGLEAMPLITUDE_4095))

#define IS_DAC_OUTPUT_BUFFER_STATE(STATE) (((STATE) == DAC_OUTPUTBUFFER_ENABLE) || \
                                           ((STATE) == DAC_OUTPUTBUFFER_DISABLE))

#if defined(DAC_CR_OUTEN1)
#define IS_DAC_OUTPUT_SWITCH_STATE(STATE) (((STATE) == DAC_OUTPUTSWITCH_DISABLE) || \
                                           ((STATE) == DAC_OUTPUTSWITCH_ENABLE))

#endif /* DAC_CR_OUTEN1 */
#define IS_DAC_ALIGN(ALIGN) (((ALIGN) == DAC_ALIGN_12B_R) || \
                             ((ALIGN) == DAC_ALIGN_12B_L) || \
                             ((ALIGN) == DAC_ALIGN_8B_R))

#define IS_DAC_DATA(DATA) ((DATA) <= 0xFFF0U) 



/**
  * @}
  */


/* Include DAC HAL Extended module */
#include "stm32f3xx_hal_dac_ex.h" 

/* Exported functions --------------------------------------------------------*/  

/** @addtogroup DAC_Exported_Functions
  * @{
  */

/** @addtogroup DAC_Exported_Functions_Group1
  * @{
  */
/* Initialization and de-initialization functions *****************************/ 
HAL_StatusTypeDef HAL_DAC_Init(DAC_HandleTypeDef* hdac);
HAL_StatusTypeDef HAL_DAC_DeInit(DAC_HandleTypeDef* hdac);
void HAL_DAC_MspInit(DAC_HandleTypeDef* hdac);
void HAL_DAC_MspDeInit(DAC_HandleTypeDef* hdac);

/**
  * @}
  */

/** @addtogroup DAC_Exported_Functions_Group2
  * @{
  */
/* IO operation functions *****************************************************/
HAL_StatusTypeDef HAL_DAC_Start(DAC_HandleTypeDef* hdac, uint32_t Channel);
HAL_StatusTypeDef HAL_DAC_Stop(DAC_HandleTypeDef* hdac, uint32_t Channel);
HAL_StatusTypeDef HAL_DAC_Start_DMA(DAC_HandleTypeDef* hdac, uint32_t Channel, uint32_t* pData, uint32_t Length, uint32_t Alignment);
HAL_StatusTypeDef HAL_DAC_Stop_DMA(DAC_HandleTypeDef* hdac, uint32_t Channel);
uint32_t HAL_DAC_GetValue(DAC_HandleTypeDef* hdac, uint32_t Channel);
HAL_StatusTypeDef HAL_DAC_ConfigChannel(DAC_HandleTypeDef* hdac, DAC_ChannelConfTypeDef* sConfig, uint32_t Channel);

void HAL_DAC_IRQHandler(DAC_HandleTypeDef* hdac);
void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef* hdac);
void HAL_DAC_ConvHalfCpltCallbackCh1(DAC_HandleTypeDef* hdac);
void HAL_DAC_ErrorCallbackCh1(DAC_HandleTypeDef *hdac);
void HAL_DAC_DMAUnderrunCallbackCh1(DAC_HandleTypeDef *hdac);

#if (USE_HAL_DAC_REGISTER_CALLBACKS == 1)
/* DAC callback registering/unregistering */
HAL_StatusTypeDef     HAL_DAC_RegisterCallback (DAC_HandleTypeDef *hdac, HAL_DAC_CallbackIDTypeDef CallbackId, pDAC_CallbackTypeDef pCallback);
HAL_StatusTypeDef     HAL_DAC_UnRegisterCallback (DAC_HandleTypeDef *hdac, HAL_DAC_CallbackIDTypeDef CallbackId);
#endif /* USE_HAL_DAC_REGISTER_CALLBACKS */

/**
  * @}
  */
  
/** @addtogroup DAC_Exported_Functions_Group3
  * @{
  */
/* Peripheral Control functions ***********************************************/
HAL_StatusTypeDef HAL_DAC_SetValue(DAC_HandleTypeDef* hdac, uint32_t Channel, uint32_t Alignment, uint32_t Data);

/**
  * @}
  */

/** @addtogroup DAC_Exported_Functions_Group4
  * @{
  */
/* Peripheral State and Error functions ***************************************/
HAL_DAC_StateTypeDef HAL_DAC_GetState(DAC_HandleTypeDef* hdac);
uint32_t HAL_DAC_GetError(DAC_HandleTypeDef *hdac);

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */
  
#ifdef __cplusplus
}
#endif

#endif /* STM32F3xx_HAL_DAC_H */

>>>>>>> b334a05 (Week 3 Tasks)
