<<<<<<< HEAD
/**
  ******************************************************************************
  * @file    stm32f3xx_ll_usb.h
  * @author  MCD Application Team
  * @brief   Header file of USB Low Layer HAL module.
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
#ifndef STM32F3xx_LL_USB_H
#define STM32F3xx_LL_USB_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal_def.h"

#if defined (USB)
/** @addtogroup STM32F3xx_HAL_Driver
  * @{
  */

/** @addtogroup USB_LL
  * @{
  */

/* Exported types ------------------------------------------------------------*/

/**
  * @brief  USB Mode definition
  */

typedef enum
{
  USB_DEVICE_MODE = 0
} USB_ModeTypeDef;


/**
  * @brief  USB Instance Initialization Structure definition
  */
typedef struct
{
  uint8_t dev_endpoints;            /*!< Device Endpoints number.
                                         This parameter depends on the used USB core.
                                         This parameter must be a number between Min_Data = 1 and Max_Data = 15 */

  uint8_t speed;                   /*!< USB Core speed.
                                        This parameter can be any value of @ref PCD_Speed/HCD_Speed
                                                                                (HCD_SPEED_xxx, HCD_SPEED_xxx) */

  uint8_t ep0_mps;                 /*!< Set the Endpoint 0 Max Packet size.                                    */

  uint8_t phy_itface;              /*!< Select the used PHY interface.
                                        This parameter can be any value of @ref PCD_PHY_Module/HCD_PHY_Module  */

  uint8_t Sof_enable;              /*!< Enable or disable the output of the SOF signal.                        */

  uint8_t low_power_enable;        /*!< Enable or disable the low Power Mode.                                  */

  uint8_t lpm_enable;              /*!< Enable or disable Link Power Management.                               */

  uint8_t battery_charging_enable; /*!< Enable or disable Battery charging.                                    */
} USB_CfgTypeDef;

typedef struct
{
  uint8_t   num;                  /*!< Endpoint number
                                       This parameter must be a number between Min_Data = 1 and Max_Data = 15   */

  uint8_t   is_in;                /*!< Endpoint direction
                                       This parameter must be a number between Min_Data = 0 and Max_Data = 1    */

  uint8_t   is_stall;             /*!< Endpoint stall condition
                                       This parameter must be a number between Min_Data = 0 and Max_Data = 1    */

  uint8_t   type;                 /*!< Endpoint type
                                       This parameter can be any value of @ref USB_LL_EP_Type                   */

  uint8_t   data_pid_start;       /*!< Initial data PID
                                       This parameter must be a number between Min_Data = 0 and Max_Data = 1    */


  uint16_t  pmaadress;            /*!< PMA Address
                                       This parameter can be any value between Min_addr = 0 and Max_addr = 1K   */

  uint16_t  pmaaddr0;             /*!< PMA Address0
                                       This parameter can be any value between Min_addr = 0 and Max_addr = 1K   */

  uint16_t  pmaaddr1;             /*!< PMA Address1
                                       This parameter can be any value between Min_addr = 0 and Max_addr = 1K   */

  uint8_t   doublebuffer;         /*!< Double buffer enable
                                       This parameter can be 0 or 1                                             */


  uint32_t  maxpacket;            /*!< Endpoint Max packet size
                                       This parameter must be a number between Min_Data = 0 and Max_Data = 64KB */

  uint8_t   *xfer_buff;           /*!< Pointer to transfer buffer                                               */

  uint32_t  xfer_len;             /*!< Current transfer length                                                  */

  uint32_t  xfer_count;           /*!< Partial transfer length in case of multi packet transfer                 */

  uint32_t  xfer_len_db;          /*!< double buffer transfer length used with bulk double buffer in            */

  uint8_t   xfer_fill_db;         /*!< double buffer Need to Fill new buffer  used with bulk_in                 */
} USB_EPTypeDef;

/* Exported constants --------------------------------------------------------*/

/** @defgroup PCD_Exported_Constants PCD Exported Constants
  * @{
  */
/** @defgroup USB_LL_EP0_MPS USB Low Layer EP0 MPS
  * @{
  */
#define EP_MPS_64                              0U
#define EP_MPS_32                              1U
#define EP_MPS_16                              2U
#define EP_MPS_8                               3U
/**
  * @}
  */

/** @defgroup USB_LL_EP_Type USB Low Layer EP Type
  * @{
  */
#define EP_TYPE_CTRL                           0U
#define EP_TYPE_ISOC                           1U
#define EP_TYPE_BULK                           2U
#define EP_TYPE_INTR                           3U
#define EP_TYPE_MSK                            3U
/**
  * @}
  */

/** @defgroup USB_LL Device Speed
  * @{
  */
#define USBD_FS_SPEED                          2U
/**
  * @}
  */


#define BTABLE_ADDRESS                         0x000U
#if defined(STM32F303xC) || defined(STM32F303x8) || defined(STM32F334x8) || defined(STM32F301x8) || defined(STM32F373xC) || defined(STM32F378xx) || defined(STM32F302xC)
#define PMA_ACCESS                             2U
#endif /* STM32F303xC                || */
/* STM32F303x8 || STM32F334x8 || */
/* STM32F301x8                || */
/* STM32F373xC || STM32F378xx    */

#if defined(STM32F302xE) || defined(STM32F303xE) || defined(STM32F302x8)
#define PMA_ACCESS                             1U
#endif /* STM32F302xE || STM32F303xE || */
/* STM32F302xC                || */
/* STM32F302x8                   */

#ifndef USB_EP_RX_STRX
#define USB_EP_RX_STRX                         (0x3U << 12)
#endif /* USB_EP_RX_STRX */

#define EP_ADDR_MSK                            0x7U

#ifndef USE_USB_DOUBLE_BUFFER
#define USE_USB_DOUBLE_BUFFER                  1U
#endif /* USE_USB_DOUBLE_BUFFER */


/**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/
/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup USB_LL_Exported_Functions USB Low Layer Exported Functions
  * @{
  */


HAL_StatusTypeDef USB_CoreInit(USB_TypeDef *USBx, USB_CfgTypeDef cfg);
HAL_StatusTypeDef USB_DevInit(USB_TypeDef *USBx, USB_CfgTypeDef cfg);
HAL_StatusTypeDef USB_EnableGlobalInt(USB_TypeDef *USBx);
HAL_StatusTypeDef USB_DisableGlobalInt(USB_TypeDef *USBx);
HAL_StatusTypeDef USB_SetCurrentMode(USB_TypeDef *USBx, USB_ModeTypeDef mode);

HAL_StatusTypeDef USB_FlushRxFifo(USB_TypeDef const *USBx);
HAL_StatusTypeDef USB_FlushTxFifo(USB_TypeDef const *USBx, uint32_t num);

#if defined (HAL_PCD_MODULE_ENABLED)
HAL_StatusTypeDef USB_ActivateEndpoint(USB_TypeDef *USBx, USB_EPTypeDef *ep);
HAL_StatusTypeDef USB_DeactivateEndpoint(USB_TypeDef *USBx, USB_EPTypeDef *ep);
HAL_StatusTypeDef USB_EPStartXfer(USB_TypeDef *USBx, USB_EPTypeDef *ep);
HAL_StatusTypeDef USB_EPSetStall(USB_TypeDef *USBx, USB_EPTypeDef *ep);
HAL_StatusTypeDef USB_EPClearStall(USB_TypeDef *USBx, USB_EPTypeDef *ep);
HAL_StatusTypeDef USB_EPStopXfer(USB_TypeDef *USBx, USB_EPTypeDef *ep);
#endif /* defined (HAL_PCD_MODULE_ENABLED) */

HAL_StatusTypeDef USB_SetDevAddress(USB_TypeDef *USBx, uint8_t address);
HAL_StatusTypeDef USB_DevConnect(USB_TypeDef *USBx);
HAL_StatusTypeDef USB_DevDisconnect(USB_TypeDef *USBx);
HAL_StatusTypeDef USB_StopDevice(USB_TypeDef *USBx);
uint32_t          USB_ReadInterrupts(USB_TypeDef const *USBx);
HAL_StatusTypeDef USB_ActivateRemoteWakeup(USB_TypeDef *USBx);
HAL_StatusTypeDef USB_DeActivateRemoteWakeup(USB_TypeDef *USBx);

void              USB_WritePMA(USB_TypeDef const *USBx, uint8_t *pbUsrBuf,
                               uint16_t wPMABufAddr, uint16_t wNBytes);

void              USB_ReadPMA(USB_TypeDef const *USBx, uint8_t *pbUsrBuf,
                              uint16_t wPMABufAddr, uint16_t wNBytes);

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
#endif /* defined (USB) */

#ifdef __cplusplus
}
#endif /* __cplusplus */


#endif /* STM32F3xx_LL_USB_H */
=======
/**
  ******************************************************************************
  * @file    stm32f3xx_ll_usb.h
  * @author  MCD Application Team
  * @brief   Header file of USB Low Layer HAL module.
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
#ifndef STM32F3xx_LL_USB_H
#define STM32F3xx_LL_USB_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal_def.h"

#if defined (USB)
/** @addtogroup STM32F3xx_HAL_Driver
  * @{
  */

/** @addtogroup USB_LL
  * @{
  */

/* Exported types ------------------------------------------------------------*/

/**
  * @brief  USB Mode definition
  */

typedef enum
{
  USB_DEVICE_MODE = 0
} USB_ModeTypeDef;


/**
  * @brief  USB Instance Initialization Structure definition
  */
typedef struct
{
  uint8_t dev_endpoints;            /*!< Device Endpoints number.
                                         This parameter depends on the used USB core.
                                         This parameter must be a number between Min_Data = 1 and Max_Data = 15 */

  uint8_t speed;                   /*!< USB Core speed.
                                        This parameter can be any value of @ref PCD_Speed/HCD_Speed
                                                                                (HCD_SPEED_xxx, HCD_SPEED_xxx) */

  uint8_t ep0_mps;                 /*!< Set the Endpoint 0 Max Packet size.                                    */

  uint8_t phy_itface;              /*!< Select the used PHY interface.
                                        This parameter can be any value of @ref PCD_PHY_Module/HCD_PHY_Module  */

  uint8_t Sof_enable;              /*!< Enable or disable the output of the SOF signal.                        */

  uint8_t low_power_enable;        /*!< Enable or disable the low Power Mode.                                  */

  uint8_t lpm_enable;              /*!< Enable or disable Link Power Management.                               */

  uint8_t battery_charging_enable; /*!< Enable or disable Battery charging.                                    */
} USB_CfgTypeDef;

typedef struct
{
  uint8_t   num;                  /*!< Endpoint number
                                       This parameter must be a number between Min_Data = 1 and Max_Data = 15   */

  uint8_t   is_in;                /*!< Endpoint direction
                                       This parameter must be a number between Min_Data = 0 and Max_Data = 1    */

  uint8_t   is_stall;             /*!< Endpoint stall condition
                                       This parameter must be a number between Min_Data = 0 and Max_Data = 1    */

  uint8_t   type;                 /*!< Endpoint type
                                       This parameter can be any value of @ref USB_LL_EP_Type                   */

  uint8_t   data_pid_start;       /*!< Initial data PID
                                       This parameter must be a number between Min_Data = 0 and Max_Data = 1    */


  uint16_t  pmaadress;            /*!< PMA Address
                                       This parameter can be any value between Min_addr = 0 and Max_addr = 1K   */

  uint16_t  pmaaddr0;             /*!< PMA Address0
                                       This parameter can be any value between Min_addr = 0 and Max_addr = 1K   */

  uint16_t  pmaaddr1;             /*!< PMA Address1
                                       This parameter can be any value between Min_addr = 0 and Max_addr = 1K   */

  uint8_t   doublebuffer;         /*!< Double buffer enable
                                       This parameter can be 0 or 1                                             */


  uint32_t  maxpacket;            /*!< Endpoint Max packet size
                                       This parameter must be a number between Min_Data = 0 and Max_Data = 64KB */

  uint8_t   *xfer_buff;           /*!< Pointer to transfer buffer                                               */

  uint32_t  xfer_len;             /*!< Current transfer length                                                  */

  uint32_t  xfer_count;           /*!< Partial transfer length in case of multi packet transfer                 */

  uint32_t  xfer_len_db;          /*!< double buffer transfer length used with bulk double buffer in            */

  uint8_t   xfer_fill_db;         /*!< double buffer Need to Fill new buffer  used with bulk_in                 */
} USB_EPTypeDef;

/* Exported constants --------------------------------------------------------*/

/** @defgroup PCD_Exported_Constants PCD Exported Constants
  * @{
  */
/** @defgroup USB_LL_EP0_MPS USB Low Layer EP0 MPS
  * @{
  */
#define EP_MPS_64                              0U
#define EP_MPS_32                              1U
#define EP_MPS_16                              2U
#define EP_MPS_8                               3U
/**
  * @}
  */

/** @defgroup USB_LL_EP_Type USB Low Layer EP Type
  * @{
  */
#define EP_TYPE_CTRL                           0U
#define EP_TYPE_ISOC                           1U
#define EP_TYPE_BULK                           2U
#define EP_TYPE_INTR                           3U
#define EP_TYPE_MSK                            3U
/**
  * @}
  */

/** @defgroup USB_LL Device Speed
  * @{
  */
#define USBD_FS_SPEED                          2U
/**
  * @}
  */


#define BTABLE_ADDRESS                         0x000U
#if defined(STM32F303xC) || defined(STM32F303x8) || defined(STM32F334x8) || defined(STM32F301x8) || defined(STM32F373xC) || defined(STM32F378xx) || defined(STM32F302xC)
#define PMA_ACCESS                             2U
#endif /* STM32F303xC                || */
/* STM32F303x8 || STM32F334x8 || */
/* STM32F301x8                || */
/* STM32F373xC || STM32F378xx    */

#if defined(STM32F302xE) || defined(STM32F303xE) || defined(STM32F302x8)
#define PMA_ACCESS                             1U
#endif /* STM32F302xE || STM32F303xE || */
/* STM32F302xC                || */
/* STM32F302x8                   */

#ifndef USB_EP_RX_STRX
#define USB_EP_RX_STRX                         (0x3U << 12)
#endif /* USB_EP_RX_STRX */

#define EP_ADDR_MSK                            0x7U

#ifndef USE_USB_DOUBLE_BUFFER
#define USE_USB_DOUBLE_BUFFER                  1U
#endif /* USE_USB_DOUBLE_BUFFER */


/**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/
/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup USB_LL_Exported_Functions USB Low Layer Exported Functions
  * @{
  */


HAL_StatusTypeDef USB_CoreInit(USB_TypeDef *USBx, USB_CfgTypeDef cfg);
HAL_StatusTypeDef USB_DevInit(USB_TypeDef *USBx, USB_CfgTypeDef cfg);
HAL_StatusTypeDef USB_EnableGlobalInt(USB_TypeDef *USBx);
HAL_StatusTypeDef USB_DisableGlobalInt(USB_TypeDef *USBx);
HAL_StatusTypeDef USB_SetCurrentMode(USB_TypeDef *USBx, USB_ModeTypeDef mode);

HAL_StatusTypeDef USB_FlushRxFifo(USB_TypeDef const *USBx);
HAL_StatusTypeDef USB_FlushTxFifo(USB_TypeDef const *USBx, uint32_t num);

#if defined (HAL_PCD_MODULE_ENABLED)
HAL_StatusTypeDef USB_ActivateEndpoint(USB_TypeDef *USBx, USB_EPTypeDef *ep);
HAL_StatusTypeDef USB_DeactivateEndpoint(USB_TypeDef *USBx, USB_EPTypeDef *ep);
HAL_StatusTypeDef USB_EPStartXfer(USB_TypeDef *USBx, USB_EPTypeDef *ep);
HAL_StatusTypeDef USB_EPSetStall(USB_TypeDef *USBx, USB_EPTypeDef *ep);
HAL_StatusTypeDef USB_EPClearStall(USB_TypeDef *USBx, USB_EPTypeDef *ep);
HAL_StatusTypeDef USB_EPStopXfer(USB_TypeDef *USBx, USB_EPTypeDef *ep);
#endif /* defined (HAL_PCD_MODULE_ENABLED) */

HAL_StatusTypeDef USB_SetDevAddress(USB_TypeDef *USBx, uint8_t address);
HAL_StatusTypeDef USB_DevConnect(USB_TypeDef *USBx);
HAL_StatusTypeDef USB_DevDisconnect(USB_TypeDef *USBx);
HAL_StatusTypeDef USB_StopDevice(USB_TypeDef *USBx);
uint32_t          USB_ReadInterrupts(USB_TypeDef const *USBx);
HAL_StatusTypeDef USB_ActivateRemoteWakeup(USB_TypeDef *USBx);
HAL_StatusTypeDef USB_DeActivateRemoteWakeup(USB_TypeDef *USBx);

void              USB_WritePMA(USB_TypeDef const *USBx, uint8_t *pbUsrBuf,
                               uint16_t wPMABufAddr, uint16_t wNBytes);

void              USB_ReadPMA(USB_TypeDef const *USBx, uint8_t *pbUsrBuf,
                              uint16_t wPMABufAddr, uint16_t wNBytes);

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
#endif /* defined (USB) */

#ifdef __cplusplus
}
#endif /* __cplusplus */


#endif /* STM32F3xx_LL_USB_H */
>>>>>>> b334a05 (Week 3 Tasks)
