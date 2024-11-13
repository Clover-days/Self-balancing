/*********************************************************************************************************//**
 * @file    bmduino_board.h
 * @version $Rev:: 7246         $
 * @date    $Date:: 2023-09-25 #$
 * @brief   Pin assignment of BMduino.
 *************************************************************************************************************
 * @attention
 *
 * Firmware Disclaimer Information
 *
 * 1. The customer hereby acknowledges and agrees that the program technical documentation, including the
 *    code, which is supplied by Holtek Semiconductor Inc., (hereinafter referred to as "HOLTEK") is the
 *    proprietary and confidential intellectual property of HOLTEK, and is protected by copyright law and
 *    other intellectual property laws.
 *
 * 2. The customer hereby acknowledges and agrees that the program technical documentation, including the
 *    code, is confidential information belonging to HOLTEK, and must not be disclosed to any third parties
 *    other than HOLTEK and the customer.
 *
 * 3. The program technical documentation, including the code, is provided "as is" and for customer reference
 *    only. After delivery by HOLTEK, the customer shall use the program technical documentation, including
 *    the code, at their own risk. HOLTEK disclaims any expressed, implied or statutory warranties, including
 *    the warranties of merchantability, satisfactory quality and fitness for a particular purpose.
 *
 * <h2><center>Copyright (C) Holtek Semiconductor Inc. All rights reserved</center></h2>
 ************************************************************************************************************/
/* Define to prevent recursive inclusion -------------------------------------------------------------------*/
#ifndef __BMDUINO_BOARD_H
#define __BMDUINO_BOARD_H

/* Settings ------------------------------------------------------------------------------------------------*/
#if defined(USE_BM53A367A_DVB)

/*----------------------------------------------------------------------------------------------------------*/
/* UART Module Setting                                                                                      */
/*----------------------------------------------------------------------------------------------------------*/
#if (BMDUINO_SERIAL == 0) // Serial
  #define HTCFG_UARTM_CH0                         USART0
  #define HTCFG_UARTM0_TX_GPIO_PORT               A
  #define HTCFG_UARTM0_TX_GPIO_PIN                8
  #define HTCFG_UARTM0_RX_GPIO_PORT               A
  #define HTCFG_UARTM0_RX_GPIO_PIN                10
  #define HTCFG_UARTM0_TX_BUFFER_SIZE             128
  #define HTCFG_UARTM0_RX_BUFFER_SIZE             128
#endif
#if (BMDUINO_SERIAL == 1) // Serial1
  #define HTCFG_UARTM_CH0                         UART1
  #define HTCFG_UARTM0_TX_GPIO_PORT               C
  #define HTCFG_UARTM0_TX_GPIO_PIN                12
  #define HTCFG_UARTM0_RX_GPIO_PORT               C
  #define HTCFG_UARTM0_RX_GPIO_PIN                13
  #define HTCFG_UARTM0_TX_BUFFER_SIZE             128
  #define HTCFG_UARTM0_RX_BUFFER_SIZE             128
#endif
#if (BMDUINO_SERIAL == 2) // Serial2
  #define HTCFG_UARTM_CH0                         UART3
  #define HTCFG_UARTM0_TX_GPIO_PORT               C
  #define HTCFG_UARTM0_TX_GPIO_PIN                14
  #define HTCFG_UARTM0_RX_GPIO_PORT               C
  #define HTCFG_UARTM0_RX_GPIO_PIN                15
  #define HTCFG_UARTM0_TX_BUFFER_SIZE             128
  #define HTCFG_UARTM0_RX_BUFFER_SIZE             128
#endif
#if (BMDUINO_SERIAL == 3) // Serial3
  #define HTCFG_UARTM_CH0                         USART1
  #define HTCFG_UARTM0_TX_GPIO_PORT               A
  #define HTCFG_UARTM0_TX_GPIO_PIN                4
  #define HTCFG_UARTM0_RX_GPIO_PORT               A
  #define HTCFG_UARTM0_RX_GPIO_PIN                5
  #define HTCFG_UARTM0_TX_BUFFER_SIZE             128
  #define HTCFG_UARTM0_RX_BUFFER_SIZE             128
#endif
#if (BMDUINO_SERIAL == 4) // Serial4
  #define HTCFG_UARTM_CH0                         UART2
  #define HTCFG_UARTM0_TX_GPIO_PORT               B
  #define HTCFG_UARTM0_TX_GPIO_PIN                2
  #define HTCFG_UARTM0_RX_GPIO_PORT               B
  #define HTCFG_UARTM0_RX_GPIO_PIN                3
  #define HTCFG_UARTM0_TX_BUFFER_SIZE             128
  #define HTCFG_UARTM0_RX_BUFFER_SIZE             128
#endif

/*----------------------------------------------------------------------------------------------------------*/
/* SPI Module Setting                                                                                       */
/*----------------------------------------------------------------------------------------------------------*/
#if (BMDUINO_SPI == 0) // SPI
  #define HTCFG_CH0_SCK_GPIO_PORT                 C
  #define HTCFG_CH0_SCK_GPIO_PIN                  11
  #define HTCFG_CH0_SEL_GPIO_PORT                 C
  #define HTCFG_CH0_SEL_GPIO_PIN                  10
  #define HTCFG_CH0_MOSI_GPIO_PORT                C
  #define HTCFG_CH0_MOSI_GPIO_PIN                 8
  #define HTCFG_CH0_MISO_GPIO_PORT                C
  #define HTCFG_CH0_MISO_GPIO_PIN                 9
  #define HTCFG_SPI_CH0                           SPI1
#endif
#if (BMDUINO_SPI == 1) // SPI1
  #define HTCFG_CH0_SCK_GPIO_PORT                 A
  #define HTCFG_CH0_SCK_GPIO_PIN                  4
  #define HTCFG_CH0_SEL_GPIO_PORT                 A
  #define HTCFG_CH0_SEL_GPIO_PIN                  7
  #define HTCFG_CH0_MOSI_GPIO_PORT                A
  #define HTCFG_CH0_MOSI_GPIO_PIN                 5
  #define HTCFG_CH0_MISO_GPIO_PORT                A
  #define HTCFG_CH0_MISO_GPIO_PIN                 11
  #define HTCFG_SPI_CH0                           SPI0
#endif
#if (BMDUINO_SPI == 2) // SPI2
  #define HTCFG_CH0_SCK_GPIO_PORT                 B
  #define HTCFG_CH0_SCK_GPIO_PIN                  3
  #define HTCFG_CH0_SEL_GPIO_PORT                 B
  #define HTCFG_CH0_SEL_GPIO_PIN                  2
  #define HTCFG_CH0_MOSI_GPIO_PORT                B
  #define HTCFG_CH0_MOSI_GPIO_PIN                 4
  #define HTCFG_CH0_MISO_GPIO_PORT                B
  #define HTCFG_CH0_MISO_GPIO_PIN                 5
  #define HTCFG_SPI_CH0                           QSPI
#endif

/*----------------------------------------------------------------------------------------------------------*/
/* I2C Master Setting                                                                                       */
/*----------------------------------------------------------------------------------------------------------*/
#if (BMDUINO_WIRE == 0) // Wire
  #define HTCFG_I2CM_CH0                          I2C1
  #define HTCFG_M0_SCL_GPIO_PORT                  C
  #define HTCFG_M0_SCL_GPIO_PIN                   4
  #define HTCFG_M0_SDA_GPIO_PORT                  C
  #define HTCFG_M0_SDA_GPIO_PIN                   5
#endif
#if (BMDUINO_WIRE == 1) // Wire1
  #define HTCFG_I2CM_CH0                          I2C0
  #define HTCFG_M0_SCL_GPIO_PORT                  C
  #define HTCFG_M0_SCL_GPIO_PIN                   12
  #define HTCFG_M0_SDA_GPIO_PORT                  C
  #define HTCFG_M0_SDA_GPIO_PIN                   13
#endif
#if (BMDUINO_WIRE == 2) // Wire2
  #define HTCFG_I2CM_CH0                          I2C0
  #define HTCFG_M0_SCL_GPIO_PORT                  C
  #define HTCFG_M0_SCL_GPIO_PIN                   14
  #define HTCFG_M0_SDA_GPIO_PORT                  C
  #define HTCFG_M0_SDA_GPIO_PIN                   15
#endif

#endif

#endif
