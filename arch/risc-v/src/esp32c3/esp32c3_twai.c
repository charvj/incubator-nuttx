/****************************************************************************
 * arch/risc-v/src/esp32c3/esp32c3_twai.c
 *
 *   Copyright (C) 2022 Jan Charvat. All rights reserved.
 *   Authors:
 *     Jan Charvat <jancharvat.charvat@gmail.com>
 *   History:
 *     2022-02-2: Initial version (Jan Charvat)
 *
 * This file is a part of NuttX:
 *
 *   Copyright (C) 2010 Gregory Nutt. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <sys/types.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>
//#include <string.h>

#include <arch/board/board.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/can/can.h>
//#include <nuttx/kmalloc.h>

//#include "arm_internal.h"
#include "riscv_arch.h"

//#include "chip.h"
#include "hardware/esp32c3_system.h"
#include "hardware/esp32c3_gpio_sigmap.h"
//#include "hardware/esp32c3_syscon.h"
//#include "hardware/esp32c3_soc.h"
//#include "hardware/esp32c3_twai.h"
#include "esp32c3_gpio.h"
#include "esp32c3_twai.h"
#include "esp32c3_irq.h"

#if defined(CONFIG_ESP32C3_TWAI)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

//Default values written to various registers on initialization
#define TWAI_INIT_TEC    0
#define TWAI_INIT_REC    0
#define TWAI_INIT_EWL    96
#define TWAI_MSG_LENGTH  8
#define TWAI_ID11_MASK   (0x7ff)   /* Bits 0-10: 11-bit Identifier (FF=0) */
                                               /* Bits 11-31: Reserved */
#define TWAI_ID29_MASK   (0x1fffffff) /* Bits 0-28: 29-bit Identifiter (FF=1) */
                                                  /* Bits 29-31: Reserved */
#define TWAI_DEFAULT_INTERRUPTS   0xE7        //Exclude data overrun (bit[3]) and brp_div (bit[4])

/* Debug ********************************************************************/

/* Non-standard debug that may be enabled just for testing TWAI */

#ifndef CONFIG_DEBUG_CAN_INFO
#  undef CONFIG_ESP32C3_TWAI_REGDEBUG
#endif


/****************************************************************************
 * Private Types
 ****************************************************************************/

struct up_dev_s
{
  uint8_t port;   /* TWAI port number */
  uint8_t clkdiv; /* CLKDIV register */
  uint32_t baud;  /* Configured baud */
  uint32_t base;  /* TWAI register base address */
  uint8_t irq;    /* IRQ associated with this TWAI */
  uint8_t cpuint;
  uint8_t periph;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* TWAI Register access */

#ifdef CONFIG_ESP32C3_TWAI_REGDEBUG
static void twai_printreg(uint32_t addr, uint32_t value);
#endif

static uint32_t twai_getreg(struct up_dev_s *priv, int offset);
static void twai_putreg(struct up_dev_s *priv, int offset, uint32_t value);

#ifdef CONFIG_ESP32C3_TWAI_REGDEBUG
static uint32_t twai_getcommon(uint32_t addr);
static void twai_putcommon(uint32_t addr, uint32_t value);
#else
#  define twai_getcommon(addr)        getreg32(addr)
#  define twai_putcommon(addr, value) putreg32(value, addr)
#endif

/* TWAI methods */

static void esp32c3twai_reset(FAR struct can_dev_s *dev);
static int  esp32c3twai_setup(FAR struct can_dev_s *dev);
static void esp32c3twai_shutdown(FAR struct can_dev_s *dev);
static void esp32c3twai_rxint(FAR struct can_dev_s *dev, bool enable);
static void esp32c3twai_txint(FAR struct can_dev_s *dev, bool enable);
static int  esp32c3twai_ioctl(FAR struct can_dev_s *dev, int cmd,
                           unsigned long arg);
static int  esp32c3twai_remoterequest(FAR struct can_dev_s *dev, uint16_t id);
static int esp32c3twai_send(FAR struct can_dev_s *dev, FAR struct can_msg_s *msg);
static bool esp32c3twai_txready(FAR struct can_dev_s *dev);
static bool esp32c3twai_txempty(FAR struct can_dev_s *dev);

/* TWAI interrupts */

static int esp32c3twai_interrupt(int irq, void *context, FAR void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct can_ops_s g_twaiops =
{
  .co_reset         = esp32c3twai_reset,
  .co_setup         = esp32c3twai_setup,
  .co_shutdown      = esp32c3twai_shutdown,
  .co_rxint         = esp32c3twai_rxint,
  .co_txint         = esp32c3twai_txint,
  .co_ioctl         = esp32c3twai_ioctl,
  .co_remoterequest = esp32c3twai_remoterequest,
  .co_send          = esp32c3twai_send,
  .co_txready       = esp32c3twai_txready,
  .co_txempty       = esp32c3twai_txempty,
};
#ifdef CONFIG_ESP32C3_TWAI0
static struct up_dev_s g_twai0priv =
{
  .port    = 0,
  //.divisor = CONFIG_LPC17_40_CAN1_DIVISOR,
  //.baud    = CONFIG_LPC17_40_CAN1_BAUD,
  .base    = DR_REG_TWAI_BASE,
  .cpuint  = -ENOMEM,
  .periph  = ESP32C3_PERIPH_TWAI,
  .irq     = ESP32C3_IRQ_TWAI,
};

static struct can_dev_s g_twai0dev =
{
  .cd_ops  = &g_twaiops,
  .cd_priv = &g_twai0priv,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: twai_printreg
 *
 * Description:
 *   Print the value read from a register.
 *
 * Input Parameters:
 *   addr - The register address
 *   value - The register value
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_ESP32C3_TWAI_REGDEBUG
static void twai_printreg(uint32_t addr, uint32_t value)
{
  static uint32_t prevaddr = 0;
  static uint32_t preval   = 0;
  static uint32_t count    = 0;

  /* Is this the same value that we read from the same register last time?
   * Are we polling the register?  If so, suppress some of the output.
   */

  if (addr == prevaddr && value == preval)
    {
      if (count == 0xffffffff || ++count > 3)
        {
          if (count == 4)
            {
              caninfo("...\n");
            }

          return;
        }
    }

  /* No this is a new address or value */

  else
    {
      /* Did we print "..." for the previous value? */

      if (count > 3)
        {
          /* Yes.. then show how many times the value repeated */

          caninfo("[repeats %d more times]\n", count - 3);
        }

      /* Save the new address, value, and count */

      prevaddr = addr;
      preval   = value;
      count    = 1;
    }

  /* Show the register value read */

  caninfo("%08x->%08x\n", addr, value);
}
#endif

/****************************************************************************
 * Name: twai_getreg
 *
 * Description:
 *   Read the value of an TWAI register.
 *
 * Input Parameters:
 *   priv - A reference to the TWAI block status
 *   offset - The offset to the register to read
 *
 * Returned Value:
 *
 ****************************************************************************/

#ifdef CONFIG_ESP32C3_TWAI_REGDEBUG
static uint32_t twai_getreg(struct up_dev_s *priv, int offset)
{
  uint32_t addr;
  uint32_t value;

  /* Read the value from the register */

  addr  = /*priv->base + */offset;
  value = getreg32(addr);
  twai_printreg(addr, value);
  return value;
}
#else
static uint32_t twai_getreg(struct up_dev_s *priv, int offset)
{
  return getreg32(/*priv->base + */offset);
}
#endif

/****************************************************************************
 * Name: twai_putreg
 *
 * Description:
 *   Set the value of an TWAI register.
 *
 * Input Parameters:
 *   priv - A reference to the TWAI block status
 *   offset - The offset to the register to write
 *   value - The value to write to the register
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_ESP32C3_TWAI_REGDEBUG
static void twai_putreg(struct up_dev_s *priv, int offset, uint32_t value)
{
  uint32_t addr = /*priv->base + */offset;

  /* Show the register value being written */

  caninfo("%08x<-%08x\n", addr, value);

  /* Write the value */

  putreg32(value, addr);
}
#else
static void twai_putreg(struct up_dev_s *priv, int offset, uint32_t value)
{
  putreg32(value, /*priv->base + */offset);
}
#endif

/****************************************************************************
 * Name: twai_getcommon
 *
 * Description:
 *   Get the value of common register.
 *
 * Input Parameters:
 *   addr - The address of the register to read
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_ESP32C3_TWAI_REGDEBUG
static uint32_t twai_getcommon(uint32_t addr)
{
  uint32_t value;

  /* Read the value from the register */

  value = getreg32(addr);
  twai_printreg(addr, value);
  return value;
}
#endif

/****************************************************************************
 * Name: twai_putcommon
 *
 * Description:
 *   Set the value of common register.
 *
 * Input Parameters:
 *   addr - The address of the register to write
 *   value - The value to write to the register
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_ESP32C3_TWAI_REGDEBUG
static void twai_putcommon(uint32_t addr, uint32_t value)
{
  /* Show the register value being written */

  caninfo("%08x<-%08x\n", addr, value);

  /* Write the value */

  putreg32(value, addr);
}
#endif

#define HAL_SWAP16(d) __builtin_bswap16((d))
#define HAL_SWAP32(d) __builtin_bswap32((d))
#define HAL_SWAP64(d) __builtin_bswap64((d))
#define MSG_ID                  0x555   //11 bit standard format ID
#define TWAI_STD_ID_MASK        0x7FF       /**< Bit mask for 11 bit Standard Frame Format ID */

/**
 * @brief   Set Acceptance Filter
 * @param hw Start address of the TWAI registers
 * @param code Acceptance Code
 * @param mask Acceptance Mask
 * @param single_filter Whether to enable single filter mode
 *
 * @note Must be called in reset mode
 */
static inline void twai_ll_set_acc_filter(uint32_t code, uint32_t mask, bool single_filter)
{
  uint32_t code_swapped = HAL_SWAP32(code);
  uint32_t mask_swapped = HAL_SWAP32(mask);
  caninfo("set_acc_filter\n");
  for (int i = 0; i < 4; i++)
    {
      twai_putreg(0, TWAI_DATA_0_REG + (i * 4), ((code_swapped >> (i * 8)) & 0xFF));
      twai_putreg(0, TWAI_DATA_4_REG + (i * 4), ((mask_swapped >> (i * 8)) & 0xFF));
      //HAL_FORCE_MODIFY_U32_REG_FIELD(hw->acceptance_filter.acr[i], byte, ((code_swapped >> (i * 8)) & 0xFF));
      //HAL_FORCE_MODIFY_U32_REG_FIELD(hw->acceptance_filter.amr[i], byte, ((mask_swapped >> (i * 8)) & 0xFF));
    }
  //hw->mode_reg.afm = single_filter;
}

/****************************************************************************
 * Name: esp32c3twai_reset
 *
 * Description:
 *   Reset the TWAI device.  Called early to initialize the hardware. This
 *   function is called, before esp32_twai_setup() and on error conditions.
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" TWAI driver state structure.
 *
 * Returned Value:
 *  None
 *
 ****************************************************************************/

static void esp32c3twai_reset(FAR struct can_dev_s *dev)
{
  FAR struct up_dev_s *priv = (FAR struct up_dev_s *)dev->cd_priv;
  irqstate_t flags;
  int ret;

  caninfo("Reset function\n");
  caninfo("TWAI%d\n", priv->port);

  flags = enter_critical_section();

  /* Disable the TWAI and stop ongong transmissions */

  uint32_t mode_value = TWAI_RESET_MODE_M | TWAI_LISTEN_ONLY_MODE_M;
  twai_putreg(priv, TWAI_MODE_REG, mode_value);                 /* Enter Reset Mode */

  twai_putreg(priv, TWAI_INT_ENA_REG, 0);                       /* Disable interrupts */
  twai_getreg(priv, TWAI_STATUS_REG);                           /* Clear status bits */
  twai_putreg(priv, TWAI_CMD_REG, TWAI_ABORT_TX_M);             /* Abort transmission */

  twai_putreg(priv, TWAI_TX_ERR_CNT_REG, TWAI_INIT_TEC);        /* TEC */
  twai_putreg(priv, TWAI_RX_ERR_CNT_REG, TWAI_INIT_REC);        /* REC */
  twai_putreg(priv, TWAI_ERR_WARNING_LIMIT_REG, TWAI_INIT_EWL); /* EWL */

//  static const twai_filter_config_t f_config = {.acceptance_code = (MSG_ID << 21),
//                                             .acceptance_mask = ~(TWAI_STD_ID_MASK << 21),
//                                             .single_filter = true};

  twai_ll_set_acc_filter((MSG_ID << 21), ~(TWAI_STD_ID_MASK << 21), true);	// Tested and working

  //      #define TWAI_TIMING_CONFIG_25KBITS()    {.brp = 128, .tseg_1 = 16, .tseg_2 = 8, .sjw = 3, .triple_sampling = false}


  uint32_t timing0;
  uint32_t timing1;

  timing0 = 63;
  timing0 |= (2 << 14);

  timing1 = 15;
  timing1 |= (7 << 4);

  twai_putreg(0, TWAI_BUS_TIMING_0_REG, timing0);
  twai_putreg(0, TWAI_BUS_TIMING_1_REG, timing1);

  twai_getreg(0, TWAI_BUS_TIMING_0_REG);
  twai_getreg(0, TWAI_BUS_TIMING_1_REG);

  caninfo("Reset alive %08x\n", twai_getreg(priv, TWAI_MODE_REG));
  /* Set bit timing */

  /*ret = can_bittiming(priv);
  if (ret != OK)
    {
      canerr("ERROR: Failed to set bit timing: %d\n", ret);
    }*/

  /* Restart the TWAI */

#ifdef CONFIG_CAN_LOOPBACK
  caninfo("Leave Reset Mode, enter Test Mode\n");
  twai_putreg(priv, TWAI_MODE_REG, TWAI_SELF_TEST_MODE_M | TWAI_RX_FILTER_MODE_M); /* Leave Reset Mode, enter Test Mode */
#else
  caninfo("Leave Reset Mode\n");
  twai_putreg(priv, TWAI_MODE_REG, TWAI_RX_FILTER_MODE_M);                     /* Leave Reset Mode */
#endif
  //twai_putcommon(LPC17_40_CANAF_AFMR, CANAF_AFMR_ACCBP);   /* All RX messages accepted */ ???
  //TWAI_TX_BUF_ST ???
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: esp32c3twai_setup
 *
 * Description:
 *   Configure the TWAI. This method is called the first time that the TWAI
 *   device is opened.  This will occur when the port is first opened.
 *   This setup includes configuring and attaching TWAI interrupts.
 *   All TWAI interrupts are disabled upon return.
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" TWAI driver state structure.
 *
 * Returned Value:
 *   Zero on success; a negated errno on failure
 *
 ****************************************************************************/

static int esp32c3twai_setup(FAR struct can_dev_s *dev)
{
  caninfo("Enter setup function\n");
  FAR struct up_dev_s *priv = (FAR struct up_dev_s *)dev->cd_priv;
  uint32_t regval;
  irqstate_t flags;
  int ret;


  twai_getreg(0, TWAI_BUS_TIMING_0_REG);
  twai_getreg(0, TWAI_BUS_TIMING_1_REG);

  caninfo("setup TWAI%d\n", priv->port);

  flags = enter_critical_section();

  twai_putreg(priv, TWAI_CMD_REG, TWAI_RELEASE_BUF_M | TWAI_CLR_OVERRUN_M); // fix

  twai_putreg(priv, TWAI_INT_ENA_REG, TWAI_DEFAULT_INTERRUPTS);
  regval = twai_getreg(priv, TWAI_INT_RAW_REG);

//  leave_critical_section(flags);
//
//  ret = irq_attach(ESP32C3_IRQ_TWAI, esp32c3twai_interrupt, priv);
//  if (ret == OK)
//    {
//      caninfo("Enable interrupts\n");
//      up_enable_irq(ESP32C3_IRQ_TWAI);
//    }

  if (priv->cpuint != -ENOMEM)
    {
      /* Disable the provided CPU Interrupt to configure it. */
      up_disable_irq(priv->cpuint);
    }

  priv->cpuint = esp32c3_request_irq(priv->periph,
                                     ESP32C3_INT_PRIO_DEF,
                                     ESP32C3_INT_LEVEL);
  if (priv->cpuint < 0)
    {
      /* Failed to allocate a CPU interrupt of this type. */

      leave_critical_section(flags);

      return NULL;
    }

  if (irq_attach(priv->irq, esp32c3twai_interrupt, priv) != OK)
    {
      /* Failed to attach IRQ, so CPU interrupt must be freed. */

      esp32c3_free_cpuint(priv->periph);
      priv->cpuint = -ENOMEM;
      leave_critical_section(flags);

      return NULL;
    }

  /* Enable the CPU interrupt that is linked to the SPI device. */
  caninfo("priv->cpuint=%d ena=%08x status=%08x\n", priv->cpuint, twai_getreg(priv, TWAI_INT_ENA_REG), twai_getreg(priv, TWAI_STATUS_REG));
  up_enable_irq(priv->cpuint);

  leave_critical_section(flags);

  return ret;
}

/****************************************************************************
 * Name: esp32c3twai_shutdown
 *
 * Description:
 *   Disable the TWAI.  This method is called when the TWAI device is closed.
 *   This method reverses the operation the setup method.
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" TWAI driver state structure.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void esp32c3twai_shutdown(FAR struct can_dev_s *dev)
{
  caninfo("Enter shutdown function\n");
#ifdef CONFIG_DEBUG_CAN_INFO
  FAR struct up_dev_s *priv = (FAR struct up_dev_s *)dev->cd_priv;

  caninfo("shutdown TWAI%d\n", priv->port);
#endif

  up_disable_irq(ESP32C3_IRQ_TWAI);
  irq_detach(ESP32C3_IRQ_TWAI);
  return;
}

/****************************************************************************
 * Name: esp32c3twai_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts.
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" TWAI driver state structure.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void esp32c3twai_rxint(FAR struct can_dev_s *dev, bool enable)
{
  caninfo("Enter rxint function\n");
  FAR struct up_dev_s *priv = (FAR struct up_dev_s *)dev->cd_priv;
  uint32_t regval;
  irqstate_t flags;

  caninfo("rxint TWAI%d enable: %d\n", priv->port, enable);

  /* The EIR register is also modified from the interrupt handler, so we have
   * to protect this code section.
   */

  flags = enter_critical_section();
  regval = twai_getreg(priv, TWAI_INT_ENA_REG);
  if (enable)
    {
      caninfo("rxint interrupt enable\n");
      regval |= TWAI_RX_INT_ENA_M;
    }
  else
    {
      regval &= ~TWAI_RX_INT_ENA_M;
    }

  twai_putreg(priv, TWAI_INT_ENA_REG, regval);
  leave_critical_section(flags);
  return;
}

/****************************************************************************
 * Name: esp32c3twai_txint
 *
 * Description:
 *   Call to enable or disable TX interrupts.
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" TWAI driver state structure.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void esp32c3twai_txint(FAR struct can_dev_s *dev, bool enable)
{
  FAR struct up_dev_s *priv = (FAR struct up_dev_s *)dev->cd_priv;
  caninfo("Enter txint function + status reg %08x + err_code %08x\n", twai_getreg(priv, TWAI_STATUS_REG), twai_getreg(priv, TWAI_ERR_CODE_CAP_REG));
  caninfo("Enter txint function + rx_err %08x + tx_err %08x\n", twai_getreg(priv, TWAI_RX_ERR_CNT_REG), twai_getreg(priv, TWAI_TX_ERR_CNT_REG));
  uint32_t regval;
  irqstate_t flags;

  //caninfo("txint TWAI%d enable: %d + raw %08x\n", priv->port, enable), twai_getreg(priv, TWAI_INT_RAW_REG);

  /* Only disabling of the TX interrupt is supported here.  The TX interrupt
   * is automatically enabled just before a message is sent in order to avoid
   * lost TX interrupts.
   */

  if (!enable)
    {
      /* TX interrupts are also disabled from the interrupt handler, so we
       * have to protect this code section.
       */

      flags = enter_critical_section();

      /* Disable all TX interrupts */

      caninfo("txint interrupt disable\n");
      regval = twai_getreg(priv, TWAI_INT_ENA_REG);
      regval &= ~(TWAI_TX_INT_ENA_M);
      twai_putreg(priv, TWAI_INT_ENA_REG, regval);
      leave_critical_section(flags);
    }
}

static int esp32c3twai_ioctl(FAR struct can_dev_s *dev, int cmd,
                          unsigned long arg)
{
  caninfo("Enter ioctl function\n");
  return 0;
}

static int esp32c3twai_remoterequest(FAR struct can_dev_s *dev, uint16_t id)
{
  caninfo("Enter remoterequest function\n");
  return 0;
}

/****************************************************************************
 * Name: esp32c3twai_send
 *
 * Description:
 *    Send one TWAI message.
 *
 *    One TWAI-message consists of a maximum of 10 bytes.  A message is
 *    composed of at least the first 2 bytes (when there are no data bytes).
 *
 *    Byte 0:      Bits 0-7: Bits 3-10 of the 11-bit TWAI identifier
 *    Byte 1:      Bits 5-7: Bits 0-2 of the 11-bit TWAI identifier
 *                 Bit 4:    Remote Transmission Request (RTR)
 *                 Bits 0-3: Data Length Code (DLC)
 *    Bytes 2-10: TWAI data
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" TWAI driver state structure.
 *
 * Returned Value:
 *   Zero on success; a negated errno on failure
 *
 ****************************************************************************/

static int esp32c3twai_send(FAR struct can_dev_s *dev, FAR struct can_msg_s *msg)
{
  FAR struct up_dev_s *priv = (FAR struct up_dev_s *)dev->cd_priv;
  caninfo("Enter send function + status reg %08x\n", twai_getreg(priv, TWAI_STATUS_REG));
  caninfo("Enter send function + rx_err %08x + tx_err %08x\n", twai_getreg(priv, TWAI_RX_ERR_CNT_REG), twai_getreg(priv, TWAI_TX_ERR_CNT_REG));
//  uint32_t tid = (uint32_t)msg->cm_hdr.ch_id;
//  uint32_t tfi = (uint32_t)msg->cm_hdr.ch_dlc << 16;
  uint32_t regval;
  uint32_t i;
  uint32_t len;
  uint32_t id;
  uint32_t frame_info;
  irqstate_t flags;
  int ret = OK;

  caninfo("CAN%d ID: %" PRId32 " DLC: %d DATA0: %x\n",
          priv->port, (uint32_t)msg->cm_hdr.ch_id, msg->cm_hdr.ch_dlc, msg->cm_data[0]);

  len = (uint32_t)msg->cm_hdr.ch_dlc;
	if (len > TWAI_MSG_LENGTH) len = TWAI_MSG_LENGTH;

	frame_info = len;

  if (msg->cm_hdr.ch_rtr)
    {
      caninfo("RTR message\n");
      frame_info |= (1 << 6);
//      tfi |= CAN_TFI_RTR;
    }

  flags = enter_critical_section();

  /* Pick a transmit buffer */

//  regval = can_getreg(priv, LPC17_40_CAN_SR_OFFSET);
//  if ((regval & CAN_SR_TBS1) != 0)
//    {
      /* Make sure that buffer 1 TX interrupts are enabled BEFORE sending the
       * message. The TX interrupt is generated when the TBSn bit in CANxSR
       * goes from 0 to 1 when the TIEn bit in CANxIER is 1.  If we don't
       * enable it now, we may miss the TIE1 interrupt.
       *
       * NOTE: The IER is also modified from the interrupt handler, but the
       * following is safe because interrupts are disabled here.
       */

      regval  = twai_getreg(priv, TWAI_INT_ENA_REG);
      regval |= TWAI_TX_INT_ENA_M;
      twai_putreg(priv, TWAI_INT_ENA_REG, regval);

      /* Set up the transfer */



  /* Set the FF bit in the TFI register if this message should be sent with
   * the extended frame format (and 29-bit extended ID).
   */

#ifdef CONFIG_CAN_EXTID
  if (msg->cm_hdr.ch_extid)
    {
      /* The provided ID should be 29 bits */
      id = (uint32_t)msg->cm_hdr.ch_id;
      DEBUGASSERT((id & ~TWAI_ID29_MASK) == 0);
      frame_info |= (1 << 7);
      twai_putreg(priv, TWAI_DATA_0_REG, frame_info);
//      tfi |= CAN_TFI_FF;
      id <<= 3;
      twai_putreg(priv, TWAI_DATA_4_REG, id & 0xFF);
      id >>= 8;
      twai_putreg(priv, TWAI_DATA_3_REG, id & 0xFF);
      id >>= 8;
      twai_putreg(priv, TWAI_DATA_2_REG, id & 0xFF);
      id >>= 8;
      twai_putreg(priv, TWAI_DATA_1_REG, id & 0xFF);
      for (i = 0; i < len; i++)
        {
		  twai_putreg(priv, TWAI_DATA_5_REG + (i * 4), msg->cm_data[i]);
		}
    }
  else
#endif
    {
      /* The provided ID should be 11 bits */
      id = (uint32_t)msg->cm_hdr.ch_id;
      DEBUGASSERT((id & ~TWAI_ID11_MASK) == 0);
      caninfo("TWAI_DATA_0_REG\n");
      twai_putreg(priv, TWAI_DATA_0_REG, frame_info);
      id <<= 5;
      twai_putreg(priv, TWAI_DATA_1_REG, (id >> 8) & 0xFF);
      twai_putreg(priv, TWAI_DATA_2_REG, id & 0xFF);
      for (i = 0; i < len; i++)
        {
		  twai_putreg(priv, TWAI_DATA_3_REG + (i * 4), msg->cm_data[i]);
		}
    }


//      can_putreg(priv, LPC17_40_CAN_TFI1_OFFSET, tfi);
//      can_putreg(priv, LPC17_40_CAN_TID1_OFFSET, tid);
//      can_putreg(priv, LPC17_40_CAN_TDA1_OFFSET,
//                 *(uint32_t *)&msg->cm_data[0]);
//      can_putreg(priv, LPC17_40_CAN_TDB1_OFFSET,
//                 *(uint32_t *)&msg->cm_data[4]);

      /* Send the message */

  caninfo("Leaving send function + rx_err %08x + tx_err %08x\n", twai_getreg(priv, TWAI_RX_ERR_CNT_REG), twai_getreg(priv, TWAI_TX_ERR_CNT_REG));
#ifdef CONFIG_CAN_LOOPBACK
    caninfo("CAN_LOOPBACK %08x\n", TWAI_SELF_RX_REQ_M | TWAI_ABORT_TX_M);
    twai_putreg(priv, TWAI_CMD_REG, TWAI_SELF_RX_REQ_M | TWAI_ABORT_TX_M);
#else
    twai_putreg(priv, TWAI_CMD_REG, TWAI_TX_REQ_M);
#endif

  caninfo("Leaving 2 send function + rx_err %08x + tx_err %08x\n", twai_getreg(priv, TWAI_RX_ERR_CNT_REG), twai_getreg(priv, TWAI_TX_ERR_CNT_REG));
//    {
//      canerr("ERROR: No available transmission buffer, SR: %08" PRIx32 "\n",
//             regval);
//      ret = -EBUSY;
//    }

  leave_critical_section(flags);
  caninfo("finish send function\n");

  return ret;
}

/****************************************************************************
 * Name: esp32c3twai_txready
 *
 * Description:
 *   Return true if the TWAI hardware can accept another TX message.
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" TWAI driver state structure.
 *
 * Returned Value:
 *   True if the TWAI hardware is ready to accept another TX message.
 *
 ****************************************************************************/

static bool esp32c3twai_txready(FAR struct can_dev_s *dev)
{
  caninfo("Enter txready function\n");
  FAR struct up_dev_s *priv = (FAR struct up_dev_s *)dev->cd_priv;
  uint32_t regval = twai_getreg(priv, TWAI_STATUS_REG);
  caninfo("txready result %d\n", ((regval & TWAI_TX_BUF_ST_M) != 0));
  return ((regval & TWAI_TX_BUF_ST_M) != 0);
}

/****************************************************************************
 * Name: esp32c3twai_txempty
 *
 * Description:
 *   Return true if all message have been sent.  If for example, the TWAI
 *   hardware implements FIFOs, then this would mean the transmit FIFO is
 *   empty.  This method is called when the driver needs to make sure that
 *   all characters are "drained" from the TX hardware before calling
 *   co_shutdown().
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" TWAI driver state structure.
 *
 * Returned Value:
 *   True if there are no pending TX transfers in the TWAI hardware.
 *
 ****************************************************************************/

static bool esp32c3twai_txempty(FAR struct can_dev_s *dev)
{
  caninfo("Enter txempty function\n");
  FAR struct up_dev_s *priv = (FAR struct up_dev_s *)dev->cd_priv;
  uint32_t regval = twai_getreg(priv, TWAI_STATUS_REG);
  caninfo("txempty result %d\n", ((regval & TWAI_TX_BUF_ST_M) != 0));
  return ((regval & TWAI_TX_BUF_ST_M) != 0);
}

/****************************************************************************
 * Name: esp32c3twai_interrupt
 *
 * Description:
 *   TWAI0 RX/TX interrupt handler
 *
 * Input Parameters:
 *   irq - The IRQ number of the interrupt.
 *   context - The register state save array at the time of the interrupt.
 *
 * Returned Value:
 *   Zero on success; a negated errno on failure
 *
 ****************************************************************************/

static int esp32c3twai_interrupt(int irq, void *context, FAR void *arg)
{
  caninfo("Enter interrupt function irq: %d\n",  irq);
#ifdef CONFIG_ESP32C3_TWAI0
  FAR struct can_dev_s *dev = &g_twai0dev;
  FAR struct up_dev_s *priv = (FAR struct up_dev_s *)dev->cd_priv;
  struct can_hdr_s hdr;
  uint8_t data[8];
//  uint32_t rfs;
//  uint32_t rid;
  uint32_t frame_info;
  uint32_t len;
  uint32_t datastart;
  uint32_t regval;
  uint32_t i;

  /* Read the interrupt and capture register (also clearing most status
   * bits)
   */

  regval = twai_getreg(priv, TWAI_INT_RAW_REG);
  caninfo("CAN%d ICR: %08" PRIx32 "\n", priv->port, regval);

  /* Check for a receive interrupt */

  if ((regval & TWAI_RX_INT_ST_M) != 0)
    {
      caninfo("Receive interrupt\n");

      frame_info = twai_getreg(priv, TWAI_DATA_0_REG);

      /* Construct the CAN header */

      if (frame_info & (1 << 6))
        {
          hdr.ch_rtr    = 1;
        }

#ifdef CONFIG_CAN_EXTID
    if (frame_info & (1 << 7))
      {
        /* The provided ID should be 29 bits */
        hdr.ch_extid = 1;
        hdr.ch_id =
				(twai_getreg(priv, TWAI_DATA_1_REG) << 21) +
				(twai_getreg(priv, TWAI_DATA_2_REG) << 13) +
				(twai_getreg(priv, TWAI_DATA_3_REG) << 5) +
				(twai_getreg(priv, TWAI_DATA_4_REG) >> 3);
        datastart = TWAI_DATA_5_REG;
    }
    else
#endif
      {
        /* The provided ID should be 11 bits */
        hdr.ch_id =
				(twai_getreg(priv, TWAI_DATA_1_REG) << 3) +
				(twai_getreg(priv, TWAI_DATA_2_REG) >> 5);
        datastart = TWAI_DATA_3_REG;
      }

    for(i = 0; i < len; i++)
      {
		data[i] = twai_getreg(priv, datastart + (i * 4));
	  }


      /* Release the receive buffer */

      twai_putreg(priv, TWAI_CMD_REG, TWAI_RELEASE_BUF_M);

      len = frame_info & 0xF;
      if (len > TWAI_MSG_LENGTH) len = TWAI_MSG_LENGTH;
      hdr.ch_dlc = len;
#ifdef CONFIG_CAN_ERRORS
      hdr.ch_error  = 0; /* Error reporting not supported */
#endif
      can_receive(dev, &hdr, data);

    }

  /* Check for TX buffer 1 complete */

  if ((regval & TWAI_TX_INT_ST_M) != 0)
    {
      caninfo("Transmit interrupt\n");
      /* Disable all further TX buffer interrupts */

      regval  = twai_getreg(priv, TWAI_INT_ENA_REG);
      regval &= ~TWAI_TX_INT_ENA_M;
      twai_putreg(priv, TWAI_INT_ENA_REG, regval);

      /* Indicate that the TX is done and a new TX buffer is available */

      can_txdone(dev);
    }

#endif
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32c3_twaiinitialize
 *
 * Description:
 *   Initialize the selected TWAI port
 *
 * Input Parameters:
 *   Port number (for hardware that has multiple TWAI interfaces)
 *
 * Returned Value:
 *   Valid TWAI device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

FAR struct can_dev_s *esp32c3_twaiinitialize(int port)
{
  FAR struct can_dev_s *twaidev;
  irqstate_t flags;
  uint32_t regval;

  caninfo("Hello world!\n");
  caninfo("TWAI%d\n",  port);

  flags = enter_critical_section();

#ifdef CONFIG_ESP32C3_TWAI0
  if (port == 0)
    {
      /* Enable power to the TWAI module ???*/

      /*regval  = can_getcommon(LPC17_40_SYSCON_PCONP);
      regval |= SYSCON_PCONP_PCCAN1;
      can_putcommon(LPC17_40_SYSCON_PCONP, regval);*/


      caninfo("SYSTEM_PERIP_CLK_EN0_REG!\n");
      modifyreg32(SYSTEM_PERIP_CLK_EN0_REG, SYSTEM_TWAI_RST_M, 0);
      modifyreg32(SYSTEM_PERIP_CLK_EN0_REG, 0, SYSTEM_TWAI_CLK_EN_M);
      //modifyreg32(TWAI_CLOCK_DIVIDER_REG, 0, TWAI_CLOCK_OFF_M); ??? probably not

      /* Enable clocking to the CAN module (not necessary... already done
       * in low level clock configuration logic).
       */

/*#ifdef LPC178x_40xx
      regval  = can_getcommon(LPC17_40_SYSCON_PCLKSEL);
      regval &= SYSCON_PCLKSEL_PCLKDIV_MASK
      regval >>= SYSCON_PCLKSEL_PCLKDIV_SHIFT;
      g_can1pri.divisor = regval;
#else
      regval  = can_getcommon(LPC17_40_SYSCON_PCLKSEL0);
      regval &= ~SYSCON_PCLKSEL0_CAN1_MASK;
      regval |= (CAN1_CCLK_DIVISOR << SYSCON_PCLKSEL0_CAN1_SHIFT);
      can_putcommon(LPC17_40_SYSCON_PCLKSEL0, regval);
#endif*/

      /* Configure CAN GPIO pins */

      //esp32c3_configgpio(TWAI_RX_IDX, INPUT_PULLUP /*PULLUP*/);
      //esp32c3_configgpio(TWAI_TX_IDX, OUTPUT);



      esp32c3_gpio_matrix_out(2, TWAI_TX_IDX, 0, 0);
      esp32c3_configgpio(2, OUTPUT_FUNCTION_1);

      esp32c3_configgpio(3, INPUT_FUNCTION_1);
      esp32c3_gpio_matrix_in(3, TWAI_RX_IDX, 0);

      twaidev = &g_twai0dev;
    }
  else
#endif

    {
      canerr("ERROR: Unsupported port: %d\n", port);
      leave_critical_section(flags);
      return NULL;
    }

   /* Then just perform a CAN reset operation */

   caninfo("esp32c3twai_reset!\n");
   esp32c3twai_reset(twaidev);
   leave_critical_section(flags);
   caninfo("return twaidev!\n");
   return twaidev;
    //}
//#endif
}
#endif
