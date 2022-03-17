/****************************************************************************
 * arch/risc-v/src/esp32c3/esp32c3_twai.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
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

#include <arch/board/board.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/can/can.h>

#include "riscv_arch.h"

#include "hardware/esp32c3_system.h"
#include "hardware/esp32c3_gpio_sigmap.h"
#include "esp32c3_gpio.h"
#include "esp32c3_twai.h"
#include "esp32c3_irq.h"
#include "esp32c3_clockconfig.h"

#if defined(CONFIG_ESP32C3_TWAI)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Default values written to various registers on initialization */
#define TWAI_INIT_TEC             0
#define TWAI_INIT_REC             0
#define TWAI_INIT_EWL             96
#define TWAI_MSG_LENGTH           8

/* Pin definition */
#define TWAI_TX_PIN               2
#define TWAI_RX_PIN               3

/* Bits 0-10: 11-bit Identifier (FF=0)
 * Bits 11-31: Reserved
 */

//#define TWAI_ID11_MASK            CAN_MAX_STDMSGID

/* Bits 0-28: 29-bit Identifier (FF=1)
 * Bits 29-31: Reserved
 */

//#define TWAI_ID29_MASK            CAN_MAX_EXTMSGID

#define TWAI_DEFAULT_INTERRUPTS   0xe7  /* Exclude data overrun (bit[3]) and brp_div (bit[4]) */

#define ACCEPTANCE_CODE           0x0           /* 32-bit address to match */
#define ACCEPTANCE_MASK           0xffffffff    /* 32-bit address mask */

//#define TWAI_BTR_BRP_MAX          64      /* Maximum BTR value (without decrement) */
//#define TWAI_BTR_TSEG1_MAX        16      /* Maximum TSEG1 value (without decrement) */
//#define TWAI_BTR_TSEG2_MAX        8       /* Maximum TSEG2 value (without decrement) */

#ifdef CONFIG_ESP32C3_TWAI0

/* A TWAI bit rate must be provided */

#  ifndef CONFIG_ESP32C3_TWAI0_BITRATE
#    error "CONFIG_ESP32C3_TWAI0_BITRATE is not defined"
#  endif

/* If no sample point is provided, use a sample point of 80 */

#  ifndef CONFIG_ESP32C3_TWAI0_SAMPLEP
#    define CONFIG_ESP32C3_TWAI0_SAMPLEP 80
#  endif
#endif

/* If no Synchronization Jump Width is provided, use a SJW of 3 */

#ifndef CONFIG_ESP32C3_TWAI0_SJW
#  define CONFIG_ESP32C3_TWAI0_SJW 3
#endif

/* User-defined TSEG1 and TSEG2 settings may be used.
 *
 * CONFIG_ESP32C3_TWAI_TSEG1 = the number of CAN time quanta in segment 1
 * CONFIG_ESP32C3_TWAI_TSEG2 = the number of CAN time quanta in segment 2
 * TWAI_BIT_QUANTA   = The number of CAN time quanta in on bit time
 */

//#ifndef CONFIG_ESP32C3_TWAI_TSEG1
//#  define CONFIG_ESP32C3_TWAI_TSEG1 6
//#endif
//
//#if CONFIG_ESP32C3_TWAI_TSEG1 < 1 || CONFIG_ESP32C3_TWAI_TSEG1 > TWAI_BTR_TSEG1_MAX
//#  error "CONFIG_ESP32C3_TWAI_TSEG1 is out of range"
//#endif
//
//#ifndef CONFIG_ESP32C3_TWAI_TSEG2
//#  define CONFIG_ESP32C3_TWAI_TSEG2 7
//#endif
//
//#if CONFIG_ESP32C3_TWAI_TSEG2 < 1 || CONFIG_ESP32C3_TWAI_TSEG2 > TWAI_BTR_TSEG2_MAX
//#  error "CONFIG_ESP32C3_TWAI_TSEG2 is out of range"
//#endif
//
//#define TWAI_BIT_QUANTA (CONFIG_ESP32C3_TWAI_TSEG1 + CONFIG_ESP32C3_TWAI_TSEG2 + 1)

/* Debug ********************************************************************/

/* Non-standard debug that may be enabled just for testing TWAI */

#ifndef CONFIG_DEBUG_CAN_INFO
#  undef CONFIG_ESP32C3_TWAI_REGDEBUG
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/*
 * CAN bit-timing parameters
 *
 * For further information, please read chapter "8 BIT TIMING
 * REQUIREMENTS" of the "Bosch CAN Specification version 2.0"
 * at http://www.semiconductors.bosch.de/pdf/can2spec.pdf.
 */
//struct can_bittiming {
//  uint32_t bitrate;    /* Bit-rate in bits/second */
//  uint32_t sample_point; /* Sample point in one-tenth of a percent */
//  uint32_t tq;   /* Time quanta (TQ) in nanoseconds */
//  uint32_t prop_seg;   /* Propagation segment in TQs */
//  uint32_t phase_seg1; /* Phase buffer segment 1 in TQs */
//  uint32_t phase_seg2; /* Phase buffer segment 2 in TQs */
//  uint32_t sjw;    /* Synchronisation jump width in TQs */
//  uint32_t brp;    /* Bit-rate prescaler */
//};

/*
 * CAN hardware-dependent bit-timing constant
 *
 * Used for calculating and checking bit-timing parameters
 */
struct can_bittiming_const {
  uint32_t tseg1_min;  /* Time segment 1 = prop_seg + phase_seg1 */
  uint32_t tseg1_max;
  uint32_t tseg2_min;  /* Time segment 2 = phase_seg2 */
  uint32_t tseg2_max;
  uint32_t sjw_min;    /* Synchronisation jump width */
  uint32_t sjw_max;
  uint32_t brp_min;    /* Bit-rate prescaler */
  uint32_t brp_max;
  uint32_t brp_inc;
};

//struct can_config_s
//{
//  uint8_t periph; /* Peripheral ID */
//  uint8_t irq;    /* IRQ associated with this TWAI */
//};

struct twai_dev_s
{
  /* Device configuration */

  const struct can_bittiming_const *bittiming_const;
  uint8_t port;     /* TWAI port number */
  uint8_t clkdiv;   /* CLKDIV register */
  uint32_t bitrate;    /* Configured bit rate */
  uint32_t samplep; /* Configured sample point */
  uint32_t sjw;     /* Synchronisation jump width */
  uint8_t periph; /* Peripheral ID */
  uint8_t irq;    /* IRQ associated with this TWAI */
  uint32_t base;    /* TWAI register base address */
  uint8_t cpuint;   /* CPU interrupt assigned to this TWAI */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* TWAI Register access */

#ifdef CONFIG_ESP32C3_TWAI_REGDEBUG
static void twai_printreg(uint32_t addr, uint32_t value);
#endif

static uint32_t twai_getreg(uint32_t addr);
static void twai_putreg(uint32_t addr, uint32_t value);

/* TWAI methods */

static void esp32c3twai_reset(FAR struct can_dev_s *dev);
static int  esp32c3twai_setup(FAR struct can_dev_s *dev);
static void esp32c3twai_shutdown(FAR struct can_dev_s *dev);
static void esp32c3twai_rxint(FAR struct can_dev_s *dev, bool enable);
static void esp32c3twai_txint(FAR struct can_dev_s *dev, bool enable);
static int  esp32c3twai_ioctl(FAR struct can_dev_s *dev, int cmd,
                           unsigned long arg);
static int  esp32c3twai_remoterequest(FAR struct can_dev_s *dev,
                                      uint16_t id);
static int  esp32c3twai_send(FAR struct can_dev_s *dev,
                             FAR struct can_msg_s *msg);
static bool esp32c3twai_txready(FAR struct can_dev_s *dev);
static bool esp32c3twai_txempty(FAR struct can_dev_s *dev);

/* TWAI interrupts */

static int esp32c3twai_interrupt(int irq, void *context, FAR void *arg);

/* TWAI acceptance filter */

static void esp32c3twai_set_acc_filter(uint32_t code, uint32_t mask,
                                       bool single_filter);

/* Initialization */

static int twai_bittiming(struct twai_dev_s *priv);
static int twai_baud_rate(FAR struct twai_dev_s *priv, int rate, int clock,
    int sjw, int sampl_pt, int flags);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct can_bittiming_const esp32c3_twai_bittiming_const = {
  .tseg1_min = 1,
  .tseg1_max = 16,
  .tseg2_min = 1,
  .tseg2_max = 8,
  .sjw_min = 1,
  .sjw_max = 3,
  .brp_min = 1,
  .brp_max = 64,
  .brp_inc = 1,
};

//static const struct can_config_s esp32c3_twai_config =
//{
//  .periph = ESP32C3_PERIPH_TWAI,
//  .irq    = ESP32C3_IRQ_TWAI,
//};

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
static struct twai_dev_s g_twai0priv =
{
  .bittiming_const = &esp32c3_twai_bittiming_const,
  .port    = 0,
  .periph = ESP32C3_PERIPH_TWAI,
  .irq    = ESP32C3_IRQ_TWAI,
  .bitrate    = CONFIG_ESP32C3_TWAI0_BITRATE,
  .samplep = CONFIG_ESP32C3_TWAI0_SAMPLEP,
  .sjw = CONFIG_ESP32C3_TWAI0_SJW,
  .base    = DR_REG_TWAI_BASE,
  .cpuint  = -ENOMEM,
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
 *   addr - The address to the register to read
 *
 * Returned Value:
 *
 ****************************************************************************/

#ifdef CONFIG_ESP32C3_TWAI_REGDEBUG
static uint32_t twai_getreg(uint32_t addr)
{
  uint32_t value;

  /* Read the value from the register */

  value = getreg32(addr);
  twai_printreg(addr, value);
  return value;
}
#else
static uint32_t twai_getreg(uint32_t addr)
{
  return getreg32(addr);
}
#endif

/****************************************************************************
 * Name: twai_putreg
 *
 * Description:
 *   Set the value of an TWAI register.
 *
 * Input Parameters:
 *   addr - The address to the register to write
 *   value - The value to write to the register
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_ESP32C3_TWAI_REGDEBUG
static void twai_putreg(uint32_t addr, uint32_t value)
{
  /* Show the register value being written */

  caninfo("%08x<-%08x\n", addr, value);

  /* Write the value */

  putreg32(value, addr);
}
#else
static void twai_putreg(uint32_t addr, uint32_t value)
{
  putreg32(value, addr);
}
#endif

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
  FAR struct twai_dev_s *priv = (FAR struct twai_dev_s *)dev->cd_priv;
  irqstate_t flags;
  int ret;

  caninfo("Reset function\n");
  caninfo("TWAI%d\n", priv->port);

  flags = enter_critical_section();

  /* Disable the TWAI and stop ongoing transmissions */

  uint32_t mode_value = TWAI_RESET_MODE_M | TWAI_LISTEN_ONLY_MODE_M;
  twai_putreg(TWAI_MODE_REG, mode_value);                 /* Enter Reset Mode */

  twai_putreg(TWAI_INT_ENA_REG, 0);                       /* Disable interrupts */
  twai_getreg(TWAI_STATUS_REG);                           /* Clear status bits */

  twai_putreg(TWAI_TX_ERR_CNT_REG, TWAI_INIT_TEC);        /* TEC */
  twai_putreg(TWAI_RX_ERR_CNT_REG, TWAI_INIT_REC);        /* REC */
  twai_putreg(TWAI_ERR_WARNING_LIMIT_REG, TWAI_INIT_EWL); /* EWL */

  esp32c3twai_set_acc_filter(ACCEPTANCE_CODE, ACCEPTANCE_MASK, true);

  caninfo("Reset alive %08x\n", twai_getreg(TWAI_MODE_REG));

  /* Set bit timing */

  //ret = twai_bittiming(priv);
  ret = twai_baud_rate(priv, priv->bitrate, esp32c3_clk_apb_freq(),
      priv->sjw, priv->samplep, 0);

  if (ret != OK)
    {
      canerr("ERROR: Failed to set bit timing: %d\n", ret);
    }

  /* Restart the TWAI */

#ifdef CONFIG_CAN_LOOPBACK
  caninfo("Leave Reset Mode, enter Test Mode\n");
  twai_putreg(TWAI_MODE_REG, TWAI_SELF_TEST_MODE_M); /* Leave Reset Mode, enter Test Mode */
#else
  caninfo("Leave Reset Mode\n");
  twai_putreg(TWAI_MODE_REG, 0);                     /* Leave Reset Mode */
#endif

  /* Abort transmission, release RX buffer and clear overrun.
   * Command register can only be modified when in Operation Mode.
   */

  twai_putreg(TWAI_CMD_REG, TWAI_ABORT_TX_M | TWAI_RELEASE_BUF_M |
              TWAI_CLR_OVERRUN_M);

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
  FAR struct twai_dev_s *priv = (FAR struct twai_dev_s *)dev->cd_priv;
  irqstate_t flags;
  int ret = OK;

  caninfo("setup TWAI%d\n", priv->port);

  flags = enter_critical_section();

  twai_putreg(TWAI_INT_ENA_REG, TWAI_DEFAULT_INTERRUPTS);

  twai_getreg(TWAI_INT_RAW_REG);          /* clear latched interrupts */

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

      ret = priv->cpuint;
      leave_critical_section(flags);

      return ret;
    }

  ret = irq_attach(priv->irq, esp32c3twai_interrupt, priv);
  if (ret != OK)
    {
      /* Failed to attach IRQ, so CPU interrupt must be freed. */

      esp32c3_free_cpuint(priv->periph);
      priv->cpuint = -ENOMEM;
      leave_critical_section(flags);

      return ret;
    }

  /* Enable the CPU interrupt that is linked to the SPI device. */

  caninfo("priv->cpuint=%d ena=%08x status=%08x\n", priv->cpuint,
          twai_getreg(TWAI_INT_ENA_REG),
          twai_getreg(TWAI_STATUS_REG));

  /* irq_attach(ESP32C3_IRQ_TWAI, esp32c3twai_interrupt, priv); */

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
  FAR struct twai_dev_s *priv = (FAR struct twai_dev_s *)dev->cd_priv;

#ifdef CONFIG_DEBUG_CAN_INFO
  caninfo("shutdown TWAI%d\n", priv->port);
#endif

  if (priv->cpuint != -ENOMEM)
    {
      /* Disable cpu interrupt */

      up_disable_irq(priv->cpuint);

      /* Dissociate the IRQ from the ISR */

      irq_detach(priv->irq);

      /* Free cpu interrupt that is attached to this peripheral */

      esp32c3_free_cpuint(priv->periph);
      priv->cpuint = -ENOMEM;
    }

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
  FAR struct twai_dev_s *priv = (FAR struct twai_dev_s *)dev->cd_priv;
  uint32_t regval;
  irqstate_t flags;

  caninfo("rxint TWAI%d enable: %d\n", priv->port, enable);

  /* The INT_ENA register is also modified from the interrupt handler,
   * so we have to protect this code section.
   */

  flags = enter_critical_section();
  regval = twai_getreg(TWAI_INT_ENA_REG);
  if (enable)
    {
//      caninfo("rxint interrupt enable\n");
      regval |= TWAI_RX_INT_ENA_M;
    }
  else
    {
      regval &= ~TWAI_RX_INT_ENA_M;
    }

  twai_putreg(TWAI_INT_ENA_REG, regval);
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
  /* FAR struct twai_dev_s *priv = (FAR struct twai_dev_s *)dev->cd_priv; */

  caninfo("Enter txint function + status reg %08x + err_code %08x\n",
          twai_getreg(TWAI_STATUS_REG),
          twai_getreg(TWAI_ERR_CODE_CAP_REG));
  caninfo("Enter txint function + rx_err %08x + tx_err %08x\n",
          twai_getreg(TWAI_RX_ERR_CNT_REG),
          twai_getreg(TWAI_TX_ERR_CNT_REG));
  uint32_t regval;
  irqstate_t flags;

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
      regval = twai_getreg(TWAI_INT_ENA_REG);
      regval &= ~(TWAI_TX_INT_ENA_M);
      twai_putreg(TWAI_INT_ENA_REG, regval);
      leave_critical_section(flags);
    }
}

static int esp32c3twai_ioctl(FAR struct can_dev_s *dev, int cmd,
                          unsigned long arg)
{
  int ret = -ENOTTY;

  caninfo("Enter ioctl function + cmd=%04x arg=%lu\n", cmd, arg);

  /* Handle the command */

  switch (cmd)
    {
      /* CANIOC_GET_BITTIMING:
       *   Description:    Return the current bit timing settings
       *   Argument:       A pointer to a write-able instance of struct
       *                   canioc_bittiming_s in which current bit timing
       *                   values will be returned.
       *   Returned Value: Zero (OK) is returned on success.  Otherwise -1
       *                   (ERROR) is returned with the errno variable set
       *                   to indicate the nature of the error.
       *   Dependencies:   None
       */

      case CANIOC_GET_BITTIMING:
        {
          FAR struct canioc_bittiming_s *bt =
            (FAR struct canioc_bittiming_s *)arg;
          uint32_t timing0;
          uint32_t timing1;
          uint32_t brp;

          DEBUGASSERT(bt != NULL);

          timing0 = twai_getreg(TWAI_BUS_TIMING_0_REG);
          timing1 = twai_getreg(TWAI_BUS_TIMING_1_REG);

          brp = ((timing0 & TWAI_BAUD_PRESC_M) + 1) * 2;
          bt->bt_sjw = ((timing0 & TWAI_SYNC_JUMP_WIDTH_M) >>
                       TWAI_SYNC_JUMP_WIDTH_S) + 1;

          bt->bt_tseg1 = ((timing1 & TWAI_TIME_SEG1_M) >>
                         TWAI_TIME_SEG1_S) + 1;
          bt->bt_tseg2 = ((timing1 & TWAI_TIME_SEG2_M) >>
                         TWAI_TIME_SEG2_S)+ 1;
          bt->bt_baud = esp32c3_clk_apb_freq() /
              (brp * (bt->bt_tseg1 + bt->bt_tseg2 + 1));

          caninfo("regval %04x bt_tseg1= %d bt_tseg1_after= %d \n",
              timing1, timing1 & timing1 & TWAI_TIME_SEG2_M, bt->bt_tseg2);

          ret = OK;
        }
        break;

      /* Unsupported/unrecognized command */

      default:
        canerr("ERROR: Unrecognized command: %04x\n", cmd);
        break;
    }

  return ret;
}

static int esp32c3twai_remoterequest(FAR struct can_dev_s *dev, uint16_t id)
{
  caninfo("Enter remoterequest function\n");
  return -ENOSYS;
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

static int esp32c3twai_send(FAR struct can_dev_s *dev,
                            FAR struct can_msg_s *msg)
{
  FAR struct twai_dev_s *priv = (FAR struct twai_dev_s *)dev->cd_priv;
  caninfo("Enter send function + status reg %08x------------------\n",
          twai_getreg(TWAI_STATUS_REG));
  caninfo("Enter send function + rx_err %08x + tx_err %08x\n",
          twai_getreg(TWAI_RX_ERR_CNT_REG),
          twai_getreg(TWAI_TX_ERR_CNT_REG));

  uint32_t regval;
  uint32_t i;
  uint32_t len;
  uint32_t id;
  uint32_t frame_info;
  irqstate_t flags;
  int ret = OK;

  caninfo("CAN%d ID: %" PRId32 " DLC: %d DATA0: %x\n",
          priv->port, (uint32_t)msg->cm_hdr.ch_id,
          msg->cm_hdr.ch_dlc, msg->cm_data[0]);

  len = (uint32_t)msg->cm_hdr.ch_dlc;
  if (len > TWAI_MSG_LENGTH) len = TWAI_MSG_LENGTH;

  frame_info = len;

  if (msg->cm_hdr.ch_rtr)
    {
      caninfo("RTR message\n");
      frame_info |= (1 << 6);
    }

  flags = enter_critical_section();

  /* Make sure that TX interrupts are enabled BEFORE sending the
   * message.
   *
   * NOTE: The INT_ENA is also modified from the interrupt handler, but the
   * following is safe because interrupts are disabled here.
   */

  regval  = twai_getreg(TWAI_INT_ENA_REG);
  regval |= TWAI_TX_INT_ENA_M;
  twai_putreg(TWAI_INT_ENA_REG, regval);

  /* Set up the transfer */

#ifdef CONFIG_CAN_EXTID
  if (msg->cm_hdr.ch_extid)
    {
      /* The provided ID should be 29 bits */

      id = (uint32_t)msg->cm_hdr.ch_id;
      DEBUGASSERT((id & ~CAN_MAX_EXTMSGID) == 0);
      frame_info |= (1 << 7);
      twai_putreg(TWAI_DATA_0_REG, frame_info);

      id <<= 3;
      twai_putreg(TWAI_DATA_4_REG, id & 0xff);
      id >>= 8;
      twai_putreg(TWAI_DATA_3_REG, id & 0xff);
      id >>= 8;
      twai_putreg(TWAI_DATA_2_REG, id & 0xff);
      id >>= 8;
      twai_putreg(TWAI_DATA_1_REG, id & 0xff);
      for (i = 0; i < len; i++)
        {
          twai_putreg(TWAI_DATA_5_REG + (i * 4), msg->cm_data[i]);
        }
    }
  else
#endif
    {
      /* The provided ID should be 11 bits */

      id = (uint32_t)msg->cm_hdr.ch_id;
      DEBUGASSERT((id & ~CAN_MAX_STDMSGID) == 0);
      twai_putreg(TWAI_DATA_0_REG, frame_info);
      id <<= 5;
      twai_putreg(TWAI_DATA_1_REG, (id >> 8) & 0xff);
      twai_putreg(TWAI_DATA_2_REG, id & 0xff);
      for (i = 0; i < len; i++)
        {
          twai_putreg(TWAI_DATA_3_REG + (i * 4), msg->cm_data[i]);
          caninfo("Send data[%d]: %d\n", i, msg->cm_data[i]);
        }
    }

      /* Send the message */

  caninfo("Leaving send function + rx_err %08x + tx_err %08x\n",
          twai_getreg(TWAI_RX_ERR_CNT_REG),
          twai_getreg(TWAI_TX_ERR_CNT_REG));
#ifdef CONFIG_CAN_LOOPBACK
    caninfo("CAN_LOOPBACK %08x\n", TWAI_SELF_RX_REQ_M | TWAI_ABORT_TX_M);
    twai_putreg(TWAI_CMD_REG, TWAI_SELF_RX_REQ_M | TWAI_ABORT_TX_M);
#else
    twai_putreg(TWAI_CMD_REG, TWAI_TX_REQ_M);
#endif

  caninfo("Leaving 2 send function + rx_err %08x + tx_err %08x\n",
          twai_getreg(TWAI_RX_ERR_CNT_REG),
          twai_getreg(TWAI_TX_ERR_CNT_REG));

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

  uint32_t regval = twai_getreg(TWAI_STATUS_REG);
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

  uint32_t regval = twai_getreg(TWAI_STATUS_REG);
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
  FAR struct twai_dev_s *priv = (FAR struct twai_dev_s *)dev->cd_priv;
  struct can_hdr_s hdr;
  uint8_t data[8];

  uint32_t frame_info;
  uint32_t len;
  uint32_t datastart;
  uint32_t regval;
  uint32_t i;

  /* Read the interrupt register results in clearing bits  */

  regval = twai_getreg(TWAI_INT_RAW_REG);
  caninfo("CAN%d ICR: %08" PRIx32 "\n", priv->port, regval);

  /* Check for a receive interrupt */

  if ((regval & TWAI_RX_INT_ST_M) != 0)
    {
      caninfo("Receive interrupt\n");

      memset(&hdr, 0, sizeof(hdr));
      memset(data, 0, 8);

      frame_info = twai_getreg(TWAI_DATA_0_REG);

      /* Construct the TWAI header */

      if (frame_info & (1 << 6))
        {
          caninfo("RTR frame\n");
          hdr.ch_rtr    = 1;
        }

#ifdef CONFIG_CAN_EXTID
      if (frame_info & (1 << 7))
        {
          /* The provided ID should be 29 bits */

          hdr.ch_extid = 1;
          hdr.ch_id =
          (twai_getreg(TWAI_DATA_1_REG) << 21) +
          (twai_getreg(TWAI_DATA_2_REG) << 13) +
          (twai_getreg(TWAI_DATA_3_REG) << 5) +
          (twai_getreg(TWAI_DATA_4_REG) >> 3);
          datastart = TWAI_DATA_5_REG;
        }
      else
#endif
        {
          /* The provided ID should be 11 bits */

          hdr.ch_id =
          (twai_getreg(TWAI_DATA_1_REG) << 3) +
          (twai_getreg(TWAI_DATA_2_REG) >> 5);
          caninfo("Receive id: %d\n", hdr.ch_id);
          datastart = TWAI_DATA_3_REG;
        }

      len = frame_info & 0xf;
      if (len > TWAI_MSG_LENGTH) len = TWAI_MSG_LENGTH;
      hdr.ch_dlc = len;

      for (i = 0; i < len; i++)
        {
          data[i] = twai_getreg(datastart + (i * 4));
          caninfo("Receive data[%d]: %d\n", i, data[i]);
        }

      /* Release the receive buffer */

      twai_putreg(TWAI_CMD_REG, TWAI_RELEASE_BUF_M);

#ifdef CONFIG_CAN_ERRORS
      hdr.ch_error  = 0; /* Error reporting not supported */
#endif
      caninfo("Receive interrupt %d %d %d\n", hdr.ch_id, hdr.ch_dlc,
              hdr.ch_rtr);
      can_receive(dev, &hdr, data);
    }

  /* Check for TX buffer 1 complete */

  if ((regval & TWAI_TX_INT_ST_M) != 0)
    {
      caninfo("Transmit interrupt\n");

      /* Disable all further TX buffer interrupts */

      regval  = twai_getreg(TWAI_INT_ENA_REG);
      regval &= ~TWAI_TX_INT_ENA_M;
      twai_putreg(TWAI_INT_ENA_REG, regval);

      /* Indicate that the TX is done and a new TX buffer is available */

      can_txdone(dev);
    }

#endif
  return OK;
}

/****************************************************************************
 * Name: esp32c3twai_set_acc_filter
 *
 * Description:
 *   Call to set acceptance filter.
 *   Must be called in reset mode.
 *
 * Input Parameters:
 *   code - Acceptance Code.
 *   mask - Acceptance Mask.
 *   single_filter - Whether to enable single filter mode.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void esp32c3twai_set_acc_filter(uint32_t code, uint32_t mask,
                                       bool single_filter)
{
  caninfo("set_acc_filter\n");
  uint32_t regval;
  uint32_t code_swapped = __builtin_bswap32(code);
  uint32_t mask_swapped = __builtin_bswap32(mask);

  regval = twai_getreg(TWAI_MODE_REG);
  if (single_filter)
    {
      regval |= TWAI_RX_FILTER_MODE_M;
    }
  else
    {
      regval &= ~(TWAI_RX_FILTER_MODE_M);
    }

  twai_putreg(TWAI_MODE_REG, regval);

  for (int i = 0; i < 4; i++)
    {
      twai_putreg(TWAI_DATA_0_REG + (i * 4),
                  ((code_swapped >> (i * 8)) & 0xff));
      twai_putreg(TWAI_DATA_4_REG + (i * 4),
                  ((mask_swapped >> (i * 8)) & 0xff));
    }
}

/****************************************************************************
 * Name: twai_bittiming
 *
 * Description:
 *   Set the CAN bus timing registers based on the configured BAUD, etc.
 *
 * The bit timing logic monitors the serial bus-line and performs sampling
 * and adjustment of the sample point by synchronizing on the start-bit edge
 * and resynchronizing on the following edges.
 *
 * Its operation may be explained simply by splitting nominal bit time into
 * three segments as follows:
 *
 * 1. Synchronization segment (SYNC_SEG): a bit change is expected to occur
 *    within this time segment. It has a fixed length of one time quantum
 *    (1 x tCAN).
 * 2. Bit segment 1 (BS1): defines the location of the sample point. It
 *    includes the PROP_SEG and PHASE_SEG1 of the CAN standard. Its duration
 *    is programmable between 1 and 16 time quanta but may be automatically
 *    lengthened to compensate for positive phase drifts due to differences
 *    in the frequency of the various nodes of the network.
 * 3. Bit segment 2 (BS2): defines the location of the transmit point. It
 *    represents the PHASE_SEG2 of the CAN standard. Its duration is
 *    programmable between 1 and 8 time quanta but may also be automatically
 *    shortened to compensate for negative phase drifts.
 *
 * Pictorially:
 *
 *  |<----------------- NOMINAL BIT TIME ----------------->|
 *  |<- SYNC_SEG ->|<------ BS1 ------>|<------ BS2 ------>|
 *  |<---- Tq ---->|<----- Tbs1 ------>|<----- Tbs2 ------>|
 *
 * Where
 *   Tbs1 is the duration of the BS1 segment
 *   Tbs2 is the duration of the BS2 segment
 *   Tq is the "Time Quantum"
 *
 * Relationships:
 *
 *   baud = 1 / bit_time
 *   bit_time = Tq + Tbs1 + Tbs2
 *   Tbs1 = Tq * ts1
 *   Tbs2 = Tq * ts2
 *   Tq = brp * Tcan
 *
 * Where:
 *   Tcan is the period of the APB clock
 *
 * Input Parameters:
 *   priv - A reference to the CAN block status
 *
 * Returned Value:
 *   Zero on success; a negated errno on failure
 *
 ****************************************************************************/

//static int twai_bittiming(struct twai_dev_s *priv)
//{
//  uint32_t btr;
//  uint32_t nclks;
//  uint32_t brp;
//  uint32_t ts1;
//  uint32_t ts2;
//  uint32_t sjw;
//  uint32_t timing0;
//  uint32_t timing1;
//
//  caninfo("CAN%d PCLK: %" PRId32 " baud: %" PRId32 "\n", priv->port,
//          esp32c3_clk_apb_freq() /* / priv->divisor */ , priv->bitrate);
//
//  /* Try to get CAN_BIT_QUANTA quanta in one bit_time.
//   *
//   *   bit_time = Tq*(ts1 + ts2 + 1)
//   *   nquanta  = bit_time/Tq
//   *   Tq       = brp * Tcan
//   *   nquanta  = (ts1 + ts2 + 1)
//   *
//   *   bit_time = brp * Tcan * (ts1 + ts2 + 1)
//   *   nquanta  = bit_time / brp / Tcan
//   *   brp      = Fcan / baud / nquanta;
//   *
//   * First, calculate the number of CAN clocks in one bit time: Fcan / baud
//   */
//
//  nclks = esp32c3_clk_apb_freq() /* / priv->divisor */ / priv->baud;
//  if (nclks < TWAI_BIT_QUANTA)
//    {
//      /* At the smallest brp value (1), there are already too few bit times
//       * (CAN_CLOCK / baud) to meet our goal.  brp must be one and we need
//       * make some reasonable guesses about ts1 and ts2.
//       */
//
//      brp = 1;
//
//      /* In this case, we have to guess a good value for ts1 and ts2 */
//
//      ts1 = (nclks - 1) >> 1;
//      ts2 = nclks - ts1 - 1;
//      if (ts1 == ts2 && ts1 > 1 && ts2 < TWAI_BTR_TSEG2_MAX)
//        {
//          ts1--;
//          ts2++;
//        }
//    }
//
//  /* Otherwise, nquanta is CAN_BIT_QUANTA, ts1 is CONFIG_ESP32C3_TWAI_TSEG1,
//   * ts2 is CONFIG_ESP32C3_TWAI_TSEG2 and we calculate brp to achieve
//   * CAN_BIT_QUANTA quanta in the bit time
//   */
//
//  else
//    {
//      ts1 = CONFIG_ESP32C3_TWAI_TSEG1;
//      ts2 = CONFIG_ESP32C3_TWAI_TSEG2;
//      brp = (nclks + (TWAI_BIT_QUANTA / 2)) / TWAI_BIT_QUANTA;
//      DEBUGASSERT(brp >= 1 && brp <= TWAI_BTR_BRP_MAX);
//    }
//
//  sjw = CONFIG_ESP32C3_TWAI0_SJW;
//
//  caninfo("TS1: %" PRId32 " TS2: %" PRId32
//          " BRP: %" PRId32 " SJW= %" PRId32 "\n",
//          ts1, ts2, brp, sjw);
//
//  /* Configure bit timing */
//
//  timing0 = (priv->baud / 2) - 1;                 /* 63; */
//  timing0 |= (sjw - 1) << TWAI_SYNC_JUMP_WIDTH_S; /* (2 << TWAI_SYNC_JUMP_WIDTH_S); */
//
//  timing1 = ts1 - 1;                              /* 15 */;
//  timing1 |= (ts2 - 1) << TWAI_TIME_SEG2_S;       /* (7 << TWAI_TIME_SEG2_S); */
//
//#ifdef CONFIG_ESP32C3_TWAI0_SAM
//  /* The bus is sampled 3 times (recommended for low to medium speed buses
//   * to spikes on the bus-line).
//   */
//
//    timing1 |= CONFIG_ESP32C3_TWAI0_SAM << TWAI_TIME_SAMP_S;
//#endif
//
//  caninfo("Setting CANxBTR= 0x%08" PRIx32 "\n", btr);
//
//  twai_putreg(TWAI_BUS_TIMING_0_REG, timing0);
//  twai_putreg(TWAI_BUS_TIMING_1_REG, timing1);
//
//  return OK;
//}

static int twai_baud_rate(struct twai_dev_s *priv, int rate, int clock,
                          int sjw, int sampl_pt, int flags)
{
  FAR const struct can_bittiming_const *timing = &esp32c3_twai_bittiming_const;
  int best_error = 1000000000, error;
  int best_tseg = 0, best_brp = 0, best_rate = 0, brp = 0;
  int tseg = 0, tseg1 = 0, tseg2 = 0;
  uint32_t timing0;
  uint32_t timing1;

  unsigned short tempCR = 0;

  caninfo("(c%d)calling c_can_baud_rate(...)\n", priv->port);

  /*if (c_can_enable_configuration(pchip))
          return -ENODEV;*/

  /* tseg even = round down, odd = round up */
  for (tseg = (0 + 0 + 2) * 2;
       tseg <= (timing->tseg2_max + timing->tseg1_max + 2) * 2 + 1; tseg++)
    {
      brp = clock / ((1 + tseg / 2) * rate) + tseg % 2;
      if (brp == 0 || brp > 64)
          continue;
      error = rate - clock / (brp * (1 + tseg / 2));
      if (error < 0)
          error = -error;
      if (error <= best_error)
        {
          best_error = error;
          best_tseg = tseg / 2;
          best_brp = brp;
          best_rate = clock / (brp * (1 + tseg / 2));
        }
    }
    if (best_error && (rate / best_error < 10))
      {
        caninfo("baud rate %d is not possible with %d Hz clock\n",
               rate, clock);
        caninfo("%d bps. brp=%d, best_tseg=%d, tseg1=%d, tseg2=%d\n",
               best_rate, best_brp, best_tseg, tseg1, tseg2);
        return -EINVAL;
      }
  tseg2 = best_tseg - (sampl_pt * (best_tseg + 1)) / 100 + 1;
  caninfo("tseg2=%d, best_tseg=%d, sampl_pt=%d\n",
      tseg2, best_tseg, sampl_pt);
  if (tseg2 < 0)
      tseg2 = 0;
  if (tseg2 > timing->tseg2_max)
      tseg2 = timing->tseg2_max;
  tseg1 = best_tseg - tseg2;
  if (tseg1 > timing->tseg1_max) {
      tseg1 = timing->tseg1_max;
      tseg2 = best_tseg - tseg1;
  }

  caninfo("-> Setting %d bps.\n", best_rate);
  caninfo("->brp=%d, best_tseg=%d, tseg1=%d, tseg2=%d, sampl_pt=%d\n",
           best_brp, best_tseg, tseg1, tseg2,
           (100 * (best_tseg - tseg2) / (best_tseg + 1)));

  //read Control Register
  /*tempCR = c_can_read_reg_w(pchip, CCCR);
  //Configuration Change Enable
  c_can_write_reg_w(pchip, tempCR | CR_CCE, CCCR);
  c_can_write_reg_w(pchip,
                    ((unsigned short)tseg2) << 12 | ((unsigned short)
                                                     tseg1) << 8 |
                    (unsigned short)sjw << 6 | (unsigned short)best_brp,
                    CCBT);*/

  /*if (c_can_disable_configuration(pchip))
          return -ENODEV;*/

  /* Configure bit timing */

    timing0 = (best_brp - 1) / 2;                 /* 63; */
    timing0 |= (sjw - 1) << TWAI_SYNC_JUMP_WIDTH_S; /* (2 << TWAI_SYNC_JUMP_WIDTH_S); */

    timing1 = tseg1 - 1;
    timing1 |= (tseg2 - 1) << TWAI_TIME_SEG2_S;

  #ifdef CONFIG_ESP32C3_TWAI0_SAM
    /* The bus is sampled 3 times (recommended for low to medium speed buses
     * to spikes on the bus-line).
     */

      timing1 |= CONFIG_ESP32C3_TWAI0_SAM << TWAI_TIME_SAMP_S;
  #endif

    caninfo("Setting CANxBTR= 0x%08" PRIx32 "\n", best_brp);

    twai_putreg(TWAI_BUS_TIMING_0_REG, timing0);
    twai_putreg(TWAI_BUS_TIMING_1_REG, timing1);

    return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef CONFIG_ESP32C3_TWAI_REGDEBUG_MONITOR
static void thread_body(void *argument)
{
  irqstate_t flags;
  while (true)
    {
      caninfo("---------------------------\nInfo thread!\n");
      usleep(3000000);
      flags = enter_critical_section();
      caninfo("data0: %08x data1 %08x data2 %08x data3 %08x\n",
                twai_getreg(TWAI_DATA_0_REG),
                twai_getreg(TWAI_DATA_1_REG),
                twai_getreg(TWAI_DATA_2_REG),
                twai_getreg(TWAI_DATA_3_REG));
      caninfo("status: %08x arb %08x err %08x rx %08x tx %08x cnt %08x\n",
                      twai_getreg(TWAI_STATUS_REG),
                      twai_getreg(TWAI_ARB_LOST_CAP_REG),
                      twai_getreg(TWAI_ERR_CODE_CAP_REG),
                      twai_getreg(TWAI_RX_ERR_CNT_REG),
                      twai_getreg(TWAI_TX_ERR_CNT_REG),
                      twai_getreg(TWAI_RX_MESSAGE_CNT_REG));
      leave_critical_section(flags);
    }
}
#endif

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

  caninfo("Hello world!\n");

#ifdef CONFIG_ESP32C3_TWAI_REGDEBUG_MONITOR
  pthread_t thread;
  pthread_create(&thread, NULL, thread_body, NULL);
#endif

  caninfo("TWAI%d\n",  port);

  flags = enter_critical_section();

#ifdef CONFIG_ESP32C3_TWAI0
  if (port == 0)
    {
      /* Enable power to the TWAI module */

      caninfo("SYSTEM_PERIP_CLK_EN0_REG!\n");
      modifyreg32(SYSTEM_PERIP_CLK_EN0_REG, SYSTEM_TWAI_RST_M, 0);
      modifyreg32(SYSTEM_PERIP_CLK_EN0_REG, 0, SYSTEM_TWAI_CLK_EN_M);

      /* modifyreg32(TWAI_CLOCK_DIVIDER_REG, 0,
       * CONFIG_ESP32C3_TWAI0_DIVISOR);
       */

      /* Enable clocking to the TWAI module (not necessary... already done
       * in low level clock configuration logic).
       */

      /* Configure CAN GPIO pins */

      esp32c3_gpio_matrix_out(TWAI_TX_PIN, TWAI_TX_IDX, 0, 0);
      esp32c3_configgpio(TWAI_TX_PIN, OUTPUT_FUNCTION_1);

      esp32c3_configgpio(TWAI_RX_PIN, INPUT_FUNCTION_1);
      esp32c3_gpio_matrix_in(TWAI_RX_PIN, TWAI_RX_IDX, 0);

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
}
#endif
