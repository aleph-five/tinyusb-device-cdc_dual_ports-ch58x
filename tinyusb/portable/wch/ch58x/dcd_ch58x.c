/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Nathan Conrad
 *
 * Portions:
 * Copyright (c) 2016 STMicroelectronics
 * Copyright (c) 2019 Ha Thach (tinyusb.org)
 * Copyright (c) 2022 Simon K¨¹ppers (skuep)
 * Copyright (c) 2022 HiFiPhile
 * Copyright (c) 2023 Nikita Maltsev (aleph-five)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * This file is part of the TinyUSB stack.
 */

/**********************************************
 * This driver has been tested with the following MCUs:
 *  - CH582M
 *
 * To use this driver, you must:
 * - call tud_init();
 * - periodically call tud_task();
 *
 * Assumptions of the driver:
 * - No external D+ pull up resistor (internal pull up is used)
 * - You have the CH58x SDK in your project (https://github.com/openwch/ch583)
 *
 * Current driver limitations (i.e., a list of features for you to add):
 * - No STALL handling
 * - No double-buffering
 * - No error handling
 *
 * USB documentation and Reference implementations
 * - CH583 Datasheet (https://github.com/openwch/ch583/tree/main/Datasheet)
 * - USB 2.0 Specification
 * - CH32 CherryUSB port (https://github.com/cherry-embedded/CherryUSB/tree/master/port/ch32)
 *
 *
 * Notes:
 * - The endpoint buffers are statically allocated
 * - After reset USB peripheral clocking is enabled by default
 * - USB peripheral isn't clocked from system clock, so there is no dependency on it.
 *   There have been success tests on 60 MHz with example CDC echo project,
 *   but lower frequencies haven't been tested yet.
 */

#include "tusb_option.h"

#if CFG_TUD_ENABLED && ( CFG_TUSB_MCU == OPT_MCU_CH58X )

#include "device/dcd.h"
#include "ch58x_usb_regs.h"

#if (CFG_TUD_ENDPOINT0_SIZE != 64)
  #error "CH58x EP0 buffer size should be equal to 64"
#endif
#if TUD_OPT_HIGH_SPEED
  #error "CH58x doesn\'t support high-speed"
#endif

#if (TUP_DCD_ENDPOINT_MAX < 1)
  #error "Device should support at least one endpoint 0"
#endif

#if (TUP_DCD_ENDPOINT_MAX > 8)
  #error "CH58x supports only 8 endpoints"
#endif
  #if !defined(CH58x_MAX_PORTS)
    #define CH58x_MAX_PORTS       2
  #endif
  #if (CH58x_MAX_PORTS > 2)
    #error "CH58x supports only 2 ports"
  #endif

#if (TUSB_DIR_OUT != 0)
  #error "TUSB_DIR_OUT is not 0"
#endif

#if (TUSB_DIR_IN != 0)
  #error "TUSB_DIR_IN is not 1"
#endif

#define EP_BUF_SIZE             64
#define EP0_BUF_SIZE            CFG_TUD_ENDPOINT0_SIZE

#if EP_BUF_SIZE > 64
  #error "Max packet size for each endpoint is 64"
#endif

#define EP_INVALID_XFER_TYPE    (0xFF)

#define IS_XFER_TYPE(xfer_type) ((TUSB_XFER_CONTROL == xfer_type)\
                              || (TUSB_XFER_INTERRUPT == xfer_type)\
                              || (TUSB_XFER_BULK == xfer_type)\
                              || (TUSB_XFER_ISOCHRONOUS == xfer_type) \
                                )
#define IS_DIR(dir) ((dir == TUSB_DIR_IN) || (dir == TUSB_DIR_OUT))

typedef struct {
    uint8_t *buffer;
    uint16_t total_bytes;
    uint16_t queued_bytes;
}
xfer_t;

typedef struct {
    uint8_t *edpt_buffer;
    uint8_t xfer_type;
    uint8_t packet_size;
}
ep_config_t;

//--------------------------------------------------------------------+
// INTERNAL OBJECT & FUNCTION DECLARATION
//--------------------------------------------------------------------+

//////////////////////////////////////////////////////////////////////////////
////// ENDPOINT BUFFERS RELATED DEFINITIONS
//////////////////////////////////////////////////////////////////////////////

static uint8_t device_address;
static ep_config_t ep_config[TUP_DCD_ENDPOINT_MAX][2];
static xfer_t      ep_xfer[TUP_DCD_ENDPOINT_MAX][2];

////////// BUFFER DEFINITIONS ////////////
#if(TUP_DCD_ENDPOINT_MAX < 5)
static CFG_TUSB_MEM_ALIGN  uint8_t EP0_4_Buffer[64]; // ep0 64 bytes IN + OUT
#else
static CFG_TUSB_MEM_ALIGN  uint8_t EP0_4_Buffer[64 + 64 + 64]; // ep0 64 bytes IN + OUT, ep 64 bytes OUT, ep 64 bytes IN
#endif
static CFG_TUSB_MEM_ALIGN  uint8_t EP1_Buffer[64 + 64];
static CFG_TUSB_MEM_ALIGN  uint8_t EP2_Buffer[64 + 64];
static CFG_TUSB_MEM_ALIGN  uint8_t EP3_Buffer[64 + 64];
static CFG_TUSB_MEM_ALIGN  uint8_t EP5_Buffer[64 + 64];
static CFG_TUSB_MEM_ALIGN  uint8_t EP6_Buffer[64 + 64];
static CFG_TUSB_MEM_ALIGN  uint8_t EP7_Buffer[64 + 64];

static void device_interrupt_config(uint8_t rhport)
{
  USB_INT_EN(rhport) = RB_UIE_SUSPEND | RB_UIE_BUS_RST | RB_UIE_TRANSFER;
}

static void device_tx_rx_config(uint8_t rhport, uint8_t epnum, bool enable)
{
  if(enable)
  {
    switch(epnum)
    {
      case 1:
      {
        USB_UEP4_1_MOD(rhport) |= (RB_UEP1_RX_EN | RB_UEP1_TX_EN);
        break;
      }
      case 2:
      {
        USB_UEP2_3_MOD(rhport) |= (RB_UEP2_RX_EN | RB_UEP2_TX_EN);
        break;
      }
      case 3:
      {
        USB_UEP2_3_MOD(rhport) |= (RB_UEP3_RX_EN | RB_UEP3_TX_EN);
        break;
      }
      case 4:
      {
        USB_UEP4_1_MOD(rhport) |= (RB_UEP4_RX_EN | RB_UEP4_TX_EN);
        break;
      }
      case 5:
      {
        USB_UEP567_MOD(rhport) |= (RB_UEP5_RX_EN | RB_UEP5_TX_EN);
        break;
      }
      case 6:
      {
        USB_UEP567_MOD(rhport) |= (RB_UEP6_RX_EN | RB_UEP6_TX_EN);
        break;
      }
      case 7:
      {
        USB_UEP567_MOD(rhport) |= (RB_UEP7_RX_EN | RB_UEP7_TX_EN);
        break;
      }
    }
  }
  else
  {
    switch(epnum)
    {
      case 1:
      {
        USB_UEP4_1_MOD(rhport) &= ~(RB_UEP1_RX_EN | RB_UEP1_TX_EN);
        break;
      }
      case 2:
      {
        USB_UEP2_3_MOD(rhport) &= ~(RB_UEP2_RX_EN | RB_UEP2_TX_EN);
        break;
      }
      case 3:
      {
        USB_UEP2_3_MOD(rhport) &= ~(RB_UEP3_RX_EN | RB_UEP3_TX_EN);
        break;
      }
      case 4:
      {
        USB_UEP4_1_MOD(rhport) &= ~(RB_UEP4_RX_EN | RB_UEP4_TX_EN);
        break;
      }
      case 5:
      {
        USB_UEP567_MOD(rhport) &= ~(RB_UEP5_RX_EN | RB_UEP5_TX_EN);
        break;
      }
      case 6:
      {
        USB_UEP567_MOD(rhport) &= ~(RB_UEP6_RX_EN | RB_UEP6_TX_EN);
        break;
      }
      case 7:
      {
        USB_UEP567_MOD(rhport) &= ~(RB_UEP7_RX_EN | RB_UEP7_TX_EN);
        break;
      }
    }
  }
}

static void device_endpoints_initial_config(uint8_t rhport)
{
  ep_config[0][TUSB_DIR_OUT].edpt_buffer = EP0_4_Buffer;
  ep_config[0][TUSB_DIR_OUT].xfer_type = TUSB_XFER_CONTROL;
  ep_config[0][TUSB_DIR_OUT].packet_size = EP0_BUF_SIZE;
  ep_config[0][TUSB_DIR_IN].edpt_buffer = EP0_4_Buffer;
  ep_config[0][TUSB_DIR_IN].xfer_type = TUSB_XFER_CONTROL;
  ep_config[0][TUSB_DIR_IN].packet_size = EP0_BUF_SIZE;

#if(TUP_DCD_ENDPOINT_MAX >= 2)
  ep_config[1][TUSB_DIR_OUT].edpt_buffer = EP1_Buffer;
  ep_config[1][TUSB_DIR_OUT].xfer_type = EP_INVALID_XFER_TYPE;
  ep_config[1][TUSB_DIR_IN].edpt_buffer = &EP1_Buffer[64];
  ep_config[1][TUSB_DIR_IN].xfer_type = EP_INVALID_XFER_TYPE;
#endif
#if(TUP_DCD_ENDPOINT_MAX >= 3)
  ep_config[2][TUSB_DIR_OUT].edpt_buffer = EP2_Buffer;
  ep_config[2][TUSB_DIR_OUT].xfer_type = EP_INVALID_XFER_TYPE;
  ep_config[2][TUSB_DIR_IN].edpt_buffer = &EP2_Buffer[64];
  ep_config[2][TUSB_DIR_IN].xfer_type = EP_INVALID_XFER_TYPE;
#endif
#if(TUP_DCD_ENDPOINT_MAX >= 4)
  ep_config[3][TUSB_DIR_OUT].edpt_buffer = EP3_Buffer;
  ep_config[3][TUSB_DIR_OUT].xfer_type = EP_INVALID_XFER_TYPE;
  ep_config[3][TUSB_DIR_IN].edpt_buffer = &EP3_Buffer[64];
  ep_config[3][TUSB_DIR_IN].xfer_type = EP_INVALID_XFER_TYPE;
#endif
#if(TUP_DCD_ENDPOINT_MAX >= 5)
  ep_config[4][TUSB_DIR_OUT].edpt_buffer = &(EP0_4_Buffer[64]);
  ep_config[4][TUSB_DIR_OUT].xfer_type = EP_INVALID_XFER_TYPE;
  ep_config[4][TUSB_DIR_IN].edpt_buffer = &(EP0_4_Buffer[128]);
  ep_config[4][TUSB_DIR_IN].xfer_type = EP_INVALID_XFER_TYPE;
#endif
#if(TUP_DCD_ENDPOINT_MAX >= 6)
  ep_config[5][TUSB_DIR_OUT].edpt_buffer = EP5_Buffer;
  ep_config[5][TUSB_DIR_OUT].xfer_type = EP_INVALID_XFER_TYPE;
  ep_config[5][TUSB_DIR_IN].edpt_buffer = &EP5_Buffer[64];
  ep_config[5][TUSB_DIR_IN].xfer_type = EP_INVALID_XFER_TYPE;
#endif
#if(TUP_DCD_ENDPOINT_MAX >= 7)
  ep_config[6][TUSB_DIR_OUT].edpt_buffer = EP6_Buffer;
  ep_config[6][TUSB_DIR_OUT].xfer_type = EP_INVALID_XFER_TYPE;
  ep_config[6][TUSB_DIR_IN].edpt_buffer = &EP6_Buffer[64];
  ep_config[6][TUSB_DIR_IN].xfer_type = EP_INVALID_XFER_TYPE;
#endif
#if(TUP_DCD_ENDPOINT_MAX >= 8)
  ep_config[7][TUSB_DIR_OUT].edpt_buffer = EP7_Buffer;
  ep_config[7][TUSB_DIR_OUT].xfer_type = EP_INVALID_XFER_TYPE;
  ep_config[7][TUSB_DIR_IN].edpt_buffer = &EP7_Buffer[64];
  ep_config[7][TUSB_DIR_IN].xfer_type = EP_INVALID_XFER_TYPE;
#endif

  for(uint8_t epnum = 0; epnum < TUP_DCD_ENDPOINT_MAX; epnum++)
  {
    EPn_CTRL(rhport, epnum) = UEP_R_RES_NAK | UEP_T_RES_NAK;
    if(epnum != 4)
    {
      EP_DMA_ADDR(rhport, epnum) = (uint16_t)(uint32_t)(ep_config[epnum][TUSB_DIR_OUT].edpt_buffer);
    }
    if(epnum != 0 && epnum != 4)
    {
      EPn_CTRL(rhport, epnum) |= RB_UEP_AUTO_TOG;
    }
    device_tx_rx_config(rhport, epnum, true);
  }

  EPn_CTRL(rhport, 0) = UEP_R_RES_ACK | UEP_T_RES_NAK;
}

static void device_endpoints_initial_state(uint8_t rhport)
{
  // TODO: implement initial state
}

static void process_bus_reset(uint8_t rhport)
{
  USB_DEV_AD(rhport) = 0;
  device_address = 0;
  device_endpoints_initial_config(rhport);
  device_endpoints_initial_state(rhport);
  device_interrupt_config(rhport);
  dcd_connect(rhport);
}

static void process_bus_inactive(uint8_t rhport)
{
  // TODO: Implement later
}

static void process_bus_active(uint8_t rhport)
{
  // TODO: Implement later
}

static void device_configure_pins(uint8_t rhport)
{
  if(rhport == 0)
  {
    R16_PIN_ANALOG_IE |= RB_PIN_USB_IE | RB_PIN_USB_DP_PU;
  }
  else if (rhport == 1)
  {
    R16_PIN_ANALOG_IE |= RB_PIN_USB2_IE | RB_PIN_USB2_DP_PU;
  }
}

static void device_process_out_transaction(uint8_t rhport, uint8_t epnum)
{
  //if ( USB_INT_ST(rhport) & RB_UIS_TOG_OK )
  {
    xfer_t *endpoint_xfer = &(ep_xfer[epnum][TUSB_DIR_OUT]);
    ep_config_t *endpoint_config = &(ep_config[epnum][TUSB_DIR_OUT]);
    endpoint_xfer->queued_bytes = USB_RX_LEN(rhport);

    if(endpoint_xfer->queued_bytes != 0)
    {
      if(endpoint_xfer->buffer == NULL)
      {
        return;
      }
      if(endpoint_config->packet_size < endpoint_xfer->queued_bytes)
      {
        return;
      }
      memcpy(endpoint_xfer->buffer, endpoint_config->edpt_buffer, endpoint_xfer->queued_bytes);
      if(endpoint_config->xfer_type == TUSB_XFER_CONTROL)
      {
        uint8_t datax_mask = (EPn_CTRL(rhport, epnum) & ~(MASK_UEP_R_RES | MASK_UEP_T_RES));
        datax_mask ^= (RB_UEP_R_TOG | RB_UEP_T_TOG);
        EPn_CTRL(rhport, epnum) = datax_mask | UEP_R_RES_NAK | UEP_T_RES_NAK;
      }
      else if(endpoint_config->xfer_type == TUSB_XFER_INTERRUPT)
      {
        uint8_t datax_mask = (EPn_CTRL(rhport, epnum) & ~(MASK_UEP_R_RES));
        if(epnum == 4)
        {
          datax_mask ^= (RB_UEP_R_TOG);
        }
        EPn_CTRL(rhport, epnum) = datax_mask | UEP_R_RES_NAK;
      }
      else if(endpoint_config->xfer_type == TUSB_XFER_BULK)
      {
        uint8_t datax_mask = (EPn_CTRL(rhport, epnum) & ~(MASK_UEP_R_RES));
        if(epnum == 4)
        {
          datax_mask ^= (RB_UEP_R_TOG);
        }
        EPn_CTRL(rhport, epnum) = datax_mask | UEP_R_RES_NAK;
      }
    }
    else
    {
      if(endpoint_config->xfer_type == TUSB_XFER_CONTROL)
      {
        EPn_CTRL(rhport, epnum) = UEP_R_RES_ACK | UEP_T_RES_NAK;
      }
      else if(endpoint_config->xfer_type == TUSB_XFER_INTERRUPT)
      {
        uint8_t datax_mask = (EPn_CTRL(rhport, epnum) & ~(MASK_UEP_R_RES));
        if(epnum == 4)
        {
          datax_mask ^= (RB_UEP_R_TOG);
        }
        EPn_CTRL(rhport, epnum) = datax_mask | UEP_R_RES_NAK;
      }
      else if(endpoint_config->xfer_type == TUSB_XFER_BULK)
      {
        uint8_t datax_mask = (EPn_CTRL(rhport, epnum) & ~(MASK_UEP_R_RES));
        if(epnum == 4)
        {
          datax_mask ^= (RB_UEP_R_TOG);
        }
        EPn_CTRL(rhport, epnum) = datax_mask | UEP_R_RES_NAK;
      }
    }
    uint8_t ep_addr = tu_edpt_addr(epnum, TUSB_DIR_OUT);
    dcd_event_xfer_complete(rhport, ep_addr, endpoint_xfer->queued_bytes, XFER_RESULT_SUCCESS, true);
  }
}

static void device_process_in_transaction(uint8_t rhport, uint8_t epnum)
{
  uint8_t ep_addr = tu_edpt_addr(epnum, TUSB_DIR_IN);// (epnum | TUSB_DIR_IN_MASK);
  xfer_t *endpoint_xfer = &(ep_xfer[epnum][TUSB_DIR_IN]);
  ep_config_t *endpoint_config = &(ep_config[epnum][TUSB_DIR_IN]);

  uint16_t left_bytes = endpoint_xfer->total_bytes - endpoint_xfer->queued_bytes;
  if(left_bytes != 0)
  {
    uint8_t ep_size = endpoint_config->packet_size;
    uint16_t packet_size = TU_MIN(left_bytes, ep_size);
    memcpy(endpoint_config->edpt_buffer, &(endpoint_xfer->buffer[endpoint_xfer->queued_bytes]), packet_size);
    endpoint_xfer->queued_bytes += packet_size;
    EPn_TX_LEN(rhport, epnum) = packet_size;

    if(endpoint_config->xfer_type == TUSB_XFER_CONTROL)
    {
      EPn_CTRL(rhport, epnum) ^= (RB_UEP_T_TOG | RB_UEP_R_TOG);
    }
    else if(endpoint_config->xfer_type == TUSB_XFER_CONTROL)
    {
      if(epnum == 4)      EPn_CTRL(rhport, epnum) ^= (RB_UEP_T_TOG);
    }
    else if(endpoint_config->xfer_type == TUSB_XFER_BULK)
    {
      if(epnum == 4)      EPn_CTRL(rhport, epnum) ^= (RB_UEP_T_TOG);
    }
  }
  else
  {
    if(USB_DEV_AD(rhport) & RB_UDA_GP_BIT)
    {
      USB_DEV_AD(rhport) &= ~(RB_UDA_GP_BIT);
      USB_DEV_AD(rhport) = device_address;
    }
    if(endpoint_xfer->total_bytes != 0)
    {
      if(endpoint_config->xfer_type == TUSB_XFER_CONTROL)
      {
        uint8_t datax_mask = (EPn_CTRL(rhport, epnum) & ~(MASK_UEP_R_RES | MASK_UEP_T_RES));
        datax_mask ^= (RB_UEP_R_TOG | RB_UEP_T_TOG);
        EPn_CTRL(rhport, epnum) = datax_mask | UEP_R_RES_ACK | UEP_T_RES_NAK;
      }
    }
    else
    {
      if(endpoint_config->xfer_type == TUSB_XFER_CONTROL)
      {
        EPn_CTRL(rhport, epnum) = UEP_R_RES_ACK | UEP_T_RES_NAK;
      }
    }
    if(endpoint_config->xfer_type == TUSB_XFER_INTERRUPT)
    {
      uint8_t datax_mask = (EPn_CTRL(rhport, epnum) & ~(MASK_UEP_T_RES));
      if(epnum == 4)       datax_mask ^= (RB_UEP_T_TOG);
      EPn_CTRL(rhport, epnum) = datax_mask | UEP_T_RES_NAK;
    }
    else if(endpoint_config->xfer_type == TUSB_XFER_BULK)
    {
      uint8_t datax_mask = (EPn_CTRL(rhport, epnum) & ~(MASK_UEP_T_RES));
      if(epnum == 4)       datax_mask ^= (RB_UEP_T_TOG);
      EPn_CTRL(rhport, epnum) = datax_mask | UEP_T_RES_NAK;
    }
    dcd_event_xfer_complete(rhport, ep_addr, endpoint_xfer->total_bytes, XFER_RESULT_SUCCESS, true);
  }
}

/*------------------------------------------------------------------*/
/* Device API
 *------------------------------------------------------------------*/
void dcd_init(uint8_t rhport)
{
  TU_ASSERT(rhport < CH58x_MAX_PORTS,);

  USB_CTRL(rhport) = 0x00;
  USB_DEV_AD(rhport) = 0x00;
  device_address = 0;

  device_endpoints_initial_config(rhport);
  device_endpoints_initial_state(rhport);

  USB_CTRL(rhport) = RB_UC_DEV_PU_EN | RB_UC_INT_BUSY | RB_UC_DMA_EN;
  device_configure_pins(rhport);
  USB_INT_FG(rhport) = USB_INT_FG_RESET_ALL_MASK;
  device_interrupt_config(rhport);

  dcd_connect(rhport);
}

void dcd_int_enable(uint8_t rhport)
{
  if(rhport == 0)
  {
    PFIC_EnableIRQ(USB_IRQn);
  }
  else if(rhport == 1)
  {
    PFIC_EnableIRQ(USB2_IRQn);
  }
}

void dcd_int_disable(uint8_t rhport)
{
  if(rhport == 0)
  {
    PFIC_DisableIRQ(USB_IRQn);
  }
  else if(rhport == 1)
  {
    PFIC_DisableIRQ(USB2_IRQn);
  }
}

void dcd_set_address(uint8_t rhport, uint8_t dev_addr)
{
  // we use general-purpose bit of device address register,
  // the real address will be set after zero-length IN Status stage
  // is complete
  USB_DEV_AD(rhport) |= RB_UDA_GP_BIT;
  device_address = dev_addr;

  dcd_edpt_xfer(rhport, tu_edpt_addr(0, TUSB_DIR_IN), NULL, 0);
}

void dcd_remote_wakeup(uint8_t rhport)
{
  (void)rhport;

}

void dcd_connect(uint8_t rhport)
{
  USB_DEV_CTRL(rhport)  = RB_UD_PD_DIS | RB_UD_PORT_EN;
}

void dcd_disconnect(uint8_t rhport)
{
  USB_DEV_CTRL(rhport) &= ~(RB_UD_PD_DIS | RB_UD_PORT_EN);
}

void dcd_sof_enable(uint8_t rhport, bool en)
{
  if(en)
  {
    USB_INT_EN(rhport) |= (RB_UIE_DEV_SOF);
  }
  else
  {
    USB_INT_EN(rhport) &= ~(RB_UIE_DEV_SOF);
  }
}

//--------------------------------------------------------------------+
// Endpoint API
//--------------------------------------------------------------------+
bool dcd_edpt_open(uint8_t rhport, tusb_desc_endpoint_t const * desc_ep)
{
  uint8_t const epnum = tu_edpt_number(desc_ep->bEndpointAddress);
  tusb_dir_t const dir = tu_edpt_dir(desc_ep->bEndpointAddress);
  tusb_xfer_type_t const xfer_type = desc_ep->bmAttributes.xfer;
  uint16_t edpt_packet_size = tu_edpt_packet_size(desc_ep);

  TU_ASSERT(epnum < TUP_DCD_ENDPOINT_MAX);
  TU_ASSERT(edpt_packet_size >= 8);
  TU_ASSERT(edpt_packet_size <= EP_BUF_SIZE);
  TU_ASSERT(IS_XFER_TYPE(xfer_type));
  TU_ASSERT(xfer_type != TUSB_XFER_CONTROL);

  ep_config_t *endpoint_config = &(ep_config[epnum][dir]);

  dcd_int_disable(rhport);
  endpoint_config->xfer_type = xfer_type;
  endpoint_config->packet_size = (uint8_t)(edpt_packet_size);

  switch(dir)
  {
    case TUSB_DIR_OUT:
    {
      uint8_t clr_mask = EPn_CTRL(rhport, epnum) & ~(MASK_UEP_R_RES);
      EPn_CTRL(rhport, epnum) = clr_mask | UEP_R_RES_NAK;
      break;
    }
    case TUSB_DIR_IN:
    {
      uint8_t clr_mask = EPn_CTRL(rhport, epnum) & ~(MASK_UEP_T_RES);
      EPn_CTRL(rhport, epnum) = clr_mask | UEP_T_RES_NAK;
      break;
    }
  }

  dcd_int_enable(rhport);

  return true;
}

void dcd_edpt_close_all (uint8_t rhport)
{
  for(uint8_t epnum = 1; epnum < TUP_DCD_ENDPOINT_MAX; epnum++)
  {
    uint8_t ep_addr = tu_edpt_addr(epnum, TUSB_DIR_OUT);
    dcd_edpt_close( rhport, ep_addr);
    ep_addr = tu_edpt_addr(epnum, TUSB_DIR_IN);
    dcd_edpt_close( rhport, ep_addr);
  }
}

void dcd_edpt_close(uint8_t rhport, uint8_t ep_addr)
{
  const uint8_t epnum = tu_edpt_number(ep_addr);
  const uint8_t dir = tu_edpt_dir(ep_addr);

  TU_ASSERT(epnum < TUP_DCD_ENDPOINT_MAX, );

  xfer_t *endpoint_xfer = &(ep_xfer[epnum][dir]);
  ep_config_t *endpoint_config = &(ep_config[epnum][dir]);

  endpoint_xfer->buffer = NULL;
  endpoint_xfer->total_bytes = 0;
  endpoint_xfer->queued_bytes = 0;

  endpoint_config->xfer_type = EP_INVALID_XFER_TYPE;
  endpoint_config->packet_size = 0;

  switch(dir)
  {
    case TUSB_DIR_OUT:
    {
      uint8_t ctrl_mask = EPn_CTRL(rhport, epnum) & ~(MASK_UEP_R_RES);
      EPn_CTRL(rhport, epnum) = ctrl_mask | UEP_R_RES_NAK;
      break;
    }
    case TUSB_DIR_IN:
    {
      uint8_t ctrl_mask = EPn_CTRL(rhport, epnum) & ~(MASK_UEP_T_RES);
      EPn_CTRL(rhport, epnum) = ctrl_mask | UEP_T_RES_NAK;
      break;
    }
  }
}

bool dcd_edpt_xfer(uint8_t rhport, uint8_t ep_addr, uint8_t* buffer, uint16_t total_bytes)
{
  dcd_int_disable(rhport);
  uint8_t epnum = tu_edpt_number(ep_addr);
  uint8_t dir   = tu_edpt_dir(ep_addr);

  TU_ASSERT(IS_DIR(dir));

  xfer_t *endpoint_xfer = &(ep_xfer[epnum][dir]);
  ep_config_t *endpoint_config = &(ep_config[epnum][dir]);

  switch(dir)
  {
    case TUSB_DIR_OUT:
    {
      endpoint_xfer->buffer = buffer;
      endpoint_xfer->total_bytes = total_bytes;
      uint8_t mask = EPn_CTRL(rhport, epnum) & ~(MASK_UEP_R_RES);
      if(endpoint_config->xfer_type == TUSB_XFER_CONTROL)
      {
        mask = mask & ~(MASK_UEP_T_RES);
        EPn_CTRL(rhport, epnum) = mask | UEP_R_RES_ACK | UEP_T_RES_NAK;
      }
      else if(endpoint_config->xfer_type == TUSB_XFER_INTERRUPT)
      {
        EPn_CTRL(rhport, epnum) = mask | UEP_R_RES_ACK;
      }
      else if(endpoint_config->xfer_type == TUSB_XFER_BULK)
      {
        EPn_CTRL(rhport, epnum) = mask | UEP_R_RES_ACK;
      }
      break;
    }
    case TUSB_DIR_IN:
    {
      memset(endpoint_xfer, 0, sizeof(xfer_t));
      endpoint_xfer->buffer = buffer;
      endpoint_xfer->total_bytes = total_bytes;
      uint8_t ep_size = endpoint_config->packet_size;
      uint16_t packet_size = TU_MIN(total_bytes, ep_size);

      if(buffer != NULL)
      {
        TU_ASSERT(endpoint_config->edpt_buffer != NULL);
        memcpy(endpoint_config->edpt_buffer, buffer, packet_size);
      }

      endpoint_xfer->queued_bytes = packet_size;
      EPn_TX_LEN(rhport, epnum) = packet_size;
      uint8_t mask = EPn_CTRL(rhport, epnum) & ~(MASK_UEP_T_RES);

      if(endpoint_config->xfer_type == TUSB_XFER_CONTROL)
      {
        mask = mask & ~(MASK_UEP_R_RES);
        EPn_CTRL(rhport, epnum) = mask | UEP_R_RES_NAK | UEP_T_RES_ACK;
      }
      else if(endpoint_config->xfer_type == TUSB_XFER_INTERRUPT)
      {
        EPn_CTRL(rhport, epnum) = mask | UEP_T_RES_ACK;
      }
      else if(endpoint_config->xfer_type == TUSB_XFER_BULK)
      {
        EPn_CTRL(rhport, epnum) = mask | UEP_T_RES_ACK;
      }
      break;
    }
  }

  dcd_int_enable(rhport);
  return true;
}

void dcd_edpt_stall(uint8_t rhport, uint8_t ep_addr)
{
  const uint8_t epn = tu_edpt_number(ep_addr);
  const uint8_t dir = tu_edpt_dir(ep_addr);

  switch(dir)
  {
    case TUSB_DIR_OUT:
    {
      uint8_t mask = EP_CTRL(rhport, epn) & ~(MASK_UEP_R_RES);
      EP_CTRL(rhport, epn) = mask | UEP_R_RES_STALL;
      break;
    }
    case TUSB_DIR_IN:
    {
      uint8_t mask = EP_CTRL(rhport, epn) & ~(MASK_UEP_T_RES);
      EP_CTRL(rhport, epn) = mask | UEP_T_RES_STALL;
      break;
    }
  }
}

void dcd_edpt_clear_stall(uint8_t rhport, uint8_t ep_addr)
{
  const uint8_t epn = tu_edpt_number(ep_addr);
  const uint8_t dir = tu_edpt_dir(ep_addr);

  // TODO: Clear stall for ep0?

  switch(dir)
  {
    case TUSB_DIR_OUT:
    {
      uint8_t mask = EP_CTRL(rhport, epn) & ~(MASK_UEP_R_RES | RB_UEP_R_TOG);
      EP_CTRL(rhport, epn) = mask | UEP_R_RES_ACK;
      break;
    }
    case TUSB_DIR_IN:
    {
      uint8_t mask = EP_CTRL(rhport, epn) & ~(MASK_UEP_T_RES | RB_UEP_T_TOG);
      EP_CTRL(rhport, epn) = mask | UEP_T_RES_ACK;
      break;
    }
  }
}

//--------------------------------------------------------------------+
// ISR
//--------------------------------------------------------------------+
void dcd_int_handler(uint8_t rhport)
{
  uint8_t intFlag = USB_INT_FG(rhport);
  if (intFlag & RB_UIF_TRANSFER)
  {
    uint8_t intStatus = USB_INT_ST(rhport);
    uint8_t token = (intStatus & MASK_UIS_TOKEN);
    uint8_t epnum = intStatus & MASK_UIS_ENDP;
    if(token != UIS_TOKEN_NONE_IDLE)
    {
      if(token == UIS_TOKEN_SOF)
      {
        dcd_event_bus_signal(rhport, DCD_EVENT_SOF, true);
      }
      else if(token == UIS_TOKEN_OUT)
      {
        device_process_out_transaction(rhport, epnum);
      }
      else if(token == UIS_TOKEN_IN)
      {
        device_process_in_transaction(rhport, epnum);
      }
    }
    if (USB_INT_ST(rhport) & RB_UIS_SETUP_ACT)
    {
      dcd_event_setup_received(rhport, EP0_4_Buffer, true);
      EPn_CTRL(rhport, 0) = RB_UEP_R_TOG | RB_UEP_T_TOG | UEP_R_RES_NAK | UEP_T_RES_NAK;
    }
    USB_INT_FG(rhport) |= RB_UIF_TRANSFER;
  }
  else if (intFlag & RB_UIF_BUS_RST)
  {
    process_bus_reset(rhport);
    dcd_event_bus_reset(rhport, TUSB_SPEED_FULL, true);
    USB_INT_FG(rhport) |= RB_UIF_BUS_RST;
  }
  else if (intFlag & RB_UIF_SUSPEND)
  {
    if (USB_MIS_ST(rhport) & RB_UMS_SUSPEND)
    {
      process_bus_inactive(rhport);
      dcd_event_bus_signal(rhport, DCD_EVENT_SUSPEND, true);
    }
    else
    {
      process_bus_active(rhport);
      dcd_event_bus_signal(rhport, DCD_EVENT_RESUME, true);
    }
    USB_INT_FG(rhport) |= RB_UIF_SUSPEND;
  }
  else
  {
    USB_INT_FG(rhport) = intFlag;
  }
}

#endif // CFG_TUD_ENABLED && ( CFG_TUSB_MCU == OPT_MCU_CH58X )
