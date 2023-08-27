#ifndef _CH58X_USB_REGS_H_
#define _CH58X_USB_REGS_H_

#include "CH583SFR.h"
#include "core_riscv.h"

#define USB_BASE                   0x40008000

#define USB_CTRL_BASE              USB_BASE
#define USB_DEV_CTRL_BASE          0x40008001
#define USB_INT_EN_BASE            0x40008002
#define USB_DEV_AD_BASE            0x40008003
#define USB_MIS_ST_BASE            0x40008005
#define USB_INT_FG_BASE            0x40008006
#define USB_INT_ST_BASE            0x40008007
#define USB_RX_LEN_BASE            0x40008008

#define USB_UEP4_1_MOD_BASE        0x4000800c
#define USB_UEP2_3_MOD_BASE        0x4000800d
#define USB_UEP567_MOD_BASE        0x4000800e
#define USB_EP_DMA_REG_BASE        0x40008010
#define USB_EP_CTRL_REG_BASE       0x40008022
#define USB_EP_T_LEN_REG_BASE      0x40008020
#define USB_EP_MOD_REG_BASE        0x4000800C

#define USB2_OFFSET           0x400  // offset for USB2

#define USB_CTRL(rhport) \
  *(volatile uint8_t *)(USB2_OFFSET*rhport + USB_CTRL_BASE)
#define USB_DEV_CTRL(rhport) \
  *(volatile uint8_t *)(USB2_OFFSET*rhport + USB_DEV_CTRL_BASE)
#define USB_INT_EN(rhport) \
  *(volatile uint8_t *)(USB2_OFFSET*rhport + USB_INT_EN_BASE)
#define USB_DEV_AD(rhport) \
  *(volatile uint8_t *)(USB2_OFFSET*rhport + USB_DEV_AD_BASE)
#define USB_MIS_ST(rhport) \
  *(volatile uint8_t *)(USB2_OFFSET*rhport + USB_MIS_ST_BASE)
#define USB_INT_FG(rhport) \
  *(volatile uint8_t *)(USB2_OFFSET*rhport + USB_INT_FG_BASE)
#define USB_INT_ST(rhport) \
  *(volatile uint8_t *)(USB2_OFFSET*rhport + USB_INT_ST_BASE)
#define USB_RX_LEN(rhport) \
  *(volatile uint8_t *)(USB2_OFFSET*rhport + USB_RX_LEN_BASE)

#define USB_UEP4_1_MOD(rhport) \
  *(volatile uint8_t *)(USB2_OFFSET*rhport + USB_UEP4_1_MOD_BASE)
#define USB_UEP2_3_MOD(rhport) \
  *(volatile uint8_t *)(USB2_OFFSET*rhport + USB_UEP2_3_MOD_BASE)
#define USB_UEP567_MOD(rhport) \
  *(volatile uint8_t *)(USB2_OFFSET*rhport + USB_UEP567_MOD_BASE)

/*!< 8-bit value of endpoint control register */
#define EPn_CTRL(rhport, epn) \
  *((volatile uint8_t *)(USB2_OFFSET*rhport + USB_EP_CTRL_REG_BASE + epn * 4 + (epn / 5) * 48))

/*!< The length register value of the endpoint send buffer */
#define EPn_TX_LEN(rhport, epn) \
  *((volatile uint8_t *)(USB2_OFFSET*rhport + 0x40008020 + epn * 4 + (epn / 5) * 48))
#define EP_DMA_ADDR(rhport, epn) \
  *((volatile uint16_t *)(USB_EP_DMA_REG_BASE + rhport*USB2_OFFSET + (epn != 4)*((epn/5)*48 + 4*epn)))
#define EP_T_LEN(rhport, epn) \
  *((volatile uint8_t *)(USB_EP_T_LEN_REG_BASE + rhport*USB2_OFFSET + (epn/5)*48 + 4*epn))
#define EP_CTRL(rhport, epn) \
  *((volatile uint8_t *)(USB_EP_CTRL_REG_BASE + rhport*USB2_OFFSET + (epn/5)*48 + 4*epn))

#define UIS_TOKEN_NONE_IDLE    MASK_UIS_TOKEN
#define USB_INT_FG_RESET_ALL_MASK 0xFF

#endif /* _CH58X_USB_REGS_H_ */
